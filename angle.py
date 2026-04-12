from math import *
import numpy as np

# ─────────────────────────────────────────────
# Paramètres géométriques du bras PX-100
# ─────────────────────────────────────────────
L        = [10.5, 10.0, 12]   # longueurs des segments (cm)
h_base   = 10                  # hauteur de la base (cm)
max_iter = 100
tol      = 1e-3


def fabrik_3d(target):
    """
    Cinématique inverse 3D par algorithme FABRIK.
    Retourne (theta1, joints) — joints : tableau 4×2 dans le plan 2D (R, Z).
    """
    theta1   = np.arctan2(target[1], target[0])
    R_target = np.hypot(target[0], target[1])
    z_target = target[2]

    joints    = np.zeros((4, 2))
    joints[0] = [0, h_base]
    for i in range(1, 4):
        joints[i] = joints[i-1] + [L[i-1], 0]

    target_2d    = np.array([R_target, z_target])
    total_length = sum(L)
    dist_target  = np.linalg.norm(target_2d - joints[0])

    # Cible hors de portée → extension maximale
    if dist_target > total_length:
        for i in range(1, 4):
            ratio    = L[i-1] / dist_target
            joints[i] = joints[i-1] + (target_2d - joints[0]) * ratio
        return theta1, joints

    for _ in range(max_iter):
        # Passe arrière
        joints[-1] = target_2d.copy()
        for i in reversed(range(3)):
            d = joints[i] - joints[i+1]
            n = np.linalg.norm(d)
            if n < 1e-9: d, n = np.array([1.0, 0.0]), 1.0
            joints[i] = joints[i+1] + d / n * L[i]
            if joints[i][1] < 0: joints[i][1] = 0.0

        # Passe avant
        joints[0] = np.array([0.0, h_base])
        for i in range(3):
            d = joints[i+1] - joints[i]
            n = np.linalg.norm(d)
            if n < 1e-9: d, n = np.array([1.0, 0.0]), 1.0
            joints[i+1] = joints[i] + d / n * L[i]
            if joints[i+1][1] < 0: joints[i+1][1] = 0.0

        if np.linalg.norm(joints[-1] - target_2d) < tol:
            break

    return theta1, joints


def segment_angles(joints):
    v1 = joints[1] - joints[0]
    theta2 = np.arctan2(v1[1], v1[0])
    v2 = joints[2] - joints[1]
    theta3 = np.arctan2(v2[1], v2[0]) - theta2
    v3 = joints[3] - joints[2]
    theta4 = np.arctan2(v3[1], v3[0]) - (theta2 + theta3)
    return theta2, theta3, theta4


def ki(x, y):
    """
    Cinématique inverse vers (x, y, 0).
    Retourne [pos_ID1, pos_ID2, pos_ID3, pos_ID4, 0] en ticks Dynamixel.
    Facteur : 2000 ticks = 180° → 2000/π rad⁻¹
    """
    theta1, joints = fabrik_3d([x, y, 0])
    theta2, theta3, theta4 = segment_angles(joints)

    deg = 2000.0 / np.pi   # rad → ticks

    return [
        int(theta1 * deg),
        int(theta2 * deg),
        int(theta3 * deg),
        int(theta4 * deg),
        0
    ]