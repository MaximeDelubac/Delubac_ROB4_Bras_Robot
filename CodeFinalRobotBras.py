from flask import Flask, Response, render_template, request
import cv2
from ultralytics import YOLO
import math
import serial
import threading
import time
#ajout
import mvtbras

# ----------------------
# Robot COM port
# ----------------------
arduino = serial.Serial(port='COM6', baudrate=9600, timeout=1)

#ajout
center = False

def attraper():
    print("attraper objet")
    mvtbras.attraper_objet(get_dist(),0)

def get_dist():
    arduino.reset_input_buffer()
    line = arduino.readline().decode().strip()
    print("test:",line)
    if line.isdigit():
            distance = int(line)
            print(distance)
    return distance

#ajoutfin

def envoyer_commande(action):
    actions = ['Avancer','Reculer','Droite','Gauche','Arret']
    cmd = actions.index(action)
    arduino.write(str(cmd+1).encode())
    print(f"Commande envoyée : {action} ({cmd+1})")

def recadrage_objet():
    pass

# ----------------------
# YOLO & tracking params
# ----------------------
CONF_THRESHOLD = 0.5
TRACKER_TYPE = "CSRT"
YOLO_REFRESH = 15
MAX_MATCH_DIST = 60
PREVIEW_COLOR = (0, 0, 255)
TRACK_COLOR = (0, 255, 0)

click_point = None
tracking = False
tracker = None
frame_id = 0
current_frame = None
lock = threading.Lock()

# ----------------------
# YOLO model
# ----------------------
model = YOLO("yolov8n.pt")

# ----------------------
# Initialize camera
# ----------------------
CAM_INDEX = 0  # your camera index (built-in or external)
camera = cv2.VideoCapture(CAM_INDEX)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


cam_cx, cam_cy = 640 // 2, 480 // 2

# Wait for camera to warm up (non-blocking)
time.sleep(2)

# ----------------------
# Tracker factory
# ----------------------
def create_tracker():
    return cv2.TrackerCSRT_create()

# ----------------------
# Video + YOLO thread
# ----------------------
def video_loop():
    global current_frame, click_point, tracking, tracker, frame_id,center
    while True:
        ret, frame = camera.read()
        if not ret:
            time.sleep(0.1)
            continue

        frame_id += 1

        # ------------------ Phase 1: YOLO preview before click
        if not tracking:
            results = model(frame, conf=CONF_THRESHOLD, verbose=False)
            boxes = []

            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    cx, cy = x1 + w // 2, y1 + h // 2
                    boxes.append((x1, y1, w, h, cx, cy))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), PREVIEW_COLOR, 2)
                    cv2.circle(frame, (cx, cy), 3, PREVIEW_COLOR, -1)

            if click_point is not None:
                best = None
                best_dist = float("inf")
                for (x, y, w, h, cx, cy) in boxes:
                    d = math.hypot(cx - click_point[0], cy - click_point[1])
                    if d < best_dist:
                        best_dist = d
                        best = (x, y, w, h)

                if best is not None:
                    tracker = create_tracker()
                    tracker.init(frame, best)
                    tracking = True
                    click_point = None
                    frame_id = 0
                    print("Objet sélectionné")

        # ------------------ Phase 2: Tracking
        else:
            #ajout
            dist = get_dist()

            if tracking and center and dist>20:
                envoyer_commande('Avancer')

            if tracking and center and dist<20:
                attraper()
            #ajout fin
            success, bbox = tracker.update(frame)
            if not success:
                print("Objet perdu — recliquez pour sélectionner")
                tracking = False
                tracker = None
            else:
                x, y, w, h = map(int, bbox)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), TRACK_COLOR, 3)
                cv2.circle(frame, (cx, cy), 5, TRACK_COLOR, -1)

                dx = cx - cam_cx  # positif → objet à droite
                dy = cy - cam_cy  
                print(dx, 'distance', dist,'center:', center, 'objet:', tracking)
                if dx>30:
                    envoyer_commande('Droite')
                    center = False
                if dx<-30:
                    envoyer_commande('Gauche')
                    center = False
                if -30<dx<30 and not center:
                    #ajout
                    center = True
                    #ajoutfin
                    envoyer_commande('Arret')

                if frame_id % YOLO_REFRESH == 0:
                    results = model(frame, conf=CONF_THRESHOLD, verbose=False)
                    best_box = None
                    min_dist = float("inf")
                    for r in results:
                        for box in r.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            bw, bh = x2 - x1, y2 - y1
                            bx, by = x1 + bw // 2, y1 + bh // 2
                            d = math.hypot(bx - cx, by - cy)
                            if d < min_dist:
                                min_dist = d
                                best_box = (x1, y1, bw, bh)
                    if best_box is not None and min_dist < MAX_MATCH_DIST:
                        tracker = create_tracker()
                        tracker.init(frame, best_box)
                        #print("Taille corrigée par YOLO")

        with lock:
            current_frame = frame.copy()

# ----------------------
# Flask app
# ----------------------
app = Flask(__name__)

def gen_frames():
    global current_frame
    while True:
        with lock:
            if current_frame is None:
                continue
            ret, buffer = cv2.imencode('.jpg', current_frame)
            frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index_tracking.html')  # use your new HTML

@app.route('/video')
def video():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/command', methods=['POST'])
def command():
    data = request.get_json(force=True)
    cmd = data.get("cmd")
    envoyer_commande(cmd)
    return {"status": "ok"}

@app.route('/click', methods=['POST'])
def click():
    global click_point
    data = request.get_json(force=True)
    x = int(data.get('x') * 640 / 640)
    y = int(data.get('y') * 480 / 480)
    click_point = (x, y)
    print(f"Clic reçu : x={x}, y={y}")
    return {"status": "ok"}

# ----------------------
# Run
# ----------------------
threading.Thread(target=video_loop, daemon=True).start()
app.run(host='0.0.0.0', port=5000)
