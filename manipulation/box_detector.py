import cv2
import zmq
import numpy as np
import sys
import time
from ultralytics import YOLO

ROBOT_IP = "127.0.0.1"  
ZMQ_PORT = "6002"             
TARGET_CLASS = 'bottle'
CONF_THRESHOLD = 0.5

def main():
    print("\n" + "="*60)
    print(f"🚀 IA YOLO FLUIDA (Frame Skipping activado)")
    print("="*60)

    model = YOLO('yolov8n.pt') 
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVHWM, 1) 
    socket.setsockopt(zmq.RCVTIMEO, 1000) 
    
    socket.connect(f"tcp://{ROBOT_IP}:{ZMQ_PORT}")
    socket.setsockopt_string(zmq.SUBSCRIBE, "") 
    
    print(f"📡 Sincronizado con g1_server.py... (Ctrl+C para salir)")

    frames_procesados = 0
    last_print_time = time.time()

    try:
        while True:
            try:
                zmq_msg = socket.recv()
                
                while True:
                    try:
                        zmq_msg = socket.recv(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                
                np_arr = np.frombuffer(zmq_msg, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is None: continue

                results = model(frame, stream=True, verbose=False)

                frames_procesados += 1
                current_time = time.time()
                print_now = (current_time - last_print_time) >= 1.0 

                best_bottle = None
                highest_conf = 0.0

                for r in results:
                    for box in r.boxes:
                        cls_name = model.names[int(box.cls[0])]
                        conf = float(box.conf[0])
                        if cls_name == TARGET_CLASS and conf > CONF_THRESHOLD:
                            if conf > highest_conf:
                                highest_conf = conf
                                best_bottle = box

                if best_bottle is not None:
                    x1, y1, x2, y2 = best_bottle.xyxy[0].cpu().numpy()
                    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                    if print_now:
                        sys.stdout.write(f"\r✅ [BOTELLA] Conf: {highest_conf:.2f} | Centro: [{cx}, {cy}] (Real-time)\n")
                        sys.stdout.flush()
                        last_print_time = current_time
                else:
                    if print_now:
                        sys.stdout.write(f"\r❌ Buscando... (Analizando a {frames_procesados} FPS de calidad)\n")
                        sys.stdout.flush()
                        last_print_time = current_time
                        frames_procesados = 0 

            except zmq.Again:
                sys.stdout.write("\r⚠️ Esperando señal de video...")
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\n👋 Cerrando detector.")
    finally:
        socket.close()
        context.term()

if __name__ == '__main__':
    main()
