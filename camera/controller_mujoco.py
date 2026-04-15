import cv2
import numpy as np
import time
import collections
import torch
import socket
import sounddevice as sd
from PIL import Image

from insightface.app import FaceAnalysis
from inference import EmotionInference

# 🔥 CONFIGURACIÓN DE RED (IP de TU ordenador en la red local)
SIMULATOR_IP = "192.168.1.100"  # <-- QUE TU COMPAÑERA CAMBIE ESTO A TU IP
SIMULATOR_PORT = 5005

# Mapeo de la salida del modelo (0-5) a los textos que entiende tu simulador
EMOTION_MAP = {
    0: "HAPPY",
    1: "SAD",
    2: "NEUTRAL",
    3: "FRUSTRATED",
    4: "NEUTRAL",
    5: "ANGRY" 
}

# ======================
# 🔥 Face Detector
# ======================
face_app = FaceAnalysis(name='buffalo_l')
face_app.prepare(ctx_id=0)

def crop_face(frame):
    faces = face_app.get(frame)
    if len(faces) == 0:
        return None
    areas = [(f.bbox[2]-f.bbox[0])*(f.bbox[3]-f.bbox[1]) for f in faces]
    face = faces[np.argmax(areas)]
    x1, y1, x2, y2 = face.bbox.astype(int)
    H, W = frame.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(W, x2), min(H, y2)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None
    return cv2.resize(crop, (224,224))

# ======================
# 🔥 Buffers y Audio
# ======================
frame_buffer = collections.deque(maxlen=300)
audio_buffer = collections.deque(maxlen=16000*10)

def audio_callback(indata, frames, time_info, status):
    now = time.time()
    audio_data = (indata[:, 0] * 32768).astype(np.int16)
    for sample in audio_data:
        audio_buffer.append((now, sample))

def slice_by_time(buffer, t_start, t_end):
    return [x for (t, x) in buffer if t_start <= t <= t_end]

def send_emotion_command(emotion_str):
    print(f"🚀 ENVIANDO EMOCIÓN A SIMULACIÓN: {emotion_str}")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(emotion_str.encode('utf-8'), (SIMULATOR_IP, SIMULATOR_PORT))
    except Exception as e:
        print("❌ Error de red:", e)

# ======================
# 🔥 MAIN
# ======================
def main():
    print("✅ Iniciando cámara y micrófono locales...")
    
    # Iniciar captura de audio
    stream = sd.InputStream(samplerate=16000, channels=1, callback=audio_callback)
    stream.start()

    # Iniciar captura de video
    cap = cv2.VideoCapture(0)

    infer = EmotionInference("best_epoch17_val0.5943.pth")

    state = "idle"
    human_start = None
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        now = time.time()

        # Face crop
        if frame_count % 3 == 0:
            face = crop_face(frame)
            if face is not None:
                frame_buffer.append((now, face))
        frame_count += 1

        cv2.imshow("Webcam Local", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if face is None:
            continue

        # ======================
        # 🔥 STATE MACHINE
        # ======================
        if state == "idle":
            print("🟡 IDLE → collecting")
            human_start = now
            infer.frame_buffer.clear()
            state = "collecting"

        elif state == "collecting":
            infer.update_frame(face)
            if now - human_start > 3.0:  # Predecir cada 3 segundos
                human_end = now
                state = "predict"

        elif state == "predict":
            print("🟢 Predicting...")
            frames = slice_by_time(frame_buffer, human_start, human_end)
            audio = slice_by_time(audio_buffer, human_start, human_end)

            if len(frames) < 8:
                state = "idle"
                continue

            frames = frames[-16:]
            processed = []
            for f in frames:
                img = Image.fromarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))
                img = infer.transform(img)
                processed.append(img)

            frames_tensor = torch.stack(processed).unsqueeze(0).to(infer.device)
            wav = np.array(audio)
            max_len = 16000 * 6

            if len(wav) < max_len:
                wav = np.pad(wav, (0, max_len - len(wav)))
            else:
                wav = wav[:max_len]

            wav_tensor = torch.tensor(wav).float().unsqueeze(0).to(infer.device)
            
            emotion_idx = infer.predict(wav_tensor)
            emotion_str = EMOTION_MAP.get(emotion_idx, "NEUTRAL")
            
            send_emotion_command(emotion_str)
            state = "idle"

    cap.release()
    stream.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
