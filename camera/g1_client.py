import tkinter as tk
from tkinter import ttk
import socket, threading, zmq, cv2, numpy as np
from PIL import Image, ImageTk
import time
import sounddevice as sd  # Nueva dependencia para audio

ROBOT_IP = '192.168.0.107'

class G1RemoteControl:
    def __init__(self, root):
        self.root = root
        self.root.title("G1 Remote Emulation Pro (Dual Cam + Realtime Audio)")
        self.root.geometry("950x880")
        self.root.configure(bg="#212121")

        # --- PROTECCIÓN DE FOCO ---
        def handle_click(event):
            try:
                if event.widget != self.tts_entry:
                    self.root.focus_set()
            except AttributeError:
                pass
                
        self.root.bind_all("<Button-1>", handle_click)

        # --- CAM SELECTOR ---
        top_frame = tk.Frame(self.root, bg="#212121")
        top_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(top_frame, text="📷 See camera:", bg="#212121", fg="white", font=("Arial", 11, "bold")).pack(side=tk.LEFT, padx=5)
        
        self.cam_var = tk.StringVar(value="RealSense")
        self.cam_dropdown = ttk.Combobox(top_frame, textvariable=self.cam_var, 
                                         values=["RealSense", "USB Camera (Port 6)"], 
                                         state="readonly", width=25)
        self.cam_dropdown.pack(side=tk.LEFT, padx=5)

        # Video
        self.video_label = tk.Label(self.root, bg="black")
        self.video_label.pack(pady=5, fill=tk.BOTH, expand=True)

        controls_frame = tk.Frame(self.root, bg="#212121")
        controls_frame.pack(fill=tk.X, padx=10)

        # Paneles de Control
        pwr_frame = tk.LabelFrame(controls_frame, text=" Energy (L2 + B) ", bg="#2c3e50", fg="white")
        pwr_frame.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.Y)
        ttk.Button(pwr_frame, text="💀 Zero Torque", command=lambda: self.send('zero')).pack(pady=5, padx=10)
        ttk.Button(pwr_frame, text="🛡️ Damping", command=lambda: self.send('damp')).pack(pady=5, padx=10)

        pos_frame = tk.LabelFrame(controls_frame, text=" Posture (L2 + A) ", bg="#2c3e50", fg="white")
        pos_frame.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.Y)
        ttk.Button(pos_frame, text="⬆️ Squat -> Stand", command=lambda: self.send('stand')).pack(pady=5, padx=10)
        ttk.Button(pos_frame, text="⬇️ Stand -> Squat", command=lambda: self.send('squat')).pack(pady=5, padx=10)

        arm_frame = tk.LabelFrame(controls_frame, text=" Arm RPC ", bg="#2c3e50", fg="white")
        arm_frame.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.Y)
        ttk.Button(arm_frame, text="🤝 Shake Hands", command=lambda: self.send('1')).pack(pady=2, padx=10)
        ttk.Button(arm_frame, text="👋 Say Hi!", command=lambda: self.send('2')).pack(pady=2, padx=10)
        ttk.Button(arm_frame, text="👏 Congrats!", command=lambda: self.send('3')).pack(pady=2, padx=10)
        
        crane_frame = tk.LabelFrame(controls_frame, text=" Rack ", bg="#d35400", fg="white")
        crane_frame.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.Y)
        ttk.Button(crane_frame, text="🏗️ Ready (L2+UP)", command=lambda: self.send('ready')).pack(pady=5, padx=10)
        ttk.Button(crane_frame, text="🏃 Motion (R2+A)", command=lambda: self.send('motion')).pack(pady=5, padx=10)
        
        move_frame = tk.LabelFrame(controls_frame, text=" Locomotion ", bg="#8e44ad", fg="white")
        move_frame.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(move_frame, text="W / S : Forward / Backward\nA / D : Sides\nQ / E : Rotate", 
                 bg="#8e44ad", fg="white", font=("Arial", 10, "bold"), justify=tk.LEFT).pack(pady=10, padx=10)

        self.status = tk.StringVar(value="Ready")
        tk.Label(self.root, textvariable=self.status, bg="#333", fg="#00FF00").pack(side=tk.BOTTOM, fill=tk.X)
        
        # --- AUDIO TTS PANEL ---
        tts_frame = tk.LabelFrame(controls_frame, text=" Audio TTS (English) ", bg="#16a085", fg="white")
        tts_frame.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.Y)
        
        self.tts_entry = ttk.Entry(tts_frame, width=25)
        self.tts_entry.pack(pady=5, padx=10)
        self.tts_entry.insert(0, "Hello, I am G1")
        ttk.Button(tts_frame, text="🗣️ Speak it!", 
                   command=lambda: self.send(f"speak:{self.tts_entry.get()}")).pack(pady=5, padx=10)

        # Keyboard setup
        self.current_key = None
        self.release_timer = None
        self.valid_keys = ['w', 'a', 's', 'd', 'q', 'e']
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.root.focus_set()

        # --- RED ZMQ (VIDEO + AUDIO) ---
        self.ctx = zmq.Context()
        
        # Socket 1 (RealSense)
        self.sock_sdk = self.ctx.socket(zmq.SUB)
        self.sock_sdk.setsockopt(zmq.CONFLATE, 1)
        self.sock_sdk.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # Socket 2 (USB)
        self.sock_usb = self.ctx.socket(zmq.SUB)
        self.sock_usb.setsockopt(zmq.CONFLATE, 1)
        self.sock_usb.setsockopt_string(zmq.SUBSCRIBE, "")

        # Socket 3 (Audio Streaming - Puerto 6003)
        self.sock_audio = self.ctx.socket(zmq.SUB)
        self.sock_audio.setsockopt(zmq.CONFLATE, 1)
        self.sock_audio.setsockopt_string(zmq.SUBSCRIBE, "")

        try: 
            self.sock_sdk.connect(f"tcp://{ROBOT_IP}:6001")
            self.sock_usb.connect(f"tcp://{ROBOT_IP}:6002")
            self.sock_audio.connect(f"tcp://{ROBOT_IP}:6003")
        except Exception as e: 
            print("Error connecting to ZMQ:", e)
        
        self.alive = True
        
        # Inicializar flujo de audio de salida (Altavoces del portátil)
        self.audio_out = sd.OutputStream(samplerate=16000, channels=1, dtype='float32')
        self.audio_out.start()

        # Iniciar hilos
        threading.Thread(target=self.ping_loop, daemon=True).start()
        threading.Thread(target=self.audio_receive_loop, daemon=True).start()
        self.update_img()

    def ping_loop(self):
        while self.alive:
            self.send('ping')
            time.sleep(0.5)

    def audio_receive_loop(self):
        print("[*] Receptor de audio en tiempo real activo...")
        while self.alive:
            try:
                raw_audio = self.sock_audio.recv(zmq.NOBLOCK)
                data = np.frombuffer(raw_audio, dtype='float32')
                self.audio_out.write(data)
            except zmq.Again:
                time.sleep(0.01)
            except Exception as e:
                pass # Errores de buffer vacio normales

    def on_key_press(self, event):
        if self.root.focus_get() == self.tts_entry:
            return
        key = event.keysym.lower()
        if key not in self.valid_keys: return
        if self.release_timer is not None:
            self.root.after_cancel(self.release_timer)
            self.release_timer = None
        if self.current_key != key:
            self.current_key = key
            self.send(key)

    def on_key_release(self, event):
        if self.root.focus_get() == self.tts_entry:
            return
        key = event.keysym.lower()
        if key == self.current_key:
            self.release_timer = self.root.after(50, self.execute_stop)

    def execute_stop(self):
        self.current_key = None
        self.send('stop')
        self.release_timer = None

    def update_img(self):
        if not self.alive: return
        selected = self.cam_var.get()
        active_sock = self.sock_sdk if "RealSense" in selected else self.sock_usb
        inactive_sock = self.sock_usb if "RealSense" in selected else self.sock_sdk
        
        try: inactive_sock.recv(zmq.NOBLOCK)
        except: pass

        try:
            raw = active_sock.recv(zmq.NOBLOCK)
            frame = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                img = ImageTk.PhotoImage(img.resize((640, 360)))
                self.video_label.configure(image=img)
                self.video_label.image = img
        except: pass
        
        self.root.after(30, self.update_img)

    def send(self, cmd):
        if cmd != 'ping': self.status.set(f"Sending {cmd.upper()}...")
        threading.Thread(target=self._worker, args=(cmd,), daemon=True).start()

    def _worker(self, cmd):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2.0); s.connect((ROBOT_IP, 6000))
                s.sendall(cmd.encode('utf-8'))
                s.settimeout(15.0); s.recv(1024)
                if cmd != 'ping': self.status.set(f"OK: {cmd.upper()} ✅")
        except Exception as e:
            if cmd != 'ping': self.status.set(f"Error: {e} ❌")

if __name__ == "__main__":
    root = tk.Tk()
    app = G1RemoteControl(root)
    root.mainloop()
