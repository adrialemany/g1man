#!/usr/bin/env python3
"""
g1_client_mujoco.py
===================
Cliente de teleoperación para el robot G1 en simulación MuJoCo.

Características:
  - Feed de cámara en vivo (ZMQ puerto 5555) con overlay de estado
  - Control WASD + QE por teclado
  - Joystick virtual en pantalla (click + arrastrar)
  - Panel de telemetría: velocidad, comando activo, conexión
  - Botones de control en pantalla
  - Indicador de caída del robot

Conexiones:
  - ZMQ SUB tcp://127.0.0.1:5555  → frames de cámara JPEG
  - TCP 127.0.0.1:6000             → comandos de locomoción

Uso:
    python3 teleop/g1_client_mujoco.py

Controles:
    W / S     Avanzar / Retroceder
    A / D     Strafe izquierda / derecha
    Q / E     Rotar izquierda / derecha
    ESPACIO   Stop de emergencia
    R         Resetear posición del robot
"""

import socket
import threading
import time
import os
import math

import zmq
import cv2
import numpy as np
from PIL import Image, ImageTk, ImageDraw, ImageFont
import tkinter as tk

# ============================================================
# Configuración
# ============================================================
ROBOT_IP   = '127.0.0.1'
CAM_PORT   = "5555"
CMD_PORT   = 6000
RESET_PORT = 6005   # UDP — reset de posición (mensaje "reset")

TARGET_FPS = 30
FRAME_MS   = int(1000 / TARGET_FPS)

# Dimensiones ventana
WIN_W = 900
WIN_H = 700

# Dimensiones feed cámara
CAM_W = 640
CAM_H = 480

# Joystick virtual
JOY_R     = 70     # radio del joystick en px
JOY_CX    = 800    # centro X (en la vista de joystick, relativo al panel)
JOY_CY    = 120    # centro Y

# Colores (BGR para OpenCV, RGB para todo lo demás)
C_GREEN   = (0, 255, 80)
C_RED     = (60, 20, 220)
C_YELLOW  = (0, 220, 255)
C_WHITE   = (255, 255, 255)
C_DARK    = (20, 20, 20)
C_ACCENT  = (0, 200, 255)

CMD_LABELS = {
    'w': ('FORWARD',  C_GREEN),
    's': ('BACKWARD', C_YELLOW),
    'a': ('STRAFE L', C_ACCENT),
    'd': ('STRAFE R', C_ACCENT),
    'q': ('ROTATE L', (200, 100, 255)),
    'e': ('ROTATE R', (200, 100, 255)),
    'stop': ('STOP',  C_RED),
    'ping': ('IDLE',  (120, 120, 120)),
}

# ============================================================
# Utilidades de dibujo
# ============================================================

def draw_rounded_rect(img, x1, y1, x2, y2, r, color, thickness=-1, alpha=1.0):
    """Rectángulo redondeado sobre array numpy (BGR)."""
    overlay = img.copy()
    cv2.rectangle(overlay, (x1 + r, y1), (x2 - r, y2), color, thickness)
    cv2.rectangle(overlay, (x1, y1 + r), (x2, y2 - r), color, thickness)
    cv2.circle(overlay, (x1 + r, y1 + r), r, color, thickness)
    cv2.circle(overlay, (x2 - r, y1 + r), r, color, thickness)
    cv2.circle(overlay, (x1 + r, y2 - r), r, color, thickness)
    cv2.circle(overlay, (x2 - r, y2 - r), r, color, thickness)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def put_text_centered(img, text, cx, cy, scale, color, thickness=1):
    font = cv2.FONT_HERSHEY_DUPLEX
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    cv2.putText(img, text, (cx - tw // 2, cy + th // 2), font, scale, color, thickness, cv2.LINE_AA)


def put_text(img, text, x, y, scale, color, thickness=1):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_DUPLEX, scale, color, thickness, cv2.LINE_AA)


# ============================================================
# Aplicación principal
# ============================================================
class G1TeleopClient:

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("G1 Teleop — MuJoCo Simulator")
        self.root.geometry(f"{WIN_W}x{WIN_H}")
        self.root.configure(bg="#0d0d0d")
        self.root.resizable(False, False)

        # Canvas único que cubre toda la ventana
        self.canvas = tk.Canvas(root, width=WIN_W, height=WIN_H,
                                bg="#0d0d0d", highlightthickness=0)
        self.canvas.pack()

        # Imagen Tkinter (se actualiza en el loop)
        self._tk_img = None

        # ---- Estado ----
        self.current_key  = None
        self.release_timer = None
        self.valid_keys    = ['w', 'a', 's', 'd', 'q', 'e', 'space', 'r']
        self._cmd_label    = 'ping'
        self._fps_counter  = 0
        self._fps_display  = 0
        self._last_fps_t   = time.time()
        self._frame_ok     = False
        self._conn_ok      = False
        self._fall_flag    = False

        # ---- Joystick virtual ----
        self._joy_active   = False
        self._joy_dx       = 0.0   # -1..1 horizontal (vy)
        self._joy_dy       = 0.0   # -1..1 vertical   (vx)
        self._joy_px       = 0     # posición del panel en canvas
        self._joy_py       = 0

        # ---- ZMQ cámara ----
        self.ctx      = zmq.Context()
        self.sock_cam = self.ctx.socket(zmq.SUB)
        self.sock_cam.setsockopt(zmq.CONFLATE, 1)
        self.sock_cam.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sock_cam.connect(f"tcp://{ROBOT_IP}:{CAM_PORT}")

        self._last_frame = self._make_no_signal_frame()
        self._alive       = True

        # ---- Bindings teclado ----
        root.bind('<KeyPress>',   self._on_key_press)
        root.bind('<KeyRelease>', self._on_key_release)
        root.focus_set()

        # ---- Bindings joystick ----
        self.canvas.bind('<ButtonPress-1>',   self._joy_press)
        self.canvas.bind('<B1-Motion>',        self._joy_motion)
        self.canvas.bind('<ButtonRelease-1>', self._joy_release)

        # ---- Hilos ----
        threading.Thread(target=self._cam_loop,  daemon=True).start()
        threading.Thread(target=self._ping_loop, daemon=True).start()

        # ---- Loop de render ----
        self._render_loop()

    # ============================================================
    # Cámara
    # ============================================================
    def _cam_loop(self):
        while self._alive:
            try:
                raw = self.sock_cam.recv(zmq.NOBLOCK)
                arr = np.frombuffer(raw, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    self._last_frame = frame
                    self._frame_ok   = True
                    self._fps_counter += 1
            except zmq.Again:
                time.sleep(0.005)
            except Exception:
                time.sleep(0.01)

    # ============================================================
    # Ping / keepalive
    # ============================================================
    def _ping_loop(self):
        while self._alive:
            ok = self._send_tcp('ping', silent=True)
            self._conn_ok = ok
            time.sleep(0.5)

    # ============================================================
    # Teclado
    # ============================================================
    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key not in self.valid_keys:
            return
        if self.release_timer is not None:
            self.root.after_cancel(self.release_timer)
            self.release_timer = None

        if key == 'space':
            self._send('stop')
            self.current_key = None
            return
        if key == 'r':
            self._send_reset()
            return

        if self.current_key != key:
            self.current_key = key
            self._send(key)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        if key == self.current_key:
            self.release_timer = self.root.after(60, self._do_stop)

    def _do_stop(self):
        self.current_key   = None
        self.release_timer = None
        if not self._joy_active:
            self._send('stop')

    # ============================================================
    # Joystick virtual
    # ============================================================
    def _joy_in_zone(self, ex, ey):
        """True si el click está dentro de la zona del joystick."""
        cx = self._joy_px + JOY_CX
        cy = self._joy_py + JOY_CY
        return math.hypot(ex - cx, ey - cy) < JOY_R * 2.5

    def _joy_press(self, event):
        if self._joy_in_zone(event.x, event.y):
            self._joy_active = True
            self._update_joy(event.x, event.y)

    def _joy_motion(self, event):
        if self._joy_active:
            self._update_joy(event.x, event.y)

    def _joy_release(self, event):
        if self._joy_active:
            self._joy_active = False
            self._joy_dx = 0.0
            self._joy_dy = 0.0
            self._send('stop')

    def _update_joy(self, mx, my):
        cx = self._joy_px + JOY_CX
        cy = self._joy_py + JOY_CY
        dx = mx - cx
        dy = my - cy
        dist = math.hypot(dx, dy)
        if dist > JOY_R:
            dx = dx / dist * JOY_R
            dy = dy / dist * JOY_R
        self._joy_dx =  dx / JOY_R   # +1 = derecha (strafe right)
        self._joy_dy = -dy / JOY_R   # +1 = arriba  (forward)

        # Mapear a comando discreto dominante
        if abs(self._joy_dy) >= abs(self._joy_dx):
            cmd = 'w' if self._joy_dy > 0.15 else ('s' if self._joy_dy < -0.15 else 'stop')
        else:
            cmd = 'd' if self._joy_dx > 0.15 else ('a' if self._joy_dx < -0.15 else 'stop')

        self._send(cmd)

    # ============================================================
    # Envío de comandos
    # ============================================================
    def _send(self, cmd):
        self._cmd_label = cmd
        threading.Thread(target=self._send_tcp, args=(cmd,), daemon=True).start()

    def _send_tcp(self, cmd, silent=False):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.4)
                s.connect((ROBOT_IP, CMD_PORT))
                s.sendall(cmd.encode())
                s.recv(128)
            return True
        except Exception:
            return False

    def _send_reset(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(b"reset", (ROBOT_IP, RESET_PORT))
            s.close()
        except Exception:
            pass

    # ============================================================
    # Render principal
    # ============================================================
    def _render_loop(self):
        if not self._alive:
            return

        # FPS
        now = time.time()
        if now - self._last_fps_t >= 1.0:
            self._fps_display = self._fps_counter
            self._fps_counter = 0
            self._last_fps_t  = now

        # Construir frame compuesto
        canvas_img = self._build_frame()

        # Convertir a PhotoImage y mostrar
        rgb = cv2.cvtColor(canvas_img, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(rgb)
        self._tk_img = ImageTk.PhotoImage(pil)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self._tk_img)

        self.root.after(FRAME_MS, self._render_loop)

    # ============================================================
    # Construcción del frame
    # ============================================================
    def _build_frame(self):
        """Compone la imagen completa: cámara + panel lateral + overlays."""
        canvas = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
        canvas[:] = (13, 13, 13)

        # --- Cámara (izquierda, 640x480 centrada verticalmente) ---
        cam_y_off = (WIN_H - CAM_H) // 2
        frame = cv2.resize(self._last_frame, (CAM_W, CAM_H))
        canvas[cam_y_off:cam_y_off + CAM_H, 0:CAM_W] = frame

        # Borde cámara
        border_color = C_GREEN if self._frame_ok else (80, 80, 80)
        cv2.rectangle(canvas, (0, cam_y_off), (CAM_W - 1, cam_y_off + CAM_H - 1),
                      border_color, 2)

        # --- Overlay cámara: estado conexión y FPS ---
        self._draw_camera_overlay(canvas, cam_y_off)

        # --- Panel derecho ---
        panel_x = CAM_W + 4
        self._joy_px  = panel_x
        self._joy_py  = 0
        self._draw_right_panel(canvas, panel_x)

        return canvas

    def _draw_camera_overlay(self, canvas, cam_y_off):
        """Overlays sobre el feed de cámara."""
        # Barra superior semitransparente
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, cam_y_off), (CAM_W, cam_y_off + 32), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0, canvas)

        # Título
        put_text(canvas, "G1 LIVE FEED", 8, cam_y_off + 22, 0.6, C_WHITE, 1)

        # FPS
        fps_color = C_GREEN if self._fps_display >= 20 else C_YELLOW
        put_text(canvas, f"{self._fps_display} fps", CAM_W - 80, cam_y_off + 22, 0.55, fps_color, 1)

        # Estado conexión simulador
        conn_txt   = "SIM CONNECTED" if self._conn_ok else "SIM OFFLINE"
        conn_color = C_GREEN if self._conn_ok else C_RED
        dot_color  = C_GREEN if self._conn_ok else C_RED
        cv2.circle(canvas, (8, cam_y_off + CAM_H - 14), 6, dot_color, -1)
        put_text(canvas, conn_txt, 20, cam_y_off + CAM_H - 8, 0.48, conn_color, 1)

        # Estado señal cámara
        cam_txt   = "CAM OK" if self._frame_ok else "NO SIGNAL"
        cam_color = C_GREEN if self._frame_ok else (80, 80, 80)
        put_text(canvas, cam_txt, CAM_W - 90, cam_y_off + CAM_H - 8, 0.48, cam_color, 1)

        # Comando activo — badge grande centrado en la parte baja
        label, color = CMD_LABELS.get(self._cmd_label, ('...', C_WHITE))
        if label not in ('IDLE',):
            overlay2 = canvas.copy()
            bw = 180
            bx = (CAM_W - bw) // 2
            by = cam_y_off + CAM_H - 60
            draw_rounded_rect(overlay2, bx, by, bx + bw, by + 36, 8, (0, 0, 0), -1, 1.0)
            cv2.addWeighted(overlay2, 0.6, canvas, 0.4, 0, canvas)
            put_text_centered(canvas, label, CAM_W // 2, by + 18, 0.75, color, 2)

    def _draw_right_panel(self, canvas, px):
        """Panel derecho: título, joystick, teclado, botones, telemetría."""
        pw = WIN_W - px
        ph = WIN_H

        # Fondo panel
        cv2.rectangle(canvas, (px, 0), (WIN_W - 1, ph - 1), (18, 18, 18), -1)
        cv2.line(canvas, (px, 0), (px, ph - 1), (45, 45, 45), 1)

        # ---- Título ----
        put_text_centered(canvas, "TELEOP", px + pw // 2, 22, 0.7, C_ACCENT, 1)
        cv2.line(canvas, (px + 8, 36), (WIN_W - 8, 36), (45, 45, 45), 1)

        # ---- WASD layout ----
        self._draw_wasd(canvas, px + pw // 2, 90)

        # ---- Joystick virtual ----
        jcx = px + JOY_CX
        jcy = JOY_CY + 190
        self._joy_px = px
        self._joy_py = 190
        self._draw_joystick(canvas, jcx, jcy)

        # ---- Botones ----
        self._draw_buttons(canvas, px + 8, 370, pw - 16)

        # ---- Leyenda teclas ----
        self._draw_key_legend(canvas, px + 8, 480)

    def _draw_wasd(self, canvas, cx, cy):
        """Dibuja el layout WASD con la tecla activa iluminada."""
        key_size = 36
        gap      = 4
        keys_layout = [
            ('q', -2, -1), ('w', -1, -1), ('e', 0, -1),
            ('a', -2,  0), ('s', -1,  0), ('d', 0,  0),
        ]
        active = self.current_key

        for (k, col, row) in keys_layout:
            kx = cx + col * (key_size + gap) - key_size // 2 + (key_size + gap)
            ky = cy + row * (key_size + gap)

            is_active = (k == active)
            bg = (0, 150, 60) if is_active else (35, 35, 35)
            border = C_GREEN if is_active else (70, 70, 70)

            draw_rounded_rect(canvas, kx, ky, kx + key_size, ky + key_size, 5, bg, -1, 1.0)
            cv2.rectangle(canvas, (kx, ky), (kx + key_size, ky + key_size), border, 1)

            color = C_WHITE if is_active else (160, 160, 160)
            put_text_centered(canvas, k.upper(), kx + key_size // 2, ky + key_size // 2, 0.6, color, 1)

        # Etiqueta SPACE (stop)
        sx = cx - key_size // 2 - gap // 2
        sy = cy + (key_size + gap) + 6
        sw = key_size * 3 + gap * 2
        cv2.rectangle(canvas, (sx, sy), (sx + sw, sy + 22), (50, 20, 20), -1)
        cv2.rectangle(canvas, (sx, sy), (sx + sw, sy + 22), (120, 40, 40), 1)
        put_text_centered(canvas, "SPACE = STOP", sx + sw // 2, sy + 11, 0.38, (180, 80, 80), 1)

    def _draw_joystick(self, canvas, cx, cy):
        """Joystick virtual: círculo externo + punto de posición."""
        # Base
        cv2.circle(canvas, (cx, cy), JOY_R, (40, 40, 40), -1)
        cv2.circle(canvas, (cx, cy), JOY_R, (70, 70, 70), 2)

        # Cruces guía
        cv2.line(canvas, (cx - JOY_R, cy), (cx + JOY_R, cy), (55, 55, 55), 1)
        cv2.line(canvas, (cx, cy - JOY_R), (cx, cy + JOY_R), (55, 55, 55), 1)

        # Punto stick
        if self._joy_active:
            jx = int(cx + self._joy_dx * JOY_R)
            jy = int(cy - self._joy_dy * JOY_R)
            cv2.circle(canvas, (jx, jy), 16, (0, 180, 80), -1)
            cv2.circle(canvas, (jx, jy), 16, C_GREEN, 2)
        else:
            cv2.circle(canvas, (cx, cy), 16, (55, 55, 55), -1)
            cv2.circle(canvas, (cx, cy), 16, (90, 90, 90), 2)

        # Label
        put_text_centered(canvas, "VIRTUAL JOYSTICK",
                          cx, cy + JOY_R + 16, 0.38, (100, 100, 100), 1)
        put_text_centered(canvas, "(click + drag)",
                          cx, cy + JOY_R + 30, 0.35, (70, 70, 70), 1)

    def _draw_buttons(self, canvas, bx, by, bw):
        """Botones de acción."""
        # Usamos coordenadas absolutas en canvas para los bindings
        # El binding está en el canvas completo, así que guardamos las zonas
        half = (bw - 8) // 2

        # Botón STOP
        sx, sy = bx, by
        sw, sh = bw, 34
        cv2.rectangle(canvas, (sx, sy), (sx + sw, sy + sh), (100, 20, 20), -1)
        cv2.rectangle(canvas, (sx, sy), (sx + sw, sy + sh), (200, 40, 40), 2)
        put_text_centered(canvas, "STOP [SPACE]", sx + sw // 2, sy + sh // 2, 0.55, (255, 80, 80), 1)

        # Botón RESET
        rx2, ry2 = bx, by + sh + 6
        cv2.rectangle(canvas, (rx2, ry2), (rx2 + bw, ry2 + 28), (20, 40, 80), -1)
        cv2.rectangle(canvas, (rx2, ry2), (rx2 + bw, ry2 + 28), (40, 80, 160), 1)
        put_text_centered(canvas, "RESET ROBOT [R]", rx2 + bw // 2, ry2 + 14, 0.45, (80, 140, 220), 1)

    def _draw_key_legend(self, canvas, lx, ly):
        """Leyenda compacta de controles."""
        put_text(canvas, "CONTROLS", lx, ly, 0.45, (100, 100, 100), 1)
        cv2.line(canvas, (lx, ly + 5), (lx + WIN_W - CAM_W - 20, ly + 5), (40, 40, 40), 1)
        lines = [
            "W / S  —  Forward / Backward",
            "A / D  —  Strafe Left / Right",
            "Q / E  —  Rotate Left / Right",
            "SPACE  —  Emergency Stop",
            "R      —  Reset Position",
        ]
        for i, line in enumerate(lines):
            put_text(canvas, line, lx, ly + 22 + i * 18, 0.38, (130, 130, 130), 1)

    # ============================================================
    # Frame "sin señal"
    # ============================================================
    def _make_no_signal_frame(self):
        img = np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8)
        img[:] = (18, 18, 18)
        # Patrón de ruido tenue
        noise = np.random.randint(0, 30, img.shape, dtype=np.uint8)
        img = cv2.add(img, noise)
        put_text_centered(img, "NO SIGNAL", CAM_W // 2, CAM_H // 2 - 20, 1.2, (60, 60, 60), 2)
        put_text_centered(img, "Waiting for simulator...", CAM_W // 2, CAM_H // 2 + 20,
                          0.55, (50, 50, 50), 1)
        return img

    # ============================================================
    # Cierre
    # ============================================================
    def close(self):
        self._alive = False
        try:
            self.sock_cam.close()
            self.ctx.term()
        except Exception:
            pass


# ============================================================
# Entrypoint
# ============================================================
def main():
    root = tk.Tk()
    app  = G1TeleopClient(root)

    def on_close():
        app.close()
        root.destroy()
        os._exit(0)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
