"""
Deteção de cor (GPU) com stream de video ao vivo pela rede WiFi.

Diferenca desta versao: o pipeline de processamento de imagem
(blur, conversao BGR->HSV, threshold de cor, erosao/dilatacao)
corre na GPU via torch/CUDA. So o cv2.findContours final fica na CPU,
porque nao ha alternativa GPU pratica para esse passo.

Qualquer PC na mesma rede pode ver o video (com as deteções desenhadas)
abrindo no navegador:
    http://IP_DO_JETSON:5000

Requisitos:
    pip3 install flask opencv-python numpy torch

Uso:
    python3 color_stream_gpu.py
"""

import threading
import time
import subprocess
from dataclasses import dataclass
from pymavlink import mavutil

drone = mavutil.mavlink_connection('/dev/ttyACM0', dialect="ardupilotmega")
drone.wait_heartbeat()

tune_string = 'MFT200A'.encode('ascii')

target_system = drone.target_system
target_component = drone.target_component

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from flask import Flask, Response

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_INDEX = 0  # muda conforme o /dev/videoX da tua câmara
ultimo_beep = 0
lasttime = 0
lasttime2 = 0

DEFAULT_MIN_AREA = 250
FPS_SMOOTHING_WINDOW = 30
PORT = 5000


def reiniciar_nvargus_daemon(espera_segundos=2.0):
    try:
        print("A reiniciar nvargus-daemon...")
        subprocess.run(
            ["sudo", "systemctl", "restart", "nvargus-daemon"],
            check=True,
            timeout=10,
        )
        time.sleep(espera_segundos)
        print("nvargus-daemon reiniciado com sucesso.")
    except subprocess.CalledProcessError as e:
        print(f"[Aviso] Falha a reiniciar nvargus-daemon: {e}")
    except FileNotFoundError:
        print("[Aviso] systemctl não encontrado (não estás num Jetson/Linux com systemd?)")
    except subprocess.TimeoutExpired:
        print("[Aviso] Timeout a reiniciar nvargus-daemon.")


@dataclass
class ColorTarget:
    name: str
    hsv_lower: tuple
    hsv_upper: tuple
    min_area: int = DEFAULT_MIN_AREA
    bgr_draw: tuple = (0, 255, 0)


@dataclass
class Detection:
    name: str
    area: float
    bbox: tuple
    center: tuple


TARGETS = [
    ColorTarget("vermelho", (0, 60, 45), (10, 255, 255), DEFAULT_MIN_AREA, (0, 0, 255)),
]


class ColorDetectorGPU:
    """
    Deteção de cor com o pipeline pesado (blur, HSV, threshold, morfologia)
    a correr em GPU via torch. So o findContours final e' feito na CPU.
    """

    def __init__(self, targets, frame_width, frame_height, device="cuda"):
        self.targets = targets
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        print(f"[ColorDetectorGPU] a usar device: {self.device}")

        self.kernel_gauss = self._make_gaussian_kernel(5, 1.0).to(self.device)
        self.morph_k = 3  # kernel 3x3, aproxima a elipse usada na versao CPU

    def _make_gaussian_kernel(self, size, sigma):
        coords = torch.arange(size, dtype=torch.float32) - size // 2
        g = torch.exp(-(coords ** 2) / (2 * sigma ** 2))
        g = g / g.sum()
        kernel_2d = g[:, None] * g[None, :]
        # depthwise: um kernel igual para cada um dos 3 canais (B, G, R)
        kernel_2d = kernel_2d.expand(3, 1, size, size).contiguous()
        return kernel_2d

    def _bgr_to_hsv_gpu(self, bgr):
        """bgr: tensor (1,3,H,W) float32 em [0,1], canais B,G,R. Devolve H,S,V
        na mesma convencao do OpenCV: H em [0,179], S e V em [0,255]."""
        b, g, r = bgr[:, 0], bgr[:, 1], bgr[:, 2]
        maxc, _ = torch.max(bgr, dim=1)
        minc, _ = torch.min(bgr, dim=1)
        v = maxc
        delta = maxc - minc
        s = torch.where(maxc > 0, delta / (maxc + 1e-8), torch.zeros_like(maxc))

        h = torch.zeros_like(maxc)
        mask = delta > 1e-8

        r_is_max = (maxc == r) & mask
        g_is_max = (maxc == g) & mask & (~r_is_max)
        b_is_max = mask & (~r_is_max) & (~g_is_max)

        h[r_is_max] = (60 * ((g[r_is_max] - b[r_is_max]) / delta[r_is_max]) + 360) % 360
        h[g_is_max] = (60 * ((b[g_is_max] - r[g_is_max]) / delta[g_is_max]) + 120)
        h[b_is_max] = (60 * ((r[b_is_max] - g[b_is_max]) / delta[b_is_max]) + 240)

        h_cv = h / 2.0
        s_cv = s * 255.0
        v_cv = v * 255.0

        return h_cv, s_cv, v_cv

    def detect(self, frame_bgr):
        target = self.targets[0]  # so' "vermelho" por agora

        # numpy (H,W,3) uint8 -> tensor (1,3,H,W) float32 [0,1] na GPU
        t = torch.from_numpy(frame_bgr).to(self.device).float() / 255.0
        t = t.permute(2, 0, 1).unsqueeze(0)

        # blur (substitui o medianBlur da versao CPU - conv gaussiana é mais rapida na GPU)
        pad = self.kernel_gauss.shape[-1] // 2
        t_blur = F.conv2d(t, self.kernel_gauss, padding=pad, groups=3)

        h_ch, s_ch, v_ch = self._bgr_to_hsv_gpu(t_blur)

        # mascara vermelho: dois intervalos de H (wrap-around 0/179)
        mask1 = (h_ch <= 10) & (s_ch >= 60) & (v_ch >= 45)
        mask2 = (h_ch >= 170) & (s_ch >= 60) & (v_ch >= 45)
        mask = (mask1 | mask2).float().unsqueeze(1)  # (1,1,H,W)

        k = self.morph_k
        pad_m = k // 2
        mask = -F.max_pool2d(-mask, kernel_size=k, stride=1, padding=pad_m)  # erosao
        mask = F.max_pool2d(mask, kernel_size=k, stride=1, padding=pad_m)    # dilatacao 1
        mask = F.max_pool2d(mask, kernel_size=k, stride=1, padding=pad_m)    # dilatacao 2

        # só volta para CPU aqui, para o findContours
        mask_np = (mask.squeeze().detach().cpu().numpy() * 255).astype(np.uint8)

        contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        if not contours:
            return detections

        maior = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(maior)
        if area < target.min_area:
            return detections

        x, y, w, h = cv2.boundingRect(maior)
        cx, cy = x + w // 2, y + h // 2
        detections.append(Detection(target.name, area, (x, y, w, h), (cx, cy)))
        return detections

    def draw_debug(self, frame_bgr, detections):
        debug = frame_bgr.copy()
        color_by_name = {t.name: t.bgr_draw for t in self.targets}

        for d in detections:
            x, y, w, h = d.bbox
            cor = color_by_name.get(d.name, (255, 255, 255))
            cv2.rectangle(debug, (x, y), (x + w, y + h), cor, 2)
            cv2.circle(debug, d.center, 4, cor, -1)
            cv2.putText(
                debug,
                f"{d.name} ({int(d.area)}px)",
                (x, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                cor,
                2,
            )
        return debug


def calcular_desvios(cx, cy, center_x, center_y):
    desvio_x = cx - center_x
    desvio_y = cy - center_y
    return {
        "direita": max(desvio_x, 0),
        "esquerda": max(-desvio_x, 0),
        "baixo": max(desvio_y, 0),
        "cima": max(-desvio_y, 0),
    }


_lock = threading.Lock()
_ultimo_frame_jpeg = None

_lock_desvio_vermelho = threading.Lock()
_desvio_vermelho = None


def get_desvio_vermelho():
    with _lock_desvio_vermelho:
        return _desvio_vermelho


def _set_desvio_vermelho(valor):
    global _desvio_vermelho
    with _lock_desvio_vermelho:
        _desvio_vermelho = valor


def playTone():
    tune_string = 'MFT200a'.encode('ascii')
    global ultimo_beep
    agora = time.time()
    if (agora - ultimo_beep) < .8:
        return
    ultimo_beep = agora
    drone.mav.play_tune_send(
        target_system=target_system,
        target_component=target_component,
        tune=tune_string
    )


def capturar_e_processar():
    global _ultimo_frame_jpeg
    global lasttime, lasttime2

    detector = ColorDetectorGPU(TARGETS, FRAME_WIDTH, FRAME_HEIGHT)

    reiniciar_nvargus_daemon()

    gst_pipeline = (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width={FRAME_WIDTH}, height={FRAME_HEIGHT}, "
        f"framerate=30/1, format=NV12 ! "
        f"nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
    )
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Primeira tentativa falhou, a tentar reiniciar o daemon outra vez...")
        cap.release()
        reiniciar_nvargus_daemon(espera_segundos=3.0)
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        raise RuntimeError(
            "Não foi possível abrir a câmara CSI. Verifica a ligação fisica "
            "e corre 'sudo systemctl restart nvargus-daemon'."
        )

    tempos_frame = []
    tempos_processamento = []

    print("A correr o stream (GPU). Abre http://IP_DO_JETSON:5000 no navegador.")

    try:
        while True:
            t_frame_inicio = time.time()

            ok, frame = cap.read()
            if not ok:
                print("[Câmara] Falha a ler frame")
                continue

            t_proc_inicio = time.time()
            detections = detector.detect(frame)
            debug_frame = detector.draw_debug(frame, detections)
            t_proc_fim = time.time()

            tempos_processamento.append(t_proc_fim - t_proc_inicio)
            tempos_frame.append(t_proc_fim - t_frame_inicio)
            tempos_processamento = tempos_processamento[-FPS_SMOOTHING_WINDOW:]
            tempos_frame = tempos_frame[-FPS_SMOOTHING_WINDOW:]

            fps_medio = 1.0 / (sum(tempos_frame) / len(tempos_frame))
            proc_ms_medio = (sum(tempos_processamento) / len(tempos_processamento)) * 1000

            texto_fps = f"FPS: {fps_medio:.1f}  |  Processamento (GPU): {proc_ms_medio:.1f} ms"
            cv2.putText(
                debug_frame, texto_fps, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
            )

            h, w = frame.shape[:2]
            center_x, center_y = w // 2, h // 2
            vermelho_detetado_neste_frame = False

            if detections:
                nomes = ", ".join(d.name for d in detections)
                current_time = time.time()
                if (current_time - lasttime) > 1:
                    print(f"\n[Deteção] {nomes}  |  {texto_fps}")
                    lasttime = current_time

                for d in detections:
                    if d.name == "vermelho":
                        cx, cy = d.center
                        desvios = calcular_desvios(cx, cy, center_x, center_y)
                        _set_desvio_vermelho(desvios)
                        vermelho_detetado_neste_frame = True

                        cv2.circle(debug_frame, (center_x, center_y), 5, (255, 255, 255), -1)
                        cv2.line(debug_frame, (center_x, center_y), (cx, cy), (0, 0, 255), 2)
                        cv2.putText(
                            debug_frame,
                            f"D:{desvios['direita']} E:{desvios['esquerda']} "
                            f"B:{desvios['baixo']} C:{desvios['cima']}",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2,
                        )

            if not vermelho_detetado_neste_frame:
                _set_desvio_vermelho(None)

            ok_jpeg, buffer = cv2.imencode(".jpg", debug_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok_jpeg:
                continue

            with _lock:
                _ultimo_frame_jpeg = buffer.tobytes()
    finally:
        cap.release()


def gerar_frames():
    while True:
        with _lock:
            frame_bytes = _ultimo_frame_jpeg
        if frame_bytes is None:
            time.sleep(0.05)
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )
        time.sleep(0.03)


app = Flask(__name__)


@app.route("/")
def index():
    return (
        "<html><head><title>Deteção de Cor (GPU) - Stream</title></head>"
        "<body style='margin:0; background:#000;'>"
        "<img src='/video' style='width:100%; height:auto;'>"
        "</body></html>"
    )


@app.route("/video")
def video():
    return Response(gerar_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/desvio_vermelho")
def desvio_vermelho():
    from flask import jsonify
    return jsonify(get_desvio_vermelho())


if __name__ == "__main__":
    thread_camera = threading.Thread(target=capturar_e_processar, daemon=True)
    thread_camera.start()

    print(f"A iniciar stream em http://0.0.0.0:{PORT}")
    app.run(host="0.0.0.0", port=PORT, threaded=True)