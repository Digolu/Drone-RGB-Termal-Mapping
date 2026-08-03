"""
Deteção de cor com stream de video ao vivo pela rede WiFi.

Qualquer PC na mesma rede pode ver o video (com as deteções desenhadas)
abrindo no navegador:
    http://IP_DO_JETSON:5000

Para descobrir o IP do Jetson:
    hostname -I

Requisitos:
    pip3 install flask opencv-python numpy
    (no Jetson, usa o OpenCV do sistema - ver nota no fundo do ficheiro)

Uso:
    python3 color_stream.py

Nota sobre o restart do nvargus-daemon:
    Este script já reinicia o nvargus-daemon automaticamente antes de abrir
    a câmara CSI, por isso já não precisas de correr manualmente:
        sudo systemctl restart nvargus-daemon
    Para isto funcionar sem pedir password, configura sudoers (ver README /
    instruções enviadas), permitindo NOPASSWD apenas para:
        /usr/bin/systemctl restart nvargus-daemon
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
from flask import Flask, Response

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_INDEX = 0  # muda conforme o /dev/videoX da tua câmara
ultimo_beep = 0

# Configuração de deteção mais robusta em pouca luz
LOW_LIGHT_CLAHE_CLIP = 2.0
LOW_LIGHT_BRIGHTNESS_ALPHA = 1.08
LOW_LIGHT_BRIGHTNESS_BETA = 12
DEFAULT_MIN_AREA = 250

# Janela deslizante para média de FPS (mais estável que frame-a-frame)
FPS_SMOOTHING_WINDOW = 30

# Porta onde o servidor de stream vai ficar disponivel
PORT = 5000


def reiniciar_nvargus_daemon(espera_segundos=2.0):
    """
    Reinicia o nvargus-daemon antes de abrir a câmara CSI.

    Evita o erro 'Failed to create CaptureSession' que acontece quando o
    daemon fica num estado inconsistente (ex: sessão anterior não foi
    fechada corretamente).

    Requer sudoers configurado com NOPASSWD para este comando específico,
    caso contrário o subprocess vai bloquear/falhar à espera de password.
    """
    try:
        print("A reiniciar nvargus-daemon...")
        subprocess.run(
            ["sudo", "systemctl", "restart", "nvargus-daemon"],
            check=True,
            timeout=10,
        )
        # dá tempo ao daemon para inicializar completamente antes
        # de abrir o pipeline GStreamer
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
    bgr_draw: tuple = (0, 255, 0)  # cor do retângulo/texto no debug


@dataclass
class Detection:
    name: str
    area: float
    bbox: tuple  # (x, y, w, h)
    center: tuple  # (cx, cy)


TARGETS = [
    ColorTarget("vermelho", (0, 60, 45), (10, 255, 255), DEFAULT_MIN_AREA, (0, 0, 255)),
]


class ColorDetector:
    def __init__(self, targets, frame_width, frame_height):
        self.targets = targets
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.clahe = cv2.createCLAHE(clipLimit=LOW_LIGHT_CLAHE_CLIP, tileGridSize=(8, 8))
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    def _preprocess_frame(self, frame_bgr):
        blurred = cv2.medianBlur(frame_bgr, 5)

        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l_eq = self.clahe.apply(l)
        lab_eq = cv2.merge((l_eq, a, b))

        enhanced = cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)
        enhanced = cv2.convertScaleAbs(
            enhanced,
            alpha=LOW_LIGHT_BRIGHTNESS_ALPHA,
            beta=LOW_LIGHT_BRIGHTNESS_BETA,
        )
        return enhanced

    def detect(self, frame_bgr):
        frame_for_detection = self._preprocess_frame(frame_bgr)
        hsv = cv2.cvtColor(frame_for_detection, cv2.COLOR_BGR2HSV)
        detections = []

        target = self.targets[0]
        lower = np.array(target.hsv_lower, dtype=np.uint8)
        upper = np.array(target.hsv_upper, dtype=np.uint8)

        mask1 = cv2.inRange(hsv, np.array([0, 60, 45], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8))
        mask2 = cv2.inRange(hsv, np.array([170, 60, 45], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8))
        mask = cv2.bitwise_or(mask1, mask2)

        # limpeza básica de ruído
        mask = cv2.erode(mask, self.kernel, iterations=1)
        mask = cv2.dilate(mask, self.kernel, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
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
    """
    Calcula o desvio em px do centro da imagem, separado nos 4 sentidos.
    De cada par (esquerda/direita) e (cima/baixo), só um valor fica != 0.
    """
    desvio_x = cx - center_x   # positivo = à direita do centro, negativo = à esquerda
    desvio_y = cy - center_y   # positivo = abaixo do centro, negativo = acima

    return {
        "direita":  max(desvio_x, 0),
        "esquerda": max(-desvio_x, 0),
        "baixo":    max(desvio_y, 0),
        "cima":     max(-desvio_y, 0),
    }


# --- Estado partilhado entre o thread da camara e os pedidos HTTP ---
_lock = threading.Lock()
_ultimo_frame_jpeg = None

# Desvio (em px) do vermelho relativamente ao centro da imagem, nos 4 sentidos.
# None quando não há vermelho detetado no frame atual.
_lock_desvio_vermelho = threading.Lock()
_desvio_vermelho = None  # dict: {"direita": int, "esquerda": int, "baixo": int, "cima": int}


def get_desvio_vermelho():
    """Lê de forma segura o último desvio calculado para o vermelho (ou None)."""
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
    """
    Thread unico: le a camara, corre a deteção de cor, desenha o debug,
    calcula FPS e guarda o ultimo frame codificado em JPEG.

    So existe UMA captura de camara no processo todo - todos os clientes
    HTTP partilham o mesmo frame, evitando erros de sessao duplicada.
    """
    global _ultimo_frame_jpeg

    detector = ColorDetector(TARGETS, FRAME_WIDTH, FRAME_HEIGHT)

    # Reinicia o nvargus-daemon antes de abrir a câmara CSI. Isto substitui
    # o "sudo systemctl restart nvargus-daemon" manual que era preciso
    # correr antes de iniciar o script.
    reiniciar_nvargus_daemon()

    # Camara CSI (IMX219) no Jetson Nano: precisa de passar pelo ISP via
    # GStreamer/nvarguscamerasrc. Aceder por V4L2 direto (cv2.VideoCapture(0))
    # causa erro "VIDIOC_STREAMON: Remote I/O error".
    gst_pipeline = (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width={FRAME_WIDTH}, height={FRAME_HEIGHT}, "
        f"framerate=30/1, format=NV12 ! "
        f"nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
    )
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        # Se falhou à primeira, tenta reiniciar o daemon outra vez com mais
        # tempo de espera e voltar a abrir a câmara antes de desistir.
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

    print("A correr o stream. Abre http://IP_DO_JETSON:5000 no navegador.")

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

            # métricas
            tempos_processamento.append(t_proc_fim - t_proc_inicio)
            tempos_frame.append(t_proc_fim - t_frame_inicio)
            tempos_processamento = tempos_processamento[-FPS_SMOOTHING_WINDOW:]
            tempos_frame = tempos_frame[-FPS_SMOOTHING_WINDOW:]

            fps_medio = 1.0 / (sum(tempos_frame) / len(tempos_frame))
            proc_ms_medio = (sum(tempos_processamento) / len(tempos_processamento)) * 1000

            texto_fps = f"FPS: {fps_medio:.1f}  |  Processamento: {proc_ms_medio:.1f} ms"
            cv2.putText(
                debug_frame,
                texto_fps,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            h, w = frame.shape[:2]
            center_x, center_y = w // 2, h // 2

            vermelho_detetado_neste_frame = False

            if detections:
                nomes = ", ".join(d.name for d in detections)
                print(f"[Deteção] {nomes}  |  {texto_fps}")

                for d in detections:
                    atual = str(d.name)

                    if atual == "vermelho":   # toca 1 vez
                        print("A cor detectada é vermelho")

                        cx, cy = d.center
                        desvios = calcular_desvios(cx, cy, center_x, center_y)
                        _set_desvio_vermelho(desvios)
                        vermelho_detetado_neste_frame = True

                        print(
                            f"[Vermelho] Desvio -> "
                            f"Direita: {desvios['direita']}px | "
                            f"Esquerda: {desvios['esquerda']}px | "
                            f"Baixo: {desvios['baixo']}px | "
                            f"Cima: {desvios['cima']}px"
                        )

                        # desenha centro da imagem + linha até ao objeto (debug visual)
                        cv2.circle(debug_frame, (center_x, center_y), 5, (255, 255, 255), -1)
                        cv2.line(debug_frame, (center_x, center_y), (cx, cy), (0, 0, 255), 2)
                        cv2.putText(
                            debug_frame,
                            f"D:{desvios['direita']} E:{desvios['esquerda']} "
                            f"B:{desvios['baixo']} C:{desvios['cima']}",
                            (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 255),
                            2,
                        )

                        # playTone()

            if not vermelho_detetado_neste_frame:
                _set_desvio_vermelho(None)

            ok_jpeg, buffer = cv2.imencode(
                ".jpg", debug_frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
            )
            if not ok_jpeg:
                continue

            with _lock:
                _ultimo_frame_jpeg = buffer.tobytes()
    finally:
        cap.release()


def gerar_frames():
    """Gerador usado por cada cliente HTTP: le sempre o ultimo frame guardado."""
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
        time.sleep(0.03)  # ~30 fps maximo de envio


app = Flask(__name__)


@app.route("/")
def index():
    return (
        "<html><head><title>Deteção de Cor - Stream</title></head>"
        "<body style='margin:0; background:#000;'>"
        "<img src='/video' style='width:100%; height:auto;'>"
        "</body></html>"
    )


@app.route("/video")
def video():
    return Response(
        gerar_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/desvio_vermelho")
def desvio_vermelho():
    """Endpoint JSON com o ultimo desvio do vermelho (ou null se não detetado)."""
    from flask import jsonify
    return jsonify(get_desvio_vermelho())


if __name__ == "__main__":
    thread_camera = threading.Thread(target=capturar_e_processar, daemon=True)
    thread_camera.start()

    print(f"A iniciar stream em http://0.0.0.0:{PORT}")
    app.run(host="0.0.0.0", port=PORT, threaded=True)