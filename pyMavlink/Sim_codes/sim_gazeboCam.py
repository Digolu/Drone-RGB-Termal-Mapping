"""
Deteção de cor com stream de video ao vivo vindo do Gazebo (simulação),
transmitido pela rede WiFi/local para qualquer PC.

Qualquer PC na mesma rede pode ver o video (com as deteções desenhadas)
abrindo no navegador:
    http://IP_DA_MAQUINA:5000

Para descobrir o IP da maquina:
    hostname -I

Requisitos:
    pip3 install flask opencv-python numpy pymavlink
    (o OpenCV tem de ter suporte a GStreamer - ver secção "gst-plugins"
    do README/instruções para confirmar com:
        python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i gstreamer
    )

Uso:
    python3 color_stream_gazebo.py

Nota sobre a fonte de video:
    Ao contrário da câmara CSI real (que usa nvarguscamerasrc), o Gazebo
    envia o video da câmara simulada como stream RTP/H264 sobre UDP
    (tipicamente na porta 5600). Este script usa cv2.VideoCapture com o
    backend GStreamer (cv2.CAP_GSTREAMER) para o receber - NÃO uses FFmpeg
    aqui (udp://@:5600), porque o FFmpeg espera um stream "cru" tipo
    MPEG-TS e não sabe descodificar RTP sem um SDP.

Nota sobre a ligação MAVLink:
    Este script assume que estás a ligar-te a um SITL/Gazebo, por isso a
    ligação é por UDP (MAVLINK_CONNECTION abaixo), não por porta serie
    (/dev/ttyACM0), que é para hardware real. Ajusta a string de ligação
    conforme o teu setup (ex: 'udp:127.0.0.1:14550' para SITL local, ou
    'udp:127.0.0.1:14551' se o teu simulador expõe noutra porta).
"""

import threading
import time
from dataclasses import dataclass
from pymavlink import mavutil

# --- Configuração da ligação MAVLink (SITL / Gazebo) ---
# Ajusta consoante o teu setup: SITL local costuma expor em 14550 ou 14551.
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"


import cv2
import numpy as np
from flask import Flask, Response

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
ultimo_beep = 0

# --- Configuração do stream RTP vindo do Gazebo ---
RTP_PORT = 5600
RTP_ENCODING = "H264"   # muda para "H265" se o teu simulador usar HEVC
RTP_PAYLOAD = 96        # confirma com o teu gst-launch-1.0 se for diferente

# Configuração de deteção mais robusta em pouca luz
LOW_LIGHT_CLAHE_CLIP = 2.0
LOW_LIGHT_BRIGHTNESS_ALPHA = 1.08
LOW_LIGHT_BRIGHTNESS_BETA = 12
DEFAULT_MIN_AREA = 250

# Janela deslizante para média de FPS (mais estável que frame-a-frame)
FPS_SMOOTHING_WINDOW = 30

# Porta onde o servidor de stream vai ficar disponivel
PORT = 5000


def construir_pipeline_rtp(port=RTP_PORT, encoding=RTP_ENCODING, payload=RTP_PAYLOAD):
    """
    Constrói a pipeline GStreamer para receber o video RTP/H264 enviado
    pelo Gazebo. Testada e confirmada com gst-launch-1.0 antes de ser
    usada aqui via cv2.VideoCapture(..., cv2.CAP_GSTREAMER).
    """
    return (
        f'udpsrc port={port} caps="application/x-rtp,media=video,'
        f'encoding-name={encoding},payload={payload}" ! '
        f"rtp{encoding.lower()}depay ! {encoding.lower()}parse ! "
        f"avdec_{encoding.lower()} ! videoconvert ! appsink drop=1"
    )


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




def abrir_stream_rtp(max_tentativas=10, espera_entre_tentativas=2.0):
    """
    Abre o stream RTP vindo do Gazebo via GStreamer. Tenta várias vezes
    porque o simulador pode ainda não estar a enviar video quando este
    script arranca (ex: Gazebo ainda a carregar o mundo).
    """
    pipeline = construir_pipeline_rtp()
    print(f"A abrir pipeline GStreamer: {pipeline}")

    for tentativa in range(1, max_tentativas + 1):
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            print(f"Stream RTP aberto com sucesso (tentativa {tentativa}).")
            return cap
        print(
            f"[Aviso] Tentativa {tentativa}/{max_tentativas} falhou a abrir "
            f"o stream RTP. A tentar novamente em {espera_entre_tentativas}s..."
        )
        cap.release()
        time.sleep(espera_entre_tentativas)

    raise RuntimeError(
        f"Não foi possível abrir o stream RTP na porta {RTP_PORT} depois de "
        f"{max_tentativas} tentativas. Confirma que o Gazebo está a correr "
        f"e a enviar video (testa com gst-launch-1.0 primeiro)."
    )


def capturar_e_processar():
    """
    Thread unico: le o stream RTP do Gazebo, corre a deteção de cor,
    desenha o debug, calcula FPS e guarda o ultimo frame codificado em JPEG.

    So existe UMA captura de video no processo todo - todos os clientes
    HTTP partilham o mesmo frame.
    """
    global _ultimo_frame_jpeg

    detector = ColorDetector(TARGETS, FRAME_WIDTH, FRAME_HEIGHT)

    cap = abrir_stream_rtp()

    tempos_frame = []
    tempos_processamento = []

    print("A correr o stream. Abre http://IP_DA_MAQUINA:5000 no navegador.")

    try:
        while True:
            t_frame_inicio = time.time()

            ok, frame = cap.read()
            if not ok:
                print("[Stream RTP] Falha a ler frame")
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

                    if atual == "vermelho":
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
        "<html><head><title>Deteção de Cor - Stream (Gazebo)</title></head>"
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