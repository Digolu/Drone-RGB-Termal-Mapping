"""
Servidor de simulação da câmara (substitui o Jetson/camara real durante testes).

Expõe o mesmo endpoint que o teu script já espera:
    GET http://<host>:5000/desvio_vermelho

Resposta:
    - JSON com {"esquerda":.., "direita":.., "cima":.., "baixo":..,
                "norte":.., "este":.., "tamanho":..}
    - ou 404 (corpo vazio) se o checkbox "Sem deteção" estiver ativo,
      simulando o caso em que a câmara não vê vermelho (get_desvio_vermelho
      no teu script devolve None nesse caso, exatamente como quando a
      câmara real está inacessível ou não deteta nada).

Dois modos na página web:
    - "Sliders": tal como antes, 4 sliders (Norte/Sul/Este/Oeste).
    - "Rato": aparece um retângulo a representar a câmara, com um ponto
      no centro (centro da câmara). Move o rato dentro do retângulo e os
      desvios são calculados automaticamente a partir da posição do rato
      em relação ao centro. As teclas Q / A aumentam / diminuem o
      "tamanho" do alvo (círculo à volta do cursor), útil para simular
      um objeto que se aproxima ou afasta.

Corre com:
    pip install flask
    python camera_sim_server.py

Depois abre no browser:
    http://localhost:5050/
"""

from flask import Flask, jsonify, request, Response

app = Flask(__name__)

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Amplitude máxima (em "pixels" simulados) de cada desvio, tanto nos
# sliders como no modo rato (o retângulo do rato mapeia 1:1 para este
# intervalo, de -RANGE a +RANGE em cada eixo).
RANGE = 200

TAMANHO_MIN = 5
TAMANHO_MAX = 100
TAMANHO_DEFAULT = 20

# Estado atual dos desvios (partilhado entre a página web e o endpoint da API)
state = {
    "esquerda": 0,
    "direita": 0,
    "cima": 0,
    "baixo": 0,
    "tamanho": TAMANHO_DEFAULT,
    "sem_deteccao": False,
    "x": 0,
    "y": 0
}

PAGE = """
<!doctype html>
<html lang="pt">
<head>
<meta charset="utf-8">
<title>Simulador de Desvios - Câmara</title>
<style>
  body { font-family: sans-serif; max-width: 560px; margin: 40px auto; }
  h1 { font-size: 1.3rem; }
  .row { margin-bottom: 22px; }
  label { display: flex; justify-content: space-between; font-weight: bold; }
  input[type=range] { width: 100%; }
  .val { font-weight: normal; color: #555; }
  .toggle { display: flex; align-items: center; gap: 8px; margin-top: 12px; }
  .status { margin-top: 12px; font-size: 0.9rem; color: #777; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
  .mode-select { display: flex; gap: 20px; margin-bottom: 20px; font-weight: bold; }
  .mode-select label { font-weight: normal; display: flex; align-items: center; gap: 6px; }

  #camera-rect {
    position: relative;
    width: 400px;
    height: 400px;
    margin: 20px auto;
    border: 2px solid #333;
    background: #f5f5f5;
    cursor: crosshair;
    overflow: hidden;
  }
  #crosshair-h, #crosshair-v { position: absolute; background: #ddd; }
  #crosshair-h { left: 0; right: 0; top: 200px; height: 1px; }
  #crosshair-v { top: 0; bottom: 0; left: 200px; width: 1px; }
  #center-dot {
    position: absolute; width: 8px; height: 8px; margin: -4px;
    left: 200px; top: 200px; background: #333; border-radius: 50%;
  }
  #mouse-target {
    position: absolute; border: 2px solid #d33; border-radius: 50%;
    background: rgba(211,51,51,0.15); pointer-events: none;
  }
  .hint { font-size: 0.85rem; color: #777; text-align: center; }
</style>
</head>
<body>

<h1>Simulador de desvios da câmara (vermelho)</h1>

<div class="mode-select">
  <label><input type="radio" name="modo" value="sliders" checked> Sliders</label>
  <label><input type="radio" name="modo" value="rato"> Rato</label>
</div>

<div class="toggle">
  <input type="checkbox" id="sem_deteccao">
  <label for="sem_deteccao" style="font-weight:normal;">Sem deteção (simula câmara sem vermelho)</label>
</div>

<!-- ===== Modo sliders ===== -->
<div id="sliders-section">
  <div class="grid">
    <div class="row">
      <label>Norte <span class="val" id="v_esquerda">0</span>px</label>
      <input type="range" min="0" max="200" value="0" id="esquerda">
    </div>
    <div class="row">
      <label>Sul <span class="val" id="v_direita">0</span>px</label>
      <input type="range" min="0" max="200" value="0" id="direita">
    </div>
    <div class="row">
      <label>Este <span class="val" id="v_cima">0</span>px</label>
      <input type="range" min="0" max="200" value="0" id="cima">
    </div>
    <div class="row">
      <label>Oeste <span class="val" id="v_baixo">0</span>px</label>
      <input type="range" min="0" max="200" value="0" id="baixo">
    </div>
  </div>
</div>

<!-- ===== Modo rato ===== -->
<div id="mouse-section" style="display:none;">
  <p class="hint">
    Move o rato dentro do retângulo (o ponto é o centro da câmara).<br>
    Usa <b>Q</b> / <b>A</b> para aumentar / diminuir o tamanho do alvo.
  </p>
  <div id="camera-rect">
    <div id="crosshair-h"></div>
    <div id="crosshair-v"></div>
    <div id="mouse-target"></div>
    <div id="center-dot"></div>
  </div>
</div>

<div class="status" id="status">a atualizar...</div>

<script>
const RECT_SIZE = 400;      // px do retângulo, mapeado 1:1 para o intervalo -200..200
const CENTER = RECT_SIZE / 2;
const TAMANHO_MIN = {tamanho_min};
const TAMANHO_MAX = {tamanho_max};
const TAMANHO_STEP = 5;

let modo = "sliders";
let tamanho = {tamanho_default};
let lastX = CENTER;
let lastY = CENTER;
let sending = false;
let pendingBody = null;

const sliderIds = ["esquerda", "direita", "cima", "baixo"];
const rectEl = document.getElementById("camera-rect");
const targetEl = document.getElementById("mouse-target");
const statusEl = document.getElementById("status");

function throttledSend(body) {
  pendingBody = body;
  if (sending) return;
  sending = true;
  fetch("/set", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(pendingBody)
  }).finally(() => {
    sending = false;
    statusEl.textContent = "Última atualização: " + new Date().toLocaleTimeString();
  });
}

// ---------- Modo sliders ----------
function sendSlidersState() {
  const body = {};
  sliderIds.forEach(id => body[id] = parseInt(document.getElementById(id).value, 10));
  body.sem_deteccao = document.getElementById("sem_deteccao").checked;
  throttledSend(body);
}

sliderIds.forEach(id => {
  const el = document.getElementById(id);
  el.addEventListener("input", () => {
    document.getElementById("v_" + id).textContent = el.value;
    sendSlidersState();
  });
});

document.getElementById("sem_deteccao").addEventListener("change", () => {
  if (modo === "sliders") sendSlidersState();
  else sendMouseState(lastX, lastY);
});

// ---------- Modo rato ----------
function updateTargetVisual(x, y) {
  targetEl.style.width = (tamanho * 2) + "px";
  targetEl.style.height = (tamanho * 2) + "px";
  targetEl.style.left = (x - tamanho) + "px";
  targetEl.style.top = (y - tamanho) + "px";
}

function sendMouseState(x, y) {
  lastX = x;
  lastY = y;
  updateTargetVisual(x, y);

  const dx = x - CENTER;   // positivo = rato à direita do centro
  const dy = y - CENTER;   // positivo = rato abaixo do centro

  const norte = -dx;       // esquerda (norte) positivo quando o rato está à esquerda
  const este = -dy;        // cima (este) positivo quando o rato está acima

  const esquerda = Math.max(0, norte);
  const direita = Math.max(0, -norte);
  const cima = Math.max(0, este);
  const baixo = Math.max(0, -este);

  statusEl.textContent =
    `norte: ${Math.round(norte)}  este: ${Math.round(este)}  tamanho: ${tamanho}`;

  throttledSend({
    esquerda, direita, cima, baixo,
    norte, este, tamanho,
    sem_deteccao: document.getElementById("sem_deteccao").checked
  });
}

rectEl.addEventListener("mousemove", (e) => {
  if (modo !== "rato") return;
  const bounds = rectEl.getBoundingClientRect();
  let x = e.clientX - bounds.left;
  let y = e.clientY - bounds.top;
  x = Math.max(0, Math.min(RECT_SIZE, x));
  y = Math.max(0, Math.min(RECT_SIZE, y));
  sendMouseState(x, y);
});

document.addEventListener("keydown", (e) => {
  if (modo !== "rato") return;
  if (e.key === "q" || e.key === "Q") {
    tamanho = Math.min(TAMANHO_MAX, tamanho + TAMANHO_STEP);
  } else if (e.key === "a" || e.key === "A") {
    tamanho = Math.max(TAMANHO_MIN, tamanho - TAMANHO_STEP);
  } else {
    return;
  }
  sendMouseState(lastX, lastY);
});

// ---------- Alternar entre modos ----------
document.querySelectorAll('input[name="modo"]').forEach(radio => {
  radio.addEventListener("change", (e) => {
    modo = e.target.value;
    document.getElementById("sliders-section").style.display =
      (modo === "sliders") ? "block" : "none";
    document.getElementById("mouse-section").style.display =
      (modo === "rato") ? "block" : "none";

    if (modo === "sliders") {
      sendSlidersState();
    } else {
      updateTargetVisual(lastX, lastY);
      sendMouseState(lastX, lastY);
    }
  });
});

// estado inicial
updateTargetVisual(lastX, lastY);
</script>
</body>
</html>
""".replace("{tamanho_min}", str(TAMANHO_MIN)) \
   .replace("{tamanho_max}", str(TAMANHO_MAX)) \
   .replace("{tamanho_default}", str(TAMANHO_DEFAULT))


@app.route("/")
def index():
    return PAGE


@app.route("/set", methods=["POST"])
def set_state():
    data = request.get_json(force=True)
    for k in ("esquerda", "direita", "cima", "baixo"):
        if k in data:
            state[k] = int(data[k])
    if "tamanho" in data:
        state["tamanho"] = max(TAMANHO_MIN, min(TAMANHO_MAX, int(data["tamanho"])))
    if "sem_deteccao" in data:
        state["sem_deteccao"] = bool(data["sem_deteccao"])
    return jsonify(ok=True)


@app.route("/desvio_vermelho")
def desvio_vermelho():
    if state["sem_deteccao"]:
        # Simula "sem vermelho detetado": devolve 404, que faz o
        # r.raise_for_status() do teu script disparar RequestException,
        # levando get_desvio_vermelho() a devolver None -- tal como
        # quando a câmara real não vê nada.
        return Response(status=404)

    norte = state["esquerda"] - state["direita"]
    este = state["cima"] - state["baixo"]

    if state['direita'] > 0: 
      x = state['direita']
    elif state['esquerda'] > 0:
      x = -state['esquerda']
    else:
      x = 0

    if state['cima'] > 0: 
      y = state['cima']
    elif state['baixo'] > 0:
      y = -state['baixo']
    else:
      y = 0
    
    return jsonify({
        "esquerda": state["esquerda"],
        "direita": state["direita"],
        "cima": state["cima"],
        "baixo": state["baixo"],
        "norte": norte,
        "este": este,
        "tamanho": state["tamanho"],
        "x": x,
        "y": y
    })


if __name__ == "__main__":
    # Corre apenas em localhost -- só acessível a partir desta máquina.
    app.run(host="127.0.0.1", port=5050, debug=True)