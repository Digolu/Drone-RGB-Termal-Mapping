"""
Servidor de simulação da câmara (substitui o Jetson/camara real durante testes).

Expõe o mesmo endpoint que o teu script já espera:
    GET http://<host>:5000/desvio_vermelho

Resposta:
    - JSON com {"esquerda":.., "direita":.., "cima":.., "baixo":..}
    - ou 404 (corpo vazio) se o checkbox "Sem deteção" estiver ativo,
      simulando o caso em que a câmara não vê vermelho (get_desvio_vermelho
      no teu script devolve None nesse caso, exatamente como quando a
      câmara real está inacessível ou não deteta nada).

Corre com:
    pip install flask
    python camera_sim_server.py

Depois abre no browser:
    http://localhost:5000/
para mexeres nos sliders e veres o drone a reagir na simulação.
"""

from flask import Flask, jsonify, request, Response

app = Flask(__name__)

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Estado atual dos desvios (partilhado entre a página web e o endpoint da API)
state = {
    "esquerda": 0,
    "direita": 0,
    "cima": 0,
    "baixo": 0,
    "sem_deteccao": False,
}

PAGE = """
<!doctype html>
<html lang="pt">
<head>
<meta charset="utf-8">
<title>Simulador de Desvios - Câmara</title>
<style>
  body { font-family: sans-serif; max-width: 520px; margin: 40px auto; }
  h1 { font-size: 1.3rem; }
  .row { margin-bottom: 22px; }
  label { display: flex; justify-content: space-between; font-weight: bold; }
  input[type=range] { width: 100%; }
  .val { font-weight: normal; color: #555; }
  .toggle { display: flex; align-items: center; gap: 8px; margin-top: 24px; }
  .status { margin-top: 12px; font-size: 0.9rem; color: #777; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
</style>
</head>
<body>
<h1>Simulador de desvios da câmara (vermelho)</h1>

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

<div class="toggle">
  <input type="checkbox" id="sem_deteccao">
  <label for="sem_deteccao" style="font-weight:normal;">Sem deteção (simula câmara sem vermelho)</label>
</div>

<div class="status" id="status">a atualizar...</div>

<script>
const ids = ["esquerda", "direita", "cima", "baixo"];

async function sendState() {
  const body = {};
  ids.forEach(id => body[id] = parseInt(document.getElementById(id).value, 10));
  body.sem_deteccao = document.getElementById("sem_deteccao").checked;

  const res = await fetch("/set", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body)
  });
  document.getElementById("status").textContent =
    "Última atualização: " + new Date().toLocaleTimeString();
}

ids.forEach(id => {
  const el = document.getElementById(id);
  el.addEventListener("input", () => {
    document.getElementById("v_" + id).textContent = el.value;
    sendState();
  });
});

document.getElementById("sem_deteccao").addEventListener("change", sendState);
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return PAGE


@app.route("/set", methods=["POST"])
def set_state():
    data = request.get_json(force=True)
    for k in ("esquerda", "direita", "cima", "baixo"):
        if k in data:
            state[k] = int(data[k])
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

    return jsonify({
        "esquerda": state["esquerda"],
        "direita": state["direita"],
        "cima": state["cima"],
        "baixo": state["baixo"],
    })


if __name__ == "__main__":
    # Corre apenas em localhost -- só acessível a partir desta máquina.
    app.run(host="127.0.0.1", port=5050, debug=True)