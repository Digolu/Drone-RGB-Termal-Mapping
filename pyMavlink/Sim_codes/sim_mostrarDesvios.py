from flask import Flask, jsonify, request, render_template_string
from collections import deque
import logging, time

app = Flask(__name__)
logging.getLogger('werkzeug').setLevel(logging.ERROR)

MAX_PONTOS = 200
historico = {k: deque(maxlen=MAX_PONTOS) for k in
             ["t","DBM","DCM","DDM","DEM","moveN","moveE","moveD"]}
t0 = time.time()


HTML = """
<!doctype html>
<html>
<head>
    <title>Desvios & Movimentos</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
    <canvas id="grafico" width="900" height="450"></canvas>
    <script>
    const ctx = document.getElementById('grafico').getContext('2d');
    const chart = new Chart(ctx, {
        type: 'line',
        data: { labels: [], datasets: [
        {label: 'DBM', borderColor: 'red', data: [], fill:false},
        {label: 'DCM', borderColor: 'orange', data: [], fill:false},
        {label: 'DDM', borderColor: 'green', data: [], fill:false},
        {label: 'DEM', borderColor: 'blue', data: [], fill:false},
        {label: 'moveN', borderColor: 'black', data: [], fill:false, borderDash:[5,5]},
        {label: 'moveE', borderColor: 'gray', data: [], fill:false, borderDash:[5,5]},
        {label: 'moveD', borderColor: 'purple', data: [], fill:false, borderDash:[5,5]},
        ]},
        options: { animation:false, responsive:false, scales:{x:{title:{display:true,text:'tempo (s)'}}} }
    });

    async function atualizar() {
        const r = await fetch('/dados');
        const d = await r.json();
        chart.data.labels = d.t;
        chart.data.datasets[0].data = d.DBM;
        chart.data.datasets[1].data = d.DCM;
        chart.data.datasets[2].data = d.DDM;
        chart.data.datasets[3].data = d.DEM;
        chart.data.datasets[4].data = d.moveN;
        chart.data.datasets[5].data = d.moveE;
        chart.data.datasets[6].data = d.moveD;
        chart.update();
    }
    setInterval(atualizar, 300);
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/dados')
def dados():
    return jsonify({k: list(v) for k, v in historico.items()})

@app.route('/update', methods=['POST'])
def update():
    d = request.get_json()
    historico["t"].append(round(time.time() - t0, 2))
    for k in ["DBM","DCM","DDM","DEM","moveN","moveE","moveD"]:
        historico[k].append(d.get(k, 0))
    return jsonify({"ok": True})

if __name__ == '__main__':
    print("Gráfico disponível em http://localhost:5001/")
    app.run(host='0.0.0.0', port=5001, threaded=True)