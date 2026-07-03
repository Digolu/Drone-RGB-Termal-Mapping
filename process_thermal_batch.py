#!/usr/bin/env python3
"""
Processa em lote imagens térmicas R-JPEG da DJI (M4TD e outros modelos
suportados pelo DJI Thermal SDK) e gera versões em tons de cinzento
normalizadas com a MESMA escala de temperatura em todas as imagens.

Em vez de usar o wrapper Python "thermal_parser" (que teve problemas de
compatibilidade/segfault com o modelo M4TD), este script chama diretamente
o binário oficial "dji_irp" que vem no DJI Thermal SDK, via subprocess.
Essa é a via mais fiável, porque é o próprio executável da DJI a fazer o
trabalho -- já confirmámos que funciona corretamente com "dji_irp --help"
e um teste manual.

Requisitos:
    pip install numpy pillow
    (não precisa de mais nada -- o dji_irp já trata da descodificação)

Uso básico (escala automática, calculada a partir do min/max de todas as
imagens da pasta):
    python process_thermal_batch.py --input PASTA_ENTRADA --output PASTA_SAIDA

Uso com escala fixa (ex: 15 a 45 graus):
    python process_thermal_batch.py --input PASTA_ENTRADA --output PASTA_SAIDA --tmin 15 --tmax 45

Guardar também TIFF 16-bit sem perdas (temperatura real em décimas de grau):
    python process_thermal_batch.py --input PASTA_ENTRADA --output PASTA_SAIDA --save-tiff16
"""
import argparse
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image

# --- Caminhos do DJI Thermal SDK (ajusta se a tua pasta tiver outro nome/local) ---
SDK_ROOT = Path(__file__).resolve().parent / "DJI_TermalSDK"
DJI_IRP_BIN = SDK_ROOT / "utility" / "bin" / "linux" / "release_x64" / "dji_irp"
DJI_IRP_LIBDIR = SDK_ROOT / "tsdk-core" / "lib" / "linux" / "release_x64"

# Resolução do sensor térmico do R-JPEG (confirmado no teu ficheiro: 640x512)
THERMAL_WIDTH = 640
THERMAL_HEIGHT = 512


def find_images(folder: Path):
    exts = {".jpg", ".jpeg", ".JPG", ".JPEG"}
    return sorted(p for p in folder.iterdir() if p.is_file() and p.suffix in exts)


def load_temperature(path: Path, tmp_dir: Path) -> np.ndarray:
    """Chama o dji_irp para extrair a temperatura de um R-JPEG.

    Devolve uma matriz 2D (numpy, float) de temperaturas em graus Celsius.
    """
    if not DJI_IRP_BIN.exists():
        sys.exit(
            f"Não encontrei o binário dji_irp em: {DJI_IRP_BIN}\n"
            "Confirma o caminho do SDK ou ajusta SDK_ROOT no topo do script."
        )

    raw_path = tmp_dir / (path.stem + ".raw")

    cmd = [
        str(DJI_IRP_BIN),
        "-s", str(path),
        "-a", "measure",
        "--measurefmt", "0",  # 0 = int16 (valor = temperatura * 10)
        "-o", str(raw_path),
    ]
    env = {"LD_LIBRARY_PATH": str(DJI_IRP_LIBDIR)}

    result = subprocess.run(cmd, env=env, capture_output=True, text=True)
    if result.returncode != 0 or not raw_path.exists():
        raise RuntimeError(
            f"dji_irp falhou para {path.name} (código {result.returncode}).\n"
            f"stdout: {result.stdout}\nstderr: {result.stderr}"
        )

    raw = np.fromfile(raw_path, dtype=np.int16)
    expected_len = THERMAL_WIDTH * THERMAL_HEIGHT
    if raw.size != expected_len:
        raise RuntimeError(
            f"{path.name}: esperava {expected_len} pixels, veio {raw.size}. "
            "Confirma THERMAL_WIDTH/THERMAL_HEIGHT no topo do script."
        )

    temp_celsius = raw.reshape(THERMAL_HEIGHT, THERMAL_WIDTH).astype(np.float32) / 10.0
    return temp_celsius


def write_report(
    out_dir: Path,
    started_at: datetime,
    elapsed: float,
    in_dir: Path,
    total_found: int,
    per_image_stats: list,
    failed: list,
    tmin: float,
    tmax: float,
    tmin_manual: bool,
    save_tiff16: bool,
):
    """Escreve um relatório .txt legível com o que foi feito e estatísticas."""
    n_ok = len(per_image_stats)
    n_failed = len(failed)

    lines = []
    lines.append("=" * 60)
    lines.append("RELATÓRIO DE PROCESSAMENTO DE IMAGENS TÉRMICAS")
    lines.append("=" * 60)
    lines.append(f"Data/hora de início:   {started_at.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"Duração total:         {elapsed:.1f} s")
    lines.append(f"Pasta de entrada:      {in_dir.resolve()}")
    lines.append(f"Pasta de saída:        {out_dir.resolve()}")
    lines.append("")
    lines.append("-" * 60)
    lines.append("RESUMO")
    lines.append("-" * 60)
    lines.append(f"Imagens encontradas:   {total_found}")
    lines.append(f"Processadas com sucesso: {n_ok}")
    lines.append(f"Falhadas:              {n_failed}")
    escala = "manual (definida pelo utilizador)" if tmin_manual else "automática (min/max de todo o conjunto)"
    lines.append(f"Escala de temperatura usada: {tmin:.1f}°C a {tmax:.1f}°C ({escala})")
    lines.append(f"TIFF 16-bit gerado:    {'sim' if save_tiff16 else 'não'}")
    lines.append("")

    if per_image_stats:
        todas_min = [s["min"] for s in per_image_stats]
        todas_max = [s["max"] for s in per_image_stats]
        todas_media = [s["media"] for s in per_image_stats]
        lines.append("-" * 60)
        lines.append("ESTATÍSTICAS GLOBAIS (sobre as imagens processadas)")
        lines.append("-" * 60)
        lines.append(f"Temperatura mínima observada:  {min(todas_min):.1f}°C")
        lines.append(f"Temperatura máxima observada:  {max(todas_max):.1f}°C")
        lines.append(f"Média das médias por imagem:   {sum(todas_media) / len(todas_media):.1f}°C")
        lines.append("")

    lines.append("-" * 60)
    lines.append("DETALHE POR IMAGEM")
    lines.append("-" * 60)
    for s in per_image_stats:
        lines.append(f"* {s['nome']}")
        lines.append(f"    min: {s['min']:.1f}°C   max: {s['max']:.1f}°C   "
                      f"média: {s['media']:.1f}°C   desvio padrão: {s['desvio_padrao']:.1f}°C")
        extra = f"    -> {s['png']}"
        if s["tiff"]:
            extra += f", {s['tiff']}"
        lines.append(extra)
    lines.append("")

    if failed:
        lines.append("-" * 60)
        lines.append("IMAGENS QUE FALHARAM")
        lines.append("-" * 60)
        for nome, motivo in failed:
            lines.append(f"* {nome}: {motivo}")
        lines.append("")

    lines.append("=" * 60)

    report_path = out_dir / "relatorio.txt"
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--input", required=True, help="pasta com as imagens R-JPEG")
    ap.add_argument("--output", required=True, help="pasta de saída")
    ap.add_argument("--tmin", type=float, default=None, help="temperatura mínima fixa (°C)")
    ap.add_argument("--tmax", type=float, default=None, help="temperatura máxima fixa (°C)")
    ap.add_argument(
        "--save-tiff16",
        action="store_true",
        help="também guarda um TIFF 16-bit com a temperatura em décimas de °C por pixel",
    )
    args = ap.parse_args()

    in_dir = Path(args.input)
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    start_time = time.time()
    started_at = datetime.now()

    images = find_images(in_dir)
    if not images:
        sys.exit(f"Nenhuma imagem .jpg/.jpeg encontrada em {in_dir}")

    print(f"A ler {len(images)} imagens térmicas...")
    temps = {}
    failed = []  # lista de (nome_ficheiro, motivo_erro)
    with tempfile.TemporaryDirectory() as tmp:
        tmp_dir = Path(tmp)
        for path in images:
            try:
                t = load_temperature(path, tmp_dir)
            except Exception as e:
                print(f"  [aviso] falhou a ler {path.name}: {e}")
                failed.append((path.name, str(e)))
                continue
            temps[path] = t
            print(f"  {path.name}: min={t.min():.1f}°C  max={t.max():.1f}°C")

    if not temps:
        sys.exit("Nenhuma imagem foi lida com sucesso.")

    # --- calcular escala global (Tmin/Tmax) ---
    if args.tmin is not None and args.tmax is not None:
        tmin, tmax = args.tmin, args.tmax
        print(f"\nA usar escala manual: {tmin:.1f}°C – {tmax:.1f}°C\n")
    else:
        tmin = min(t.min() for t in temps.values())
        tmax = max(t.max() for t in temps.values())
        print(f"\nEscala global calculada automaticamente: {tmin:.1f}°C – {tmax:.1f}°C\n")

    # --- normalizar e exportar ---
    per_image_stats = []  # lista de dicts com estatísticas por imagem, para o relatório
    for path, t in temps.items():
        norm = (t - tmin) / (tmax - tmin)
        norm = np.clip(norm, 0.0, 1.0)
        gray8 = (norm * 255).astype(np.uint8)

        out_png = out_dir / (path.stem + "_gray.png")
        Image.fromarray(gray8, mode="L").save(out_png)

        out_tiff = None
        if args.save_tiff16:
            tiff16 = (t * 10).astype(np.uint16)
            out_tiff = out_dir / (path.stem + "_temp16.tiff")
            Image.fromarray(tiff16, mode="I;16").save(out_tiff)

        per_image_stats.append({
            "nome": path.name,
            "min": float(t.min()),
            "max": float(t.max()),
            "media": float(t.mean()),
            "desvio_padrao": float(t.std()),
            "png": out_png.name,
            "tiff": out_tiff.name if out_tiff else None,
        })

        print(f"  -> {out_png.name}")

    elapsed = time.time() - start_time
    write_report(
        out_dir=out_dir,
        started_at=started_at,
        elapsed=elapsed,
        in_dir=in_dir,
        total_found=len(images),
        per_image_stats=per_image_stats,
        failed=failed,
        tmin=tmin,
        tmax=tmax,
        tmin_manual=args.tmin is not None and args.tmax is not None,
        save_tiff16=args.save_tiff16,
    )

    print(f"\nConcluído. Relatório em: {out_dir / 'relatorio.txt'}")


if __name__ == "__main__":
    main()