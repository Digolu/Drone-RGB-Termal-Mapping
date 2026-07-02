#!/usr/bin/env python3
"""
raw_to_tiff.py

Converte o output .raw (float32) do dji_irp (DJI Thermal SDK) num
GeoTIFF de banda unica pronto para o ODM.

IMPORTANTE: a resolucao do sensor termico do M4TD e 640x512
(nao confundir com a resolucao da imagem JPEG completa, que e 1280x1024
e inclui overlay/paleta de cores).

Uso:
    python3 raw_to_tiff.py <ficheiro.raw> <largura> <altura> <saida.tif>

Exemplo:
    python3 raw_to_tiff.py DJI_0031_T.raw 640 512 DJI_0031_T_temp.tif
"""

import sys
import numpy as np
import tifffile


def raw_to_tiff(raw_path: str, width: int, height: int, out_path: str) -> None:
    expected_pixels = width * height

    data = np.fromfile(raw_path, dtype=np.float32)

    if data.size != expected_pixels:
        raise ValueError(
            f"Tamanho inesperado: ficheiro tem {data.size} valores, "
            f"esperava {expected_pixels} ({width}x{height}). "
            f"Confirma a resolucao ou o --measurefmt usado no dji_irp."
        )

    temp_c = data.reshape((height, width))

    # Guarda como float32 de banda unica -- 1 pixel = 1 valor de
    # temperatura em graus Celsius. O ODM le isto como imagem
    # radiometrica de banda unica.
    tifffile.imwrite(out_path, temp_c.astype(np.float32))

    print(f"OK: {out_path}  min={temp_c.min():.1f}C  max={temp_c.max():.1f}C  "
          f"media={temp_c.mean():.1f}C")


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(1)

    raw_file, w, h, out_file = sys.argv[1:5]
    raw_to_tiff(raw_file, int(w), int(h), out_file)
