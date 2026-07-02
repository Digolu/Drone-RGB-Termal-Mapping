#!/bin/bash
#
# batch_thermal_convert.sh
#
# Converte todos os *_T.jpeg (R-JPEG DJI) de uma pasta em TIFFs de
# temperatura de banda unica, prontos para o ODM, com as tags GPS/gimbal
# copiadas do JPEG original.
#
# Requisitos:
#   - dji_irp (DJI Thermal SDK) compilado -- ver DJI_IRP_BIN abaixo
#   - exiftool
#   - python3 com numpy + tifffile
#     (pip install numpy tifffile --break-system-packages)
#
# Resolucao do sensor termico M4TD: 640x512 (default abaixo).
#
# Uso:
#   ./batch_thermal_convert.sh <pasta_entrada> <pasta_saida> [opcoes]
#
# Opcoes (todas opcionais, com defaults sensatos):
#   --width N          largura do sensor (default: 640)
#   --height N         altura do sensor  (default: 512)
#   --distance N       distancia ao alvo em metros, [1,25] (default: 25)
#   --humidity N       humidade relativa %, [20,100]       (default: 70)
#   --emissivity N     emissividade do alvo, [0.10,1.00]   (default: 0.98)
#   --reflection N     temperatura refletida C, [-40,500]  (default: 23)
#   --dji-irp PATH     caminho para o binario dji_irp
#                       (default: procura no PATH)
#
# Exemplos:
#   # usar tudo por defeito
#   ./batch_thermal_convert.sh ./voo_polo2 ./voo_polo2_tiff
#
#   # sobrepor parametros ambientais
#   ./batch_thermal_convert.sh ./voo_polo2 ./voo_polo2_tiff \
#       --distance 15 --humidity 60 --emissivity 0.95

set -e

# ---- defaults ----
WIDTH=640
HEIGHT=512
DISTANCE=25
HUMIDITY=70
EMISSIVITY=0.98
REFLECTION=23
DJI_IRP_BIN="dji_irp"

IN_DIR="$1"
OUT_DIR="$2"
shift 2 2>/dev/null || true

while [ $# -gt 0 ]; do
    case "$1" in
        --width) WIDTH="$2"; shift 2 ;;
        --height) HEIGHT="$2"; shift 2 ;;
        --distance) DISTANCE="$2"; shift 2 ;;
        --humidity) HUMIDITY="$2"; shift 2 ;;
        --emissivity) EMISSIVITY="$2"; shift 2 ;;
        --reflection) REFLECTION="$2"; shift 2 ;;
        --dji-irp) DJI_IRP_BIN="$2"; shift 2 ;;
        *) echo "Opcao desconhecida: $1"; exit 1 ;;
    esac
done

if [ -z "$IN_DIR" ] || [ -z "$OUT_DIR" ]; then
    echo "Uso: $0 <pasta_entrada> <pasta_saida> [--distance N] [--humidity N] [--emissivity N] [--reflection N] [--width N] [--height N] [--dji-irp PATH]"
    exit 1
fi

for cmd in "$DJI_IRP_BIN" exiftool python3; do
    if ! command -v "$cmd" &> /dev/null; then
        echo "ERRO: '$cmd' nao encontrado no PATH."
        echo "  Se for o dji_irp, define --dji-irp /caminho/para/dji_irp"
        echo "  e confirma que exportaste LD_LIBRARY_PATH para a pasta das .so."
        exit 1
    fi
done

mkdir -p "$OUT_DIR"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Parametros:"
echo "  resolucao   : ${WIDTH}x${HEIGHT}"
echo "  distancia   : ${DISTANCE} m"
echo "  humidade    : ${HUMIDITY} %"
echo "  emissividade: ${EMISSIVITY}"
echo "  reflexao    : ${REFLECTION} C"
echo ""

count=0
fail=0

for jpg in "$IN_DIR"/*_T.jpeg "$IN_DIR"/*_T.jpg; do
    [ -e "$jpg" ] || continue

    base="$(basename "$jpg")"
    name="${base%.*}"
    raw_file="$OUT_DIR/${name}.raw"
    tif_file="$OUT_DIR/${name}.tif"

    echo "==> Processando $base"

    if ! "$DJI_IRP_BIN" -s "$jpg" -a measure -o "$raw_file" --measurefmt float32 \
        --distance "$DISTANCE" --humidity "$HUMIDITY" \
        --emissivity "$EMISSIVITY" --reflection "$REFLECTION" > /dev/null; then
        echo "    FALHOU dji_irp em $base"
        fail=$((fail + 1))
        continue
    fi

    if ! python3 "$SCRIPT_DIR/raw_to_tiff.py" "$raw_file" "$WIDTH" "$HEIGHT" "$tif_file"; then
        echo "    FALHOU conversao raw->tif em $base"
        fail=$((fail + 1))
        continue
    fi

    exiftool -TagsFromFile "$jpg" \
        -GPSLatitude -GPSLatitudeRef -GPSLongitude -GPSLongitudeRef \
        -GPSAltitude -GPSAltitudeRef \
        -"XMP-drone-dji:all" \
        -overwrite_original \
        "$tif_file" > /dev/null

    rm -f "$raw_file"
    count=$((count + 1))
done

echo ""
echo "Concluido: $count imagens convertidas, $fail falhas."
echo "TIFFs prontos em: $OUT_DIR"
echo ""
echo "Agora corre o ODM apontando para essa pasta, por exemplo:"
echo "  docker run --rm -ti -v $OUT_DIR:/code/images -v $OUT_DIR/../odm_output:/code/odm_orthophoto \\"
echo "    opendronemap/odm --radiometric-calibration camera --auto-boundary"
