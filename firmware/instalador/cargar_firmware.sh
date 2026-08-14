#!/usr/bin/env bash
# cargar_firmware.sh — Instala Arduino CLI y el core ESP32 de Espressif,
# descarga el firmware de la plataforma DCMotor desde GitHub, detecta el
# puerto donde está conectada la placa y compila/sube el firmware.
#
# Uso:
#   ./cargar_firmware.sh
#
# Requiere: bash, curl, git, python3 (para el detector de puerto).

set -euo pipefail

REPO_URL="https://github.com/nebisman/DCMotor.jl.git"
FQBN="esp32:esp32:esp32s3usbotg"
ESP32_BOARD_URL="https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"
SKETCH_NAME="esp32_DCMotor"
WORKDIR="$(pwd)"
BINDIR="$HOME/.local/bin"

log() { echo "==> $1"; }
die() { echo "ERROR: $1" >&2; exit 1; }

for dep in curl git python3; do
    command -v "$dep" &>/dev/null || die "Se requiere '$dep', pero no está instalado."
done

# ── 1. Instalar Arduino CLI ──────────────────────────────────────────────
if command -v arduino-cli &>/dev/null; then
    log "Arduino CLI ya está instalado: $(arduino-cli version | head -1)"
else
    log "Arduino CLI no está instalado. Iniciando instalación..."
    mkdir -p "$BINDIR"
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$BINDIR" sh
    export PATH="$BINDIR:$PATH"
    command -v arduino-cli &>/dev/null || die "No se pudo instalar Arduino CLI."
    log "Arduino CLI instalado correctamente: $(arduino-cli version | head -1)"
fi

# ── 2. Instalar el core (librerías) ESP32 de Espressif ──────────────────
if arduino-cli core list 2>/dev/null | grep -q "^esp32:esp32"; then
    log "Las librerías de ESP32 (core esp32:esp32) ya están instaladas."
else
    log "Las librerías de ESP32 (core esp32:esp32) no están instaladas. Iniciando instalación..."
    arduino-cli core update-index --additional-urls "$ESP32_BOARD_URL"
    arduino-cli core install esp32:esp32 --additional-urls "$ESP32_BOARD_URL"
    log "Librerías de ESP32 instaladas correctamente."
fi

# ── 3. Descargar el firmware desde el repositorio de GitHub ─────────────
if [[ -f "$WORKDIR/$SKETCH_NAME/$SKETCH_NAME.ino" && -f "$WORKDIR/$SKETCH_NAME/definitions.h" ]]; then
    log "El directorio '$SKETCH_NAME/' ya contiene $SKETCH_NAME.ino y definitions.h; se omite la descarga."
else
    log "Descargando firmware/$SKETCH_NAME desde $REPO_URL..."
    TMP_CLONE="$(mktemp -d)"
    trap 'rm -rf "$TMP_CLONE"' EXIT

    git clone --quiet --depth 1 --filter=blob:none --sparse "$REPO_URL" "$TMP_CLONE"
    git -C "$TMP_CLONE" sparse-checkout set --quiet "firmware/$SKETCH_NAME"

    [[ -d "$TMP_CLONE/firmware/$SKETCH_NAME" ]] ||
        die "No se encontró firmware/$SKETCH_NAME en el repositorio."

    if [[ -d "$WORKDIR/$SKETCH_NAME" ]]; then
        log "Ya existe '$SKETCH_NAME/' en el directorio actual; se sobrescribirá."
        rm -rf "$WORKDIR/$SKETCH_NAME"
    fi
    cp -r "$TMP_CLONE/firmware/$SKETCH_NAME" "$WORKDIR/$SKETCH_NAME"
    log "Firmware descargado en $WORKDIR/$SKETCH_NAME"
fi

# ── 4. Detectar el puerto de la placa ────────────────────────────────────
log "Buscando el puerto de la placa ESP32..."
PORTS="$(arduino-cli board list --format json | python3 -c '
import json, sys

known_vid_pid = {
    ("0x10c4", "0xea60"),  # Silicon Labs CP2102/CP2104
    ("0x1a86", "0x7523"),  # WCH CH340/CH341
    ("0x1a86", "0x55d4"),  # WCH CH9102
    ("0x0403", "0x6001"),  # FTDI FT232
    ("0x303a", "0x1001"),  # Espressif USB JTAG/serial debug unit
}
keywords = ("cp210", "ch340", "ch9102", "wch", "qinheng", "silicon labs",
            "espressif", "esp32", "usb-serial", "usb serial", "uart")

data = json.load(sys.stdin)
for entry in data.get("detected_ports", []):
    port = entry.get("port", {})
    props = port.get("properties") or {}
    vid = str(props.get("vid", "")).lower()
    pid = str(props.get("pid", "")).lower()
    text = " ".join(str(props.get(k, "")) for k in ("manufacturer", "product", "serialNumber")).lower()
    if (vid, pid) in known_vid_pid or any(kw in text for kw in keywords):
        print(port.get("address", ""))
')"

readarray -t PORT_LIST <<<"$PORTS"
PORT_LIST=("${PORT_LIST[@]/#/}")
[[ ${#PORT_LIST[@]} -eq 1 && -z "${PORT_LIST[0]}" ]] && PORT_LIST=()

if [[ ${#PORT_LIST[@]} -eq 0 ]]; then
    die "No se pudo detectar ningún puerto USB-serial compatible con la placa ESP32. Verifique que la plataforma esté conectada y encendida."
elif [[ ${#PORT_LIST[@]} -gt 1 ]]; then
    die "Se detectaron varios puertos candidatos: ${PORT_LIST[*]}. Edite este script para indicar manualmente cuál usar."
fi
PORT="${PORT_LIST[0]}"
log "Puerto detectado: $PORT"

# ── 5. Compilar y subir el firmware ──────────────────────────────────────
log "Compilando el firmware ($FQBN)..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH_NAME"

log "Subiendo el firmware al puerto $PORT..."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_NAME"

log "Firmware cargado exitosamente."
