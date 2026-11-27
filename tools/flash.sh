#!/usr/bin/env bash
# NanoLink one-shot firmware flashing.
#
# Usage:
#   ./tools/flash.sh aircraft      # flash the aircraft FC (F401, ST-Link)
#   ./tools/flash.sh gcs           # flash the ground station (F401, ST-Link)
#   ./tools/flash.sh tx            # flash the Nano transmitter (USB serial)
#   ./tools/flash.sh build         # compile all three firmwares (no upload)
#   ./tools/flash.sh monitor <t>   # open the serial monitor for a target
#   ./tools/flash.sh devices       # show connected ST-Link / serial devices
#
# The script waits for the ST-Link programmer to appear and retries the upload,
# so a single command recovers from the usual "OpenOCD init failed / open
# failed" errors caused by the probe not being ready yet.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STLINK_VIDPID="0483:3748"  # ST-Link/V2 (also matches clones)

aircraft_env="blackpill_f401ce"
gcs_env="blackpill_f401ce_usb"
tx_env="nanoatmega328"

usage() {
  sed -n '2,12p' "${BASH_SOURCE[0]}"
  exit 1
}

have_command() { command -v "$1" >/dev/null 2>&1; }

list_devices() {
  echo "ST-Link programmers (0483:3748):"
  if have_command lsusb; then
    lsusb | grep -i "$STLINK_VIDPID" || echo "  (none found)"
  else
    echo "  (lsusb not installed)"
  fi
  echo "Serial ports:"
  for p in /dev/ttyACM* /dev/ttyUSB* /dev/ttyS*; do
    [ -e "$p" ] && echo "  $p"
  done
}

# Wait up to N seconds for an ST-Link programmer to appear.
wait_stlink() {
  local timeout="${1:-30}"
  echo "Waiting for ST-Link (${STLINK_VIDPID})..."
  local waited=0
  while [ "$waited" -lt "$timeout" ]; do
    if have_command lsusb && lsusb | grep -qi "$STLINK_VIDPID"; then
      echo "ST-Link detected."
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "ERROR: no ST-Link found after ${timeout}s. Connect it (SWD + USB) and retry." >&2
  return 1
}

upload() {
  local dir="$1" env="$2" attempts="${3:-3}"
  local try
  for try in $(seq 1 "$attempts"); do
    echo "Upload attempt ${try}/${attempts} (env ${env})..."
    if (cd "$dir" && pio run -e "$env" -t upload); then
      echo "Upload OK."
      return 0
    fi
    if [ "$try" -lt "$attempts" ]; then
      echo "Retrying in 2s..."
      sleep 2
    fi
  done
  echo "ERROR: upload failed after ${attempts} attempts." >&2
  return 1
}

flash_stm32() {
  local dir="$1" env="$2"
  wait_stlink 30 || return 1
  upload "$dir" "$env"
}

flash_tx() {
  local dir="$ROOT/firmware/transmitter_nano"
  echo "Nano upload uses the USB-serial bootloader."
  if ! ls /dev/ttyACM* /dev/ttyUSB* >/dev/null 2>&1; then
    echo "WARNING: no serial port found; connect the Nano via USB."
  fi
  upload "$dir" "$tx_env"
}

monitor_target() {
  case "$1" in
    aircraft) (cd "$ROOT/firmware/aircraft_f401" && pio device monitor -b 115200) ;;
    gcs)      (cd "$ROOT/firmware/ground_station_f401" && pio device monitor -b 115200) ;;
    tx)       (cd "$ROOT/firmware/transmitter_nano" && pio device monitor -b 115200) ;;
    *) usage ;;
  esac
}

build_all() {
  local ok=1
  (cd "$ROOT/firmware/aircraft_f401" && pio run -e "$aircraft_env") || ok=0
  (cd "$ROOT/firmware/ground_station_f401" && pio run -e "$gcs_env") || ok=0
  (cd "$ROOT/firmware/transmitter_nano" && pio run -e "$tx_env") || ok=0
  return $((1 - ok))
}

case "${1:-}" in
  aircraft) flash_stm32 "$ROOT/firmware/aircraft_f401" "$aircraft_env" ;;
  gcs)      flash_stm32 "$ROOT/firmware/ground_station_f401" "$gcs_env" ;;
  tx)       flash_tx ;;
  build)    build_all ;;
  monitor)  monitor_target "${2:-}" ;;
  devices)  list_devices ;;
  *) usage ;;
esac
