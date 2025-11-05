#!/usr/bin/env bash
# Remote flash helper for mros2-mbed-chassis-dynamics
# Usage: ./scripts/remote_flash_chassis.sh
# Runs on the remote machine that has the board attached.

set -euo pipefail

BIN_PATH="/home/yupi/almondmatcha/mros2-mbed-chassis-dynamics/build/mros2-mbed.bin"
FLASH_ADDR=0x08000000

if [ ! -f "$BIN_PATH" ]; then
  echo "ERROR: firmware not found at $BIN_PATH"
  exit 2
fi

echo "Using firmware: $BIN_PATH"

# try to detect automounted removable mount (GNOME: /run/media/$USER/NAME)
MP=$(ls -d /run/media/$USER/* 2>/dev/null | head -n1 || true)
if [ -z "$MP" ]; then
  MP=$(ls -d /media/$USER/* 2>/dev/null | head -n1 || true)
fi

if [ -n "$MP" ]; then
  echo "Detected mount point: $MP"
  echo "Copying firmware to mass storage device..."
  cp "$BIN_PATH" "$MP/" && sync
  echo "File copied. Attempting safe unmount..."
  DEV=$(findmnt -n -o SOURCE --target "$MP" || true)
  if [ -n "$DEV" ]; then
    echo "Unmounting $DEV"
    if command -v udisksctl >/dev/null 2>&1; then
      udisksctl unmount -b "$DEV" || sudo umount "$MP"
      # try power-off whole device (strip partition number)
      udisksctl power-off -b "${DEV%[0-9]*}" || true
    else
      sudo umount "$MP"
    fi
  else
    echo "Warning: could not determine backing device; you may need to unmount manually."
  fi
  echo "Done — please press RESET on the board to run the new firmware."
  exit 0
fi

echo "No removable mass storage detected. Falling back to programmer (st-flash)."
if command -v st-flash >/dev/null 2>&1; then
  echo "Running: st-flash write $BIN_PATH $FLASH_ADDR"
  st-flash write "$BIN_PATH" "$FLASH_ADDR"
  echo "st-flash finished."
else
  echo "st-flash not found. Please install stlink tools or use STM32_Programmer_CLI."
  exit 3
fi
