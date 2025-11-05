# Flashing NUCLEO-F767ZI from a remote SSH session — Quick reference

This is a short, copy‑pasteable reference for flashing firmware when you are SSH'd into the remote PC which both built the firmware and has the NUCLEO‑F767ZI physically attached.

Assumptions
- You are logged in over SSH to the machine that has the board attached.
- The firmware file exists on that remote machine (example path: `/path/to/mros2-mbed.bin`).
- STM32F7 flash base address: `0x08000000`.

Quick detection (run once to decide method)
```bash
# check for automounted removable drives (GNOME usually uses /run/media/$USER)
ls -d /run/media/$USER/* 2>/dev/null | head -n1 || ls -d /media/$USER/* 2>/dev/null | head -n1

# check kernel messages for USB device attach
dmesg | tail -n 40

# see ST-Link on USB if present
lsusb | grep -i st || true

# list block devices & mounts
lsblk -o NAME,SIZE,MOUNTPOINT,LABEL
```

Flow A — mass‑storage (drag & drop equivalent)
1. If detection returns a mount point (example `/run/media/yupi/MBED`), copy the file:
```bash
MP="/run/media/$USER/MBED"   # replace with actual mount
cp /path/to/mros2-mbed.bin "$MP"/
sync
```
2. Safely unmount/eject:
```bash
DEV=$(findmnt -n -o SOURCE --target "$MP")   # e.g. /dev/sdb1
udisksctl unmount -b "$DEV"    # may require polkit
udisksctl power-off -b "${DEV%[0-9]*}" || true
# or if udisksctl isn't available: sudo umount "$MP"
```
3. Press RESET or power‑cycle the board to run the new firmware.

Flow B — programmer tools (recommended when no mass storage)
st‑flash (stlink):
```bash
st-flash write /path/to/mros2-mbed.bin 0x08000000

# optional readback verify
st-flash read /tmp/verify.bin 0x08000000 0x2000
md5sum /tmp/verify.bin
```

STM32CubeProgrammer CLI (official):
```bash
STM32_Programmer_CLI -c port=SWD -d /path/to/mros2-mbed.bin 0x08000000
```

OpenOCD (debug + flash):
```bash
openocd -f interface/stlink.cfg -f target/stm32f7x.cfg \
  -c "init; reset init; program /path/to/mros2-mbed.bin 0x08000000 verify reset; exit"
```

Verification quick checks
- If firmware prints to UART: open serial console on remote host (e.g. `screen /dev/ttyUSB0 115200`).
- Read a small region of flash and compare an md5 or known headers if needed.

Permissions notes
- If flashing tools cannot access the USB probe, either run them with `sudo` or add an appropriate udev rule to allow non-root access to ST‑Link devices.
- Avoid building as root. If build files are root-owned, restore ownership:
```bash
sudo chown -R $USER:$USER /path/to/project/*/build
```

One‑line alias (recommended for programmer flow)
```bash
# put this in ~/.bashrc on the remote host
alias flash_st='st-flash write /path/to/mros2-mbed.bin 0x08000000'
```

If you want, I can add a small helper script to `scripts/remote_flash.sh` that:
- auto-detects a removable mount (Flow A) and copies the .bin, or
- falls back to `st-flash` (Flow B) and performs a readback verify.

---
File created for quick reference: `docs/FLASHING_REMOTE_CLI_QUICK.md`
