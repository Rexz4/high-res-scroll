# High-Res Scroll Wheel

ZMK firmware for AS5600 magnetic rotary encoder BLE scroll wheel on promicro_nrf52840.

## Setup

1. Clone ZMK and set up west workspace normally
2. Apply the patch to default-app:
```
   git apply patches/default-app-CMakeLists.patch
```
3. Copy or symlink this folder to `applications/high-res-scroll/`

## Build
```bash
west build -p -b promicro_nrf52840 \
    -d build/high-res-scroll applications/default-app \
    -- \
    -DEXTRA_CONF_FILE="$(pwd)/applications/high-res-scroll/prj.conf" \
    -DEXTRA_DTC_OVERLAY_FILE="$(pwd)/applications/high-res-scroll/app.overlay;$(pwd)/applications/high-res-scroll/high-res-scroll.keymap"
```
