cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ble_audio_receiver)

# Clean, scannable, properly indented compilation targets
target_sources(app PRIVATE
    src/main.c
    src/ble/ble_conn_control.c
    src/ble/ble_bap_unicast_server.c
    src/ble/ble_auracast.c
)

# Architectural Best Practice: Expose your source root globally.
# This lets your C files use `#include "ble_auracast.h"` directly 
# instead of relying on sloppy relative subfolder tracks.
zephyr_include_directories(src/ble)