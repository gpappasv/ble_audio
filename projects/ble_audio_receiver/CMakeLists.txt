cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ble_audio_receiver)

target_sources(app PRIVATE
src/main.c
src/ble/ble_conn_control.c
src/ble/ble_bap_unicast_server.c
)

# Enable network core as a child image
if (CONFIG_SOC_NRF5340_CPUAPP)
    set_property(GLOBAL APPEND PROPERTY TFM_EXTRA_GENERATED_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/child_image
    )
endif()