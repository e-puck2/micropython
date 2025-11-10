message(STATUS "Configuring EPUCK2.0 board")

set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    #boards/sdkconfig.ble
    boards/EPUCK_20/sdkconfig.flash_4MiB
    boards/ESP32_GENERIC/sdkconfig.d2wd    
)

list(APPEND MICROPY_DEF_BOARD
    MICROPY_HW_MCU_NAME="E-puck2"
    # Disable some options to reduce firmware size.
    MICROPY_OPT_COMPUTED_GOTO=0
    MICROPY_PY_NETWORK_LAN=0
)
