message(STATUS "Configuring EPUCK2.2 board")

set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    #boards/sdkconfig.ble
    boards/EPUCK/sdkconfig.flash_8MiB
    boards/sdkconfig.spiram    
)

list(APPEND MICROPY_DEF_BOARD
    MICROPY_HW_BOARD_NAME="E-puck 2.2"
    MICROPY_PY_NETWORK_LAN=0
)
