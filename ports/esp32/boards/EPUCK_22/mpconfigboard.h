// Both of these can be set by mpconfigboard.cmake if a BOARD_VARIANT is
// specified.

#ifndef MICROPY_HW_BOARD_NAME
#define MICROPY_HW_BOARD_NAME "Generic ESP32 module"
#endif

#ifndef MICROPY_HW_MCU_NAME
#define MICROPY_HW_MCU_NAME "ESP32"
#endif

#define MICROPY_HW_UART_REPL_BAUD (230400)
#define MICROPY_HW_ENABLE_USBDEV            (0)
#define MICROPY_HW_ENABLE_USB_RUNTIME_DEVICE    (0)
#define MICROPY_HW_ESP_USB_SERIAL_JTAG      (0)
#define MICROPY_PY_BLUETOOTH                (0)
#define MICROPY_HW_ENABLE_SDCARD            (0)
#define MICROPY_PY_MACHINE_DAC              (0)
#define MICROPY_PY_MACHINE_I2S             (0)
#define MICROPY_PY_MACHINE_I2C_TARGET       (0)
#define MICROPY_PY_ESP32_PCNT               (0)
#define MICROPY_PY_ULAB                     (1)