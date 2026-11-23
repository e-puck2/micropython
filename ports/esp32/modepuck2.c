/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 "Eric Poulsen" <eric@zyxod.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>

#include <time.h>
#include <sys/time.h>
#include "soc/rtc_cntl_reg.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_heap_caps.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "shared/timeutils/timeutils.h"
#include "modmachine.h"
#include "modepuck2.h"
#include "../../../../../main/rgb_led_e-puck2.h"
#include "../../../../../main/uart_e-puck2.h"
#include "../../../../../main/button_e-puck2.h"
#include "../../../../../main/spi_e-puck2.h"
#include "py/mpthread.h"
#include "epuck2_image.h"

/// \module epuck2
/// The `epuck2` module gives access to the sensors and actuators of the e-puck2 robot from the MicroPython firmware of the ESP32 radio module. The ESP32 exchanges data with the main microcontroller (STM32F407) through UART, using the advanced sercom protocol, and receives the camera images through SPI.
///
/// The functions need the robot selector in position 3 or 15, where the main microcontroller runs the advanced sercom protocol. The camera images are streamed continuously through SPI only in position 15 (faster); in position 3 every image is requested through UART.
///
/// The same API is available on e-puck2.0/2.1 (4 MB flash, no PSRAM) and on e-puck2.2 (8 MB flash and 2 MB PSRAM, so more RAM for the user programs).
/// \example A first program: print the API version and the proximity sensors, then turn on the front LED for one second
///     import epuck2
///     import time
///
///     print("API version:", epuck2.get_api_version())
///     print("proximity:", epuck2.get_proximity())
///     epuck2.set_leds(0x20)  # Front LED on.
///     time.sleep(1)
///     epuck2.set_leds(0)
/// \endexample
/// \section System: get_api_version, get_selector, button_pressed, get_battery, sd_available
/// API version and general state of the robot.
/// \section Motors: set_motors_speed
/// The two wheels are driven by stepper motors: 1000 steps make one wheel revolution, about 12.9 cm.
/// \section LEDs: set_leds, set_rgb, set_all_rgb
/// The robot has 8 LEDs around its body: LED1, LED3, LED5 and LED7 are red, LED2, LED4, LED6 and LED8 are RGB. It also has a green body LED and a red front LED.
/// \section Sound: play_sound, play_freq
/// Sounds and tones played by the speaker of the robot.
/// \section Sensors: get_proximity, get_distance, get_acc_raw, get_gyro_raw, get_mic_volume, get_mic_data, get_tv_remote, set_magnetometer
/// The values are the last ones received from the main microcontroller, which sends all the sensors continuously.
/// \section Bulk functions: set_all_actuators, get_all_sensors
/// Set all the actuators or read all the sensors with a single call, in the same format as the advanced sercom / WiFi packets.
/// \section Camera: camera_get_image, camera_get_format, camera_set_format, camera_get_stats, camera_reset_stats, CAM_IMAGE_SIZE
/// The camera is connected to the main microcontroller. The image source depends on the selector position, read at the first camera call:
/// - position 15: the main microcontroller streams 160x120 RGB565 images to the ESP32 through SPI, continuously. The ESP32 can crop, downscale and convert them to greyscale (`camera_set_format()`).
/// - position 3: every image is requested to the main microcontroller through UART (115200 baud: a few images per second). The image is at most 4136 bytes.
///
/// Images are stored row by row, from the top-left pixel. RGB565 images use 2 bytes per pixel, big endian (`RRRRRGGG GGGBBBBB`); greyscale images use 1 byte per pixel (0 black, 255 white).
/// \section Image processing: image_get_color, image_find_blobs_color, image_find_blobs_grey, image_diff
/// Simple vision functions, implemented in C, working on the images returned by `camera_get_image()` (any buffer with the same layout can be used).
///
/// Common parameters:
/// - `img`: the image (bytearray or any buffer).
/// - `fmt`: the image format `(width, height, bytes_per_pixel)`, as returned by `camera_get_format()`.
/// - `roi`: region of interest `(x, y, w, h)` to process, `None` for the whole image.
/// - `min_pixels`: blobs with fewer pixels are ignored (noise), default 10.
/// - `max_blobs`: maximum number of blobs returned, 1..64, default 8.
///
/// A blob is a group of selected pixels touching each other, also diagonally: two separate objects give two blobs. Each blob is returned as a tuple `(pixels, cx, cy, x_min, y_min, x_max, y_max)`: number of pixels, centroid and bounding box (included), in image coordinates. The blobs are sorted by size, biggest first; the list is empty if nothing is found.
///
/// The work memory is allocated at each call and depends only on the width of the region (about 3 KB for 80 pixels, 6 KB for 160 pixels), so the functions can be used also on e-puck2.0/2.1. Invalid arguments raise `ValueError`.
/// \example Find the biggest red object in a 80x60 image (selector 15)
///     import epuck2
///
///     FMT = (80, 60, 2)
///     epuck2.camera_set_format(80, 60, 2, 8)  # Whole view, half resolution.
///     img = bytearray(80 * 60 * 2)
///     if epuck2.camera_get_image(img) is not None:
///         blobs = epuck2.image_find_blobs_color(img, FMT, 340, 20, max_blobs=1)
///         if blobs:
///             pixels, cx, cy, x_min, y_min, x_max, y_max = blobs[0]
///             print("red object at", cx, cy, "size", pixels)
/// \endexample

char api_version[6] = "XX.XX\0";
char temp_buff[512];

/// \function get_api_version()
/// Get the version of the epuck2 module API, as "major.minor" (each number is padded with spaces to 2 characters, e.g. " 1.2 "). The minor version is incremented when functions are added, the major version when the existing functions change.
/// \return string.
/// \example Print the API version
///     import epuck2
///     print(epuck2.get_api_version())
/// \endexample
static mp_obj_t get_api_version(void) {
    snprintf(api_version, 6, "%2d.%-2d", EPUCK2_API_MAJOR_VERSION, EPUCK2_API_MINOR_VERSION);
    return mp_obj_new_str(api_version, strlen(api_version));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_api_version_obj, get_api_version);

/// \function set_rgb(led, red, green, blue)
/// Set the color of one RGB LED.
/// \param led LED to set: 0 = LED2, 1 = LED4, 2 = LED6, 3 = LED8 (other values are ignored).
/// \param red intensity of the red component, between 0 (off) and 100 (full on).
/// \param green intensity of the green component, between 0 and 100.
/// \param blue intensity of the blue component, between 0 and 100.
/// \example Set LED2 to half intensity blue
///     import epuck2
///     epuck2.set_rgb(0, 0, 0, 50)
/// \endexample
mp_obj_t epuck2_set_rgb(size_t n_args, const mp_obj_t *args) {
    int l = mp_obj_get_int(args[0]);
    int red = mp_obj_get_int(args[1]);
    int green = mp_obj_get_int(args[2]);
    int blue = mp_obj_get_int(args[3]);
    if(l == 0)
    {
        //rgb_update_led2(red, green, blue);
        uart_set_rgb_led2(red, green, blue);
    }
    else if(l == 1)
    {
        //rgb_update_led4(red, green, blue);
        uart_set_rgb_led4(red, green, blue);
    }
    else if(l == 2)
    {
        //rgb_update_led6(red, green, blue);
        uart_set_rgb_led6(red, green, blue);
    }
    else if(l == 3)
    {
        //rgb_update_led8(red, green, blue);
        uart_set_rgb_led8(red, green, blue);
    }
    
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(epuck2_set_rgb_obj, 4, 4, epuck2_set_rgb);

/// \function set_all_rgb(red2, green2, blue2, red4, green4, blue4, red6, green6, blue6, red8, green8, blue8)
/// Set the color of the four RGB LEDs at once: red, green and blue intensities of LED2, LED4, LED6 and LED8, each between 0 (off) and 100 (full on).
/// \example Set LED2 to red and LED4 to green, turn off LED6 and LED8
///     import epuck2
///     epuck2.set_all_rgb(100, 0, 0,   # LED2
///                        0, 100, 0,   # LED4
///                        0, 0, 0,     # LED6
///                        0, 0, 0)     # LED8
/// \endexample
mp_obj_t epuck2_set_all_rgb(size_t n_args, const mp_obj_t *args) {
    uint8_t rgb_values[12];
    rgb_values[0] = (uint8_t)mp_obj_get_int(args[0]);
    rgb_values[1] = (uint8_t)mp_obj_get_int(args[1]);
    rgb_values[2] = (uint8_t)mp_obj_get_int(args[2]);
    rgb_values[3] = (uint8_t)mp_obj_get_int(args[3]);
    rgb_values[4] = (uint8_t)mp_obj_get_int(args[4]);
    rgb_values[5] = (uint8_t)mp_obj_get_int(args[5]);
    rgb_values[6] = (uint8_t)mp_obj_get_int(args[6]);
    rgb_values[7] = (uint8_t)mp_obj_get_int(args[7]);
    rgb_values[8] = (uint8_t)mp_obj_get_int(args[8]);
    rgb_values[9] = (uint8_t)mp_obj_get_int(args[9]);
    rgb_values[10] = (uint8_t)mp_obj_get_int(args[10]);
    rgb_values[11] = (uint8_t)mp_obj_get_int(args[11]);
    uart_set_rgb_leds(rgb_values);    
    
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(epuck2_set_all_rgb_obj, 12, 12, epuck2_set_all_rgb);

/// \function set_motors_speed(left, right)
/// Set the speed of the two motors. The motors keep the speed until it is changed: call `set_motors_speed(0, 0)` to stop the robot.
/// \param left speed of the left motor in steps per second, between -1100 and 1100 (negative values move the wheel backward, values outside the range are limited to it). 1000 steps per second are one wheel revolution per second, about 12.9 cm/s.
/// \param right speed of the right motor, same range as `left`.
/// \example Move forward, turn in place, then stop
///     import epuck2
///     import time
///     epuck2.set_motors_speed(500, 500)    # Forward at about 6.5 cm/s.
///     time.sleep(1)
///     epuck2.set_motors_speed(300, -300)   # Turn right in place.
///     time.sleep(1)
///     epuck2.set_motors_speed(0, 0)        # Stop.
/// \endexample
mp_obj_t epuck2_set_motors_speed(mp_obj_t left, mp_obj_t right) {
    int l = mp_obj_get_int(left);
    int r = mp_obj_get_int(right);
    uart_set_motors_speed(l, r);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(epuck2_set_motors_speed_obj, epuck2_set_motors_speed);

/// \function set_leds(value)
/// Turn on or off the red LEDs (LED1, LED3, LED5, LED7), the body LED and the front LED.
/// \param value bitmask, a bit set to 1 turns the corresponding LED on:
/// | Bit | Value | LED |
/// | 0 | 0x01 | LED1 (front) |
/// | 1 | 0x02 | LED3 (right) |
/// | 2 | 0x04 | LED5 (back) |
/// | 3 | 0x08 | LED7 (left) |
/// | 4 | 0x10 | Body LED (green) |
/// | 5 | 0x20 | Front LED (red) |
/// \example Turn on LED1 and the front LED, then turn all of them off
///     import epuck2
///     import time
///     epuck2.set_leds(0x01 | 0x20)
///     time.sleep(1)
///     epuck2.set_leds(0)
/// \endexample
mp_obj_t epuck2_set_leds(mp_obj_t value) {
    uint8_t val = mp_obj_get_int(value);
    uart_set_leds(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(epuck2_set_leds_obj, epuck2_set_leds);

/// \function play_sound(id)
/// Play one of the sounds stored in the robot.
/// \param id sound to play:
/// | Value | Sound |
/// | 0x01 | Mario |
/// | 0x02 | Underworld |
/// | 0x04 | Star Wars |
/// | 0x08 | 4 kHz tone |
/// | 0x10 | 10 kHz tone |
/// | 0x20 | Stop the sound |
/// | 0x40 | Christmas song |
/// \example Play the Mario song for 3 seconds
///     import epuck2
///     import time
///     epuck2.play_sound(0x01)
///     time.sleep(3)
///     epuck2.play_sound(0x20)
/// \endexample
mp_obj_t epuck2_play_sound(mp_obj_t value) {
    uint8_t val = mp_obj_get_int(value);
    uart_set_sound(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(epuck2_play_sound_obj, epuck2_play_sound);

/// \function set_all_actuators(settings, left, right, leds, red2, green2, blue2, red4, green4, blue4, red6, green6, blue6, red8, green8, blue8, sound)
/// Set all the actuators with a single call.
/// \param settings bitmask: bit 0 = 1 calibrates the proximity sensors (keep the robot away from objects); bit 1 = 1 enables the onboard obstacle avoidance (not implemented yet); bit 2 = 0 means that `left` and `right` are speeds, 1 that they are motor steps (position).
/// \param left left motor speed (-1100..1100 steps per second, see `set_motors_speed()`) or steps.
/// \param right right motor speed or steps.
/// \param leds red LEDs, body LED and front LED bitmask (see `set_leds()`).
/// \param red2 the 12 values `red2` ... `blue8` are the intensities of the RGB LEDs LED2, LED4, LED6 and LED8, between 0 and 100 (see `set_all_rgb()`).
/// \param sound sound to play (see `play_sound()`), 0 for none.
/// \example Move forward with LED1 and the front LED on, all RGB LEDs blue, no sound
///     import epuck2
///     epuck2.set_all_actuators(0, 300, 300, 0x21,
///                              0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50,
///                              0)
/// \endexample
mp_obj_t epuck2_set_all_actuators(size_t n_args, const mp_obj_t *args) {
    uint8_t values[19];
    int16_t temp = 0;
    values[0] = (uint8_t)mp_obj_get_int(args[0]);
    temp = mp_obj_get_int(args[1]);
    values[1] = temp & 0xFF;
    values[2] = (temp>>8) & 0xFF;
    temp = mp_obj_get_int(args[2]);
    values[3] = temp & 0xFF;
    values[4] = (temp>>8) & 0xFF;
    values[5] = (uint8_t)mp_obj_get_int(args[3]);
    values[6] = (uint8_t)mp_obj_get_int(args[4]);
    values[7] = (uint8_t)mp_obj_get_int(args[5]);
    values[8] = (uint8_t)mp_obj_get_int(args[6]);
    values[9] = (uint8_t)mp_obj_get_int(args[7]);
    values[10] = (uint8_t)mp_obj_get_int(args[8]);
    values[11] = (uint8_t)mp_obj_get_int(args[9]);
    values[12] = (uint8_t)mp_obj_get_int(args[10]);
    values[13] = (uint8_t)mp_obj_get_int(args[11]);
    values[14] = (uint8_t)mp_obj_get_int(args[12]);
    values[15] = (uint8_t)mp_obj_get_int(args[13]);
    values[16] = (uint8_t)mp_obj_get_int(args[14]);
    values[17] = (uint8_t)mp_obj_get_int(args[15]);
    values[18] = (uint8_t)mp_obj_get_int(args[16]);
    uart_set_all_actuators(values);    
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(epuck2_set_all_actuators_obj, 17, 17, epuck2_set_all_actuators);

/// \function get_proximity()
/// Get the values of the 8 infrared proximity sensors placed around the robot. The higher the value, the closer the object.
/// \return list of 8 integers between 0 (no object) and 4095 (object very close): index 0 is the front right sensor (Prox0), then the sensors follow clockwise up to the front left sensor (Prox7).
/// \example Stop when an object is in front of the robot
///     import epuck2
///     prox = epuck2.get_proximity()
///     if prox[0] > 500 or prox[7] > 500:
///         epuck2.set_motors_speed(0, 0)
/// \endexample
mp_obj_t epuck2_get_proximity(void) {
    mp_obj_list_t *data = MP_OBJ_TO_PTR(mp_obj_new_list(8, NULL));
    uint8_t prox_temp[16];
    uart_get_proximity(prox_temp);
    data->items[0] = mp_obj_new_int(prox_temp[0] + (((uint16_t)prox_temp[1])<<8));
    data->items[1] = mp_obj_new_int(prox_temp[2] + (((uint16_t)prox_temp[3])<<8));
    data->items[2] = mp_obj_new_int(prox_temp[4] + (((uint16_t)prox_temp[5])<<8));
    data->items[3] = mp_obj_new_int(prox_temp[6] + (((uint16_t)prox_temp[7])<<8));
    data->items[4] = mp_obj_new_int(prox_temp[8] + (((uint16_t)prox_temp[9])<<8));
    data->items[5] = mp_obj_new_int(prox_temp[10] + (((uint16_t)prox_temp[11])<<8));
    data->items[6] = mp_obj_new_int(prox_temp[12] + (((uint16_t)prox_temp[13])<<8));
    data->items[7] = mp_obj_new_int(prox_temp[14] + (((uint16_t)prox_temp[15])<<8));
    return MP_OBJ_FROM_PTR(data);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_proximity_obj, epuck2_get_proximity);

/// \function button_pressed()
/// Get the state of the user button of the ESP32 radio module.
/// \return `True` if the button is pressed, `False` otherwise.
/// \example Wait until the button is pressed and released
///     import epuck2
///     import time
///     while not epuck2.button_pressed():
///         time.sleep_ms(10)
///     while epuck2.button_pressed():
///         time.sleep_ms(10)
///     print("button pressed")
/// \endexample
mp_obj_t epuck2_button_pressed(void) {
    return mp_obj_new_bool(button_is_pressed());
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_button_pressed_obj, epuck2_button_pressed);

/// \function get_mic_volume()
/// Get the volume (amplitude) measured by the 4 microphones.
/// \return list of 4 integers between 0 and 4095: `[right, left, back, front]`.
/// \example Find the loudest microphone
///     import epuck2
///     vol = epuck2.get_mic_volume()
///     names = ("right", "left", "back", "front")
///     print("loudest:", names[vol.index(max(vol))])
/// \endexample
mp_obj_t epuck2_get_mic(void) {
    mp_obj_list_t *data = MP_OBJ_TO_PTR(mp_obj_new_list(4, NULL));
    uint8_t temp[8];
    uart_get_mic(temp);
    data->items[0] = mp_obj_new_int(temp[0] + (((uint16_t)temp[1])<<8));
    data->items[1] = mp_obj_new_int(temp[2] + (((uint16_t)temp[3])<<8));
    data->items[2] = mp_obj_new_int(temp[4] + (((uint16_t)temp[5])<<8));
    data->items[3] = mp_obj_new_int(temp[6] + (((uint16_t)temp[7])<<8));
    return MP_OBJ_FROM_PTR(data);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_mic_obj, epuck2_get_mic);

/// \function get_distance()
/// Get the distance measured by the front time of flight (ToF) sensor.
/// \return distance in millimeters, between 0 and 2000.
/// \example Print the distance of the object in front of the robot
///     import epuck2
///     print(epuck2.get_distance(), "mm")
/// \endexample
mp_obj_t epuck2_get_distance(void) {
    uint8_t temp[2];
    uart_get_distance(temp);    
    uint16_t dist = temp[0] + (((uint16_t)temp[1])<<8);
    if(dist > 2000)
    {
        dist = 2000;
    }
    return mp_obj_new_int(dist);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_distance_obj, epuck2_get_distance);

/// \function sd_available()
/// Check whether a micro SD card is inserted in the robot and can be read and written.
/// \return `True` if the micro SD card is available, `False` otherwise.
/// \example Check the micro SD card
///     import epuck2
///     print("micro SD available:", epuck2.sd_available())
/// \endexample
mp_obj_t epuck2_sd_available(void) {
    uint8_t temp;
    uart_get_sd_state(&temp);
    return mp_obj_new_bool(temp);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_sd_available_obj, epuck2_sd_available);

/// \function get_acc_raw()
/// Get the raw values of the accelerometer (range +-2 g).
/// \return list `[x, y, z]`, each value between -1500 and 1500.
/// \example Print the accelerometer values
///     import epuck2
///     x, y, z = epuck2.get_acc_raw()
///     print(x, y, z)
/// \endexample
mp_obj_t epuck2_get_acc_raw(void) {
    mp_obj_list_t *data = MP_OBJ_TO_PTR(mp_obj_new_list(3, NULL));
    uint8_t temp[6];
    uart_get_acc_raw(temp);
    data->items[0] = mp_obj_new_int((int16_t)(temp[0] + (temp[1]<<8)));
    data->items[1] = mp_obj_new_int((int16_t)(temp[2] + (temp[3]<<8)));
    data->items[2] = mp_obj_new_int((int16_t)(temp[4] + (temp[5]<<8)));
    return MP_OBJ_FROM_PTR(data);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_acc_raw_obj, epuck2_get_acc_raw);

/// \function get_battery()
/// Get the battery level as raw ADC value.
/// \return integer, raw value (the higher, the more charged the battery).
/// \example Print the battery level
///     import epuck2
///     print("battery:", epuck2.get_battery())
/// \endexample
mp_obj_t epuck2_get_battery(void) {
    uint8_t temp[2];
    uart_get_battery(temp);
    return mp_obj_new_int(temp[0] + (((uint16_t)temp[1])<<8));
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_battery_obj, epuck2_get_battery);

/// \function get_gyro_raw()
/// Get the raw values of the gyroscope (range +-250 degrees per second).
/// \return list `[x, y, z]`, each value between -32768 and 32767.
/// \example Print the rotation speed around the vertical axis
///     import epuck2
///     print(epuck2.get_gyro_raw()[2])
/// \endexample
mp_obj_t epuck2_get_gyro_raw(void) {
    mp_obj_list_t *data = MP_OBJ_TO_PTR(mp_obj_new_list(3, NULL));
    uint8_t temp[6];
    uart_get_gyro_raw(temp);
    data->items[0] = mp_obj_new_int((int16_t)(temp[0] + (temp[1]<<8)));
    data->items[1] = mp_obj_new_int((int16_t)(temp[2] + (temp[3]<<8)));
    data->items[2] = mp_obj_new_int((int16_t)(temp[4] + (temp[5]<<8)));
    return MP_OBJ_FROM_PTR(data);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_gyro_raw_obj, epuck2_get_gyro_raw);

/// \function get_tv_remote()
/// Get the last command received from a TV remote control (RC5 protocol) by the infrared receiver.
/// \return integer, data field of the RC5 command (e.g. the number of the button pressed).
/// \example Print the TV remote command
///     import epuck2
///     print(epuck2.get_tv_remote())
/// \endexample
mp_obj_t epuck2_get_tv_remote(void) {
    uint8_t temp;
    uart_get_tv_remote(&temp);
    return mp_obj_new_int(temp);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_tv_remote_obj, epuck2_get_tv_remote);

/// \function set_magnetometer(enable)
/// Enable or disable the magnetometer. Its values (x, y, z in micro Tesla) are returned by `get_all_sensors()` at
/// the indexes 9, 10 and 11.
/// The behavior depends on the IMU of the robot:
/// - MPU-9250: with the selector in position 15 the magnetometer is enabled at startup, in position 3 it must be
///   enabled with this function (values updated 8 times per second). The main microcontroller firmware doesn't
///   implement the disable yet: once enabled, the magnetometer keeps running until the robot is reset.
/// - ICM-20948: the magnetometer is always enabled (values updated 50 times per second), this function is not
///   needed.
/// \param enable `True` to enable the magnetometer, `False` to disable it.
/// \return `True` on success, `False` otherwise.
/// \example Enable the magnetometer and print its values
///     import epuck2
///     import time
///     epuck2.set_magnetometer(True)
///     time.sleep_ms(200)  # Wait for the first measures.
///     print(epuck2.get_all_sensors()[9:12])
/// \endexample
static mp_obj_t epuck2_set_magnetometer(mp_obj_t enable) {
    bool on = mp_obj_is_true(enable);
    int8_t ret;
    MP_THREAD_GIL_EXIT();
    ret = uart_set_magnetometer(on ? 1 : 0);
    MP_THREAD_GIL_ENTER();
    return mp_obj_new_bool(ret == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(epuck2_set_magnetometer_obj, epuck2_set_magnetometer);

/// \function get_selector()
/// Get the position of the rotary selector on top of the robot.
/// \return integer between 0 and 15.
/// \example Run a different behavior depending on the selector
///     import epuck2
///     if epuck2.get_selector() == 15:
///         print("camera images through SPI")
/// \endexample
mp_obj_t epuck2_get_selector(void) {
    uint8_t temp;
    uart_get_selector(&temp);
    return mp_obj_new_int(temp);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_selector_obj, epuck2_get_selector);

/// \function get_all_sensors()
/// Get the values of all the sensors with a single call.
/// \return list of 49 values:
/// | Index | Value | Range |
/// | 0, 1, 2 | Accelerometer x, y, z (raw) | -1500..1500, +-2 g |
/// | 3 | Acceleration magnitude sqrt(x^2 + y^2 + z^2) (float) | 0.0..about 2600.0 (about 3.46 g) |
/// | 4 | Orientation (float) | 0.0..360.0 degrees |
/// | 5 | Inclination (float) | 0.0..90.0 degrees, when tilted in any direction |
/// | 6, 7, 8 | Gyroscope x, y, z (raw) | -32768..32767, +-250 degrees per second |
/// | 9, 10, 11 | Magnetometer x, y, z (float), see `set_magnetometer()` | +-4912.0 uT (micro Tesla) |
/// | 12 | Temperature | degrees Celsius |
/// | 13..20 | Proximity sensors 0..7 | 0 (no object)..4095 (object very close) |
/// | 21..28 | Ambient light of the proximity sensors 0..7 | 0 (strong light)..4095 (dark) |
/// | 29 | Time of flight distance | millimeters |
/// | 30..33 | Microphones volume 0..3 (right, left, back, front) | 0..4095 |
/// | 34, 35 | Left and right motor steps | 1000 steps per wheel revolution |
/// | 36 | Battery (raw) | raw ADC value |
/// | 37 | Micro SD state | 1 if available, 0 otherwise |
/// | 38, 39, 40 | TV remote toggle, address, data | RC5 protocol |
/// | 41 | Selector position | 0..15 |
/// | 42, 43, 44 | Ground sensors 0..2 (ground sensors extension) | 0 (no surface or black)..1023 (white) |
/// | 45, 46, 47 | Ambient light of the ground sensors 0..2 | 0 (strong light)..1023 (dark) |
/// | 48 | User button | 1 pressed, 0 released |
/// \example Print the proximity sensors and the selector position
///     import epuck2
///     s = epuck2.get_all_sensors()
///     print("proximity:", s[13:21])
///     print("selector:", s[41])
/// \endexample
mp_obj_t epuck2_get_all_sensors(void) {
    mp_obj_list_t *data = MP_OBJ_TO_PTR(mp_obj_new_list(49, NULL));
    uint8_t values[103];
    float tempf = 0;
    uart_get_all_sensors(values);
    data->items[0] = mp_obj_new_int((int16_t)(values[0] + (values[1]<<8)));
    data->items[1] = mp_obj_new_int((int16_t)(values[2] + (values[3]<<8)));
    data->items[2] = mp_obj_new_int((int16_t)(values[4] + (values[5]<<8)));
    memcpy(&tempf, &values[6], sizeof(float));
    data->items[3] = mp_obj_new_float(tempf);
    memcpy(&tempf, &values[10], sizeof(float));
    data->items[4] = mp_obj_new_float(tempf);
    memcpy(&tempf, &values[14], sizeof(float));
    data->items[5] = mp_obj_new_float(tempf);
    data->items[6] = mp_obj_new_int((int16_t)(values[18] + (values[19]<<8)));
    data->items[7] = mp_obj_new_int((int16_t)(values[20] + (values[21]<<8)));
    data->items[8] = mp_obj_new_int((int16_t)(values[22] + (values[23]<<8)));
    // Magnetometer: floats, like the acceleration magnitude, orientation and inclination.
    memcpy(&tempf, &values[24], sizeof(float));
    data->items[9] = mp_obj_new_float(tempf);
    memcpy(&tempf, &values[28], sizeof(float));
    data->items[10] = mp_obj_new_float(tempf);
    memcpy(&tempf, &values[32], sizeof(float));
    data->items[11] = mp_obj_new_float(tempf);
    data->items[12] = mp_obj_new_int(values[36]);
    data->items[13] = mp_obj_new_int(values[37] + (((uint16_t)values[38])<<8));
    data->items[14] = mp_obj_new_int(values[39] + (((uint16_t)values[40])<<8));
    data->items[15] = mp_obj_new_int(values[41] + (((uint16_t)values[42])<<8));
    data->items[16] = mp_obj_new_int(values[43] + (((uint16_t)values[44])<<8));
    data->items[17] = mp_obj_new_int(values[45] + (((uint16_t)values[46])<<8));
    data->items[18] = mp_obj_new_int(values[47] + (((uint16_t)values[48])<<8));
    data->items[19] = mp_obj_new_int(values[49] + (((uint16_t)values[50])<<8));
    data->items[20] = mp_obj_new_int(values[51] + (((uint16_t)values[52])<<8));
    data->items[21] = mp_obj_new_int(values[53] + (((uint16_t)values[54])<<8));
    data->items[22] = mp_obj_new_int(values[55] + (((uint16_t)values[56])<<8));
    data->items[23] = mp_obj_new_int(values[57] + (((uint16_t)values[58])<<8));
    data->items[24] = mp_obj_new_int(values[59] + (((uint16_t)values[60])<<8));
    data->items[25] = mp_obj_new_int(values[61] + (((uint16_t)values[62])<<8));
    data->items[26] = mp_obj_new_int(values[63] + (((uint16_t)values[64])<<8));
    data->items[27] = mp_obj_new_int(values[65] + (((uint16_t)values[66])<<8));
    data->items[28] = mp_obj_new_int(values[67] + (((uint16_t)values[68])<<8));
    data->items[29] = mp_obj_new_int(values[69] + (((uint16_t)values[70])<<8));
    data->items[30] = mp_obj_new_int(values[71] + (((uint16_t)values[72])<<8));
    data->items[31] = mp_obj_new_int(values[73] + (((uint16_t)values[74])<<8));
    data->items[32] = mp_obj_new_int(values[75] + (((uint16_t)values[76])<<8));
    data->items[33] = mp_obj_new_int(values[77] + (((uint16_t)values[78])<<8));
    data->items[34] = mp_obj_new_int((int16_t)(values[79] + (values[80]<<8)));
    data->items[35] = mp_obj_new_int((int16_t)(values[81] + (values[82]<<8)));
    data->items[36] = mp_obj_new_int(values[83] + (((uint16_t)values[84])<<8));
    data->items[37] = mp_obj_new_int(values[85]);
    data->items[38] = mp_obj_new_int(values[86]);
    data->items[39] = mp_obj_new_int(values[87]);
    data->items[40] = mp_obj_new_int(values[88]);
    data->items[41] = mp_obj_new_int(values[89]);
    data->items[42] = mp_obj_new_int(values[90] + (((uint16_t)values[91])<<8));
    data->items[43] = mp_obj_new_int(values[92] + (((uint16_t)values[93])<<8));
    data->items[44] = mp_obj_new_int(values[94] + (((uint16_t)values[95])<<8));    
    data->items[45] = mp_obj_new_int(values[96] + (((uint16_t)values[97])<<8));
    data->items[46] = mp_obj_new_int(values[98] + (((uint16_t)values[99])<<8));
    data->items[47] = mp_obj_new_int(values[100] + (((uint16_t)values[101])<<8));
    data->items[48] = mp_obj_new_int(values[102]);
    return MP_OBJ_FROM_PTR(data);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_all_sensors_obj, epuck2_get_all_sensors);

/// \function play_freq(value)
/// Play a tone at the given frequency, until another frequency or 0 is given.
/// \param value frequency in Hz, between 100 and 10000 (values outside the range are limited to it); 0 stops the tone.
/// \example Play 500 Hz, then 2000 Hz, then stop
///     import epuck2
///     import time
///     epuck2.play_freq(500)
///     time.sleep(1)
///     epuck2.play_freq(2000)
///     time.sleep(1)
///     epuck2.play_freq(0)
/// \endexample
mp_obj_t epuck2_play_freq(mp_obj_t value) {
    uint16_t val = mp_obj_get_int(value);
    if((val < 100) && (val != 0)) // 0 to stop sound
    {
        val = 100;
    }
    else if(val > 10000)
    {
        val = 10000;
    }
    uart_set_sound_freq(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(epuck2_play_freq_obj, epuck2_play_freq);

/// \function get_mic_data(buffer)
/// Get the samples of the 4 microphones: 160 samples per microphone, 10 ms of sound (16 kHz sampling rate). The function waits until the data is received.
/// \param buffer bytearray of at least 1280 bytes, filled with 16-bit signed samples (little endian) interleaved as right, left, back, front: `R0, L0, B0, F0, R1, L1, ..., F159`.
/// \return `True` on success, `False` otherwise. Raise `ValueError` if the buffer is smaller than 1280 bytes.
/// \example Get the samples of the front microphone
///     import epuck2
///     import struct
///     buf = bytearray(1280)
///     if epuck2.get_mic_data(buf):
///         front = [struct.unpack_from("<h", buf, (i * 4 + 3) * 2)[0] for i in range(160)]
///         print(max(front), min(front))
/// \endexample
mp_obj_t epuck2_get_mic_data(mp_obj_t buffer_obj) {
    // Get a pointer to the internal memory of the MicroPython bytearray
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_obj, &bufinfo, MP_BUFFER_WRITE);

    // Ensure the buffer is large enough (160 samples * 4 mics * 2 bytes = 1280)
    if (bufinfo.len < 1280) {
        mp_raise_ValueError(MP_ERROR_TEXT("Buffer too small"));
    }

    // Call your C function directly into the bytearray's memory
    int8_t status = uart_get_mic_data((uint8_t *)bufinfo.buf);
    
    if (status == 0) {
        return mp_const_true;
    } else {
        return mp_const_false;
    }
}
// Update to MP_DEFINE_CONST_FUN_OBJ_1 because it now takes 1 argument
static MP_DEFINE_CONST_FUN_OBJ_1(epuck2_get_mic_data_obj, epuck2_get_mic_data);

/******************************************************************************/
// Camera.
// The image source depends on the F407 selector position (read at the first camera call):
// - selector 15: the F407 streams 160x120 RGB565 images to the ESP32 through SPI, the format set with
//   camera_set_format() (crop, downscale, greyscale) is applied by the ESP32 while copying the image;
// - other positions (e.g. 3): the image is requested to the F407 with the asercom command -'I' through UART
//   (default 40x40 RGB565, format set on the F407 side, at most 4136 bytes).

#define CAM_SRC_UNKNOWN 0
#define CAM_SRC_SPI     1
#define CAM_SRC_UART    2

static uint8_t cam_source = CAM_SRC_UNKNOWN;
static uint32_t cam_last_id = 0;
static uint8_t cam_width = 0, cam_height = 0, cam_bpp = 0; // Format of the last image (0 = unknown).
static uint32_t cam_uart_frames = 0, cam_uart_errors = 0;
static spi_img_fmt_t cam_spi_fmt = {0, 0, SPI_IMG_WIDTH, SPI_IMG_HEIGHT, 1, false}; // Conversion of the SPI image.

static uint8_t camera_source(void) {
    if (cam_source == CAM_SRC_UNKNOWN) {
        uint8_t selector = 0;
        MP_THREAD_GIL_EXIT();
        uart_get_selector(&selector);
        MP_THREAD_GIL_ENTER();
        if (selector == 15) {
            cam_source = CAM_SRC_SPI;
            cam_width = SPI_IMG_WIDTH;
            cam_height = SPI_IMG_HEIGHT;
            cam_bpp = 2;
        } else {
            cam_source = CAM_SRC_UART;
        }
    }
    return cam_source;
}

static mp_obj_t camera_get_image_spi(mp_buffer_info_t *bufinfo, mp_int_t timeout_ms) {
    if (bufinfo->len < (size_t)cam_width * cam_height * cam_bpp) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer too small"));
    }
    mp_uint_t start = mp_hal_ticks_ms();
    for (;;) {
        bool available;
        MP_THREAD_GIL_EXIT();
        available = spi_wait_image(cam_last_id, pdMS_TO_TICKS(10));
        MP_THREAD_GIL_ENTER();

        if (available) {
            uint32_t id = 0;
            int32_t n;
            MP_THREAD_GIL_EXIT();
            n = spi_copy_image((uint8_t *)bufinfo->buf, bufinfo->len, &id, &cam_spi_fmt);
            MP_THREAD_GIL_ENTER();
            if (n > 0) {
                cam_last_id = id;
                return mp_obj_new_int_from_uint(id);
            }
        }

        mp_handle_pending(true); // Allow Ctrl-C.
        if (timeout_ms >= 0 && (mp_hal_ticks_ms() - start) >= (mp_uint_t)timeout_ms) {
            return mp_const_none;
        }
    }
}

static mp_obj_t camera_get_image_uart(mp_buffer_info_t *bufinfo) {
    uint8_t mode = 0, width = 0, height = 0;
    int32_t n;
    MP_THREAD_GIL_EXIT();
    n = uart_get_image((uint8_t *)bufinfo->buf, bufinfo->len, &mode, &width, &height);
    MP_THREAD_GIL_ENTER();
    if (n <= 0) {
        cam_uart_errors++;
        return mp_const_none;
    }
    cam_uart_frames++;
    cam_width = width;
    cam_height = height;
    cam_bpp = (mode == UART_IMG_MODE_GREY) ? 1 : 2;
    if (++cam_last_id == 0) {
        cam_last_id = 1;
    }
    return mp_obj_new_int_from_uint(cam_last_id);
}

/// \function camera_get_image(buffer, timeout_ms=1000)
/// Copy a new camera image into `buffer`, in the format returned by `camera_get_format()`.
/// - Selector 15 (SPI): wait for an image newer than the one returned by the previous call, at most `timeout_ms` milliseconds (-1 waits forever, Ctrl-C stops the wait).
/// - Selector 3 (UART): request a new image to the main microcontroller and wait for it. `timeout_ms` is not used: the request fails by itself after about 1 s without answer.
/// \param buffer bytearray of at least width * height * bytes_per_pixel bytes of the current format. `CAM_IMAGE_SIZE` bytes are enough for every format.
/// \param timeout_ms maximum waiting time in milliseconds (SPI only), default 1000.
/// \return the image id, incremented at each image received (images skipped by the program can be detected by comparing consecutive ids), or `None` on timeout or error.
/// \example Get an image and print the value of the central pixel
///     import epuck2
///     img = bytearray(epuck2.CAM_IMAGE_SIZE)
///     if epuck2.camera_get_image(img) is not None:
///         width, height, bpp = epuck2.camera_get_format()
///         i = ((height // 2) * width + width // 2) * bpp
///         print(img[i:i + bpp])
/// \endexample
static mp_obj_t epuck2_camera_get_image(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[0], &bufinfo, MP_BUFFER_WRITE);
    mp_int_t timeout_ms = (n_args > 1) ? mp_obj_get_int(args[1]) : 1000;

    if (camera_source() == CAM_SRC_SPI) {
        return camera_get_image_spi(&bufinfo, timeout_ms);
    }
    return camera_get_image_uart(&bufinfo);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(epuck2_camera_get_image_obj, 1, 2, epuck2_camera_get_image);

/// \function camera_get_format()
/// Get the format of the images returned by `camera_get_image()`.
/// \return tuple `(width, height, bytes_per_pixel)`: `bytes_per_pixel` is 2 for RGB565 and 1 for greyscale. With selector 3 (UART) the format is known only after the first image, so `None` is returned before it.
/// \example Print the image format
///     import epuck2
///     img = bytearray(epuck2.CAM_IMAGE_SIZE)
///     epuck2.camera_get_image(img)
///     print(epuck2.camera_get_format())  # (160, 120, 2) with selector 15.
/// \endexample
static mp_obj_t epuck2_camera_get_format(void) {
    camera_source();
    if (cam_bpp == 0) {
        return mp_const_none;
    }
    mp_obj_t items[3] = {
        MP_OBJ_NEW_SMALL_INT(cam_width),
        MP_OBJ_NEW_SMALL_INT(cam_height),
        MP_OBJ_NEW_SMALL_INT(cam_bpp),
    };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_camera_get_format_obj, epuck2_camera_get_format);

#define CAM_SENSOR_WIDTH        640
#define CAM_SENSOR_HEIGHT       480
#define CAM_UART_MAX_IMAGE_SIZE 4136 // Asercom image buffer on the F407.

/// \function camera_set_format(width, height, bytes_per_pixel=2, zoom=4, x1=-1, y1=-1)
/// Set the format of the images returned by `camera_get_image()`. The image covers an area of (width * zoom) x (height * zoom) pixels of the 640x480 camera sensor: a smaller zoom gives a more magnified image.
/// \param width image width in pixels.
/// \param height image height in pixels.
/// \param bytes_per_pixel 2 for RGB565 (color), 1 for greyscale.
/// \param zoom sensor pixels per image pixel, see below.
/// \param x1 left side of the area on the sensor, -1 to center the area horizontally.
/// \param y1 top side of the area on the sensor, -1 to center the area vertically.
/// The allowed values depend on the selector position:
/// - Selector 15 (SPI): the main microcontroller always streams the whole sensor at zoom 4 (160x120 RGB565) and the ESP32 converts the image while copying it. Zoom 4 crops the stream at full resolution, zoom 8 averages 2x2 stream pixels, zoom 16 averages 4x4 stream pixels. `x1` and `y1` are rounded down to multiples of 4. Default 160x120 RGB565 zoom 4.
/// - Selector 3 (UART): the format is set on the main microcontroller. Zoom 1, 2 or 4, width and height between 1 and 255, image size (width * height * bytes_per_pixel) at most 4136 bytes. Default 40x40 RGB565 zoom 4.
/// \return `True` if the format is set (UART: confirmed by the main microcontroller), `False` otherwise. Raise `ValueError` if a parameter is out of range.
/// \example Selector 15: whole view in greyscale at half resolution (80x60, 4800 bytes)
///     import epuck2
///     epuck2.camera_set_format(80, 60, 1, 8)
///     img = bytearray(80 * 60)
///     epuck2.camera_get_image(img)
/// \endexample
/// \example Selector 15: central 80x60 area at full resolution
///     epuck2.camera_set_format(80, 60, 2, 4)
/// \endexample
/// \example Selector 3: 64x64 greyscale image
///     epuck2.camera_set_format(64, 64, 1, 4)
/// \endexample
static mp_obj_t epuck2_camera_set_format(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_width, ARG_height, ARG_bytes_per_pixel, ARG_zoom, ARG_x1, ARG_y1 };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_width, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_height, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_bytes_per_pixel, MP_ARG_INT, {.u_int = 2} },
        { MP_QSTR_zoom, MP_ARG_INT, {.u_int = 4} },
        { MP_QSTR_x1, MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_y1, MP_ARG_INT, {.u_int = -1} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    mp_int_t width = args[ARG_width].u_int;
    mp_int_t height = args[ARG_height].u_int;
    mp_int_t bpp = args[ARG_bytes_per_pixel].u_int;
    mp_int_t zoom = args[ARG_zoom].u_int;
    mp_int_t x1 = args[ARG_x1].u_int;
    mp_int_t y1 = args[ARG_y1].u_int;

    bool spi = (camera_source() == CAM_SRC_SPI);
    bool valid = width >= 1 && height >= 1 && (bpp == 1 || bpp == 2) && x1 >= -1 && y1 >= -1
        && width * zoom <= CAM_SENSOR_WIDTH && height * zoom <= CAM_SENSOR_HEIGHT
        && (x1 < 0 || x1 + width * zoom <= CAM_SENSOR_WIDTH)
        && (y1 < 0 || y1 + height * zoom <= CAM_SENSOR_HEIGHT);
    if (spi) {
        valid = valid && (zoom == 4 || zoom == 8 || zoom == 16);
    } else {
        // The F407 doesn't report configuration errors, so check here everything the camera driver would refuse:
        // size 1..255 (1 byte in the image header), image at most 4136 bytes, zoom 1/2/4.
        valid = valid && width <= 255 && height <= 255 && width * height * bpp <= CAM_UART_MAX_IMAGE_SIZE
            && (zoom == 1 || zoom == 2 || zoom == 4);
    }
    if (!valid) {
        // A single error message is used on purpose: every new MP_ERROR_TEXT message makes all the error texts of
        // the firmware (ulab included) more expensive to compile; 7 messages made the firmware about 17 KB bigger.
        mp_raise_ValueError(MP_ERROR_TEXT("invalid camera format"));
    }

    if (spi) {
        // Area in stream pixels (1 stream pixel = 4x4 sensor pixels).
        mp_int_t step = zoom / 4;
        mp_int_t x0 = (x1 < 0) ? (SPI_IMG_WIDTH - width * step) / 2 : x1 / 4;
        mp_int_t y0 = (y1 < 0) ? (SPI_IMG_HEIGHT - height * step) / 2 : y1 / 4;
        cam_spi_fmt.x0 = x0;
        cam_spi_fmt.y0 = y0;
        cam_spi_fmt.width = width;
        cam_spi_fmt.height = height;
        cam_spi_fmt.step = step;
        cam_spi_fmt.grey = (bpp == 1);
        cam_width = width;
        cam_height = height;
        cam_bpp = bpp;
        return mp_const_true;
    }

    int8_t ret;
    MP_THREAD_GIL_EXIT();
    ret = uart_set_camera(bpp == 1 ? UART_IMG_MODE_GREY : UART_IMG_MODE_RGB565, width, height, zoom, x1, y1);
    MP_THREAD_GIL_ENTER();
    if (ret != 0) {
        return mp_const_false;
    }
    cam_width = width;
    cam_height = height;
    cam_bpp = bpp;
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(epuck2_camera_set_format_obj, 2, epuck2_camera_set_format);

/// \function camera_get_stats()
/// Get the image counters since boot or since the last `camera_reset_stats()`.
/// \return tuple `(frames, errors)`. Selector 15 (SPI): images received by the ESP32 (streamed continuously, also when the program doesn't read them) and images discarded because of transfer errors. Selector 3 (UART): images received and failed requests.
/// \example Measure the frame rate of the SPI stream
///     import epuck2
///     import time
///     epuck2.camera_reset_stats()
///     time.sleep(5)
///     frames, errors = epuck2.camera_get_stats()
///     print(frames / 5, "fps,", errors, "errors")
/// \endexample
static mp_obj_t epuck2_camera_get_stats(void) {
    uint32_t frames, errors;
    if (camera_source() == CAM_SRC_SPI) {
        spi_get_image_stats(&frames, &errors);
    } else {
        frames = cam_uart_frames;
        errors = cam_uart_errors;
    }
    mp_obj_t items[2] = {
        mp_obj_new_int_from_uint(frames),
        mp_obj_new_int_from_uint(errors),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_camera_get_stats_obj, epuck2_camera_get_stats);

/// \function camera_reset_stats()
/// Reset the counters returned by `camera_get_stats()`.
static mp_obj_t epuck2_camera_reset_stats(void) {
    spi_reset_image_stats();
    cam_uart_frames = 0;
    cam_uart_errors = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_camera_reset_stats_obj, epuck2_camera_reset_stats);

/******************************************************************************/
// Image processing on the images returned by camera_get_image() (any buffer can be used).
// fmt: (width, height, bytes_per_pixel) as returned by camera_get_format().
// roi: (x, y, w, h) area to process, None for the whole image.
// Blobs: groups of selected pixels touching each other, returned as a list of
// (pixels, cx, cy, x_min, y_min, x_max, y_max) sorted by size (biggest first).
// Existing error messages are used on purpose (see camera_set_format()).

#define IMAGE_MAX_BLOBS 64

static void image_parse_fmt(mp_obj_t fmt_obj, img_fmt_t *fmt) {
    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(fmt_obj, &len, &items);
    if (len != 3) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    }
    mp_int_t w = mp_obj_get_int(items[0]), h = mp_obj_get_int(items[1]), bpp = mp_obj_get_int(items[2]);
    if (w < 1 || w > 0xFFFF || h < 1 || h > 0xFFFF || (bpp != 1 && bpp != 2)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    }
    fmt->width = w;
    fmt->height = h;
    fmt->bpp = bpp;
}

static const uint8_t *image_get_buffer(mp_obj_t buf_obj, const img_fmt_t *fmt) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_obj, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len < (size_t)fmt->width * fmt->height * fmt->bpp) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer too small"));
    }
    return (const uint8_t *)bufinfo.buf;
}

static void image_parse_roi(mp_obj_t roi_obj, const img_fmt_t *fmt, img_roi_t *roi) {
    if (roi_obj == mp_const_none) {
        roi->x = roi->y = 0;
        roi->w = fmt->width;
        roi->h = fmt->height;
        return;
    }
    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(roi_obj, &len, &items);
    if (len != 4) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    }
    mp_int_t x = mp_obj_get_int(items[0]), y = mp_obj_get_int(items[1]);
    mp_int_t w = mp_obj_get_int(items[2]), h = mp_obj_get_int(items[3]);
    if (x < 0 || y < 0 || w < 1 || h < 1 || x + w > fmt->width || y + h > fmt->height) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    }
    roi->x = x;
    roi->y = y;
    roi->w = w;
    roi->h = h;
}

static mp_obj_t image_blobs(img_sel_t *sel, mp_obj_t roi_obj, mp_int_t min_pixels, mp_int_t max_blobs) {
    img_roi_t roi;
    if (min_pixels < 1 || max_blobs < 1 || max_blobs > IMAGE_MAX_BLOBS) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    }
    image_parse_roi(roi_obj, &sel->fmt, &roi);
    size_t work_size = img_blobs_work_size(roi.w);
    uint8_t *work = m_new(uint8_t, work_size);
    img_blob_t *blobs = m_new(img_blob_t, max_blobs);
    int n = img_find_blobs(sel, &roi, min_pixels, blobs, max_blobs, work);
    mp_obj_list_t *list = MP_OBJ_TO_PTR(mp_obj_new_list(n, NULL));
    for (int i = 0; i < n; i++) {
        mp_obj_t items[7] = {
            mp_obj_new_int_from_uint(blobs[i].pixels),
            MP_OBJ_NEW_SMALL_INT(blobs[i].cx),
            MP_OBJ_NEW_SMALL_INT(blobs[i].cy),
            MP_OBJ_NEW_SMALL_INT(blobs[i].x_min),
            MP_OBJ_NEW_SMALL_INT(blobs[i].y_min),
            MP_OBJ_NEW_SMALL_INT(blobs[i].x_max),
            MP_OBJ_NEW_SMALL_INT(blobs[i].y_max),
        };
        list->items[i] = mp_obj_new_tuple(7, items);
    }
    m_del(img_blob_t, blobs, max_blobs);
    m_del(uint8_t, work, work_size);
    return MP_OBJ_FROM_PTR(list);
}

/// \function image_get_color(img, fmt, roi=None)
/// Get the mean color of an area of an RGB565 image, as HSV (hue, saturation, value). It can be used to learn the color of an object placed in front of the camera, then search it with `image_find_blobs_color()`.
/// \param img RGB565 image.
/// \param fmt image format `(width, height, 2)`.
/// \param roi area `(x, y, w, h)`, `None` for the whole image.
/// \return tuple `(hue, saturation, value)`: hue between 0 and 359 degrees (0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta), saturation (0 grey, 100 pure color) and value (brightness, 0 black) between 0 and 100.
/// \example Learn the color of the object in the middle of a 80x60 image
///     hue, sat, val = epuck2.image_get_color(img, (80, 60, 2), roi=(35, 25, 10, 10))
///     hue_min, hue_max = (hue - 15) % 360, (hue + 15) % 360
/// \endexample
static mp_obj_t epuck2_image_get_color(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_img, ARG_fmt, ARG_roi };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_img, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_fmt, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_roi, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    img_fmt_t fmt;
    img_roi_t roi;
    image_parse_fmt(args[ARG_fmt].u_obj, &fmt);
    if (fmt.bpp != 2) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    }
    const uint8_t *img = image_get_buffer(args[ARG_img].u_obj, &fmt);
    image_parse_roi(args[ARG_roi].u_obj, &fmt, &roi);
    uint16_t h;
    uint8_t s, v;
    img_get_color(img, &fmt, &roi, &h, &s, &v);
    mp_obj_t items[3] = { MP_OBJ_NEW_SMALL_INT(h), MP_OBJ_NEW_SMALL_INT(s), MP_OBJ_NEW_SMALL_INT(v) };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(epuck2_image_get_color_obj, 2, epuck2_image_get_color);

/// \function image_find_blobs_color(img, fmt, hue_min, hue_max, sat_min=30, sat_max=100, val_min=10, val_max=100, roi=None, min_pixels=10, max_blobs=8)
/// Find the blobs of the pixels of an RGB565 image whose color is in the given HSV range. The hue doesn't change much when the object is in shadow or more illuminated, so it is more reliable than the RGB values.
/// \param hue_min minimum hue, between 0 and 359 degrees.
/// \param hue_max maximum hue, between 0 and 359 degrees. If `hue_min` is greater than `hue_max` the range wraps around 0: for example 340, 20 selects the red colors.
/// \param sat_min minimum saturation, between 0 and 100 (default 30: greyish pixels, whose hue is unreliable, are ignored).
/// \param sat_max maximum saturation, between 0 and 100.
/// \param val_min minimum value (brightness), between 0 and 100 (default 10: dark pixels, whose hue is unreliable, are ignored).
/// \param val_max maximum value, between 0 and 100.
/// See the "Image processing" section for `img`, `fmt`, `roi`, `min_pixels` and `max_blobs`.
/// \return list of blobs `[(pixels, cx, cy, x_min, y_min, x_max, y_max), ...]`, biggest first, `[]` if none.
/// \example Follow a green object (selector 15)
///     import epuck2
///     FMT = (80, 60, 2)
///     epuck2.camera_set_format(80, 60, 2, 8)
///     img = bytearray(80 * 60 * 2)
///     while True:
///         if epuck2.camera_get_image(img) is None:
///             continue
///         blobs = epuck2.image_find_blobs_color(img, FMT, 90, 150, max_blobs=1)
///         if blobs:
///             err = blobs[0][1] - 40  # Horizontal distance from the image center.
///             epuck2.set_motors_speed(300 + 10 * err, 300 - 10 * err)
///         else:
///             epuck2.set_motors_speed(0, 0)
/// \endexample
static mp_obj_t epuck2_image_find_blobs_color(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_img, ARG_fmt, ARG_hue_min, ARG_hue_max, ARG_sat_min, ARG_sat_max, ARG_val_min, ARG_val_max,
           ARG_roi, ARG_min_pixels, ARG_max_blobs };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_img, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_fmt, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_hue_min, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_hue_max, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_sat_min, MP_ARG_INT, {.u_int = 30} },
        { MP_QSTR_sat_max, MP_ARG_INT, {.u_int = 100} },
        { MP_QSTR_val_min, MP_ARG_INT, {.u_int = 10} },
        { MP_QSTR_val_max, MP_ARG_INT, {.u_int = 100} },
        { MP_QSTR_roi, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_min_pixels, MP_ARG_INT, {.u_int = 10} },
        { MP_QSTR_max_blobs, MP_ARG_INT, {.u_int = 8} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    img_sel_t sel = { .type = IMG_SEL_COLOR };
    image_parse_fmt(args[ARG_fmt].u_obj, &sel.fmt);
    mp_int_t hue_min = args[ARG_hue_min].u_int, hue_max = args[ARG_hue_max].u_int;
    mp_int_t sat_min = args[ARG_sat_min].u_int, sat_max = args[ARG_sat_max].u_int;
    mp_int_t val_min = args[ARG_val_min].u_int, val_max = args[ARG_val_max].u_int;
    if (sel.fmt.bpp != 2 || hue_min < 0 || hue_min > 359 || hue_max < 0 || hue_max > 359
        || sat_min < 0 || sat_min > sat_max || sat_max > 100 || val_min < 0 || val_min > val_max || val_max > 100) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    }
    sel.img = image_get_buffer(args[ARG_img].u_obj, &sel.fmt);
    sel.hue_min = hue_min;
    sel.hue_max = hue_max;
    sel.sat_min = sat_min;
    sel.sat_max = sat_max;
    sel.val_min = val_min;
    sel.val_max = val_max;
    return image_blobs(&sel, args[ARG_roi].u_obj, args[ARG_min_pixels].u_int, args[ARG_max_blobs].u_int);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(epuck2_image_find_blobs_color_obj, 4, epuck2_image_find_blobs_color);

/// \function image_find_blobs_grey(img, fmt, grey_min, grey_max, roi=None, min_pixels=10, max_blobs=8)
/// Find the blobs of the pixels whose grey level is in the given range. It works with greyscale and RGB565 images (for RGB565 images the luminance of the pixel is used).
/// \param grey_min minimum grey level, between 0 (black) and 255 (white).
/// \param grey_max maximum grey level, between `grey_min` and 255.
/// See the "Image processing" section for `img`, `fmt`, `roi`, `min_pixels` and `max_blobs`.
/// \return list of blobs `[(pixels, cx, cy, x_min, y_min, x_max, y_max), ...]`, biggest first, `[]` if none.
/// \example Find the dark objects in a greyscale image
///     blobs = epuck2.image_find_blobs_grey(img, (80, 60, 1), 0, 50)
///     print(len(blobs), "dark objects")
/// \endexample
static mp_obj_t epuck2_image_find_blobs_grey(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_img, ARG_fmt, ARG_grey_min, ARG_grey_max, ARG_roi, ARG_min_pixels, ARG_max_blobs };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_img, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_fmt, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_grey_min, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_grey_max, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_roi, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_min_pixels, MP_ARG_INT, {.u_int = 10} },
        { MP_QSTR_max_blobs, MP_ARG_INT, {.u_int = 8} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    img_sel_t sel = { .type = IMG_SEL_GREY };
    image_parse_fmt(args[ARG_fmt].u_obj, &sel.fmt);
    mp_int_t grey_min = args[ARG_grey_min].u_int, grey_max = args[ARG_grey_max].u_int;
    if (grey_min < 0 || grey_min > grey_max || grey_max > 255) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    }
    sel.img = image_get_buffer(args[ARG_img].u_obj, &sel.fmt);
    sel.grey_min = grey_min;
    sel.grey_max = grey_max;
    return image_blobs(&sel, args[ARG_roi].u_obj, args[ARG_min_pixels].u_int, args[ARG_max_blobs].u_int);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(epuck2_image_find_blobs_grey_obj, 4, epuck2_image_find_blobs_grey);

/// \function image_diff(img, prev, fmt, threshold=20, roi=None, min_pixels=10, max_blobs=8)
/// Find the areas that changed between two images of the same format (motion detection): the pixels whose grey level (luminance for RGB565 images) differs by at least `threshold`. The robot must not move, otherwise the whole image changes.
/// \param img current image.
/// \param prev previous image, same format as `img`.
/// \param threshold minimum difference of grey level, between 1 and 255 (default 20).
/// See the "Image processing" section for `fmt`, `roi`, `min_pixels` and `max_blobs`.
/// \return list of blobs `[(pixels, cx, cy, x_min, y_min, x_max, y_max), ...]`, biggest first, `[]` if none.
/// \example Detect motion in front of the robot (selector 15)
///     import epuck2
///     FMT = (80, 60, 1)
///     epuck2.camera_set_format(80, 60, 1, 8)
///     img = bytearray(80 * 60)
///     prev = bytearray(80 * 60)
///     epuck2.camera_get_image(prev)
///     while True:
///         if epuck2.camera_get_image(img) is None:
///             continue
///         moving = epuck2.image_diff(img, prev, FMT, min_pixels=30, max_blobs=1)
///         if moving:
///             print("motion at", moving[0][1], moving[0][2])
///         img, prev = prev, img  # The current image becomes the previous one.
/// \endexample
static mp_obj_t epuck2_image_diff(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_img, ARG_prev, ARG_fmt, ARG_threshold, ARG_roi, ARG_min_pixels, ARG_max_blobs };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_img, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_prev, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_fmt, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_threshold, MP_ARG_INT, {.u_int = 20} },
        { MP_QSTR_roi, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_min_pixels, MP_ARG_INT, {.u_int = 10} },
        { MP_QSTR_max_blobs, MP_ARG_INT, {.u_int = 8} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    img_sel_t sel = { .type = IMG_SEL_DIFF };
    image_parse_fmt(args[ARG_fmt].u_obj, &sel.fmt);
    mp_int_t threshold = args[ARG_threshold].u_int;
    if (threshold < 1 || threshold > 255) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    }
    sel.img = image_get_buffer(args[ARG_img].u_obj, &sel.fmt);
    sel.prev = image_get_buffer(args[ARG_prev].u_obj, &sel.fmt);
    sel.diff_threshold = threshold;
    return image_blobs(&sel, args[ARG_roi].u_obj, args[ARG_min_pixels].u_int, args[ARG_max_blobs].u_int);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(epuck2_image_diff_obj, 3, epuck2_image_diff);

/// \constant CAM_IMAGE_SIZE
/// Size in bytes of the biggest camera image (160x120 RGB565, 38400 bytes): a buffer of this size can hold the images of every format.
/// \example Allocate the image buffer
///     import epuck2
///     img = bytearray(epuck2.CAM_IMAGE_SIZE)
/// \endexample

mp_rom_map_elem_t epuck2_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_epuck2) },

    { MP_ROM_QSTR(MP_QSTR_get_api_version), MP_ROM_PTR(&get_api_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_rgb), MP_ROM_PTR(&epuck2_set_rgb_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_all_rgb), MP_ROM_PTR(&epuck2_set_all_rgb_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_proximity), MP_ROM_PTR(&epuck2_get_proximity_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_motors_speed), MP_ROM_PTR(&epuck2_set_motors_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_button_pressed), MP_ROM_PTR(&epuck2_button_pressed_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_mic_volume), MP_ROM_PTR(&epuck2_get_mic_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_distance), MP_ROM_PTR(&epuck2_get_distance_obj) },
    { MP_ROM_QSTR(MP_QSTR_sd_available), MP_ROM_PTR(&epuck2_sd_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_acc_raw), MP_ROM_PTR(&epuck2_get_acc_raw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_battery), MP_ROM_PTR(&epuck2_get_battery_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_gyro_raw), MP_ROM_PTR(&epuck2_get_gyro_raw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tv_remote), MP_ROM_PTR(&epuck2_get_tv_remote_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_magnetometer), MP_ROM_PTR(&epuck2_set_magnetometer_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_selector), MP_ROM_PTR(&epuck2_get_selector_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_leds), MP_ROM_PTR(&epuck2_set_leds_obj) },
    { MP_ROM_QSTR(MP_QSTR_play_sound), MP_ROM_PTR(&epuck2_play_sound_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_all_actuators), MP_ROM_PTR(&epuck2_set_all_actuators_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_all_sensors), MP_ROM_PTR(&epuck2_get_all_sensors_obj) },
    { MP_ROM_QSTR(MP_QSTR_play_freq), MP_ROM_PTR(&epuck2_play_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_mic_data), MP_ROM_PTR(&epuck2_get_mic_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_camera_get_image), MP_ROM_PTR(&epuck2_camera_get_image_obj) },
    { MP_ROM_QSTR(MP_QSTR_camera_get_format), MP_ROM_PTR(&epuck2_camera_get_format_obj) },
    { MP_ROM_QSTR(MP_QSTR_camera_set_format), MP_ROM_PTR(&epuck2_camera_set_format_obj) },
    { MP_ROM_QSTR(MP_QSTR_camera_get_stats), MP_ROM_PTR(&epuck2_camera_get_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_camera_reset_stats), MP_ROM_PTR(&epuck2_camera_reset_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_CAM_IMAGE_SIZE), MP_ROM_INT(SPI_IMG_SIZE) },
    { MP_ROM_QSTR(MP_QSTR_image_get_color), MP_ROM_PTR(&epuck2_image_get_color_obj) },
    { MP_ROM_QSTR(MP_QSTR_image_find_blobs_color), MP_ROM_PTR(&epuck2_image_find_blobs_color_obj) },
    { MP_ROM_QSTR(MP_QSTR_image_find_blobs_grey), MP_ROM_PTR(&epuck2_image_find_blobs_grey_obj) },
    { MP_ROM_QSTR(MP_QSTR_image_diff), MP_ROM_PTR(&epuck2_image_diff_obj) },
};

static MP_DEFINE_CONST_DICT(epuck2_module_globals, epuck2_module_globals_table);

const mp_obj_module_t epuck2_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&epuck2_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_epuck2, epuck2_module);
