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

char api_version[6] = "XX.XX\0";
char temp_buff[512];

// Get current e-puck2 API version.
static mp_obj_t get_api_version(void) {
    snprintf(api_version, 6, "%2d.%-2d", EPUCK2_API_MAJOR_VERSION, EPUCK2_API_MINOR_VERSION);
    return mp_obj_new_str(api_version, strlen(api_version));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_api_version_obj, get_api_version);

/// \method epuck2_set_rgb([led, value red, value green, value blue])
/// Set the LED intensities.  Intensity ranges between 0 (off) and 100 (full on).
/// Return `None`.
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

/// \method eupck2_set_all_rgb([red2, green2, blue2, red4, green4, blue4, red6, green6, blue6, red8, green8, blue8])
/// Set all LED intensities.  Intensity ranges between 0 (off) and 100 (full on).
/// Return `None`.
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

/// \method epuck2_set_speed()
/// Set motors speed. Range is between -1000 and 1000.
/// Return `None`.
mp_obj_t epuck2_set_motors_speed(mp_obj_t left, mp_obj_t right) {
    int l = mp_obj_get_int(left);
    int r = mp_obj_get_int(right);
    uart_set_motors_speed(l, r);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(epuck2_set_motors_speed_obj, epuck2_set_motors_speed);

/// \method epuck2_set_leds()
/// Set the LEDs state (on, off).
/// \param  bit0: 0=LED1 off; 1=LED1 on
///         bit1: 0=LED3 off; 1=LED3 on
///         bit2: 0=LED5 off; 1=LED5 on
///         bit3: 0=LED7 off; 1=LED7 on
///         bit4: 0=body LED off; 1=body LED on
///         bit5: 0=front LED off; 1=front LED on
/// Return `None`.
mp_obj_t epuck2_set_leds(mp_obj_t value) {
    uint8_t val = mp_obj_get_int(value);
    uart_set_leds(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(epuck2_set_leds_obj, epuck2_set_leds);

/// \method epuck2_play_sound()
/// Play onboard sound.
/// \param  id: 0x01=MARIO, 0x02=UNDERWOLRD, 0x04=STARWARS, 0x08=4KHz, 0x10=10KHz, 0x20=stop sound
/// Return `None`.
mp_obj_t epuck2_play_sound(mp_obj_t value) {
    uint8_t val = mp_obj_get_int(value);
    uart_set_sound(val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(epuck2_play_sound_obj, epuck2_play_sound);

/// \method set_all_actuators([settings, motor left, motor right, leds, red2, green2, blue2, red4, green4, blue4, red6, green6, blue6, red8, green8, blue8, sound])
/// Set all actuators at once.
/// Return `None`.
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
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(epuck2_set_all_actuators_obj, 19, 19, epuck2_set_all_actuators);

/// \method get_proximity()
/// Get proximity sensor value (the higher the value, the closer the object). 
/// Return list: index 0 represents proximity front right, then goes clockwise.
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

/// \method button_pressed()
/// Get button state.
/// Return true if pressed.
mp_obj_t epuck2_button_pressed(void) {
    return mp_obj_new_bool(button_is_pressed());
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_button_pressed_obj, epuck2_button_pressed);

/// \method get_proximity()
/// Get microphone volume, range is [0..4095].
/// Return list: 0=mic right, 1=mic left, 2=mic back, 3=mic front
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

/// \method epuck2_get_distance()
/// Get distance from ToF sensor in millimeters.
/// Return distance between 0 and 2000
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

/// \method sd_available()
/// Get sd state.
/// Return true if micro sd is available.
mp_obj_t epuck2_sd_available(void) {
    uint8_t temp;
    uart_get_sd_state(&temp);
    return mp_obj_new_bool(temp);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_sd_available_obj, epuck2_sd_available);

/// \method epuck2_get_acc_raw()
/// Get acc raw values, range is [-1500..1500], resolution is +-2g
/// Return list: 0=X, 1=Y, 3=Z
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

/// \method epuck2_get_battery()
/// Get battery raw value.
/// Return battery value
mp_obj_t epuck2_get_battery(void) {
    uint8_t temp[2];
    uart_get_battery(temp);
    return mp_obj_new_int(temp[0] + (((uint16_t)temp[1])<<8));
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_battery_obj, epuck2_get_battery);

/// \method epuck2_get_gyro_raw()
/// Get gyro raw values, range is [-32768..32767], resolution is +-250dps
/// Return list: 0=X, 1=Y, 3=Z
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

/// \method epuck2_get_tv_remote()
/// Get the TV remote data (RC5 protocol).
/// Return data field
mp_obj_t epuck2_get_tv_remote(void) {
    uint8_t temp;
    uart_get_tv_remote(&temp);
    return mp_obj_new_int(temp);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_tv_remote_obj, epuck2_get_tv_remote);

/// \method epuck2_get_selector()
/// Get the selector position, between 0 and 15
/// Return selector position
mp_obj_t epuck2_get_selector(void) {
    uint8_t temp;
    uart_get_selector(&temp);
    return mp_obj_new_int(temp);
}
static MP_DEFINE_CONST_FUN_OBJ_0(epuck2_get_selector_obj, epuck2_get_selector);

/// \method epuck2_get_all_sensors()
/// Get all sensors values at once. 
/// Return list:
/// 0: accelerometer raw x axis, between -1500 and 1500, resolution is +-2g
/// 1: accelerometer raw y axis, between -1500 and 1500, resolution is +-2g
/// 2: accelerometer raw z axis, between -1500 and 1500, resolution is +-2g
/// 3: acceleration expressed in float: acceleration magnitude sqrt(x^2 + y^2 + z^2), between 0.0 and about 2600.0 (~3.46 g)
/// 4: orientation expressed in float: between 0.0 and 360.0 degrees
/// 5: inclination expressed in float: between 0.0 and 90.0 degrees (when tilted in any direction)
/// 6: gyroscope raw x axis, between -32768 and 32767, resolution is +-250dps
/// 7: gyroscope raw y axis, between -32768 and 32767, resolution is +-250dps
/// 8: gyroscope raw z axis, between -32768 and 32767, resolution is +-250dps
/// 9: magnetometer raw x axis expressed in float, range is +-4912.0 uT (magnetic flux density expressed in micro Tesla)
/// 10: magnetometer raw y axis expressed in float, range is +-4912.0 uT (magnetic flux density expressed in micro Tesla)
/// 11: magnetometer raw z axis expressed in float, range is +-4912.0 uT (magnetic flux density expressed in micro Tesla)
/// 12: temperature given in Celsius degrees
/// 13: IR proximity 0, between 0 (no objects detected) and 4095 (object near the sensor)
/// 14: IR proximity 1, between 0 (no objects detected) and 4095 (object near the sensor)
/// 15: IR proximity 2, between 0 (no objects detected) and 4095 (object near the sensor)
/// 16: IR proximity 3, between 0 (no objects detected) and 4095 (object near the sensor)
/// 17: IR proximity 4, between 0 (no objects detected) and 4095 (object near the sensor)
/// 18: IR proximity 5, between 0 (no objects detected) and 4095 (object near the sensor)
/// 19: IR proximity 6, between 0 (no objects detected) and 4095 (object near the sensor)
/// 20: IR proximity 7, between 0 (no objects detected) and 4095 (object near the sensor)
/// 21: IR proximity 0 ambient, between 0 (strong light) and 4095 (dark)
/// 22: IR proximity 1 ambient, between 0 (strong light) and 4095 (dark)
/// 23: IR proximity 2 ambient, between 0 (strong light) and 4095 (dark)
/// 24: IR proximity 3 ambient, between 0 (strong light) and 4095 (dark)
/// 25: IR proximity 4 ambient, between 0 (strong light) and 4095 (dark)
/// 26: IR proximity 5 ambient, between 0 (strong light) and 4095 (dark)
/// 27: IR proximity 6 ambient, between 0 (strong light) and 4095 (dark)
/// 28: IR proximity 7 ambient, between 0 (strong light) and 4095 (dark)
/// 29: Time of Flight distance: distance given in millimeters
/// 30: mic0 volume between 0 and 4095
/// 31: mic1 volume between 0 and 4095
/// 32: mic2 volume between 0 and 4095
/// 33: mic3 volume between 0 and 4095
/// 34: left motor steps: 1000 steps per wheel revolution
/// 35: right motor steps: 1000 steps per wheel revolution
/// 36: battery raw value
/// 37: uSD state: 1 if the micro sd is present and can be read/write, 0 otherwise
/// 38: TV remote toggle (RC5 protocol)
/// 39: TV remote address (RC5 protocol)
/// 40: TV remote data (RC5 protocol)
/// 41: selector position, between 0 and 15
/// 42: ground proximity 0, between 0 (no surface at all or not reflective surface e.g. black) and 1023 (very reflective surface e.g. white)
/// 43: ground proximity 1, between 0 (no surface at all or not reflective surface e.g. black) and 1023 (very reflective surface e.g. white)
/// 44: ground proximity 2, between 0 (no surface at all or not reflective surface e.g. black) and 1023 (very reflective surface e.g. white)
/// 45: ground proximity 0 ambient, between 0 (strong light) and 1023 (dark)
/// 46: ground proximity 1 ambient, between 0 (strong light) and 1023 (dark)
/// 47: ground proximity 2 ambient, between 0 (strong light) and 1023 (dark)
/// 48: button state: 1 button pressed, 0 button released
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
    data->items[9] = mp_obj_new_float(values[24] + (values[25]<<8) + (values[26]<<16) + (values[27]<<24));
    data->items[10] = mp_obj_new_float(values[28] + (values[29]<<8) + (values[30]<<16) + (values[31]<<24));
    data->items[11] = mp_obj_new_float(values[32] + (values[33]<<8) + (values[34]<<16) + (values[35]<<24));
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

/// \method epuck2_play_freq()
/// Play a tone at specified frequency.
/// \param  value: frequency in Hz from 100 to 10000 Hz
/// Return `None`.
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

/// \method epuck2_get_mic_data()
/// Get the raw data for all 4 microphones (160 samples, 16 bits for each channel, 10 ms of data). This is a blocking function.
/// \param  buufer: interleaved data (right, left, back, front))
/// Return true in case of success, false otherwise.
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

/// \method camera_get_image(buffer, timeout_ms=1000)
/// Copy a camera image into "buffer": a bytearray of width*height*bytes_per_pixel bytes of the current format
/// (camera_get_format()), CAM_IMAGE_SIZE bytes are enough for every format.
/// - selector 15 (SPI): waits for an image newer than the one returned by the previous call (at most timeout_ms);
/// - other selector positions (UART): requests a new image to the F407 and waits for it (timeout_ms isn't used,
///   the request fails by itself after about 1 s without answer).
/// The image format is returned by camera_get_format(). Pixels are stored row by row; RGB565 uses 2 bytes per
/// pixel big endian (RRRRRGGG GGGBBBBB), greyscale 1 byte per pixel.
/// Return the image id (incremented at each image received), or None on timeout/error.
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

/// \method camera_get_format()
/// Return (width, height, bytes_per_pixel) of the images returned by camera_get_image(): bytes_per_pixel is 2 for
/// RGB565 and 1 for greyscale. In UART mode the format is known only after the first image (None before).
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

/// \method camera_set_format(width, height, bytes_per_pixel=2, zoom=4, x1=-1, y1=-1)
/// Set the format of the images returned by camera_get_image():
/// - width, height: image size in pixels;
/// - bytes_per_pixel: 2 for RGB565, 1 for greyscale;
/// - zoom: sensor pixels per image pixel, the image covers (width*zoom)x(height*zoom) pixels of the 640x480 sensor;
/// - x1, y1: top-left corner of that area on the sensor, -1 to center it.
/// Allowed values depend on the image source:
/// - selector 15 (SPI): the F407 always streams the whole sensor at zoom 4 (160x120 RGB565), the ESP32 crops and
///   converts it: zoom 4 (crop at full resolution), 8 (each pixel is the average of 2x2 stream pixels) or 16 (4x4);
///   x1, y1 are rounded down to multiples of 4. Default 160x120 RGB565 zoom 4. E.g. (80, 60, 1, 8): whole view,
///   greyscale, half resolution (4800 bytes); (80, 60, 2, 4): central 80x60 area at full resolution.
/// - other selector positions (UART): the format is set on the F407 (command "J"): zoom 1, 2 or 4, width and
///   height 1..255, at most 4136 bytes. Default 40x40 RGB565 zoom 4.
/// Return True if the format is set (UART: confirmed by the F407), False otherwise.
/// Raise ValueError("invalid camera format") if a parameter is out of range.
/// (A single error message is used on purpose: every new MP_ERROR_TEXT message makes all the error texts of the
/// firmware, ulab included, more expensive to compile; 7 messages made the firmware about 17 KB bigger.)
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

/// \method camera_get_stats()
/// Return (frames, errors) since boot or since the last camera_reset_stats():
/// - SPI: images received by the ESP32 (streamed continuously) and images discarded (unexpected packet size);
/// - UART: images received and failed requests.
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

/// \method camera_reset_stats()
/// Reset the counters returned by camera_get_stats().
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

/// \method image_get_color(img, fmt, roi=None)
/// Mean color of the area "roi" of an RGB565 image.
/// Return (hue, saturation, value): hue 0..359 degrees (0 red, 60 yellow, 120 green, 180 cyan, 240 blue,
/// 300 magenta), saturation and value (brightness) 0..100.
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

/// \method image_find_blobs_color(img, fmt, hue_min, hue_max, sat_min=30, sat_max=100, val_min=10, val_max=100,
///                                 roi=None, min_pixels=10, max_blobs=8)
/// Find the blobs of an RGB565 image whose pixels have hue between hue_min and hue_max (0..359; if
/// hue_min > hue_max the range wraps around 0, e.g. 340, 20 for red), saturation between sat_min and sat_max and
/// value (brightness) between val_min and val_max (0..100).
/// Return the blobs with at least min_pixels pixels (at most max_blobs, 1..64), biggest first:
/// [(pixels, cx, cy, x_min, y_min, x_max, y_max), ...], [] if none.
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

/// \method image_find_blobs_grey(img, fmt, grey_min, grey_max, roi=None, min_pixels=10, max_blobs=8)
/// Find the blobs whose pixels have grey level between grey_min and grey_max (0..255; for RGB565 images the
/// luminance is used). Return the blobs like image_find_blobs_color().
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

/// \method image_diff(img, prev, fmt, threshold=20, roi=None, min_pixels=10, max_blobs=8)
/// Find the areas that changed between two images of the same format: pixels whose grey level (luminance for
/// RGB565) differs by at least threshold (1..255). Return the blobs like image_find_blobs_color().
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
