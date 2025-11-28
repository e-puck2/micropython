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
    uart_get_data_ptr();
    
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
};

static MP_DEFINE_CONST_DICT(epuck2_module_globals, epuck2_module_globals_table);

const mp_obj_module_t epuck2_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&epuck2_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_epuck2, epuck2_module);
