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

char api_version[6] = "XX.XX\0";
char temp_buff[512];

// Get current e-puck2 API version.
static mp_obj_t get_api_version(void) {
    snprintf(api_version, 6, "%2d.%-2d", EPUCK2_API_MAJOR_VERSION, EPUCK2_API_MINOR_VERSION);
    return mp_obj_new_str(api_version, strlen(api_version));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_api_version_obj, get_api_version);

/// \method set_rgb([led, value red, value green, value blue])
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

/// \method set_all_rgb([red2, green2, blue2, red4, green4, blue4, red6, green6, blue6, red8, green8, blue8])
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

/// \method get_proximity()
/// Get proximity sensor value (the higher the value, the closer the object).
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

/// \method set_speed()
/// Set motors speed. Range is between -1000 and 1000.
mp_obj_t set_motors_speed(mp_obj_t left, mp_obj_t right) {
    int l = mp_obj_get_int(left);
    int r = mp_obj_get_int(right);
    uart_set_motors_speed(l, r);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(set_motors_speed_obj, set_motors_speed);

static const mp_rom_map_elem_t epuck2_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_epuck2) },

    { MP_ROM_QSTR(MP_QSTR_get_api_version), MP_ROM_PTR(&get_api_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_rgb), MP_ROM_PTR(&epuck2_set_rgb_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_all_rgb), MP_ROM_PTR(&epuck2_set_all_rgb_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_proximity), MP_ROM_PTR(&epuck2_get_proximity_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_motors_speed), MP_ROM_PTR(&set_motors_speed_obj) },
};

static MP_DEFINE_CONST_DICT(epuck2_module_globals, epuck2_module_globals_table);

const mp_obj_module_t epuck2_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&epuck2_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_epuck2, epuck2_module);
