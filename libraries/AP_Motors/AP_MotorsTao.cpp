// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsTao.cpp - ArduCopter motors library
 *       Code by Tao Du. MIT CSAIL
 *
 */

#include "../../ArduCopter/Copter.h"
#include <AC_PID/AC_PID.h>
#include "AP_MotorsTao.h"

extern const AP_HAL::HAL& hal;

float remap(const float in_value, const float in_low, const float in_high,
        const float out_low, const float out_high);
float clamp(const float in_value, const float in_low, const float in_high);
float wrap180(const float x);

// Definitions of helper functions.
float remap(const float in_value, const float in_low, const float in_high,
        const float out_low, const float out_high) {
    if (in_low == in_high) return (out_low + out_high) / 2.0;
    return (in_value - in_low) / (in_high - in_low) * (out_high - out_low) + out_low;
}

float clamp(const float in_value, const float in_low, const float in_high) {
    return (in_value < in_low ? in_low : (in_value > in_high ? in_high : in_value));
}

float wrap180(const float x) {
    return x < -180.0f ? (x + 360.0f) : (x > 180.0f ? (x - 360.0f) : x);
}

void AP_MotorsTao::setup_motors() {
    // call parent
    AP_MotorsMatrix::setup_motors();

    // Add at most 6 motors. The roll/pitch/yaw factor does not really matter
    // as we are going to send desired pwm via mavlink.
    add_motor_raw(AP_MOTORS_MOT_1, 0.0f, 0.0f, 0.0f, 1);
    add_motor_raw(AP_MOTORS_MOT_2, 0.0f, 0.0f, 0.0f, 2);
    add_motor_raw(AP_MOTORS_MOT_3, 0.0f, 0.0f, 0.0f, 3);
    add_motor_raw(AP_MOTORS_MOT_4, 0.0f, 0.0f, 0.0f, 4);
    add_motor_raw(AP_MOTORS_MOT_5, 0.0f, 0.0f, 0.0f, 5);
    add_motor_raw(AP_MOTORS_MOT_6, 0.0f, 0.0f, 0.0f, 6);
}

// Do not use this function.
void AP_MotorsTao::output_armed_not_stabilizing() {
    // send output to each motor.
    for(int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if(motor_enabled[i]) {
            hal.rcout->write(i, _throttle_radio_min);
        }
    }
}

void AP_MotorsTao::output_armed_stabilizing() {
    // A simple safety switch.
    // throttle \in (0.0f, 1000.0f).
    const float throttle = (float)_copter.get_channel_throttle_control_in();
    if (throttle < 500.0f) {
        for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
            if (motor_enabled[i]) {
                hal.rcout->write(i, _throttle_radio_min);
            }
        }
    } else {
        for(int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
            if(motor_enabled[i]) {
                const float motor_output = clamp(_copter.get_mavlink_motor_output(i),
                        (float)_throttle_radio_min, (float)_throttle_radio_max);
                hal.rcout->write(i, (uint16_t)motor_output);
            }
        }
    }
}
