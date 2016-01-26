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

#ifndef TAO_COPTER_FRAME
    #define TAO_COPTER_FRAME    QUAD_COPTER
#endif

#define FIVE_COPTER     0
#define BUNNY_COPTER    1
#define QUAD_COPTER     2

// PID controllers for roll, pitch and yaw.
static AC_PID pid_roll(5.0f, 1.0f, 0.1f, 50.0f, RATE_ROLL_FILT_HZ, MAIN_LOOP_SECONDS);
static AC_PID pid_pitch(5.0f, 1.0f, 0.1f, 50.0f, RATE_PITCH_FILT_HZ, MAIN_LOOP_SECONDS);
static AC_PID pid_yaw(5.0f, 1.0f, 0.0f, 50.0f, RATE_YAW_FILT_HZ, MAIN_LOOP_SECONDS);

void AP_MotorsTao::setup_five_motors()
{
    // setup_motors - configures the motors for five rotors.
    // The order of all the 5 rotors:
    //  4 CW------ 2 CCW
    //         |---------- 5 CCW
    //  3 CCW ---- 1 CW
    //
    //     ------>x
    //     |
    //     |
    //     \/
    //     y

    // call parent
    AP_MotorsMatrix::setup_motors();
    // Optimized parameters.
    // -0.621183    0.807174    -2.87231    0.204073
    // 1.07771      0.110314    1.26664     0.21262
    // -1.64458     -1.25492    2.05629     0.21848
    // 1.43191      -0.486728   -1.32397    0.233993
    // -0.334775    0.987783    0.86673     0.172848
    add_motor_raw(AP_MOTORS_MOT_1,  -0.621183f,    0.807174f,   -2.87231f,  1);
    add_motor_raw(AP_MOTORS_MOT_2,    1.07771f,    0.110314f,    1.26664f,  2);
    add_motor_raw(AP_MOTORS_MOT_3,   -1.64458f,    -1.25492f,    2.05629f,  3);
    add_motor_raw(AP_MOTORS_MOT_4,    1.43191f,   -0.486728f,   -1.32397f,  4);
    add_motor_raw(AP_MOTORS_MOT_5,  -0.334775f,    0.987783f,    0.86673f,  5);

    // Set up the throttle factors.
    for(int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        _throttle_factor[i] = 0.0f;
    }

    const float normalized_factor = 0.233993f;
    _throttle_factor[AP_MOTORS_MOT_1] = 0.204073f / normalized_factor;
    _throttle_factor[AP_MOTORS_MOT_2] = 0.21262f / normalized_factor;
    _throttle_factor[AP_MOTORS_MOT_3] = 0.21838f / normalized_factor;
    _throttle_factor[AP_MOTORS_MOT_4] = 0.233993f / normalized_factor;
    _throttle_factor[AP_MOTORS_MOT_5] = 0.172848f / normalized_factor;
}

void AP_MotorsTao::setup_bunny_motors() {
    // TODO
}

void AP_MotorsTao::setup_quad_motors() {
    // call parent
    AP_MotorsMatrix::setup_motors();

    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,   1.0f,   1.0f,   1);
    add_motor_raw(AP_MOTORS_MOT_2,  1.0f,  -1.0f,   1.0f,   2);
    add_motor_raw(AP_MOTORS_MOT_3,  1.0f,   1.0f,  -1.0f,   3);
    add_motor_raw(AP_MOTORS_MOT_4, -1.0f,  -1.0f,  -1.0f,   4);

    // Set up the throttle factors.
    for(int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        _throttle_factor[i] = 0.0f;
    }

    _throttle_factor[AP_MOTORS_MOT_1] = 1.0f;
    _throttle_factor[AP_MOTORS_MOT_2] = 1.0f;
    _throttle_factor[AP_MOTORS_MOT_3] = 1.0f;
    _throttle_factor[AP_MOTORS_MOT_4] = 1.0f;
}

void AP_MotorsTao::setup_motors() {
#if TAO_COPTER_FRAME == FIVE_COPTER
    setup_five_motors();
#elif TAO_COPTER_FRAME == BUNNY_COPER
    setup_bunny_motors();
#elif TAO_COPTER_FRAME == QUAD_COPTER
    setup_quad_motors();
#else
    // TODO.
#endif
}

// Do not use this function.
void AP_MotorsTao::output_armed_not_stabilizing()
{
    // send output to each motor.
    hal.rcout->cork();
    for(int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if(motor_enabled[i]) {
            rc_write(i, _throttle_radio_min);
        }
    }
    hal.rcout->push();
}

float remap(const float in_value, const float in_low, const float in_high,
        const float out_low, const float out_high) {
    if (in_low == in_high) return (out_low + out_high) / 2.0;
    return (in_value - in_low) / (in_high - in_low) * (out_high - out_low) + out_low;
}

float clamp(const float in_value, const float in_low, const float in_high) {
    return (in_value < in_low ? in_low : (in_value > in_high ? in_high : in_value));
}

void AP_MotorsTao::output_armed_stabilizing()
{
    // Implement our own output_armed_stabilizing function.
    const float throttle_in = (float)_copter.get_channel_throttle_control_in();

    pid_roll.set_input_filter_d(_desired_roll - _actual_roll);
    const float roll_torque = pid_roll.get_pid();
    pid_pitch.set_input_filter_d(_desired_pitch - _actual_pitch);
    const float pitch_torque = pid_pitch.get_pid();
    pid_yaw.set_input_filter_d(_desired_yaw - _actual_yaw);
    const float yaw_torque = pid_yaw.get_pid();

    // Compute the contribution of each motor.
    uint16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];
    for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        motor_out[i] = _throttle_radio_min;
        if (motor_enabled[i]) {
            // Convert it to pwm.
            const float pwm = (float)_throttle_radio_min
                    + _roll_factor[i] * roll_torque
                    + _pitch_factor[i] * pitch_torque
                    + _yaw_factor[i] * yaw_torque
                    + _throttle_factor[i] * throttle_in;
            // Clamp.
            motor_out[i] = (uint16_t)clamp(pwm, (float)_throttle_radio_min,
                    (float)_throttle_radio_max);
        }
    }

    // send output to each motor
    hal.rcout->cork();
    for(int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if(motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}


void AP_MotorsTao::test_input_from_rc() {
    // Get input from RC transmitter.
    // Min/max input values from the transmitter.
    const float roll_min = 1040.0f;
    const float roll_max = 1869.0f;
    const float pitch_min = 1042.0f;
    const float pitch_max = 1870.0f;
    const float thrust_min = 1042.0f;
    const float thrust_max = 1870.0f;
    const float yaw_min = 1043.0f;
    const float yaw_max = 1870.0f;

    const float roll_in = remap((float)_copter.get_channel_roll_control_in(), -4500.0f,
            4500.0f, roll_min, roll_max);
    const float pitch_in = remap((float)_copter.get_channel_pitch_control_in(), -4500.0f,
            4500.0f, pitch_min, pitch_max);
    const float throttle_in = remap((float)_copter.get_channel_throttle_control_in(), 0.0f,
            1000.0f, thrust_min, thrust_max);
    const float yaw_in = remap((float)_copter.get_channel_yaw_control_in(), -4500.0f,
            4500.0f, yaw_min, yaw_max);

    uint16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];
    motor_out[AP_MOTORS_MOT_1] = (uint16_t)roll_in;
    motor_out[AP_MOTORS_MOT_2] = (uint16_t)pitch_in;
    motor_out[AP_MOTORS_MOT_3] = (uint16_t)throttle_in;
    motor_out[AP_MOTORS_MOT_4] = (uint16_t)yaw_in;

    hal.rcout->cork();
    for(int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if(motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}

void AP_MotorsTao::test_vicon_data() {
    // Test vicon data can be received successfully.
    // Remember that all the angles are in degree, and are roughly
    // between -15 to 15 degrees.

    float motor_out[AP_MOTORS_MAX_NUM_MOTORS];
    motor_out[AP_MOTORS_MOT_1] = _desired_roll;
    motor_out[AP_MOTORS_MOT_2] = _desired_pitch;
    motor_out[AP_MOTORS_MOT_3] = _desired_yaw;
    motor_out[AP_MOTORS_MOT_4] = _actual_roll;
    motor_out[AP_MOTORS_MOT_5] = _actual_pitch;
    motor_out[AP_MOTORS_MOT_6] = _actual_yaw;

    const float angle_scale = 20.0f;
    hal.rcout->cork();
    for(int i = 0; i < 6; ++i) {
        rc_write(i, (uint16_t)(1500.0f + motor_out[i] * angle_scale));
    }
    hal.rcout->push();
}
