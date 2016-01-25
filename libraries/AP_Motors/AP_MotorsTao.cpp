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
#include <AC_PID/AC_P.h>
#include <AC_PID/AC_PID.h>
#include "AP_MotorsTao.h"

#define USE_UNOPTIMIZED
#define USE_VICON

extern const AP_HAL::HAL& hal;

// PID controllers for roll, pitch, yaw angles and climb rate.
static AC_PID pid_roll(0.5f, 0.4f, 0.3f, 50.0f, RATE_ROLL_FILT_HZ, MAIN_LOOP_SECONDS);
static AC_PID pid_pitch(1.0f, 0.8f, 0.5f, 50.0f, RATE_ROLL_FILT_HZ, MAIN_LOOP_SECONDS);
static AC_P p_throttle(20.0f);
static AC_P p_yaw(1.5f);

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
void AP_MotorsTao::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

#ifdef USE_UNOPTIMIZED
    // Unoptimized parameters.
    add_motor_raw(AP_MOTORS_MOT_1,  -1.1930f,   0.5418f,  -3.0075f,  1);
    add_motor_raw(AP_MOTORS_MOT_2,   1.1930f,   0.0500f,   1.6198f,  2);
    add_motor_raw(AP_MOTORS_MOT_3,  -1.1930f,  -1.0336f,   2.6348f,  3);
    add_motor_raw(AP_MOTORS_MOT_4,   1.1930f,  -0.5418f,  -1.9925f,  4);
    add_motor_raw(AP_MOTORS_MOT_5,      0.0f,   0.9836f,   0.7454f,  5);

    // Set up the throttle factors.
    for(int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        _throttle_factor[i] = 0.0f;
    }

    const float normalized_factor = 0.2681f;
    _throttle_factor[AP_MOTORS_MOT_1] = 0.2319f / normalized_factor;
    _throttle_factor[AP_MOTORS_MOT_2] = 0.1650f / normalized_factor;
    _throttle_factor[AP_MOTORS_MOT_3] = 0.2011f / normalized_factor;
    _throttle_factor[AP_MOTORS_MOT_4] = 0.2681f / normalized_factor;
    _throttle_factor[AP_MOTORS_MOT_5] = 0.1339f / normalized_factor;
#else
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
    // Input:
    // Desired input:
    // roll/pitch/yaw: +/-30 degree or degree/second(actually in radians)
    // velocity_z: -0.25m/s to 0.25m/s.
    // These values have been tested.
#ifdef USE_VICON
    const float roll_in = ToRad(_desired_roll);
    const float pitch_in = ToRad(_desired_pitch);
    const float yaw_rate_in = ToRad(_desired_yaw_rate);
#else
    const float roll_in = ToRad(remap((float)_copter.get_channel_roll_control_in(), -4500.0f,
            4500.0f, -30.0f, 30.0f));
    const float pitch_in = ToRad(remap((float)_copter.get_channel_pitch_control_in(), -4500.0f,
            4500.0f, -30.0f, 30.0f));
    const float yaw_rate_in = ToRad(remap((float)_copter.get_channel_yaw_control_in(), -4500.0f,
            4500.0f, -30.0f, 30.0f));
#endif
    const float throttle_in = (float)_copter.get_channel_throttle_control_in();
    const float velocity_z_in = remap(throttle_in, 0.0f, 1000.0f, -0.25f, 0.25f);

    // Actual input:
    // Euler angles in radians or radians/seconds. Use ToDeg to convert to degrees when necessary.
    const float roll_actual = _copter.get_ahrs_roll();
    const float pitch_actual = _copter.get_ahrs_pitch();
    const float yaw_rate_actual = _copter.get_ahrs_yaw_rate_earth();
    // altitude in meters.
    const float altitude_actual = _copter.get_altitude() / 100.0;
    // climb rate in m/s, positive up.
    const float velocity_z = _copter.get_velocity_z() / 100.0;

    // Output:
    // signals to each motor.
    pid_roll.set_input_filter_d(roll_in - roll_actual);
    const float roll_torque = pid_roll.get_pid();
    pid_pitch.set_input_filter_d(pitch_in - pitch_actual);
    const float pitch_torque = pid_pitch.get_pid();
    const float yaw_torque = p_yaw.get_p(yaw_rate_in - yaw_rate_actual);

    // Compute the contribution of each motor.
    uint16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];
    for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        motor_out[i] = _throttle_radio_min;
        if (motor_enabled[i]) {
            const float attitude_thrust = _roll_factor[i] * roll_torque
                    + _pitch_factor[i] * pitch_torque
                    + _yaw_factor[i] * yaw_torque;
            // Convert it to pwm.
            const float pwm = (float)_throttle_radio_min + _throttle_factor[i] * throttle_in
                    + remap(attitude_thrust, -2.0f, 2.0f, -300.0f, 300.0f);
            // Clamp.
            motor_out[i] = (uint16_t)clamp(pwm, (float)_throttle_radio_min, (float)_throttle_radio_max);
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


void AP_MotorsTao::test_input_output() {
    // Test whether we can linearly map the input signals. Ideally RCIN and RCOUT curves should
    // overlap perfectly.
#ifdef USE_VICON
    const float roll_in = ToRad(_desired_roll);
    const float pitch_in = ToRad(_desired_pitch);
    const float yaw_rate_in = ToRad(_desired_yaw_rate);
#else
    // Get input from RC transmitter.
    const float roll_in = ToRad(remap((float)_copter.get_channel_roll_control_in(), -4500.0f,
            4500.0f, -30.0f, 30.0f));
    const float pitch_in = ToRad(remap((float)_copter.get_channel_pitch_control_in(), -4500.0f,
            4500.0f, -30.0f, 30.0f));
    const float yaw_rate_in = ToRad(remap((float)_copter.get_channel_yaw_control_in(), -4500.0f,
            4500.0f, -30.0f, 30.0f));
#endif
    const float throttle_in = (float)_copter.get_channel_throttle_control_in();
    const float velocity_z_in = remap(throttle_in, 0.0f, 1000.0f, -0.25f, 0.25f);

    // Send output to each motor.
    uint16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];
    for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        motor_out[i] = _throttle_radio_min;
    }
    // Min/max input values from the transmitter.
    const float roll_min = 1103.0f;
    const float roll_max = 1924.0f;
    const float pitch_min = 1103.0f;
    const float pitch_max = 1924.0f;
    const float altitude_min = 1094.0f;
    const float altitude_max = 1924.0f;
    const float yaw_min = 1103.0f;
    const float yaw_max = 1924.0f;
    motor_out[0] = (uint16_t)remap(roll_in, ToRad(-30.0f), ToRad(30.0f), roll_min, roll_max);
    motor_out[1] = (uint16_t)remap(pitch_in, ToRad(-30.0f), ToRad(30.0f), pitch_min, pitch_max);
    motor_out[2] = (uint16_t)remap(throttle_in, 0.0f, 1000.0f, altitude_min, altitude_max);
    motor_out[3] = (uint16_t)remap(yaw_rate_in, ToRad(-30.0f), ToRad(30.0f), yaw_min, yaw_max);

    hal.rcout->cork();
    for(int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if(motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}
