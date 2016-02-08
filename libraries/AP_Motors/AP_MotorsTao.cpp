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
    #define TAO_COPTER_FRAME    FIVE_COPTER
#endif

#define FIVE_COPTER     0
#define BUNNY_COPTER    1
#define QUAD_COPTER     2

// Definitions of helper functions.
float remap(const float in_value, const float in_low, const float in_high,
        const float out_low, const float out_high) {
    if (in_low == in_high) return (out_low + out_high) / 2.0f;
    return (in_value - in_low) / (in_high - in_low) * (out_high - out_low) + out_low;
}

float clamp(const float in_value, const float in_low, const float in_high) {
    return (in_value < in_low ? in_low : (in_value > in_high ? in_high : in_value));
}

float wrap180(const float x) {
    return x < -180.0f ? (x + 360.0f) : (x > 180.0f ? (x - 360.0f) : x);
}

void AP_MotorsTao::setup_five_motors()
{
    // setup_motors - configures the motors for five rotors.
    // The order of all the 5 rotors:
    //  2 CW------ 1 CCW
    //         |---------- 5 CCW
    //  4 CCW ---- 3 CW
    //
    //     ------>x
    //     |
    //     |
    //     \/
    //     y
    // 1: (14cm, -20cm)
    // 2: (-27cm, -20cm)
    // 3: (14cm, 20cm)
    // 4: (-27cm, 20cm)
    // 5: (50cm, 0cm)

    // call parent
    AP_MotorsMatrix::setup_motors();

    const float yaw_factor = 0.25f;
    add_motor_raw(AP_MOTORS_MOT_1,   1.25f,    0.04f,   0.1625f / yaw_factor,  1);
    add_motor_raw(AP_MOTORS_MOT_2,   1.25f,   -0.54f,  -0.1993f / yaw_factor,  2);
    add_motor_raw(AP_MOTORS_MOT_3,  -1.25f,    0.54f,  -0.3007f / yaw_factor,  3);
    add_motor_raw(AP_MOTORS_MOT_4,  -1.25f,   -1.03f,   0.2639f / yaw_factor,  4);
    add_motor_raw(AP_MOTORS_MOT_5,    0.0f,    0.99f,   0.0736f / yaw_factor,  5);

    // Set up the throttle factors.
    for(int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        _throttle_factor[i] = 0.0f;
    }

    const float throttle_factor = 0.25f;
    _throttle_factor[AP_MOTORS_MOT_1] = 0.1654f / throttle_factor;
    _throttle_factor[AP_MOTORS_MOT_2] = 0.2657f / throttle_factor;
    _throttle_factor[AP_MOTORS_MOT_3] = 0.2343f / throttle_factor;
    _throttle_factor[AP_MOTORS_MOT_4] = 0.1968f / throttle_factor;
    _throttle_factor[AP_MOTORS_MOT_5] = 0.1378f / throttle_factor;
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
    for(int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if(motor_enabled[i]) {
            hal.rcout->write(i, _throttle_radio_min);
        }
    }
}

void AP_MotorsTao::output_armed_stabilizing()
{
    ///////////////////////////////////////////////////////////////////////////
    // Get actual angles in degrees.
    ///////////////////////////////////////////////////////////////////////////
    const float actual_roll = _copter.get_roll();
    const float actual_pitch = _copter.get_pitch();
    // Note that we choose to use vicon's yaw instead of yaw angles from the
    // compass.
    const float actual_yaw = _copter.get_vicon_actual_yaw();
    const float actual_roll_rate = _copter.get_roll_rate();
    const float actual_pitch_rate = _copter.get_pitch_rate();
    const float actual_yaw_rate = _copter.get_yaw_rate();
    ///////////////////////////////////////////////////////////////////////////
    // Get actual position and speed in centimeter or centimeter/second.
    ///////////////////////////////////////////////////////////////////////////
    const float actual_x = _copter.get_vicon_x() * 100.0f;
    const float actual_y = _copter.get_vicon_y() * 100.0f;
    const float actual_z = _copter.get_vicon_z() * 100.0f;
    const float actual_z_rate = _copter.get_z_rate() * 100.0f;
    ///////////////////////////////////////////////////////////////////////////
    // Determinate the state of the copter.
    ///////////////////////////////////////////////////////////////////////////
    const float takeoff_altitude = 30.0f;
    const bool is_flying = actual_z > takeoff_altitude;
    ///////////////////////////////////////////////////////////////////////////
    // Get desired roll, pitch and yaw angles, in degrees.
    ///////////////////////////////////////////////////////////////////////////
    const float roll_pitch_min = -45.0f, roll_pitch_max = 45.0f;
    const float yaw_min = -90.0f, yaw_max = 90.0f;
    float desired_roll = 0.0f, desired_pitch = 0.0f;
    if (is_flying) {
        // Set the bound in centimeters.
        const float xy_bound = 200.0f;
        // Get desired x and y position, in the inertial frame.
        const float desired_x = remap((float)_copter.get_channel_roll_control_in(),
                -4500.0f, 4500.0f, -xy_bound, xy_bound);
        const float desired_y = -remap((float)_copter.get_channel_pitch_control_in(),
                -4500.0f, 4500.0f, -xy_bound, xy_bound);
        // Convert (desired_x, desired_y) into (desired_x_body, desired_y_body).
        const float actual_yaw_radian = ToRad(actual_yaw);
        const float x_offset = desired_x - actual_x;
        const float y_offset = desired_y - actual_y;
        const float desired_x_body = x_offset * sinf(actual_yaw_radian)
                + y_offset * cosf(actual_yaw_radian);
        const float desired_y_body = x_offset * cosf(actual_yaw_radian)
                + y_offset * -sinf(actual_yaw_radian);
        // Now we've got the desired position in the body frame, convert it into
        // roll and pitch angles.
        _copter.get_g().vicon_pos_to_roll.set_input_filter_d(desired_y_body);
        desired_roll = clamp(_copter.get_g().vicon_pos_to_roll.get_pid(), roll_pitch_min, roll_pitch_max);
        _copter.get_g().vicon_pos_to_pitch.set_input_filter_d(-desired_x_body);
        desired_pitch = clamp(_copter.get_g().vicon_pos_to_pitch.get_pid(), roll_pitch_min, roll_pitch_max);
    } else {
        // We are going to take off.
        desired_roll = remap((float)_copter.get_channel_roll_control_in(),
                -4500.0f, 4500.0f, roll_pitch_min, roll_pitch_max);
        desired_pitch = remap((float)_copter.get_channel_pitch_control_in(),
                -4500.0f, 4500.0f, roll_pitch_min, roll_pitch_max);
    }
    const float desired_yaw = remap((float)_copter.get_channel_yaw_control_in(),
            -4500.0f, 4500.0f, yaw_min, yaw_max);
    ///////////////////////////////////////////////////////////////////////////
    // Get desired altitude in centimeters.
    ///////////////////////////////////////////////////////////////////////////
    const float z_min = 25.0f, z_max = 75.0f;
    const float desired_z = remap((float)_copter.get_channel_throttle_control_in(),
            0.0f, 1000.0f, z_min, z_max);
    ///////////////////////////////////////////////////////////////////////////
    // Call stabilize pid.
    ///////////////////////////////////////////////////////////////////////////
    const float roll_pitch_rate_bound = 250.0f, yaw_rate_bound = 360.0f;
    const float desired_roll_rate = clamp(_copter.get_g().vicon_p_roll.get_p(desired_roll - actual_roll),
            -roll_pitch_rate_bound, roll_pitch_rate_bound);
    const float desired_pitch_rate = clamp(_copter.get_g().vicon_p_pitch.get_p(desired_pitch - actual_pitch),
            -roll_pitch_rate_bound, roll_pitch_rate_bound);
    const float desired_yaw_rate = clamp(_copter.get_g().vicon_p_yaw.get_p(desired_yaw - actual_yaw),
            -yaw_rate_bound, yaw_rate_bound);
    const float z_rate_bound = 250.0f;
    const float desired_z_rate = clamp(_copter.get_g().vicon_p_z.get_p(desired_z - actual_z),
            -z_rate_bound, z_rate_bound);
    ///////////////////////////////////////////////////////////////////////////
    // Call rate pid.
    ///////////////////////////////////////////////////////////////////////////
    const float pwm_bound = 250.0f;
    _copter.get_g().vicon_roll_rate.set_input_filter_d(desired_roll_rate - actual_roll_rate);
    const float roll_pwm = clamp(_copter.get_g().vicon_roll_rate.get_pid(), -pwm_bound, pwm_bound);
    _copter.get_g().vicon_pitch_rate.set_input_filter_d(desired_pitch_rate - actual_pitch_rate);
    const float pitch_pwm = clamp(_copter.get_g().vicon_pitch_rate.get_pid(), -pwm_bound, pwm_bound);
    _copter.get_g().vicon_yaw_rate.set_input_filter_d(desired_yaw_rate - actual_yaw_rate);
    const float yaw_pwm = clamp(_copter.get_g().vicon_yaw_rate.get_pid(), -pwm_bound, pwm_bound);
    _copter.get_g().vicon_z_rate.set_input_filter_d(desired_z_rate - actual_z_rate);
    const float z_pwm = clamp(_copter.get_g().vicon_z_rate.get_pid(), -pwm_bound, pwm_bound);
    ///////////////////////////////////////////////////////////////////////////
    // Send output motor values.
    ///////////////////////////////////////////////////////////////////////////
    float motor_out[AP_MOTORS_MAX_NUM_MOTORS];
    // If we are above the takeoff altitude, the throttle will be the middle
    // throttle plus/minus the adjustment made by z controller, otherwise the throttle
    // is simply the input from the third channel.
    const float throttle_input = remap((float)_copter.get_channel_throttle_control_in(),
            0.0f, 1000.0f, 0.0, (float)(_throttle_radio_max - _throttle_radio_min));
    const float throttle_radio_mid = (float)(_throttle_radio_min + _throttle_radio_max) / 2.0f;
    for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (!motor_enabled[i]) continue;
        const float throttle =(is_flying
                ? (remap(_throttle_factor[i], 0.0f, 1.0f,
                         (float)_throttle_radio_min, throttle_radio_mid)
                   + _throttle_factor[i] * z_pwm)
                : _throttle_radio_min + _throttle_factor[i] * throttle_input);
        motor_out[i] = throttle
                + _roll_factor[i] * roll_pwm
                + _pitch_factor[i] * pitch_pwm
                + _yaw_factor[i] * yaw_pwm;
        // Clamp the output.
        motor_out[i] = clamp(motor_out[i], (float)_throttle_radio_min,
                (float)_throttle_radio_max);
    }
    // Send signals out.
    for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if(motor_enabled[i]) {
            hal.rcout->write(i, (uint16_t)motor_out[i]);
        }
    }
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

    for(int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if(motor_enabled[i]) {
            hal.rcout->write(i, motor_out[i]);
        }
    }
}

void AP_MotorsTao::test_vicon_data() {
    // Test vicon data can be received successfully.
    // Remember that all the angles are in degree, and are roughly
    // between -15 to 15 degrees.

    const float desired_roll = _copter.get_vicon_desired_roll();
    const float desired_pitch = _copter.get_vicon_desired_pitch();
    const float desired_yaw = _copter.get_vicon_desired_yaw();
    const float actual_roll = _copter.get_vicon_actual_roll();
    const float actual_pitch = _copter.get_vicon_actual_pitch();
    const float actual_yaw = _copter.get_vicon_actual_yaw();

    float motor_out[AP_MOTORS_MAX_NUM_MOTORS];
    motor_out[AP_MOTORS_MOT_1] = desired_roll;
    motor_out[AP_MOTORS_MOT_2] = desired_pitch;
    motor_out[AP_MOTORS_MOT_3] = desired_yaw;
    motor_out[AP_MOTORS_MOT_4] = actual_roll;
    motor_out[AP_MOTORS_MOT_5] = actual_pitch;
    motor_out[AP_MOTORS_MOT_6] = actual_yaw;

    const float angle_scale = 20.0f;
    for(int i = 0; i < 6; ++i) {
        hal.rcout->write(i, (uint16_t)(1500.0f + motor_out[i] * angle_scale));
    }
}
