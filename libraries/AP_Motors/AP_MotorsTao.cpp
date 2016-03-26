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

// Tao Du
// taodu@csail.mit.edu
// Mar 22, 2016

static AC_PID vicon_vx(0.0f, 0.0f, 1.0f, 10.0f, 20.0f, MAIN_LOOP_SECONDS);
static AC_PID vicon_vy(0.0f, 0.0f, 1.0f, 10.0f, 20.0f, MAIN_LOOP_SECONDS);

// Tao Du
// taodu@csail.mit.edu
// Mar 22, 2016
// Currently support three types of copters.
#define QUAD_ROTOR  1
#define FIVE_ROTOR  2

#define COPTER_NAME        QUAD_ROTOR
//#define COPTER_NAME     FIVE_ROTOR

#if COPTER_NAME == QUAD_ROTOR
 #define MAX_ROTOR_IN_COPTER 4
const float K[MAX_ROTOR_IN_COPTER][12] = {
    {-0.8660f,  -0.8660f,   -0.8660f,   -4.8564f,   4.8564f,    0.8660f,    -1.2682f,   -1.2682f,   -1.0877f,   -0.9262f,   0.9262f,    1.1349f},
    {0.8660f,   0.8660f,    -0.8660f,   4.8564f,    -4.8564f,   0.8660f,    1.2682f,    1.2682f,    -1.0877f,   0.9262f,    -0.9262f,   1.1349f},
    {-0.8660f,  0.8660f,    -0.8660f,   4.8564f,    4.8564f,    -0.8660f,   -1.2682f,   1.2682f,    -1.0877f,   0.9262f,    0.9262f,    -1.1349f},
    {0.8660f,   -0.8660f,   -0.8660f,   -4.8564f,   -4.8564f,   -0.8660f,   1.2682f,    -1.2682f,   -1.0877f,   -0.9262f,   -0.9262f,   -1.1349f}
};
const float u0[MAX_ROTOR_IN_COPTER] = {
    2.4500f,
    2.4500f,
    2.4500f,
    2.4500f
};
const float xybound = 2.0f;
const float lower_z = 5.0f;
const float upper_z = -2.0f;
#elif COPTER_NAME == FIVE_ROTOR
 #define MAX_ROTOR_IN_COPTER 5
// Used by LQR controller.
const float K[MAX_ROTOR_IN_COPTER][12] = {
    {-0.2456f,    0.5031f,   -0.3852f,    3.3259f,    1.6579f,    0.3499f,   -0.3794f,    0.7705f,   -0.6189f,    0.7689f,    0.3579f,    0.9001f},
    { 0.3697f,    0.5070f,   -0.5571f,    3.3138f,   -2.5524f,   -0.3967f,    0.5736f,    0.7753f,   -0.9328f,    0.7042f,   -0.5969f,   -1.0588f},
    {-0.2455f,   -0.4940f,   -0.4237f,   -3.2780f,    1.6728f,   -0.6145f,   -0.3795f,   -0.7567f,   -0.7208f,   -0.7820f,    0.3925f,   -1.6248f},
    { 0.3698f,   -0.4959f,   -0.5396f,   -3.2367f,   -2.5753f,    0.5624f,    0.5744f,   -0.7583f,   -0.8657f,   -0.6772f,   -0.6351f,    1.4488f},
    {-0.7784f,    0.0023f,   -0.2658f,    0.0246f,    5.3185f,    0.1622f,   -1.2049f,    0.0038f,   -0.4289f,    0.0185f,    1.2154f,    0.4131f},
};
const float u0[MAX_ROTOR_IN_COPTER] = {
    2.9240f,
    4.9958f,
    4.0495f,
    4.0497f,
    2.0716f,
};
const float xybound = 2.0f;
const float lower_z = 10.0f;
const float upper_z = -1.0f;
#endif

extern const AP_HAL::HAL& hal;

float remap(const float in_value, const float in_low, const float in_high,
        const float out_low, const float out_high);
float clamp(const float in_value, const float in_low, const float in_high);
float wrap180(const float x);
float thrust2pwm(const float value);

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

float thrust2pwm(const float value) {
    if (value <= 0.0f) return 1000.0f;
#if COPTER_NAME == QUAD_ROTOR
    const float a = 8.081f;
    const float b = -15.53f;
    const float c = 7.263f - value;
#elif COPTER_NAME == FIVE_ROTOR
    const float a = 15.15f;
    const float b = -28.6f;
    const float c = 13.14f - value;
#endif
    const float delta = b * b - 4.0f * a * c;
    if (delta < 0.0f) return 1000.0f;
    const float pwm2 = (-b + sqrtf(delta)) / 2.0f / a;
    return clamp(pwm2 * 1000.0f, 1000.0f, 1800.0f);
}

void AP_MotorsTao::setup_motors() {
    // call parent
    AP_MotorsMatrix::setup_motors();

    // Add at most 6 motors. The roll/pitch/yaw factor does not really matter
    // as we are going to send desired pwm via mavlink.
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        add_motor_raw(AP_MOTORS_MOT_1 + i, 0.0f, 0.0f, 0.0f, i + 1);
    }
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
    // Get current states:
    const Vector3f& position = _copter.get_vicon_position();
    const float x = position.x;
    const float y = position.y;
    const float z = position.z;
    const float roll = _copter.get_roll();
    const float pitch = _copter.get_pitch();
    const float yaw = _copter.get_yaw();
    const Vector3f velocity = _copter.get_ned_velocity();
    vicon_vx.set_input_filter_d(x);
    vicon_vy.set_input_filter_d(y);
    const float vx = vicon_vx.get_d();
    const float vy = vicon_vy.get_d();
    const float vz = velocity.z;
    const float rollspeed = _copter.get_roll_rate();
    const float pitchspeed = _copter.get_pitch_rate();
    const float yawspeed = _copter.get_yaw_rate();
    const float X[12] = {x, y, z, roll, pitch, yaw, vx, vy, vz, rollspeed, pitchspeed, yawspeed};

    // Get desired states.
    const float y0 = remap((float)_copter.get_channel_roll_control_in(),
            -4500.0f, 4500.0f, -xybound, xybound);
    const float x0 = remap((float)_copter.get_channel_pitch_control_in(),
            -4500.0f, 4500.0f, xybound, -xybound);
    const float thr_ctrl = (float)_copter.get_channel_throttle_control_in();
    float z0 = 0.0f;
    if (thr_ctrl < 500.0f) {
        z0 = remap(thr_ctrl, 0.0f, 500.0f, lower_z, 0.0f);
    } else {
        z0 = remap(thr_ctrl, 500.0f, 1000.0f, 0.0f, upper_z);
    }
    const float X0[12] = {x0, y0, z0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Compute the desired thrust.
    // u = -K(X - X0) + u0.
    float X_minus_X0[12];
    for (int i = 0; i < 12; ++i) {
        X_minus_X0[i] = X[i] - X0[i];
    }
    float K_times_X_minus_X0[MAX_ROTOR_IN_COPTER];
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        K_times_X_minus_X0[i] = 0.0f;
        for (int j = 0; j < 12; ++j) {
            K_times_X_minus_X0[i] += K[i][j] * X_minus_X0[j];
        }
    }
    float u[MAX_ROTOR_IN_COPTER];
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        u[i] = -K_times_X_minus_X0[i] + u0[i];
    }

    // Convert u to pwm.
    float pwm[MAX_ROTOR_IN_COPTER];
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        pwm[i] = thrust2pwm(u[i]);
    }

    // Finally write pwm to the motor.
    for(int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        const float motor_output = clamp(pwm[i],
                (float)_throttle_radio_min, (float)_throttle_radio_max);
        hal.rcout->write(i, (uint16_t)motor_output);
    }
}
