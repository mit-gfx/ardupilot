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

static AC_PID vicon_vx(0.0f, 0.0f, 1.0f, 10.0f, 20.0f, MAIN_LOOP_SECONDS);
static AC_PID vicon_vy(0.0f, 0.0f, 1.0f, 10.0f, 20.0f, MAIN_LOOP_SECONDS);
static AC_PID vicon_vz(0.0f, 0.0f, 1.0f, 10.0f, 20.0f, MAIN_LOOP_SECONDS);

#define QUAD_ROTOR  1
#define FIVE_ROTOR  2
#define BUNNY_ROTOR 3

//#define COPTER_NAME       QUAD_ROTOR
//#define COPTER_NAME       FIVE_ROTOR
#define COPTER_NAME       BUNNY_ROTOR

// The meaning of each column
#define X_COL       0
#define Y_COL       1
#define Z_COL       2
#define ROL_COL     3
#define PIT_COL     4
#define YAW_COL     5
#define X_VEL_COL   6
#define Y_VEL_COL   7
#define Z_VEL_COL   8
#define ROL_VEL_COL 9
#define PIT_VEL_COL 10
#define YAW_VEL_COL 11
#define NUM_COL     12

#if COPTER_NAME == QUAD_ROTOR
 #define MAX_ROTOR_IN_COPTER 4
const float K[MAX_ROTOR_IN_COPTER][NUM_COL] = {
    {-0.8660f,   -0.8660f,   -0.8660f,   -4.8564f,    4.8564f,    0.8660f,   -1.2682f,   -1.2682f,   -1.0877f,   -0.9262f,    0.9262f,     1.1349f},
    { 0.8660f,    0.8660f,   -0.8660f,    4.8564f,   -4.8564f,    0.8660f,    1.2682f,    1.2682f,   -1.0877f,    0.9262f,   -0.9262f,     1.1349f},
    {-0.8660f,    0.8660f,   -0.8660f,    4.8564f,    4.8564f,   -0.8660f,   -1.2682f,    1.2682f,   -1.0877f,    0.9262f,    0.9262f,    -1.1349f},
    { 0.8660f,   -0.8660f,   -0.8660f,   -4.8564f,   -4.8564f,   -0.8660f,    1.2682f,   -1.2682f,   -1.0877f,   -0.9262f,   -0.9262f,    -1.1349f},
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
// Q = [5 5 5 10 10 10 10 10 10 10 10 10]. Payload = 1554g.
const float K[MAX_ROTOR_IN_COPTER][NUM_COL] = {
    {-0.5445f,  1.1180f,    -0.8554f,   8.7558f,    4.2709f,    1.1334f,    -1.0339f,   2.1208f,    -1.5705f,   1.8072f,    0.8593f,    1.8666f},
    {0.8240f,   1.1180f,    -1.2775f,   8.7558f,    -6.5564f,   -1.2181f,   1.5686f,    2.1208f,    -2.4080f,   1.8072f,    -1.3750f,   -2.0968f},
    {-0.5470f,  -1.1180f,   -0.9682f,   -8.7558f,   4.3544f,    -1.9059f,   -1.0412f,   -2.1208f,   -1.8438f,   -1.8072f,   0.9262f,    -3.2411f},
    {0.8266f,   -1.1180f,   -1.1647f,   -8.7558f,   -6.6399f,   1.8212f,    1.5759f,    -2.1208f,   -2.1348f,   -1.8072f,   -1.4418f,   3.0109f},
    {-1.7442f,  -0.0000f,   -0.5848f,   -0.0000f,   13.8179f,   0.5315f,    -3.3174f,   -0.0000f,   -1.0768f,   0.0000f,    2.8728f,    0.8653f},
};
float u0[MAX_ROTOR_IN_COPTER] = {
    5.7380f,
    9.7297f,
    7.7339f,
    7.7339f,
    3.9917f,
};
const float xybound = 4.0f;
const float lower_z = 10.0f;
const float upper_z = -5.0f;
#elif COPTER_NAME == BUNNY_ROTOR
 #define MAX_ROTOR_IN_COPTER 4
// Used by LQR controller.
// Q = [2 2 2 4 4 4 4 4 4 4 4 4]. R = [1 2 1 1]. Mass = 1800g.
const float K[MAX_ROTOR_IN_COPTER][NUM_COL] = {
    {0.1037f,   -1.0937f,   -0.6884f,   -8.6630f,   -0.7881f,   0.7989f,    0.1955f,    -2.0804f,   -1.2981f,   -1.7918f,   -0.1562f,   1.3643f},
    {0.0259f,   0.6277f,    -0.5147f,   5.0229f,    -0.2307f,   0.8251f,    0.0506f,    1.1960f,    -0.9802f,   1.0759f,    -0.0530f,   1.3974f},
    {-1.0052f,  0.0370f,    -0.7478f,   0.2431f,    8.0925f,    -0.9263f,   -1.9185f,   0.0679f,    -1.3432f,   0.0321f,    1.7199f,    -1.4669f},
    {0.9887f,   0.1195f,    -0.6611f,   0.9734f,    -7.9556f,   -1.0687f,   1.8869f,    0.2290f,    -1.1776f,   0.1983f,    -1.6862f,   -1.7073f},
};
float u0[MAX_ROTOR_IN_COPTER] = {
    5.2691f,
    4.1127f,
    4.4793f,
    3.7790f,
};
const float xybound = 5.0f;
const float lower_z = 20.0f;
const float upper_z = -10.0f;

#else
    // Do nothing.
#endif

// For voltage estimation.
static int last_frame = 0;
static int current_frame = 0;
static float average_voltage = 0.0f;
static float voltage_sum = 0.0f;

extern const AP_HAL::HAL& hal;

float remap(const float in_value, const float in_low, const float in_high,
        const float out_low, const float out_high);
float clamp(const float in_value, const float in_low, const float in_high);
float wrap180(const float x);
float thrust2pwm_kde_14inch(const float thrust, const float voltage);
float thrust2pwm_kde_10inch(const float thrust, const float voltage);

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

float thrust2pwm_kde_14inch(const float thrust, const float voltage) {
    // Reject unreasonable data.
    if (thrust <= 0.0f || voltage <= 7.5f || voltage >= 13.0f) return 1000.0f;
#if COPTER_NAME == QUAD_ROTOR
    const float mean_throttle = 1475.0f;
    const float std_throttle = 234.2f;
    const float mean_voltage = 11.64f;
    const float std_voltage = 0.1475f;
    const float p00 = 1.901f;
    const float p10 = 2.047f;
    const float p01 = 0.1023f;
    const float p20 = 0.535f;
    const float p11 = 0.07185f;
#elif COPTER_NAME == FIVE_ROTOR || COPTER_NAME == BUNNY_ROTOR
    // Output from matlab:
    /*
    Linear model Poly21:
    a(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y
      where x is normalized by mean 1425 and std 201.8
      and where y is normalized by mean 11.38 and std 0.3678
    Coefficients (with 95% confidence bounds):
      p00 =         3.4  (3.375, 3.424)
      p10 =       3.324  (3.306, 3.341)
      p01 =      0.2254  (0.2078, 0.2431)
      p20 =      0.7846  (0.7648, 0.8043)
      p11 =      0.1704  (0.1526, 0.1883)
    */
    const float mean_throttle = 1425.0f;
    const float std_throttle = 201.8140f;
    const float mean_voltage = 11.3775f;
    const float std_voltage = 0.3678f;
    const float p00 = 3.4f;
    const float p10 = 3.324f;
    const float p01 = 0.2254f;
    const float p20 = 0.7846f;
    const float p11 = 0.1704f;
#endif
    const float y = (voltage - mean_voltage) / std_voltage;
    const float a = p20;
    const float b = p11 * y + p10;
    const float c = p01 * y + p00 - thrust;
    const float delta = b * b - 4 * a * c;
    if (delta < 0.0f) return 1000.0f;
    const float x = (-b + sqrtf(delta)) / 2.0f / a;
    const float throttle = x * std_throttle + mean_throttle;
    return clamp(throttle, 1000.0f, 2000.0f);
}

float thrust2pwm_kde_10inch(const float thrust, const float voltage) {
    // Output from matlab:
    /*
     Linear model Poly21:
     a(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y
       where x is normalized by mean 1550 and std 274.1
       and where y is normalized by mean 11.48 and std 0.3972
     Coefficients (with 95% confidence bounds):
       p00 =       2.827  (2.805, 2.848)
       p10 =       2.037  (2.023, 2.052)
       p01 =      0.1959  (0.1812, 0.2105)
       p20 =      0.1797  (0.1633, 0.196)
       p11 =      0.1362  (0.1215, 0.151)
    */
    // Reject unreasonable data.
    if (thrust <= 0.0f || voltage <= 7.5f || voltage >= 13.0f) return 1000.0f;
    const float mean_throttle = 1550.0f;
    const float std_throttle = 274.1f;
    const float mean_voltage = 11.48f;
    const float std_voltage = 0.3972f;
    const float p00 = 2.827f;
    const float p10 = 2.037f;
    const float p01 = 0.1959f;
    const float p20 = 0.1797f;
    const float p11 = 0.1362f;
    const float y = (voltage - mean_voltage) / std_voltage;
    const float a = p20;
    const float b = p11 * y + p10;
    const float c = p01 * y + p00 - thrust;
    const float delta = b * b - 4 * a * c;
    if (delta < 0.0f) return 1000.0f;
    const float x = (-b + sqrtf(delta)) / 2.0f / a;
    const float throttle = x * std_throttle + mean_throttle;
    return clamp(throttle, 1000.0f, 2000.0f);
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
    vicon_vx.set_input_filter_d(x);
    vicon_vy.set_input_filter_d(y);
    vicon_vz.set_input_filter_d(z);
    const float vx = vicon_vx.get_d();
    const float vy = vicon_vy.get_d();
    const float vz = vicon_vz.get_d();
    const float rollspeed = _copter.get_roll_rate();
    const float pitchspeed = _copter.get_pitch_rate();
    const float yawspeed = _copter.get_yaw_rate();
    const float X[NUM_COL] = {x, y, z, roll, pitch, yaw, vx, vy, vz, rollspeed, pitchspeed, yawspeed};

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
    const float X0[NUM_COL] = {x0, y0, z0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Compute the desired thrust.
    // u = -K(X - X0) + u0.
    float X_minus_X0[NUM_COL];
    for (int i = 0; i < NUM_COL; ++i) {
        X_minus_X0[i] = X[i] - X0[i];
    }
    // Take care of the yaw difference so that the abs value of the difference is smaller
    // than pi.
    float yaw_diff = X_minus_X0[5];
    if (yaw_diff > PI) {
        yaw_diff -= 2 * PI;
    } else if (yaw_diff < -PI) {
        yaw_diff += 2 * PI;
    }
    X_minus_X0[5] = yaw_diff;

    float K_times_X_minus_X0[MAX_ROTOR_IN_COPTER];
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        K_times_X_minus_X0[i] = 0.0f;
        for (int j = 0; j < NUM_COL; ++j) {
            K_times_X_minus_X0[i] += K[i][j] * X_minus_X0[j];
        }
    }
    float u[MAX_ROTOR_IN_COPTER];
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        u[i] = -K_times_X_minus_X0[i] + u0[i];
    }

    // Increment current frame.
    ++current_frame;
    // Compute the mean voltage since last update. We take the mean voltage during the last
    // 2 seconds. Note that this function gets called at 400Hz.
    if (current_frame - last_frame > 400 * 2) {
        average_voltage = voltage_sum / (current_frame - last_frame);
        voltage_sum = 0.0f;
        last_frame = current_frame;
    } else {
        voltage_sum += _copter.get_battery_voltage();
    }

    // Convert u to pwm.
    float pwm[MAX_ROTOR_IN_COPTER];
#if COPTER_NAME != BUNNY_ROTOR
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        pwm[i] = thrust2pwm_kde_14inch(u[i], average_voltage);
    }
#else
    // For bunny we have different propellers for different motors.
    pwm[0] = thrust2pwm_kde_14inch(u[0], average_voltage);
    pwm[1] = thrust2pwm_kde_10inch(u[1], average_voltage);
    pwm[2] = thrust2pwm_kde_14inch(u[2], average_voltage);
    pwm[3] = thrust2pwm_kde_14inch(u[3], average_voltage);
#endif

    // Finally write pwm to the motor.
    for(int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        const float motor_output = clamp(pwm[i],
                (float)1000.0f, (float)2000.0f);
        hal.rcout->write(i, (uint16_t)motor_output);
    }
}
