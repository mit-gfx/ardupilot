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

#define COPTER_NAME       DJI_COPTER

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

#if COPTER_NAME == DJI_COPTER
 #define MAX_ROTOR_IN_COPTER 4
// Q = [1 1 1 2 2 10 2 2 2 2 2 2]. R = [5 5 5 5]. Mass = 1.187kg.
// X frame
const float K[MAX_ROTOR_IN_COPTER][NUM_COL] = {
    {-0.223583f,  0.223594f,  -0.223643f,  2.873927f,  -0.000069f,  0.707106f,  -0.438980f,  0.439001f,  -0.482479f,  0.686363f,  -0.000016f,  1.029338f,  },
    {-0.223583f,  -0.223619f,  -0.223618f,  -0.000220f,  2.874092f,  -0.707107f,  -0.438980f,  -0.439050f,  -0.482425f,  -0.000041f,  0.686407f,  -1.029340f,  },
    {0.223630f,  -0.223619f,  -0.223571f,  -2.874391f,  -0.000069f,  0.707107f,  0.439072f,  -0.439050f,  -0.482323f,  -0.686479f,  -0.000016f,  1.029342f,  },
    {0.223630f,  0.223594f,  -0.223596f,  -0.000220f,  -2.874229f,  -0.707107f,  0.439072f,  0.439001f,  -0.482377f,  -0.000041f,  -0.686436f,  -1.029340f,  },
};
const float u0[MAX_ROTOR_IN_COPTER] = {
    2.908621f,
    2.908293f,
    2.907679f,
    2.908007f,
};
const float xybound = 1.0f;
const float lower_z = 7.0f;
const float upper_z = -1.5f;
const float min_pwm = 1100.0f;
const float max_pwm = 1900.0f;
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

float thrust2pwm_dji(const float thrust, const float voltage) {
    // Reject unreasonable data.
    if (thrust <= 0.0f || voltage <= 7.5f || voltage >= 13.0f) return 1000.0f;
    // Output from matlab:
    /*
    thrust = sf(pwm, voltage)
     x: mean: 1525  std: 231.1462
     y: mean: 11.7894 std: 0.3300
     Linear model Poly21:
     sf(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y
     Coefficients (with 95% confidence bounds):
       p00 =       2.939  (2.912, 2.966)
       p10 =       2.422  (2.401, 2.442)
       p01 =      0.1464  (0.1259, 0.1669)
       p20 =      0.3989  (0.3762, 0.4215)
       p11 =      0.1119  (0.0913, 0.1325)*/
    const float mean_throttle = 1525.0f;
    const float std_throttle = 231.1462f;
    const float mean_voltage = 11.7894f;
    const float std_voltage = 0.3300f;
    const float p00 = 2.939f;
    const float p10 = 2.422f;
    const float p01 = 0.1464f;
    const float p20 = 0.3989f;
    const float p11 = 0.1119f;
    const float y = (voltage - mean_voltage) / std_voltage;
    const float a = p20;
    const float b = p11 * y + p10;
    const float c = p01 * y + p00 - thrust;
    const float delta = b * b - 4 * a * c;
    if (delta < 0.0f) return 1000.0f;
    const float x = (-b + sqrtf(delta)) / 2.0f / a;
    const float throttle = x * std_throttle + mean_throttle;
    return clamp(throttle, min_pwm, max_pwm);
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
#if COPTER_NAME == DJI_COPTER
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        pwm[i] = thrust2pwm_dji(u[i], average_voltage);
    }
#endif

    // Finally write pwm to the motor.
    for(int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        const float motor_output = clamp(pwm[i],
                min_pwm, max_pwm);
        hal.rcout->write(i, (uint16_t)motor_output);
    }
}
