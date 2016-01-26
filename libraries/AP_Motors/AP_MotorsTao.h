// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsTao.h
/// @brief  Motor control class for frames used by Tao Du

#ifndef __AP_MOTORS_TAO_H__
#define __AP_MOTORS_TAO_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"    // Parent Motors Matrix library

// Forward declaration.
class Copter;

/// @class      AP_MotorsTao
class AP_MotorsTao : public AP_MotorsMatrix {
public:
    /// Constructor
    AP_MotorsTao(uint16_t loop_rate, uint16_t speed_hz, const Copter& cop)
        : AP_MotorsMatrix(loop_rate, speed_hz), _copter(cop) {
        _desired_roll = _desired_pitch = _desired_yaw = 0.0f;
        _actual_roll = _actual_pitch = _actual_yaw = 0.0f;
    }

    // setup_motors - configures the motors for rotors.
    virtual void        setup_motors();

    void set_desired_roll(const float desired_roll) {
        _desired_roll = desired_roll;
    }

    void set_desired_pitch(const float desired_pitch) {
        _desired_pitch = desired_pitch;
    }

    void set_desired_yaw(const float desired_yaw) {
        _desired_yaw = desired_yaw;
    }

    void set_actual_roll(const float actual_roll) {
        _actual_roll = actual_roll;
    }

    void set_actual_pitch(const float actual_pitch) {
        _actual_pitch = actual_pitch;
    }

    void set_actual_yaw(const float actual_yaw) {
        _actual_yaw = actual_yaw;
    }

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();
    void                output_armed_not_stabilizing();

private:
    // Setup motors.
    void                setup_five_motors();
    void                setup_bunny_motors();
    void                setup_quad_motors();

    // Test functions.
    void                test_input_from_rc();
    void                test_vicon_data();

    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS];

    // Points to the Copter class so that we can get all kinds of sensor's data.
    const Copter&       _copter;

    // Desired roll, pitch, yaw angles from VICON, in degrees.
    float               _desired_roll;
    float               _desired_pitch;
    float               _desired_yaw;

    // Actual roll, pitch, yaw angles from VICON, in degrees.
    float               _actual_roll;
    float               _actual_pitch;
    float               _actual_yaw;
};

#endif  // AP_MOTORSTAO
