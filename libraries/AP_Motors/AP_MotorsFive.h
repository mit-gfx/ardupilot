// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsFive.h
/// @brief  Motor control class for Five rotor frames

#ifndef __AP_MOTORS_FIVE_H__
#define __AP_MOTORS_FIVE_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"    // Parent Motors Matrix library

// Forward declaration.
class Copter;

/// @class      AP_MotorsFive
class AP_MotorsFive : public AP_MotorsMatrix {
public:
    /// Constructor
    AP_MotorsFive(uint16_t loop_rate, uint16_t speed_hz, const Copter& cop)
        : AP_MotorsMatrix(loop_rate, speed_hz), _copter(cop) {
        _desired_roll = _desired_pitch = _desired_yaw_rate = 0.0f;
    }

    // setup_motors - configures the motors for bunny rotors.
    virtual void        setup_motors();

    void set_desired_roll(const float desired_roll) {
        _desired_roll = desired_roll;
    }

    void set_desired_pitch(const float desired_pitch) {
        _desired_pitch = desired_pitch;
    }

    void set_desired_yaw_rate(const float desired_yaw_rate) {
        _desired_yaw_rate = desired_yaw_rate;
    }

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();
    void                output_armed_not_stabilizing();

private:
    // Test functions.
    void                test_input_output();


    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS];

    // Points to the Copter class so that we can get all kinds of sensor's data.
    const Copter&       _copter;

    // Desired roll, pitch, yaw angles from VICON, in degrees.
    float               _desired_roll;		// +/-30 degree.
    float               _desired_pitch;		// +/-30 degree.
    float               _desired_yaw_rate;	// +/-30 degree.
};

#endif  // AP_MOTORSFIVE
