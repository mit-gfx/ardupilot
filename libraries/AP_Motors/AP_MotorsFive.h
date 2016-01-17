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
        : AP_MotorsMatrix(loop_rate, speed_hz), _copter(cop) {}

    // setup_motors - configures the motors for bunny rotors.
    virtual void        setup_motors();

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();
    void                output_armed_not_stabilizing();

private:
    // Test functions.
    void                test_input_output();


    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS];

    // Points to the Copter class so that we can get all kinds of sensor's data.
    const Copter& _copter;
};

#endif  // AP_MOTORSFIVE
