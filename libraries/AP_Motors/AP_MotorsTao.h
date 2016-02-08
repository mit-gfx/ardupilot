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
    AP_MotorsTao(uint16_t loop_rate, uint16_t speed_hz, Copter& cop)
        : AP_MotorsMatrix(loop_rate, speed_hz), _copter(cop) { }

    // setup_motors - configures the motors for rotors.
    virtual void        setup_motors();

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
    Copter&       _copter;
};

#endif  // AP_MOTORSTAO
