// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsBunny.h
/// @brief  Motor control class for Bunny rotor frames

#ifndef __AP_MOTORS_BUNNY_H__
#define __AP_MOTORS_BUNNY_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"    // Parent Motors Matrix library

/// @class      AP_MotorsBunny
class AP_MotorsBunny : public AP_MotorsMatrix {
public:
    /// Constructor
    AP_MotorsBunny(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    { };

    // init
    virtual void        Init();

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // setup_motors - configures the motors for five rotors.
    virtual void        setup_motors();

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();
    void                output_armed_not_stabilizing();
    void                output_disarmed();

    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS];

};

#endif  // AP_MOTORSBUNNY
