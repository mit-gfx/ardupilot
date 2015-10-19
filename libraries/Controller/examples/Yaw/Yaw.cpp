// Tao Du
// taodu@csail.mit.edu
// Octo 18, 2015
//
// Implement a sample controller.

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_Common/AP_Common.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Notify/AP_Notify.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <SITL/SITL.h>
#include <Filter/Filter.h>
#include <AP_Param/AP_Param.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <PID/PID.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Inertial Sensor.
AP_InertialSensor ins;

// Yaw angles.
float yaw = 0.0;
// Timer.
int count = 0;
// PID controller.
PID pid;

// Constants.
// Number of propellers.
const int prop_num = 4;
// Minimal pwm value.
const uint16_t pwm_min = 1000;
// For safety reasons, we don't allow pwm to go beyond 1700.
const uint16_t pwm_max = 1700;
// Minimal and maximal yaw.
const uint16_t yaw_min = 1045;
const uint16_t yaw_max = 1865;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup(void) {
    // Initialize the inertial sensor.
    ins.init(AP_InertialSensor::RATE_100HZ);

    // Enable outputs to all the four rotors.
    for (int i = 0; i < prop_num; ++i) {
        hal.rcout->enable_ch(i);
        hal.rcout->write(i, pwm_min);
    }

    // PID configuration.
    pid.kP(0.7);
    pid.kI(1);
    pid.imax(50);
}

void loop(void) {
    // Update ins data.
    ins.update();

    Vector3f delta_angle;
    ins.get_delta_angle(delta_angle);
    // Update yaw.
    yaw += delta_angle.z;

    // Now get throttle and yaw input.
    // Channel 2: throttle.
    // Channel 3: yaw.
    const int ch_throttle = 2;
    const int ch_yaw = 3;
    const float desired_throttle = (float)hal.rcin->read(ch_throttle);
    const float desired_yaw = map((float)hal.rcin->read(ch_yaw),
            (float)yaw_min,
            (float)yaw_max,
            -180.0f, 180.0f);

    // Throw them into our PID controller.
    const float yaw_output = constrain_float(
        pid.get_pid((float)desired_yaw - (float)ToDeg(yaw), 1.0),
        -200.0f, 200.0f
    );

    // Send out signals to all the four rotors.
    // 2 (Red)          0 (Red)
    //           ^
    //           |
    //           |
    // 1 (White)        3 (White)
    uint16_t upper_right = (uint16_t)constrain_float(desired_throttle + yaw_output, pwm_min, pwm_max);
    uint16_t lower_left = (uint16_t)constrain_float(desired_throttle + yaw_output, pwm_min, pwm_max);
    uint16_t upper_left = (uint16_t)constrain_float(desired_throttle - yaw_output, pwm_min, pwm_max);
    uint16_t lower_right = (uint16_t)constrain_float(desired_throttle - yaw_output, pwm_min, pwm_max);

    hal.rcout->write(0, upper_right);
    hal.rcout->write(1, lower_left);
    hal.rcout->write(2, upper_left);
    hal.rcout->write(3, lower_right);

    // Display the states every two seconds.
    ++count;
    if (count % 1000 == 0) {
        hal.console->printf("yaw = %f, desired yaw = %f, desired throttle = %f. "
                "motors = [%u %u %u %u]\n",
                ToDeg(yaw), desired_yaw, desired_throttle,
                upper_right, lower_left, upper_left, lower_right);
    }
    hal.scheduler->delay(10);
}

AP_HAL_MAIN();
