#include <AP_ADC/AP_ADC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <PID/PID.h>

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// INS and Baro declaration
AP_InertialSensor ins;

Compass compass;

AP_GPS gps;
AP_Baro baro;
AP_SerialManager serial_manager;

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(ins, baro, gps);

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   1042
#define RC_THR_MAX   1871
#define RC_YAW_MIN   1043
#define RC_YAW_MAX   1871
#define RC_PIT_MIN   1043
#define RC_PIT_MAX   1870
#define RC_ROL_MIN   1040
#define RC_ROL_MAX   1869

// Motor numbers definitions
#define MOTOR_FL   2    // Front left
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

// Declaration.
long map(long x, long in_min, long in_max, long out_min, long out_max);
float constrain(float val, float min, float max);
void setup();
void loop();

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrain(float val, float min, float max) {
  return (val < min) ? min : ((val > max) ? max : val);
}

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

void setup()
{
  // Enable the motors and set at 490Hz update
  hal.rcout->set_freq(0xF, 490);
  for (int i = 0; i < 4; ++i) {
    hal.rcout->enable_ch(i);
  }

  // PID Configuration
  pids[PID_PITCH_RATE].kP(0.7);
  pids[PID_PITCH_RATE].kI(1);
  pids[PID_PITCH_RATE].imax(50);

  pids[PID_ROLL_RATE].kP(0.7);
  pids[PID_ROLL_RATE].kI(1);
  pids[PID_ROLL_RATE].imax(50);

  pids[PID_YAW_RATE].kP(2.7);
  pids[PID_YAW_RATE].kI(1);
  pids[PID_YAW_RATE].imax(50);

  pids[PID_PITCH_STAB].kP(4.5);
  pids[PID_ROLL_STAB].kP(4.5);
  pids[PID_YAW_STAB].kP(10);

  ins.init(100);
  ahrs.init();
  serial_manager.init();

  if( compass.init() ) {
      hal.console->printf("Enabling compass\n");
      ahrs.set_compass(&compass);
  } else {
      hal.console->printf("No compass detected\n");
  }
}

void loop()
{
  static uint32_t last_t, last_print, last_compass;
  uint32_t now = AP_HAL::micros();

  if (last_t == 0) {
    last_t = now;
    return;
  }
  last_t = now;

  if (now - last_compass > 100*1000UL &&
    compass.read()) {
    compass.calculate_heading(ahrs.get_rotation_body_to_ned());
    // read compass at 10Hz
    last_compass = now;
  }
  ahrs.update();

  // Ask for orientation.
  const float roll = ToDeg(ahrs.roll);
  const float pitch = ToDeg(ahrs.pitch);
  const float yaw = ToDeg(ahrs.yaw);

  // Ask for gyro data.
  Vector3f gyro = ins.get_gyro();
  float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);

  /////////////////////////////////////////////////////////////////////////////
  // The controller code.
  /////////////////////////////////////////////////////////////////////////////

  static float yaw_target = 0;
  uint16_t channels[8];

  // Read RC transmitter and map to sensible values
  hal.rcin->read(channels, 8);

  long rcthr, rcyaw, rcpit, rcroll;  // Variables to store radio in

  rcthr = channels[2];
  rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
  rcpit = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
  rcroll = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -45, 45);

  // Do the magic
  if(rcthr > RC_THR_MIN + 100) {  // Throttle raised, turn on stablisation.
    // Stablise PIDS
    float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250);
    float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5) {
      yaw_stab_output = rcyaw;
      yaw_target = yaw;   // remember this yaw for when pilot stops
    }

    // rate PIDS
    long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);
    long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);
    long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);

    // mix pid outputs and send to the motors.
    hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
    hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
    hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
    hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
  } else {
    // motors off
    hal.rcout->write(MOTOR_FL, 1000);
    hal.rcout->write(MOTOR_BL, 1000);
    hal.rcout->write(MOTOR_FR, 1000);
    hal.rcout->write(MOTOR_BR, 1000);

    // reset yaw target so we maintain this on takeoff
    yaw_target = yaw;

    // reset PID integrals whilst on the ground
    for(int i=0; i<6; i++)
      pids[i].reset_I();
  }
}

AP_HAL_MAIN();
