#include <Arduino.h>
#include <esp_task_wdt.h>

#include "Wire.h"
#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "imu_go.h"
#include "motorgo_mini.h"
#include "pid_manager.h"

TaskHandle_t loop_foc_task;
TickType_t xLastWakeTime;
void loop_foc(void* pvParameters);

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& left_motor = motorgo_mini.ch1;
MotorGo::MotorChannel& right_motor = motorgo_mini.ch0;

// Declare motor parameters
MotorGo::MotorParameters motor_left_params;
MotorGo::MotorParameters motor_right_params;

// declare PID manager object
MotorGo::PIDManager pid_manager;

// declare two pre-built PID controller objects for individual motor position
// control
MotorGo::PIDParameters position_controller_params;
LowPassFilter position_lpf_left(position_controller_params.lpf_time_constant);
PIDController position_controller_left(position_controller_params.p,
                                       position_controller_params.i,
                                       position_controller_params.d,
                                       position_controller_params.output_ramp,
                                       position_controller_params.limit);

LowPassFilter position_lpf_right(position_controller_params.lpf_time_constant);
PIDController position_controller_right(position_controller_params.p,
                                        position_controller_params.i,
                                        position_controller_params.d,
                                        position_controller_params.output_ramp,
                                        position_controller_params.limit);

// declare two pre-built PID controller objects for individual motor velocity
// control
MotorGo::PIDParameters velocity_controller_params;
LowPassFilter velocity_lpf_left(velocity_controller_params.lpf_time_constant);
PIDController velocity_controller_left(velocity_controller_params.p,
                                       velocity_controller_params.i,
                                       velocity_controller_params.d,
                                       velocity_controller_params.output_ramp,
                                       velocity_controller_params.limit);
LowPassFilter velocity_lpf_right(velocity_controller_params.lpf_time_constant);
PIDController velocity_controller_right(velocity_controller_params.p,
                                        velocity_controller_params.i,
                                        velocity_controller_params.d,
                                        velocity_controller_params.output_ramp,
                                        velocity_controller_params.limit);

// Declare and configure custom pitch controller object
MotorGo::PIDParameters pitch_controller_params;
LowPassFilter pitch_lpf(pitch_controller_params.lpf_time_constant);
PIDController pitch_controller(pitch_controller_params.p,
                               pitch_controller_params.i,
                               pitch_controller_params.d,
                               pitch_controller_params.output_ramp,
                               pitch_controller_params.limit);

// Declare and configure custom pitch rate controller object
MotorGo::PIDParameters pitch_rate_params;
LowPassFilter pitch_rate_lpf(pitch_rate_params.lpf_time_constant);
PIDController pitch_rate_controller(pitch_rate_params.p, pitch_rate_params.i,
                                    pitch_rate_params.d,
                                    pitch_rate_params.output_ramp,
                                    pitch_rate_params.limit);

// declare and configure custom steering controller object
MotorGo::PIDParameters steering_controller_params;
LowPassFilter steering_lpf(steering_controller_params.lpf_time_constant);
PIDController steering_controller(steering_controller_params.p,
                                  steering_controller_params.i,
                                  steering_controller_params.d,
                                  steering_controller_params.output_ramp,
                                  steering_controller_params.limit);

// declare and configure custom balance setpoint controller object
MotorGo::PIDParameters balance_point_params;

bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

SensorGo::Imu imu;

// Zero points for state variables
float pitch_zero = 0;
// The encoder measurement is a combination of the wheel position
// and the pitch of the robot. This variable is used to store the
// initial pitch of the robot when the robot is powered on.
float wheel_pos_imu_offset;

void enable_motors_callback(bool value)
{
  if (value)
  {
    Serial.println("Enabling motors");
    left_motor.enable();
    right_motor.enable();
  }
  else
  {
    Serial.println("Disabling motors");
    left_motor.disable();
    right_motor.disable();
  }
}

// Function to print at a maximum frequency
void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

float compute_pitch()
{
  float grav_x, grav_y, grav_z;
  imu.get_gravity_vector(&grav_x, &grav_y, &grav_z);

  return atan2(grav_x, grav_z);
}

float compute_roll()
{
  float grav_x, grav_y, grav_z;
  imu.get_gravity_vector(&grav_x, &grav_y, &grav_z);

  return atan2(grav_y, grav_z);
}

void update_pid_params(MotorGo::PIDParameters& params,
                       PIDController& controller, LowPassFilter& lpf)
{
  // Update the PID controller parameters
  controller.P = params.p;
  controller.I = params.i;
  controller.D = params.d;
  controller.output_ramp = params.output_ramp;
  controller.limit = params.limit;
  lpf.Tf = params.lpf_time_constant;
  controller.reset();
}

void setup()
{
  Serial.begin(115200);
  delay(3000);

  imu.init(false);

  // Setup motor parameters
  motor_left_params.pole_pairs = 11;
  motor_left_params.power_supply_voltage = 17.0;
  motor_left_params.voltage_limit = 17.0;
  motor_left_params.current_limit = 300;
  motor_left_params.velocity_limit = 100.0;
  motor_left_params.calibration_voltage = 1.0;
  motor_left_params.reversed = false;

  motor_right_params.pole_pairs = 11;
  motor_right_params.power_supply_voltage = 17.0;
  motor_right_params.voltage_limit = 17.0;
  motor_right_params.current_limit = 300;
  motor_right_params.velocity_limit = 100.0;
  motor_right_params.calibration_voltage = 1.0;
  motor_right_params.reversed = true;

  // Setup PID parameters
  update_pid_params(pitch_controller_params, pitch_controller, pitch_lpf);
  update_pid_params(pitch_rate_params, pitch_rate_controller, pitch_rate_lpf);

  update_pid_params(position_controller_params, position_controller_left,
                    position_lpf_left);
  update_pid_params(position_controller_params, position_controller_right,
                    position_lpf_right);
  update_pid_params(velocity_controller_params, velocity_controller_left,
                    velocity_lpf_left);
  update_pid_params(velocity_controller_params, velocity_controller_right,
                    velocity_lpf_right);
  update_pid_params(steering_controller_params, steering_controller,
                    steering_lpf);

  // Setup Ch0
  bool calibrate = false;
  left_motor.init(motor_left_params, calibrate);
  right_motor.init(motor_right_params, calibrate);

  left_motor.set_control_mode(MotorGo::ControlMode::Voltage);
  right_motor.set_control_mode(MotorGo::ControlMode::Voltage);

  // Add controllers to the PID manager
  pid_manager.add_controller("/pitch", pitch_controller_params,
                             []() {
                               update_pid_params(pitch_controller_params,
                                                 pitch_controller, pitch_lpf);
                             });
  pid_manager.add_controller("/pitch_rate", pitch_rate_params,
                             []()
                             {
                               update_pid_params(pitch_rate_params,
                                                 pitch_rate_controller,
                                                 pitch_rate_lpf);
                             });
  pid_manager.add_controller(
      "/position", position_controller_params,
      []()
      {
        update_pid_params(position_controller_params, position_controller_left,
                          position_lpf_left);
        update_pid_params(position_controller_params, position_controller_right,
                          position_lpf_right);

        // Compute new initial angle
        left_motor.zero_position();
        right_motor.zero_position();

        imu.loop();
        wheel_pos_imu_offset = compute_roll();
      });
  pid_manager.add_controller(
      "/wheel_velocity", velocity_controller_params,
      []()
      {
        update_pid_params(velocity_controller_params, velocity_controller_left,
                          velocity_lpf_left);
        update_pid_params(velocity_controller_params, velocity_controller_right,
                          velocity_lpf_right);
      });

  // wrap controller params into a configurable object, pass anonymous
  //   function
  // to allow board to update controller values after receiving input over
  pid_manager.add_controller("/steering", steering_controller_params,
                             []()
                             {
                               // Compute new initial angle
                               left_motor.zero_position();
                               right_motor.zero_position();
                               imu.loop();
                               wheel_pos_imu_offset = compute_roll();

                               update_pid_params(steering_controller_params,
                                                 steering_controller,
                                                 steering_lpf);
                             });

  // make a balance point controller, where p is the balance point.
  pid_manager.add_controller("/balance point", balance_point_params,
                             []() { pitch_zero = balance_point_params.p; });
  enable_motors.set_post_callback(enable_motors_callback);

  // initialize the PID manager
  Serial.println(WIFI_SSID);
  Serial.println(WIFI_PASSWORD);
  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  //   Loop IMU until roll stablizes to 0
  Serial.println("Letting IMU filter stabilize...");
  float roll = 1;
  while (abs(roll) > 0.1)
  {
    float grav_x, grav_y, grav_z;

    imu.loop();
    roll = compute_roll();

    freq_println("Roll: " + String(roll, 10), 10);
  }

  //   Zero wheel postions
  left_motor.zero_position();
  right_motor.zero_position();
  imu.loop();
  wheel_pos_imu_offset = compute_roll();

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  // Normally, we'd enable the motors here. However, since the GUI can
  //   enable, leave them disabled so the robot doesn't run away
  //   left_motor.enable();
  //   right_motor.enable();
}

void loop_foc(void* pvParameters)
{
  Serial.print("Loop FOC running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    left_motor.loop();
    right_motor.loop();

    //   Print loop frequency
    esp_task_wdt_reset();
  }
}

void loop()
{
  imu.loop();

  // State variables:
  // Pitch
  // Pitch rate
  // Wheel position
  // Wheel velocity

  // Retrieve state variables
  //   Roll is the forward-backward tilt of the robot
  // TODO: Confirm signs
  float pitch = pitch_lpf(compute_roll() - pitch_zero);
  float pitch_rate = pitch_rate_lpf(imu.get_raw_gyro_x() / SENSORS_RADS_TO_DPS);

  //   Wheel position and velocity are encoder measurements + pitch
  float wheel_position_left =
      position_lpf_left(left_motor.get_position() + pitch);
  float wheel_position_right =
      position_lpf_right(right_motor.get_position() + pitch);

  float wheel_velocity_left =
      velocity_lpf_left(left_motor.get_velocity() + pitch_rate);
  float wheel_velocity_right =
      velocity_lpf_right(right_motor.get_velocity() + pitch_rate);

  // Controller form
  // u = -kx

  // Compute control outputs
  float command_pitch = -pitch_controller(pitch);
  float command_pitch_rate = -0pitch_rate_controller(pitch_rate);

  // Do not use position controller for now
  //   float command_wheel_position_left =
  //       position_controller_left(wheel_position_left);
  //   float command_wheel_position_right =
  //       position_controller_right(wheel_position_right);
  float command_wheel_position_left = 0;
  float command_wheel_position_right = 0;

  float command_wheel_velocity_left =
      velocity_controller_left(wheel_velocity_left);
  float command_wheel_velocity_right =
      velocity_controller_right(wheel_velocity_right);

  float command_steer =
      steering_controller(wheel_position_left - wheel_position_right);

  // Combine controller outputs to compute motor commands
  float command_left =
      -(command_pitch + command_pitch_rate + command_wheel_position_left +
        command_wheel_velocity_left + command_steer);

  float command_right =
      -(command_pitch + command_pitch_rate + command_wheel_position_right +
        command_wheel_velocity_right - command_steer);

  // Print all state variables
  //   freq_println(
  //       "Pitch: " + String(pitch, 10) +
  //           "\tPitch rate: " + String(pitch_rate, 10) +
  //           "\tWheel position left: " + String(wheel_position_left, 10) +
  //           "\tWheel position right: " + String(wheel_position_right, 10) +
  //           "\tWheel velocity left: " + String(wheel_velocity_left, 10) +
  //           "\tWheel velocity right: " + String(wheel_velocity_right, 10),
  //       10);

  //   Print controller outputs
  freq_println("Command pitch: " + String(command_pitch, 10) +
                   "\tCommand pitch rate: " + String(command_pitch_rate, 10) +
                   "\tCommand wheel position left: " +
                   String(command_wheel_position_left, 10) +
                   "\tCommand wheel position right: " +
                   String(command_wheel_position_right, 10) +
                   "\tCommand wheel velocity left: " +
                   String(command_wheel_velocity_left, 10) +
                   "\tCommand wheel velocity right: " +
                   String(command_wheel_velocity_right, 10) +
                   "\tCommand steer: " + String(command_steer, 10),
               10);

  // Set target voltage
  left_motor.set_target_voltage(command_left);
  right_motor.set_target_voltage(command_right);
}
