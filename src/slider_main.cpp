#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>

#include "Wire.h"
#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "motorgo_mini.h"
#include "pid_manager.h"

TaskHandle_t loop_foc_task;
TickType_t xLastWakeTime;
int loop_frequency = 320;
float loop_period_us = 1000000.0 / loop_frequency;
void loop_foc(void* pvParameters);

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& left_motor = motorgo_mini.ch1;
MotorGo::MotorChannel& right_motor = motorgo_mini.ch0;

// Declare motor parameters
MotorGo::MotorParameters motor_left_params;
MotorGo::MotorParameters motor_right_params;

// declare PID manager object
MotorGo::PIDManager pid_manager;

// declare two pre-built PID controller objects for individual motor velocity
// control
// Use the Same parameters for left and right, but maintain two separate
// controllers
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

// declare PID controller object for position
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

bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

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

void setup()
{
  Serial.begin(115200);
  delay(3000);

  // Setup motor parameters
  motor_left_params.pole_pairs = 11;
  motor_left_params.power_supply_voltage = 10.0;
  motor_left_params.voltage_limit = 10.0;
  motor_left_params.current_limit = 300;
  motor_left_params.velocity_limit = 100.0;
  motor_left_params.calibration_voltage = 1.5;
  motor_left_params.reversed = false;
  motor_left_params.kv = 500;
  motor_left_params.phase_resistance = 0.1088;

  motor_right_params.pole_pairs = 11;
  motor_right_params.power_supply_voltage = 10.0;
  motor_right_params.voltage_limit = 10.0;
  motor_right_params.current_limit = 300;
  motor_right_params.velocity_limit = 100.0;
  motor_right_params.calibration_voltage = 1.5;
  motor_right_params.kv = 500;
  motor_right_params.phase_resistance = 0.1088;
  motor_right_params.reversed = true;

  velocity_controller_params.lpf_time_constant = 0.005;

  // Initialize motors
  bool calibrate = false;
  left_motor.init(motor_left_params, calibrate);
  right_motor.init(motor_right_params, calibrate);

  left_motor.set_control_mode(MotorGo::ControlMode::Voltage);
  right_motor.set_control_mode(MotorGo::ControlMode::Voltage);

  // wrap controller params into a configurable object, pass anonymous
  //   function
  // to allow board to update controller values after receiving input over
  pid_manager.add_controller(
      "/velocity", velocity_controller_params,
      []()
      {
        velocity_controller_left.P = velocity_controller_params.p;
        velocity_controller_left.I = velocity_controller_params.i;
        velocity_controller_left.D = velocity_controller_params.d;
        velocity_controller_left.output_ramp =
            velocity_controller_params.output_ramp;
        velocity_controller_left.limit = velocity_controller_params.limit;
        velocity_lpf_left.Tf = velocity_controller_params.lpf_time_constant;
        velocity_controller_left.reset();

        velocity_controller_right.P = velocity_controller_params.p;
        velocity_controller_right.I = velocity_controller_params.i;
        velocity_controller_right.D = velocity_controller_params.d;
        velocity_controller_right.output_ramp =
            velocity_controller_params.output_ramp;
        velocity_controller_right.limit = velocity_controller_params.limit;
        velocity_lpf_right.Tf = velocity_controller_params.lpf_time_constant;
        velocity_controller_right.reset();
      });

  pid_manager.add_controller(
      "/position", position_controller_params,
      []()
      {
        position_controller_left.P = position_controller_params.p;
        position_controller_left.I = position_controller_params.i;
        position_controller_left.D = position_controller_params.d;
        position_controller_left.output_ramp =
            position_controller_params.output_ramp;
        position_controller_left.limit = position_controller_params.limit;
        position_lpf_left.Tf = position_controller_params.lpf_time_constant;
        position_controller_left.reset();

        position_controller_right.P = position_controller_params.p;
        position_controller_right.I = position_controller_params.i;
        position_controller_right.D = position_controller_params.d;
        position_controller_right.output_ramp =
            position_controller_params.output_ramp;
        position_controller_right.limit = position_controller_params.limit;
        position_lpf_right.Tf = position_controller_params.lpf_time_constant;
        position_controller_right.reset();
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // initialize the PID manager
  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  left_motor.set_position_controller(velocity_controller_params);
  right_motor.set_position_controller(velocity_controller_params);
  left_motor.set_velocity_controller(velocity_controller_params);
  right_motor.set_velocity_controller(velocity_controller_params);

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  left_motor.zero_position();
  right_motor.zero_position();

  // Normally, we'd enable the motors here. However, since the GUI can
  //   enable, leave them disabled so the robot doesn't run away
  //   left_motor.enable();
  //   right_motor.enable();

  ArduinoOTA.setHostname("slider");

  ArduinoOTA
      .onStart(
          []()
          {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
              type = "sketch";
            else  // U_SPIFFS
              type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount
            // SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
          })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress(
          [](unsigned int progress, unsigned int total)
          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError(
          [](ota_error_t error)
          {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
              Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
              Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
              Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
              Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)
              Serial.println("End Failed");
          });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
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
  //  Run control loop
  //   Drive velocity and position to zero for both motors
  float velocity_command_left =
      velocity_controller_left(-velocity_lpf_left(left_motor.get_velocity()));

  float velocity_command_right = velocity_controller_right(
      -velocity_lpf_right(right_motor.get_velocity()));

  float position_command_left =
      position_controller_left(-position_lpf_left(left_motor.get_position()));

  float position_command_right = position_controller_right(
      -position_lpf_right(right_motor.get_position()));

  // Set target voltage
  left_motor.set_target_voltage(velocity_command_left + position_command_left);
  right_motor.set_target_voltage(velocity_command_right +
                                 position_command_right);

  //  Print the velocity and position of the motors
  freq_println(
      "Left velocity: " + String(left_motor.get_velocity()) + "\t" +
          "Right velocity: " + String(right_motor.get_velocity()) + "\t" +
          "Left position: " + String(left_motor.get_position()) + "\t" +
          "Right position: " + String(right_motor.get_position()),
      10);

  ArduinoOTA.handle();
}
