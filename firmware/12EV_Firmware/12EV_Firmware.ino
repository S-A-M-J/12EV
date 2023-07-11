///////////////////////////////////////////////////
// Based on pybot robotic arm project from JJRobots
// Ported and changed by S.A.M
// Last updated on 11.07.23
///////////////////////////////////////////////////

#define VERSION "Scara v0.16"
//#define DEBUG 0

// ROBOT and USER configuration parameters
#include "Configuration.h"
#include <Wire.h>
#include "EV12.h"

//added for esp32 compatability
#include <esp32-hal-timer.h>
#include "esp_system.h"
#include "esp_intr_alloc.h"

hw_timer_t *TC3 = NULL;
hw_timer_t *TC5 = NULL;
hw_timer_t *TCC2 = NULL;

//interrupt task
TaskHandle_t Task1;

//colorsensor
#define I2C_SDA 23
#define I2C_SCL 25
TwoWire I2C = TwoWire(0);
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Configuration: Pins, servos, Steppers, Wifi...
void setup() {
  // STEPPER PINS ON JJROBOTS DEVIA M0 BOARD
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  pinMode(M3_EN_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M3_DIR_PIN, OUTPUT);
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M3_STEP_PIN, OUTPUT);
  pinMode(27, INPUT_PULLDOWN);

  pinMode(EM_PIN, OUTPUT);    // Electromagnet output
  digitalWrite(EM_PIN, LOW);  //Disabled


  // Serial ports initialization
  setDelay(100);
  Serial.begin(115200);  // Serial output to console
  //Serial1.begin(115200); // Wifi initialization
  //Wire.begin();
  Serial.println("welcome");
  setDelay(100);
  enableMotors(false);
#ifdef DEBUG
  delay(10000);  // Only needed for serial debug
  Serial.println(VERSION);
#endif

  // Start TCP SERVER on port 2222, telemetry port 2223
  //Serial.println("Start TCP server");
  //ESPsendCommand("AT+CIPCLOSE=0","OK",3);
  //ESPsendCommand("AT+CIPSERVER=0","OK",3);
  //ESPsendCommand("AT+CIPMUX=1", "OK", 3);  // Multiple connection mode
  //ESPsendCommand("AT+CIPMODE=1", "OK", 3); // Transparent mode
  //ESPsendCommand("AT+CIPSERVER=1,2222", "OK", 3); // TCP server
  setDelay(100);

  // Debug: Output parameters
  //Serial.print("Max_acceleration_x: ");
  //Serial.println(acceleration_x);
  //Serial.print("Max_acceleration_y: ");
  //Serial.println(acceleration_y);
  //Serial.print("Max speed X: ");
  //Serial.println(MAX_SPEED_X);
  //Serial.print("Max speed Y: ");
  //Serial.println(MAX_SPEED_Y);
  //xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
  // STEPPER MOTORS INITIALIZATION
  Serial.println("Steper motors initialization...");
  timersConfigure();
  Serial.println("Timers initialized");
  timersStart();  //starts the timers
  Serial.println("Timers started");
  delay(100);
  Serial.println("Moving to initial position...");

  configSpeed(MAX_SPEED_M1 / 10, MAX_SPEED_M2 / 10, MAX_SPEED_M3);
  configAcceleration(MAX_ACCEL_M1 / 2, MAX_ACCEL_M2 / 2, MAX_ACCEL_M3);
  setSpeedAcc();
  Serial.println("speed set");
  //Initializing init position
  target_angleA1 = ROBOT_INITIAL_POSITION_M1;
  target_angleA2 = ROBOT_INITIAL_POSITION_M2 + ROBOT_INITIAL_POSITION_M1 * AXIS2_AXIS1_correction;
  position_M1 = target_angleA1 * M1_AXIS_STEPS_PER_UNIT;
  position_M2 = target_angleA2 * M2_AXIS_STEPS_PER_UNIT;
  position_M3 = ROBOT_INITIAL_POSITION_M3 * M3_AXIS_STEPS_PER_UNIT;

#ifdef INITIALIZE_TO_MAXIMUNS
  motorsCalibration();
#endif

  //target_angleA1 = ROBOT_INITIAL_POSITION_M1;
  //target_angleA2 = ROBOT_INITIAL_POSITION_M2 + ROBOT_INITIAL_POSITION_M1*AXIS2_AXIS1_correction;

  configSpeed(MAX_SPEED_M1, MAX_SPEED_M2, MAX_SPEED_M3);
  configAcceleration(MAX_ACCEL_M1, MAX_ACCEL_M2, MAX_ACCEL_M3);
  setSpeedAcc();
  target_position_M1 = position_M1;
  target_position_M2 = position_M2;
  target_position_M3 = position_M3;

  Serial.println("Initial position configured!");
  Serial.println(ROBOT_ARM1_LENGTH);
  Serial.println(ROBOT_ARM2_LENGTH);
  Serial.println(ROBOT_ARM1_LENGTH + ROBOT_ARM2_LENGTH);

  Serial.println(" Ready...");
  Serial.print(" JJROBOTS SCARA ");
  Serial.println(VERSION);
  timer_old = micros();
  slow_timer_old = millis();
  timeout_counter = 0;

  //colorsensor setup
  I2C.begin(I2C_SDA, I2C_SCL, 100000);
  if (colorSensor.begin(0x29, &I2C)) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
  }
  setupBLE();
  // Enable motors
  enableMotors(true);
  setBatPos();
  Serial.println("setup completed");
}


// *************** APLICATION MAIN LOOP ***********************
void loop() {
  USBMsgRead();  // Read USB messages
  if (newMessage) {
    newMessage = 0;
    //debugMsg();
    if (mode == 2) {
      // Inverse Kinematic mode
      if (iCH5 == 1) {
        elbow = 1;
        //Serial.println("->Elbow:1");
      } else
        elbow = 0;
      float x = iCH1 / 10.0;
      float y = iCH2 / 10.0;
      float A1, A2;

      Serial.print("->IK time:");
      long t0 = micros();
      InverseKinematic(x, y, ROBOT_ARM1_LENGTH, ROBOT_ARM2_LENGTH, elbow, &A1, &A2);
      long t1 = micros();
      Serial.println(t1 - t0);  // First implementation was 560us
      Serial.print("->IK:");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.print(",");
      Serial.print(iCH3);
      Serial.print(" :");
      Serial.print(A1);
      Serial.print(",");
      Serial.println(A2);
      setAxis_1_2(A1, A2);
      if (iCH3 != NODATA)
        target_position_M3 = (iCH3 / 100.0) * M3_AXIS_STEPS_PER_UNIT;
      if (iCH4 > 0)
        digitalWrite(EM_PIN, HIGH);
      else
        digitalWrite(EM_PIN, LOW);
      setSpeedAcc();
      trajectory_motor_speed_adjust();  // Function to sync both motors
    } else if (mode == 3) {
      // Trajectory mode
      //Serial.print("->T MODE:");
      //Serial.println(trajectory_num_points);
    }

    else if (mode == 4) {  // Emergency stop
      // Stop the robot in the actual position
      //float a1 = (position_M1 / M1_AXIS_STEPS_PER_UNIT);
      //float a2 = (position_M2 / M2_AXIS_STEPS_PER_UNIT) - a1 * AXIS2_AXIS1_correction;
      //float az =  position_M3 / M3_AXIS_STEPS_PER_UNIT;
      //setAxis_1_2(a1, a2);
      //setAxis3(az);
      // Disable motors
      enableMotors(false);  // Disable motors
    }

    else if (mode == 5) {  // Robot calibration
      Serial.println("->Motors calibration...");
      motorsCalibration();
      setSpeedAcc();
      working = false;
      trajectory_mode = false;
    }
  }

  timer_value = micros();
  dt = timer_value - timer_old;
  if (dt >= 1000) {  // 1Khz loop for position,speed and acceleration control
    if (dt > 1500) {
      Serial.print("!!");  // Timing warning
      Serial.println(dt);
    }
    timer_old = timer_value;

    positionControl(1000);  // position, speed and acceleration control of stepper motors
    // Trajectory mode manage...
    if (trajectory_mode) {
      trajectory_processing = false;  // end of trajectory processing...
      working = true;                 // Work to do...
      // Are we reaching next point without total stop? => Go to next point
      //if ((M1stopping || (dir_M1 == 0)) && (M2stopping || (dir_M2 == 0)) && (M3stopping || (dir_M3 == 0))) {
      diff_M1 = myAbs(target_position_M1 - position_M1);
      diff_M2 = myAbs(target_position_M2 - position_M2);
      diff_M3 = myAbs(target_position_M3 - position_M3);
      if ((diff_M1 < trajectory_tolerance_M1) && (diff_M2 < trajectory_tolerance_M2) && (diff_M3 < trajectory_tolerance_M3)) {
        // Go to next point
        if (trajectory_point <= trajectory_num_points) {
          Serial.print("->T ");
          Serial.println(trajectory_point);
          //Serial.print(" ");
          //Serial.print(trajectory_vector[trajectory_point][0]);
          //Serial.print(" ");
          //Serial.println(trajectory_vector[trajectory_point][1]);
          setAxis_1_2(trajectory_vector[trajectory_point][0], trajectory_vector[trajectory_point][1]);
          setSpeedAcc();
          trajectory_motor_speed_adjust();  // Function to sync both motors
          if (trajectory_vector[trajectory_point][2] != NODATA)
            target_position_M3 = trajectory_vector[trajectory_point][2] * M3_AXIS_STEPS_PER_UNIT;
          if (trajectory_vector[trajectory_point][3] != NODATA)
            //moveServo1(trajectory_vector[trajectory_point][3] * SERVO1_RANGE / 1000.0 + SERVO1_MIN_PULSEWIDTH);
            if (trajectory_vector[trajectory_point][4] != NODATA)
              //moveServo2(trajectory_vector[trajectory_point][4] * SERVO2_RANGE / 1000.0 + SERVO2_MIN_PULSEWIDTH);
              trajectory_point++;
        } else {
          trajectory_mode = false;
          trajectory_point = 0;
          trajectory_processing = false;
        }
      }
    }

    loop_counter += 1;

    // Debug loop counter
    if (loop_counter % 10 == 0) {
      char message[80];
      //sprintf(message, "#%d:%d,%d,%d,%d", loop_counter/10,actual_angleA1, actual_angleA2,speed_M1,speed_M2);
      //Serial.println(message);
    }

    slow_timer_value = millis();
    if ((slow_timer_value - slow_timer_old) >= 50) {  // Slow loop (20hz)
      char message[80];

      slow_timer_old = slow_timer_value;

      if (trajectory_mode)
        working = true;
      else {
        // Check if robot is stopped (reach final position)
        diff_M1 = myAbs(target_position_M1 - position_M1);
        diff_M2 = myAbs(target_position_M2 - position_M2);
        diff_M3 = myAbs(target_position_M3 - position_M3);
        if ((diff_M1 < STOP_TOLERANCE) && (diff_M2 < STOP_TOLERANCE) && (diff_M3 < STOP_TOLERANCE)) {
          working = false;
          if (EV12running) {
            EV12interimStage++;
            EV12Disassembly();
          }
        } else
          working = true;
      }

      // Timestamp for status message...
      timestamp += 1;
      if (timestamp > 999)
        timestamp = 0;

      // Calculate actual robot angles based on internal motor positions (steps)
      actual_angleA1 = (position_M1 / M1_AXIS_STEPS_PER_UNIT) * 10;
      actual_angleA2 = (position_M2 / M2_AXIS_STEPS_PER_UNIT) * 10 - actual_angleA1 * AXIS2_AXIS1_correction;
      actual_valueZ = (position_M3 / M3_AXIS_STEPS_PER_UNIT) * 10;

      //uint16_t debug_value = acceleration_M2;
      if (working) {
        sprintf(message, "$$1,%d,%d,%d,%d,%d,%d,%d", actual_angleA1, actual_angleA2, actual_valueZ, actual_valueG / 10, actual_valueW / 10, actual_distance, timestamp);
        //Serial.println(message);
      }
      if (enable_udp_output) {  // Output UDP messages if we detect an UDP external interface
        Serial1.println(message);
      }
    }  // 20hz loop
  }    // 1Khz loop
}
