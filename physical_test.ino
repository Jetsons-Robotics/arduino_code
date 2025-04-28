#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Int32 left_rpm_msg;
std_msgs::Int32 right_rpm_msg;
std_msgs::Float32 left_velocity_msg;
std_msgs::Float32 right_velocity_msg;
std_msgs::Float32 v_msg;
std_msgs::Float32 w_msg;
std_msgs::Float32 theta_msg;

ros::Publisher left_motor_pub("/left_motor_rpm", &left_rpm_msg);
ros::Publisher right_motor_pub("/right_motor_rpm", &right_rpm_msg);
ros::Publisher left_velocity_pub("/left_motor_velocity", &left_velocity_msg);
ros::Publisher right_velocity_pub("/right_motor_velocity", &right_velocity_msg);
ros::Publisher v_pub("/linear_velocity", &v_msg);
ros::Publisher w_pub("/angular_velocity", &w_msg);
ros::Publisher theta_pub("/robot_orientation", &theta_msg);

volatile float desiredLeftRPM = 0.0;
volatile float desiredRightRPM = 0.0;

void leftMotorCallback(const std_msgs::Int32& msg) {
  desiredLeftRPM = msg.data;
}

void rightMotorCallback(const std_msgs::Int32& msg) {
  desiredRightRPM = msg.data;
}

ros::Subscriber<std_msgs::Int32> left_motor_sub("/desired_left_motor_rpm", &leftMotorCallback);
ros::Subscriber<std_msgs::Int32> right_motor_sub("/desired_right_motor_rpm", &rightMotorCallback);

// Motor pins
#define MOTOR1_RV 8
#define MOTOR1_EN 10
#define MOTOR1_FR 11
#define MOTOR1_BK 6
#define MOTOR2_RV 7
#define MOTOR2_EN 13
#define MOTOR2_FR 5
#define MOTOR2_BK 4

// Encoder pins
#define LEFT_ENCODER_A 18
#define LEFT_ENCODER_B 2
#define RIGHT_ENCODER_A 19
#define RIGHT_ENCODER_B 3

// Sensor input pins
const int sensorInputPin1 = 31;
const int sensorInputPin2 = 32;
const int sensorInputPin3 = 33;
const int sensorInputPin4 = 34;

// Start/Stop switch pin
#define START_STOP_PIN 12

volatile unsigned long Lcount = 0;
volatile unsigned long Rcount = 0;
volatile int leftDirection = 1;
volatile int rightDirection = 1;

// Timing for RPM calculations
unsigned long prevTime0 = 0;
unsigned long prevTime1 = 0;
unsigned long prevLeftEncoder = 0;
unsigned long prevRightEncoder = 0;
int interval = 100;
int leftRPM = 0;
int rightRPM = 0;
const int encoderCPR = 400;
const float wheelRadius = 0.05;
const float my_pi = 3.14159265358979323846;

// PID parameters
float Kp1 = 0.001, Ki1 = 0.0, Kd1 = 0.0;
float Kp2 = 0.001, Ki2 = 0.0, Kd2 = 0.0;
float left_integral = 0.0, left_last_error = 0.0;
float right_integral = 0.0, right_last_error = 0.0;
int leftPWMCommand = 255;
int rightPWMCommand = 255;
const int leftMinPWM = 0, leftMaxPWM = 255;
const int rightMinPWM = 0, rightMaxPWM = 255;
const float MaxRPM = 100.0;

float v = 0.0;
float w = 0.0;
float Theta = 0.0;

// Debounce variables
bool robotRunning = false;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ROS message timing
unsigned long lastPublishTime = 0;
const int publishInterval = 100;

void thetaCallback(const std_msgs::Float32& msg) {
  Theta = msg.data;
}
ros::Subscriber<std_msgs::Float32> theta_sub("/theta_degrees", &thetaCallback);

enum RobotMotion { MOTION_FORWARD, MOTION_STOP, MOTION_BACKWARD, MOTION_TURN, TURN_90, DEFAULT_STATE };
RobotMotion currentMotion = DEFAULT_STATE;
bool turn90Initiated = false;
float startThetaTurn90 = 0.0;
bool stopInitiated = false;
unsigned long stopStartTime = 0;
const unsigned long backwardDuration = 3000;

void setup() {
  nh.initNode();
  nh.advertise(left_motor_pub);
  nh.advertise(right_motor_pub);
  nh.advertise(left_velocity_pub);
  nh.advertise(right_velocity_pub);
  nh.advertise(v_pub);
  nh.advertise(w_pub);
  nh.advertise(theta_pub);
  nh.subscribe(left_motor_sub);
  nh.subscribe(right_motor_sub);
  nh.subscribe(theta_sub);

  digitalWrite(MOTOR1_EN, LOW);
  digitalWrite(MOTOR2_EN, LOW);
  digitalWrite(MOTOR1_BK, HIGH);
  digitalWrite(MOTOR2_BK, HIGH);

  pinMode(sensorInputPin1, INPUT_PULLUP);
  pinMode(sensorInputPin2, INPUT_PULLUP);
  pinMode(sensorInputPin3, INPUT_PULLUP);
  pinMode(sensorInputPin4, INPUT_PULLUP);

  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  pinMode(START_STOP_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), LeftmotorISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), RightmotorISR, RISING);

  pinMode(MOTOR1_RV, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR1_FR, OUTPUT);
  pinMode(MOTOR1_BK, OUTPUT);
  pinMode(MOTOR2_RV, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_FR, OUTPUT);
  pinMode(MOTOR2_BK, OUTPUT);
}

void loop() {
  // —— 1) TWO-VARIABLE DEBOUNCE FOR START/STOP BUTTON ——  
  static int buttonState = HIGH;  
  static int lastReading = HIGH;
  // (lastDebounceTime, debounceDelay, robotRunning are globals)

  int reading = digitalRead(START_STOP_PIN);
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }
  if (millis() - lastDebounceTime > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      // only toggle on the falling edge
      if (buttonState == LOW) {
        robotRunning = !robotRunning;
        // immediate hard-stop when toggled off
        if (!robotRunning) {
          digitalWrite(MOTOR1_EN, LOW);
          digitalWrite(MOTOR2_EN, LOW);
          digitalWrite(MOTOR1_BK, HIGH);
          digitalWrite(MOTOR2_BK, HIGH);
          v = w = 0.0;
        }
      }
    }
  }
  lastReading = reading;

  // —— 2) MOTION LOGIC: CONTINUOUS BACK/FORWARD PATROL ——  
  if (robotRunning) {
    // enable motors
    digitalWrite(MOTOR1_BK, LOW);
    digitalWrite(MOTOR1_EN, HIGH);
    digitalWrite(MOTOR2_BK, LOW);
    digitalWrite(MOTOR2_EN, HIGH);

    // read sensors
    int s1 = digitalRead(sensorInputPin1); // rear-right
    int s2 = digitalRead(sensorInputPin2); // rear-left
    int s3 = digitalRead(sensorInputPin3); // front-right
    int s4 = digitalRead(sensorInputPin4); // front-left

    // ① track last motion state
    static int lastMotion = MOTION_FORWARD;
    int prevMotion = lastMotion;

    // decide motion state
    if (s3 == HIGH && s4 == HIGH) {
      currentMotion = MOTION_BACKWARD;
    } else if (s1 == HIGH && s2 == HIGH) {
      currentMotion = MOTION_FORWARD;
    } else {
      currentMotion = lastMotion;
    }

    // reset PID when the case actually changes
    if (currentMotion != prevMotion) {
      left_integral      = 0.0;
      right_integral     = 0.0;
      left_last_error    = 0.0;
      right_last_error   = 0.0;
    }
    lastMotion = currentMotion;

    // act on currentMotion
    switch (currentMotion) {
      case MOTION_FORWARD:
        digitalWrite(MOTOR1_FR, HIGH);
        digitalWrite(MOTOR2_FR, LOW);
        v = 0.15;
        w = 0.0;
        break;

      case MOTION_BACKWARD:
        digitalWrite(MOTOR1_FR, LOW);
        digitalWrite(MOTOR2_FR, HIGH);
        v = 0.15;
        w = 0.0;
        break;

      default:
        // fallback stop
        digitalWrite(MOTOR1_BK, HIGH);
        digitalWrite(MOTOR2_BK, HIGH);
        digitalWrite(MOTOR1_EN, LOW);
        digitalWrite(MOTOR2_EN, LOW);
        v = w = 0.0;
        break;
    }

    // update RPM via PID
    calculateMotorRPM();

  } else {
    // robot not running: ensure motors stay off
    digitalWrite(MOTOR1_BK, HIGH);
    digitalWrite(MOTOR2_BK, HIGH);
    digitalWrite(MOTOR1_EN, LOW);
    digitalWrite(MOTOR2_EN, LOW);
    v = w = 0.0;
  }

  // —— 3) ROS COMM & PUBLISH ——  
  nh.spinOnce();
  if (!nh.connected()) {
    // emergency stop if ROS dies
    digitalWrite(MOTOR1_BK, HIGH);
    digitalWrite(MOTOR2_BK, HIGH);
    digitalWrite(MOTOR1_EN, LOW);
    digitalWrite(MOTOR2_EN, LOW);
    analogWrite(MOTOR1_RV, 0);
    analogWrite(MOTOR2_RV, 0);
    return;
  }

  unsigned long now = millis();
  if (now - lastPublishTime >= publishInterval) {
    lastPublishTime = now;
    left_motor_pub.publish(&left_rpm_msg);
    right_motor_pub.publish(&right_rpm_msg);
    left_velocity_pub.publish(&left_velocity_msg);
    right_velocity_pub.publish(&right_velocity_msg);
    v_msg.data = v;
    w_msg.data = w;
    v_pub.publish(&v_msg);
    w_pub.publish(&w_msg);
    theta_msg.data = Theta;
    theta_pub.publish(&theta_msg);
  }
}


void velocityCallback(const std_msgs::Float32& msg_v, const std_msgs::Float32& msg_w) {
  float v_val = msg_v.data;
  float w_val = msg_w.data;
  float L = 0.3; // Example wheelbase distance
  float v_left = v_val - (w_val * L / 2.0);
  float v_right = v_val + (w_val * L / 2.0);
  desiredLeftRPM = (v_left / (2 * my_pi * wheelRadius)) * 60.0;
  desiredRightRPM = (v_right / (2 * my_pi * wheelRadius)) * 60.0;
}

void calculateMotorRPM() {
  unsigned long currentTime = millis();
  float dt = interval / 1000.0;
  bool updated = false;

  if (currentTime - prevTime0 >= interval) {
  unsigned long currentLeftEncoder = Lcount;
  int deltaLeft = currentLeftEncoder - prevLeftEncoder;
  leftRPM = ((100 * deltaLeft) / encoderCPR) * (600 / interval) * leftDirection;
  prevTime0 = currentTime;
  prevLeftEncoder = currentLeftEncoder;
  updated = true;

  float error = desiredLeftRPM - leftRPM;
  left_integral += error * dt;
  float derivative = (error - left_last_error) / dt;
  float pid_output = Kp1 * error + Ki1 * left_integral + Kd1 * derivative;
  left_last_error = error;

  float scaled_pid_output = pid_output / MaxRPM * (leftMaxPWM - leftMinPWM);
  leftPWMCommand = leftPWMCommand - scaled_pid_output;
  leftPWMCommand = constrain(leftPWMCommand, leftMinPWM, leftMaxPWM);

  left_rpm_msg.data = leftRPM;
  left_velocity_msg.data = (2 * my_pi * wheelRadius * leftRPM) / 60.0;
  }

  if (currentTime - prevTime1 >= interval) {
  unsigned long currentRightEncoder = Rcount;
  int deltaRight = currentRightEncoder - prevRightEncoder;
  rightRPM = ((100 * deltaRight) / encoderCPR) * (600 / interval) * rightDirection;
  prevTime1 = currentTime;
  prevRightEncoder = currentRightEncoder;
  updated = true;

  float error = desiredRightRPM - rightRPM;
  right_integral += error * dt;
  float derivative = (error - right_last_error) / dt;
  float pid_output = Kp2 * error + Ki2 * right_integral + Kd2 * derivative;
  right_last_error = error;

  float scaled_pid_output = pid_output / MaxRPM * (rightMaxPWM - rightMinPWM);
  rightPWMCommand = rightPWMCommand - scaled_pid_output;
  rightPWMCommand = constrain(rightPWMCommand, rightMinPWM, rightMaxPWM);

  right_rpm_msg.data = rightRPM;
  right_velocity_msg.data = (2 * my_pi * wheelRadius * rightRPM) / 60.0;
  }

  if (updated) {
  analogWrite(MOTOR1_RV, leftPWMCommand);
  analogWrite(MOTOR2_RV, rightPWMCommand);
  }
}

void LeftmotorISR() {
  int state = digitalRead(LEFT_ENCODER_B);
  leftDirection = (state == HIGH) ? 1 : -1;
  Lcount--;
}

void RightmotorISR() {
  int state = digitalRead(RIGHT_ENCODER_B);
  rightDirection = (state == HIGH) ? 1 : -1;
  Rcount++;
}
