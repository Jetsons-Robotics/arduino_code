// Combined IMU and Encoder ROS Node (Cleaned Version)
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <avr/interrupt.h>
#include <math.h>

// ===== Motor Driver Pin Definitions =====
#define MOTOR1_RV 8
#define MOTOR1_EN 10
#define MOTOR1_FR 11
#define MOTOR1_BK 6

#define MOTOR2_RV 7
#define MOTOR2_EN 13
#define MOTOR2_FR 5
#define MOTOR2_BK 4

// ===== Encoder Definitions =====
volatile long Lcount = 0, Rcount = 0;
#define LEFT_ENCODER_A  18
#define LEFT_ENCODER_B   2
#define RIGHT_ENCODER_A 19
#define RIGHT_ENCODER_B  3

// ===== RPM & Kinematics Constants =====
unsigned long prevLeftEncoder = 0, prevRightEncoder = 0;
long leftRPM = 0, rightRPM = 0;
const int encoderCPR     = 400;
const float wheel_radius = 0.05;
const float wheel_base   = 0.385;
const float dt           = 0.1;

// ===== Heading from encoders =====
float theta = 0.0;
float prevYaw = 0.0;
float accumYaw = 0.0;

// ===== Timer Flag =====
volatile bool timerFlag = false;

// ===== ROS Setup =====
ros::NodeHandle nh;
std_msgs::Float32 thetaMsg;
std_msgs::Float32 vEstMsg, wEstMsg;
std_msgs::Float32 yawMsg;

ros::Publisher theta_pub     ("theta",     &thetaMsg);
ros::Publisher v_est_pub     ("v_est",     &vEstMsg);
ros::Publisher w_est_pub     ("w_est",     &wEstMsg);
ros::Publisher yaw_pub       ("yaw",       &yawMsg);

// ===== Forward Declarations =====
void computeTargetRPMs(float v_goal, float w, float &l_target, float &r_target);
void driveMotor(int en, int fr, int bk, int rv, int pwmValue, bool forward);
void stopMotor(int en, int rv, int bk);
void connectIMU();

// ===== cmd_vel callback =====
void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  float linear  = cmd_msg.linear.x;
  float angular = cmd_msg.angular.z;

  float targetLeftRPM, targetRightRPM;
  computeTargetRPMs(linear, angular, targetLeftRPM, targetRightRPM);

  int leftPWM  = 255 - int(constrain(fabs(targetLeftRPM) / 36.0, 0.0, 1.0) * 255);
  int rightPWM = 255 - int(constrain(fabs(targetRightRPM) / 36.0, 0.0, 1.0) * 255);

  bool leftForward  = (targetLeftRPM  >= 0);
  bool rightForward = (targetRightRPM >= 0);

  driveMotor(MOTOR1_EN, MOTOR1_FR, MOTOR1_BK, MOTOR1_RV, leftPWM,  leftForward);
  driveMotor(MOTOR2_EN, MOTOR2_FR, MOTOR2_BK, MOTOR2_RV, rightPWM, rightForward);
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback);

// ===== IMU (BNO085) Setup =====
Adafruit_BNO08x bno08x(0x4B);
sh2_SensorValue_t sensorValue;
bool imuConnected            = false;
unsigned long lastAttempt    = 0;
const unsigned long reconnectTimeout = 1000;
bool referenceSet            = false;
float ref_qw, ref_qx, ref_qy, ref_qz;
float imuYaw = 0.0;

// ===== ISRs =====
void LeftmotorISR()  {
  Lcount += (digitalRead(LEFT_ENCODER_B) == LOW) ? +1 : -1;
}
void RightmotorISR() {
  Rcount += (digitalRead(RIGHT_ENCODER_B) == HIGH) ? +1 : -1;
}
ISR(TIMER1_COMPA_vect) {
  timerFlag = true;
}

// ===== Helper Functions =====
void computeTargetRPMs(float v_goal, float w, float &l_target, float &r_target) {
  float v_l = v_goal - (w * wheel_base / 2.0);
  float v_r = v_goal + (w * wheel_base / 2.0);
  l_target = (v_l / (2.0 * M_PI * wheel_radius)) * 60.0;
  r_target = (v_r / (2.0 * M_PI * wheel_radius)) * 60.0;
}

void driveMotor(int en, int fr, int bk, int rv,
                int pwmValue, bool forward) {
  if (pwmValue >= 255) {
    stopMotor(en, rv, bk);
    return;
  }
  digitalWrite(bk, LOW);
  digitalWrite(en, HIGH);
  digitalWrite(fr, forward ? LOW : HIGH);
  analogWrite(rv, pwmValue);
}

void stopMotor(int en, int rv, int bk) {
  analogWrite(rv, 255);
  digitalWrite(en, LOW);
  digitalWrite(bk, HIGH);
}

void connectIMU() {
  if (millis() - lastAttempt < reconnectTimeout) return;
  lastAttempt = millis();
  if (bno08x.begin_I2C()) {
    bno08x.enableReport(SH2_ROTATION_VECTOR);
    imuConnected  = true;
    referenceSet  = false;
  } else {
    imuConnected = false;
  }
}

// ===== setup() =====
void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(theta_pub);
  nh.advertise(v_est_pub);
  nh.advertise(w_est_pub);
  nh.advertise(yaw_pub);

  pinMode(MOTOR1_RV, OUTPUT); pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR1_FR, OUTPUT); pinMode(MOTOR1_BK, OUTPUT);
  pinMode(MOTOR2_RV, OUTPUT); pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_FR, OUTPUT); pinMode(MOTOR2_BK, OUTPUT);
  stopMotor(MOTOR1_EN, MOTOR1_RV, MOTOR1_BK);
  stopMotor(MOTOR2_EN, MOTOR2_RV, MOTOR2_BK);

  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A),  LeftmotorISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), RightmotorISR, RISING);

  Wire.begin();

  cli();
  TCCR1A = 0; TCNT1 = 0;  TCCR1B = 0;
  OCR1A  = 1562;
  TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();

  connectIMU();
}

// ===== loop() =====
void loop() {
  nh.spinOnce();

  if (!imuConnected) {
    connectIMU();
  } else if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;

      if (!referenceSet) {
        ref_qw = qw; ref_qx = qx; ref_qy = qy; ref_qz = qz;
        referenceSet = true;
      }

      float rw0 =  ref_qw, rx0 = -ref_qx,
            ry0 = -ref_qy, rz0 = -ref_qz;
      float rw = qw * rw0 - qx * rx0 - qy * ry0 - qz * rz0;
      float rx = qw * rx0 + qx * rw0 + qy * rz0 - qz * ry0;
      float ry = qw * ry0 - qx * rz0 + qy * rw0 + qz * rx0;
      float rz = qw * rz0 + qx * ry0 - qy * rx0 + qz * rw0;

      float rawYaw = atan2(2.0 * (rw * rz + rx * ry),
                           1.0 - 2.0 * (ry * ry + rz * rz));

      float deltaYaw = rawYaw - prevYaw;
      if (deltaYaw > M_PI)      deltaYaw -= 2.0 * M_PI;
      else if (deltaYaw < -M_PI) deltaYaw += 2.0 * M_PI;

      accumYaw += deltaYaw;
      prevYaw = rawYaw;

      imuYaw = accumYaw;
    }
  }

  if (timerFlag) {
    timerFlag = false;

    long dL = Lcount - prevLeftEncoder;
    long dR = Rcount - prevRightEncoder;
    prevLeftEncoder  = Lcount;
    prevRightEncoder = Rcount;
    leftRPM  = (600L * dL) / encoderCPR;
    rightRPM = (600L * dR) / encoderCPR;

    float leftLin  = (leftRPM  * 2 * M_PI * wheel_radius) / 60.0;
    float rightLin = (rightRPM * 2 * M_PI * wheel_radius) / 60.0;

    float v_est = (leftLin + rightLin) / 2.0;
    float w_est = (rightLin - leftLin) / wheel_base;

    theta += w_est * dt;

    thetaMsg.data = theta;     theta_pub.publish(&thetaMsg);
    yawMsg.data   = imuYaw;    yaw_pub.publish(&yawMsg);
    vEstMsg.data  = v_est;     v_est_pub.publish(&vEstMsg);
    wEstMsg.data  = w_est;     w_est_pub.publish(&wEstMsg);
  }
}

//CODE MODIFIED FOR GIT PULL
//IGNORE THIS 
