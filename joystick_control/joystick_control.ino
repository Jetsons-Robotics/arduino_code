#include <ros.h>
#include <geometry_msgs/Twist.h>

// ===== Motor Driver Pin Definitions =====
// Left Motor (Motor 1)
#define MOTOR1_RV 8
#define MOTOR1_EN 10
#define MOTOR1_FR 11
#define MOTOR1_BK 6

// Right Motor (Motor 2)
#define MOTOR2_RV 7
#define MOTOR2_EN 13
#define MOTOR2_FR 5
#define MOTOR2_BK 4

ros::NodeHandle nh;

// Helper: stop motor cleanly
void stopMotor(int en, int rv, int bk) {
  analogWrite(rv, 0);
  digitalWrite(en, LOW);
  digitalWrite(bk, HIGH);  // Engage brake
}

// Helper: drive motor with speed and direction
void driveMotor(int en, int fr, int bk, int rv, int speed, bool forward) {
  if (speed == 0) {
    stopMotor(en, rv, bk);
    return;
  }

  digitalWrite(bk, LOW);         // Disable brake
  digitalWrite(en, HIGH);        // Enable motor
  digitalWrite(fr, forward ? LOW : HIGH); // Direction control
  analogWrite(rv, constrain(abs(speed), 255, 0));  // PWM speed
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  float linear = cmd_msg.linear.x;
  float angular = cmd_msg.angular.z;

  // Compute base speeds
  int leftSpeed = int((linear - angular * 0.5) * 255);
  int rightSpeed = int((linear + angular * 0.5) * 255);

  // Adjust for opposite motor mounting (invert direction of one motor)
  bool leftForward = leftSpeed < 0;
  bool rightForward = rightSpeed >= 0;  // Reversed motor mounting!

  driveMotor(MOTOR1_EN, MOTOR1_FR, MOTOR1_BK, MOTOR1_RV, leftSpeed, leftForward);
  driveMotor(MOTOR2_EN, MOTOR2_FR, MOTOR2_BK, MOTOR2_RV, rightSpeed, rightForward);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  // Set all pins to OUTPUT
  pinMode(MOTOR1_RV, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR1_FR, OUTPUT);
  pinMode(MOTOR1_BK, OUTPUT);

  pinMode(MOTOR2_RV, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_FR, OUTPUT);
  pinMode(MOTOR2_BK, OUTPUT);

  // Stop both motors initially
  stopMotor(MOTOR1_EN, MOTOR1_RV, MOTOR1_BK);
  stopMotor(MOTOR2_EN, MOTOR2_RV, MOTOR2_BK);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
