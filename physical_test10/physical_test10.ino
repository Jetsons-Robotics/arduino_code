/*
  Differential Drive Robot Motor Control with Encoder Feedback, PID, and Heading Correction
*/

// ===== PIN DEFINITIONS =====
#define MOTOR1_RV 8
#define MOTOR1_EN 10
#define MOTOR1_FR 11
#define MOTOR1_BK 6

#define MOTOR2_RV 7
#define MOTOR2_EN 13
#define MOTOR2_FR 5
#define MOTOR2_BK 4

#define LEFT_ENCODER_A 18
#define LEFT_ENCODER_B 2
#define RIGHT_ENCODER_A 19
#define RIGHT_ENCODER_B 3

#define START_STOP_PIN 12
const int sensorInputPin1 = 31;
const int sensorInputPin2 = 32;
const int sensorInputPin3 = 33;
const int sensorInputPin4 = 34;

// ===== OUTER-LOOP (HEADING) =====
bool justStarted = true;      // latch to capture thetaRef once on start
float thetaRef   = 0.0;       // desired heading (degrees)
const float Ktheta = 0.6;     // outer-loop gain (RPM per degree error)
float headingInt = 0.0;       // integral term accumulator
const float Ki_theta = 0.01;  // integral gain for heading correction

// ===== CONSTANTS =====
#define PI 3.14159265358979323846
const float dt = 0.1;
const float wheel_radius = 0.05;
const float wheel_base   = 0.385;
const int encoderCPR      = 400;
const unsigned long debounceDelay = 50;

// PID gains
const float Kp_L = 1.5, Ki_L = 0.005, Kd_L = 0.05;
const float Kp_R = 1.5, Ki_R = 0.005, Kd_R = 0.05;

// ===== GLOBAL STATE =====
volatile bool calculateRPMFlag = false;
volatile long Lcount = 0, Rcount = 0;
unsigned long prevLeftEncoder = 0, prevRightEncoder = 0;
long leftRPM = 0, rightRPM = 0;

float errorL_prev = 0, integralL = 0;
float errorR_prev = 0, integralR = 0;

bool robotRunning = false;
float v_goal = 0.0, w = 0.0, thetaDeg = 0.0;

unsigned long lastDebounceTime = 0;

enum RobotMotion { MOTION_FORWARD, MOTION_BACKWARD, DEFAULT_STATE };
RobotMotion currentMotion = DEFAULT_STATE;

// ===== SETUP =====
void setup() {
  Serial.begin(9600);
  pinMode(START_STOP_PIN, INPUT_PULLUP);
  pinMode(sensorInputPin1, INPUT_PULLUP);
  pinMode(sensorInputPin2, INPUT_PULLUP);
  pinMode(sensorInputPin3, INPUT_PULLUP);
  pinMode(sensorInputPin4, INPUT_PULLUP);

  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
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

  // Timer1 for 100ms
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 1562;
  TCCR1B |= (1<<WGM12)|(1<<CS12)|(1<<CS10);
  TIMSK1 |= (1<<OCIE1A);
  sei();
}

// ===== MAIN LOOP =====
void loop() {
  static bool lastButtonState = HIGH, stableState = HIGH;
  static unsigned long lastBounceTime = 0;

  int currentButtonState = digitalRead(START_STOP_PIN);
  if (currentButtonState != lastButtonState) lastBounceTime = millis();
  if ((millis() - lastBounceTime) > debounceDelay) {
    if (currentButtonState != stableState) {
      stableState = currentButtonState;
      if (stableState == LOW) {
        robotRunning = !robotRunning;
        if (robotRunning) {
          justStarted = true;  // reset heading capture
        } else {
          // stop & brake
          digitalWrite(MOTOR1_EN, LOW);
          digitalWrite(MOTOR2_EN, LOW);
          digitalWrite(MOTOR1_BK, HIGH);
          digitalWrite(MOTOR2_BK, HIGH);
          v_goal = w = 0;
          integralL = integralR = errorL_prev = errorR_prev = 0;
        }
      }
    }
  }
  lastButtonState = currentButtonState;
  if (!robotRunning) return;

  // release brakes & enable motors
  digitalWrite(MOTOR1_BK, LOW); digitalWrite(MOTOR1_EN, HIGH);
  digitalWrite(MOTOR2_BK, LOW); digitalWrite(MOTOR2_EN, HIGH);

  int s1 = digitalRead(sensorInputPin1);
  int s2 = digitalRead(sensorInputPin2);
  int s3 = digitalRead(sensorInputPin3);
  int s4 = digitalRead(sensorInputPin4);
  static RobotMotion lastMotion = MOTION_FORWARD;
  RobotMotion prevMotion = lastMotion;
  if (s3==HIGH && s4==HIGH) currentMotion = MOTION_BACKWARD;
  else if (s1==HIGH && s2==HIGH) currentMotion = MOTION_FORWARD;
  else currentMotion = lastMotion;
  if (currentMotion != prevMotion) {
    // brief brake and reset
    digitalWrite(MOTOR1_EN, LOW);
    digitalWrite(MOTOR2_EN, LOW);
    digitalWrite(MOTOR1_BK, HIGH);
    digitalWrite(MOTOR2_BK, HIGH);
    digitalWrite(MOTOR1_BK, LOW);
    digitalWrite(MOTOR2_BK, LOW);
    digitalWrite(MOTOR1_EN, HIGH);
    digitalWrite(MOTOR2_EN, HIGH);
    v_goal = 0;
    integralL = integralR = errorL_prev = errorR_prev = 0;
  }
  lastMotion = currentMotion;

  switch (currentMotion) {
    case MOTION_FORWARD:
      digitalWrite(MOTOR1_FR, HIGH);
      digitalWrite(MOTOR2_FR, LOW);
      v_goal = 0.18; w = 0;
      break;
    case MOTION_BACKWARD:
      digitalWrite(MOTOR1_FR, LOW);
      digitalWrite(MOTOR2_FR, HIGH);
      v_goal = 0.18; w = 0;
      break;
    default:
      v_goal = w = 0;
      break;
  }

  if (calculateRPMFlag) {
    calculateRPMFlag = false;

    // // Ramp filter for velocity command
    // const float maxStep = v_goal;  
    // v_actual = constrain(v_actual + ((v_goal>v_actual)? maxStep : -maxStep), v_goal, v_goal);

    // enc->RPM
    long curL=Lcount, curR=Rcount;
    
    long dL=curL-prevLeftEncoder, dR=curR-prevRightEncoder;
    prevLeftEncoder=curL; prevRightEncoder=curR;
    leftRPM  = (600*dL)/encoderCPR;
    rightRPM = (600*dR)/encoderCPR;
    Serial.print(leftRPM); Serial.print("  "); Serial.print(rightRPM);

    // base targets
    float Target_leftRPM, Target_rightRPM;
    computeTargetRPMs(v_goal,w,Target_leftRPM,Target_rightRPM);

    // --- OUTER-LOOP Heading Correction ---
    if (justStarted) {
      thetaRef     = thetaDeg;
      headingInt   = 0;
      justStarted  = false;
    }
    float headingError = thetaRef - thetaDeg;
    if (headingError > 180) headingError -= 360;
    else if (headingError < -180) headingError += 360;
    headingInt += headingError * dt;
    float trimSign = (currentMotion == MOTION_BACKWARD) ? -1.0 : 1.0;
    float trimRPM = (Ktheta * headingError + Ki_theta * headingInt) * trimSign;
    float lTrimmed = Target_leftRPM - trimRPM;
    float rTrimmed = Target_rightRPM + trimRPM;

    // --- INNER-LOOP PIDs ---
    runLeftPID(lTrimmed,  leftRPM);
    runRightPID(rTrimmed, rightRPM);

    // odometry + theta print
    float leftLin  = (leftRPM * 2*PI*wheel_radius)/60.0;
    float rightLin = (rightRPM * 2*PI*wheel_radius)/60.0;
    float omega    = (rightLin - leftLin)/wheel_base;
    thetaDeg += (omega*dt)*180.0/PI;
    Serial.print(" "); Serial.println(thetaDeg);
  }
}

// ===== HELPER FUNCTIONS =====
void computeTargetRPMs(float v_goal, float w, float &l_target, float &r_target) {
  float v_l = v_goal - (w*wheel_base/2.0);
  float v_r = v_goal + (w*wheel_base/2.0);
  l_target = (v_l/(2*PI*wheel_radius))*60.0;
  r_target = (v_r/(2*PI*wheel_radius))*60.0;
}

void runLeftPID(float targetRPM, long actualRPM) {
  float eL = abs(targetRPM) - abs(actualRPM);
  integralL += eL * dt;
  float dEL = (eL - errorL_prev)/dt;
  float outL = Kp_L*eL + Ki_L*integralL + Kd_L*dEL;
  errorL_prev = eL;
  int pwmL = constrain((int)abs(outL), 0, 255);
  digitalWrite(MOTOR1_FR, targetRPM<0?LOW:HIGH);
  analogWrite(MOTOR1_RV, 255-pwmL);
}

void runRightPID(float targetRPM, long actualRPM) {
  float eR = abs(targetRPM) - abs(actualRPM);
  integralR += eR * dt;
  float dER = (eR - errorR_prev)/dt;
  float outR = Kp_R*eR + Ki_R*integralR + Kd_R*dER;
  errorR_prev = eR;
  int pwmR = constrain((int)abs(outR), 0, 255);
  digitalWrite(MOTOR2_FR, targetRPM<0?HIGH:LOW);
  analogWrite(MOTOR2_RV, 255-pwmR);
}

// ===== ISRs =====
ISR(TIMER1_COMPA_vect) { calculateRPMFlag = true; }
void LeftmotorISR()  { Lcount += (digitalRead(LEFT_ENCODER_B)==LOW)? 1:-1; }
void RightmotorISR() { Rcount += (digitalRead(RIGHT_ENCODER_B)==HIGH)?1:-1; }
