#define MOTOR1_RV 8    // PWM speed control
#define MOTOR1_EN 10   // Motor enable
#define MOTOR1_FR 11   // Direction control: LOW = forward
#define MOTOR1_BK 6    // Brake control

// Right Motor (Motor 2)
#define MOTOR2_RV 7    // PWM speed control
#define MOTOR2_EN 13   // Motor enable
#define MOTOR2_FR 5    // Direction control: LOW = forward
#define MOTOR2_BK 4    // Brake control

// ===== Encoder Pin Definitions =====
#define LEFT_ENCODER_A 18  // Encoder A channel for left motor
#define LEFT_ENCODER_B 2   // Encoder B channel for left motor
#define RIGHT_ENCODER_A 19 // Encoder A channel for right motor
#define RIGHT_ENCODER_B 3  // Encoder B channel for right motor

// Left Motor PID Gains
const float Kp_L = 0.5;
const float Ki_L = 0.05;
const float Kd_L = 0.0;

// Right Motor PID Gains
const float Kp_R = 0.5;
const float Ki_R = 0.05;
const float Kd_R = 0.0;


// ===== Global Variables =====
float thetaDeg       = 0.0;          // heading in degrees
#define PI 3.141592653589793
const float dt = 0.1;          // control interval in seconds (100 ms)
const float wheel_radius = 0.05;   // meters
const float wheel_base   = 0.385;  // meters
const int encoderCPR = 400;        // counts per revolution

volatile bool calculateRPMFlag = false;
volatile long Lcount = 0, Rcount = 0;
unsigned long prevLeftEncoder = 0, prevRightEncoder = 0;
long leftRPM = 0, rightRPM = 0;

// PID state
float errorL_prev = 0, integralL = 0;
float errorR_prev = 0, integralR = 0;

// Motion control
float v_goal = 0.0, v_actual = 0.0;
float w = 0.0;

// Debounce & motion logic
bool robotRunning = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
#define START_STOP_PIN 12
const int sensorInputPin1 = 31;
const int sensorInputPin2 = 32;
const int sensorInputPin3 = 33;
const int sensorInputPin4 = 34;
enum RobotMotion { MOTION_FORWARD, MOTION_BACKWARD, DEFAULT_STATE };
RobotMotion currentMotion = DEFAULT_STATE;

void setup() {
  Serial.begin(9600);
  // start/stop pin
  pinMode(START_STOP_PIN, INPUT_PULLUP);
  // sensors
  pinMode(sensorInputPin1, INPUT_PULLUP);
  pinMode(sensorInputPin2, INPUT_PULLUP);
  pinMode(sensorInputPin3, INPUT_PULLUP);
  pinMode(sensorInputPin4, INPUT_PULLUP);
  // encoders
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), LeftmotorISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), RightmotorISR, RISING);
  // motor pins
  pinMode(MOTOR1_RV, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR1_FR, OUTPUT);
  pinMode(MOTOR1_BK, OUTPUT);
  pinMode(MOTOR2_RV, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_FR, OUTPUT);
  pinMode(MOTOR2_BK, OUTPUT);
  // Timer1 for 100 ms interrupts
  cli();
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  OCR1A = 1562;                    // 100 ms @16MHz/1024
  TCCR1B |= (1<<WGM12) | (1<<CS12) | (1<<CS10);
  TIMSK1 |= (1<<OCIE1A);
  sei();
}

// Compute wheel RPM targets from v and w
void computeTargetRPMs(float v_goal, float w, float& l_target, float& r_target) {
  float v_l = v_goal - (w * wheel_base / 2.0);
  float v_r = v_goal + (w * wheel_base / 2.0);
  l_target = (v_l / (2*PI*wheel_radius))*60.0;
  r_target = (v_r / (2*PI*wheel_radius))*60.0;
}

void loop() {
  // —————— START/STOP BUTTON ——————
  static bool lastButtonState = HIGH;   // Previous stable state
  static bool stableState = HIGH;      // Debounced stable state
  static unsigned long lastBounceTime = 0;

  int currentButtonState = digitalRead(START_STOP_PIN);

  if (currentButtonState != lastButtonState) {
    // Button state changed; start debounce timer
    lastBounceTime = millis();
  }

  if ((millis() - lastBounceTime) > debounceDelay) {
    // If the state remains the same for the debounce period, consider it stable
    if (currentButtonState != stableState) {
      stableState = currentButtonState;

      if (stableState == LOW) {  // Button pressed
        robotRunning = !robotRunning;
        // Serial.print("Robot is now ");
        // Serial.println(robotRunning ? "RUNNING" : "STOPPED");

        // Stop motors immediately if robotRunning is false
        if (!robotRunning) {
          // Disable motor control signals
          digitalWrite(MOTOR1_EN, LOW);
          digitalWrite(MOTOR2_EN, LOW);
          digitalWrite(MOTOR1_BK, HIGH); // Apply brake
          digitalWrite(MOTOR2_BK, HIGH); // Apply brake
          v_goal = 0.0;  // Stop movement logic
          w = 0.0;
          v_actual = 0.0;
          // Reset PID state
          integralL = 0.0;
          integralR = 0.0;
          errorL_prev = 0.0;
          errorR_prev = 0.0;
        }
      }
    }
  }

  lastButtonState = currentButtonState;

  // ——— Stop motion logic if robotRunning is false ———
  if (!robotRunning) {
    return; // Skip the rest of the loop
  }



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
  
    if (currentMotion != prevMotion) {
      // Stop motors
      digitalWrite(MOTOR1_EN, LOW);
      digitalWrite(MOTOR2_EN, LOW);
      digitalWrite(MOTOR1_BK, HIGH);
      digitalWrite(MOTOR2_BK, HIGH);

      // Re-enable motors after pause
      digitalWrite(MOTOR1_BK, LOW);
      digitalWrite(MOTOR2_BK, LOW);
      digitalWrite(MOTOR1_EN, HIGH);
      digitalWrite(MOTOR2_EN, HIGH);
      v_actual = 0.0;
      // 2) clear PID state so it starts fresh
      integralL    = 0.0;
      integralR    = 0.0;
      errorL_prev  = 0.0;
      errorR_prev  = 0.0;
    }

    lastMotion = currentMotion;

    // act on currentMotion
    switch (currentMotion) {
      case MOTION_FORWARD:
        digitalWrite(MOTOR1_FR, HIGH);
        digitalWrite(MOTOR2_FR, LOW);
        v_goal = 0.19;  // Forward speed
        w = 0.0;   // No turning
        break;

      case MOTION_BACKWARD:
        digitalWrite(MOTOR1_FR, LOW);
        digitalWrite(MOTOR2_FR, HIGH);
        v_goal = 0.19;  // Reverse speed
        w = 0.0;
        break;

      default:
        v_goal = 0.0;
        w = 0.0;
        break;
    }
  // 3) PID block every 100 ms
    if(calculateRPMFlag) {
      calculateRPMFlag = false;

      float maxStep = 12.0;  //
      float stepL   = maxStep * Kp_L;  
      float stepR   = maxStep * Kp_R;
      // take the more conservative one so both sides ramp together
      float step    = min(stepL, stepR);

      if (v_actual < v_goal) {
        v_actual = min(v_actual + step, v_goal);
      } else if (v_actual > v_goal) {
        v_actual = max(v_actual - step, v_goal);
      }


      // read encoders
      long curL=Lcount, curR=Rcount;
      long dL = curL - prevLeftEncoder;
      long dR = curR - prevRightEncoder;
      prevLeftEncoder = curL; prevRightEncoder = curR;
      leftRPM  = ((100*dL)/encoderCPR)*6;
      rightRPM = ((100*dR)/encoderCPR)*6;
      Serial.print(leftRPM);
      Serial.print("  ");
      Serial.println(rightRPM);
    
      // targets
      float tgtL, tgtR;
      computeTargetRPMs(v_actual, w, tgtL, tgtR);
      // if (currentMotion == MOTION_BACKWARD) {
      // Serial.print("Target RPM (Left): ");
      // Serial.print(tgtL);
      // Serial.print(" | Actual RPM (Left): ");
      // Serial.print(leftRPM);
      // Serial.print(" || Target RPM (Right): ");
      // Serial.print(tgtR);
      // Serial.print(" | Actual RPM (Right): ");
      // Serial.println(rightRPM);
      // }


      // PID Left
      float eL = abs(tgtL) - abs(leftRPM);  // Use magnitude for error
      integralL += eL * dt;
      float dEL = (eL - errorL_prev) / dt;
      float outL = Kp_L * eL + Ki_L * integralL + Kd_L * dEL;
      errorL_prev = eL;
      int pwmL = constrain((int)abs(outL), 0, 255);  // Constrain output to 0-255

      // Set direction and apply PWM
      if (tgtL < 0) {  // Backward
        digitalWrite(MOTOR1_FR, LOW);
      } else {         // Forward
        digitalWrite(MOTOR1_FR, HIGH);
      }
      analogWrite(MOTOR1_RV, 255 - pwmL);

      // PID Right
      float eR = abs(tgtR) - abs(rightRPM);  // Use magnitude for error
      integralR += eR * dt;
      float dER = (eR - errorR_prev) / dt;
      float outR = Kp_R * eR + Ki_R * integralR + Kd_R * dER;
      errorR_prev = eR;
      int pwmR = constrain((int)abs(outR), 0, 255);  // Constrain output to 0-255

      // Set direction and apply PWM
      if (tgtR < 0) {  // Backward
        digitalWrite(MOTOR2_FR, HIGH);
      } else {         // Forward
        digitalWrite(MOTOR2_FR, LOW);
      }
      analogWrite(MOTOR2_RV, 255 - pwmR);



      // odometry
      float leftLin  = (leftRPM * (2*PI*wheel_radius))/60.0;
      float rightLin = (rightRPM* (2*PI*wheel_radius))/60.0;
      float omega = (rightLin - leftLin)/wheel_base;
      thetaDeg += (omega * dt)*180.0/PI;
    }
  } 
}

ISR(TIMER1_COMPA_vect) {
  calculateRPMFlag = true;
}

void LeftmotorISR() {
  Lcount += (digitalRead(LEFT_ENCODER_B)==LOW) ? 1 : -1;
}

void RightmotorISR() {
  Rcount += (digitalRead(RIGHT_ENCODER_B)==HIGH) ? 1 : -1;
}


