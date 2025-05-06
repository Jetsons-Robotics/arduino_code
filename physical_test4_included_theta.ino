// ===== Motor Driver Pin Definitions =====
// Left Motor (Motor 1)
#define MOTOR1_RV 8    // PWM speed control
#define MOTOR1_EN 10   // Motor enable
#define MOTOR1_FR 11    // Direction control: LOW = forward
#define MOTOR1_BK 6    // Brake control

// Right Motor (Motor 2)
#define MOTOR2_RV 7    // PWM speed control
#define MOTOR2_EN 13   // Motor enable
#define MOTOR2_FR 5   // Direction control: LOW = forward
#define MOTOR2_BK 4    // Brake control

// ===== Encoder Pin Definitions =====
// Left Motor Encoder
#define LEFT_ENCODER_A 18  // Encoder A channel for left motor
#define LEFT_ENCODER_B 2   // Encoder B channel for left motor

// Right Motor Encoder
#define RIGHT_ENCODER_A 19 // Encoder A channel for right motor
#define RIGHT_ENCODER_B 3  // Encoder B channel for right motor

// ===== Global Variables =====

#define PI 3.141592653589793
float v = 0.0;
float w = 0.0;
float thetaDeg       = 0.0;          // heading in degrees
const float dt       = 0.1;          // time interval in seconds (100 ms)
// ——— Robot physical constants 
const float wheel_radius = 0.05;   // meters
const float wheel_base   = 0.385;  // meters


volatile unsigned long zeroCount     = 0;   // how many times rpmError was zero
volatile unsigned long totalCycles   = 0;   // total rpmError samples

volatile bool calculateRPMFlag = false;  // Flag set by Timer1 ISR every 50ms
volatile unsigned long Lcount = 0;  // Left encoder tick count
volatile unsigned long Rcount = 0;  // Right encoder tick count
long desired_speed_motor1 = 255;  // Left motor
long desired_speed_motor2 = 255;  // Right motor

unsigned long prevLeftEncoder = 0;
unsigned long prevRightEncoder = 0;
long leftRPM = 0;
long rightRPM = 0;
const int encoderCPR = 400;  // Encoder counts per revolution

// Debounce variables
bool robotRunning = false;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
#define START_STOP_PIN 12
const int sensorInputPin1 = 31;
const int sensorInputPin2 = 32;
const int sensorInputPin3 = 33;
const int sensorInputPin4 = 34;
enum RobotMotion { MOTION_FORWARD, MOTION_STOP, MOTION_BACKWARD, DEFAULT_STATE };
RobotMotion currentMotion = DEFAULT_STATE;

void setup() {
  Serial.begin(9600);
  pinMode(START_STOP_PIN, INPUT_PULLUP);
  pinMode(sensorInputPin1, INPUT_PULLUP);
  pinMode(sensorInputPin2, INPUT_PULLUP);
  pinMode(sensorInputPin3, INPUT_PULLUP);
  pinMode(sensorInputPin4, INPUT_PULLUP);

  // Set up encoder pins as inputs with pullup resistors
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  // Attach interrupts to the encoder A channels
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), LeftmotorISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), RightmotorISR, RISING);

  // Initialize left motor control pins as outputs
  pinMode(MOTOR1_RV, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR1_FR, OUTPUT);
  pinMode(MOTOR1_BK, OUTPUT);

  // Initialize right motor control pins as outputs
  pinMode(MOTOR2_RV, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_FR, OUTPUT);
  pinMode(MOTOR2_BK, OUTPUT);

  cli(); // Disable global interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 781;                        // Compare match value for 100ms at 16MHz with 1024 prescaler
  TCCR1B |= (1 << WGM12);            // Enable CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set prescaler to 1024
  TIMSK1 |= (1 << OCIE1A);           // Enable Timer1 compare interrupt

  sei(); // Enable global interrupts

}

void computeTargetRPMs(float v, float w, float& leftRPM, float& rightRPM) {

  // Compute linear velocity of each wheel (m/s)
  float left_wheel_velocity = v - (w * wheel_base / 2.0);
  float right_wheel_velocity = v + (w * wheel_base / 2.0);

  // Convert linear velocity (m/s) to RPM
  leftRPM = (left_wheel_velocity / (2 * PI * wheel_radius)) * 60.0;
  rightRPM = (right_wheel_velocity / (2 * PI * wheel_radius)) * 60.0;
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
          
          // report zero-error percentage
          float pct = totalCycles > 0
            ? (100.0 * zeroCount) / totalCycles
            : 0.0;
          // Serial.print(" ");
          // Serial.println(pct);

          // reset counters for next run
          totalCycles = 0;
          zeroCount   = 0;

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
    }

    lastMotion = currentMotion;

    // act on currentMotion
    switch (currentMotion) {
      case MOTION_FORWARD:
        digitalWrite(MOTOR1_FR, HIGH);
        digitalWrite(MOTOR2_FR, LOW);
        v = 0.18;  // Forward speed
        w = 0.0;   // No turning
        break;

      case MOTION_BACKWARD:
        digitalWrite(MOTOR1_FR, LOW);
        digitalWrite(MOTOR2_FR, HIGH);
        v = 0.18;  // Reverse speed
        w = 0.0;
        break;

      default:
        v = 0.0;
        w = 0.0;
        break;
    }
    float leftRPM_target  = 0, rightRPM_target = 0;
    computeTargetRPMs(v, w, leftRPM_target, rightRPM_target);


    desired_speed_motor1  = map(constrain(abs(leftRPM_target),  0, 34.5), 0, 34.5, 0, 255);
    desired_speed_motor2 = map(constrain(abs(rightRPM_target), 0, 34.5), 0, 34.5, 0, 255);

    analogWrite(MOTOR1_RV, 255 - desired_speed_motor1);
    analogWrite(MOTOR2_RV, 255 - desired_speed_motor2);
    if (calculateRPMFlag)
    {
    calculateRPMFlag = false;
    calculateMotorRPM();

    // 1) compute each wheel’s linear velocity (m/s)
    float leftLin  = (leftRPM  * (2 * PI * wheel_radius)) / 60.0;
    float rightLin = (rightRPM * (2 * PI * wheel_radius)) / 60.0;

    // 2) robot’s forward velocity & angular rate (rad/s)
    float robotVel   = (leftLin + rightLin) / 2.0;
    float robotOmega = (rightLin - leftLin) / wheel_base;

    leftSpeedError = leftSpeedTarget - leftSpeedActual;
    rightSpeedError = rightSpeedTarget - rightSpeedActual;
    
    //PID Control
    leftErrorSum += leftSpeedError;
    rightErrorSum += rightSpeedError;

    speedSetLeft = KpLeft * leftSpeedError + KiLeft * leftErrorSum + KdLeft * (leftSpeedError - leftErrorPrevious);
    speedSetRight = KpRight * rightSpeedError + KiRight * rightErrorSum + KdRight * (rightSpeedError - rightErrorPrevious);

    leftErrorPrevious = leftSpeedError;
    rightErrorPrevious = rightSpeedError;
    /* PID implementation Ends */



    // 4) report
    // Serial.print(" θ="); 
    // Serial.print(thetaDeg, 1); 
    }
  }
}

ISR(TIMER1_COMPA_vect) {
  calculateRPMFlag = true;
}

void calculateMotorRPM() {

  // Calculate Left Motor RPM
  unsigned long currentLeftEncoder = Lcount;
  unsigned long deltaLeft = currentLeftEncoder - prevLeftEncoder;
  leftRPM = ((100 * deltaLeft) / encoderCPR) * 12;  // 600/100
  prevLeftEncoder = currentLeftEncoder;

  // Calculate Right Motor RPM
  unsigned long currentRightEncoder = Rcount;
  unsigned long deltaRight = currentRightEncoder - prevRightEncoder;
  rightRPM = ((100 * deltaRight) / encoderCPR) * 12;
  prevRightEncoder = currentRightEncoder;


  // Display RPM values on the Serial Monitor
  Serial.print(leftRPM);
  Serial.print("  ");
  Serial.print(rightRPM);
  // // ===== Improved RPM balancing logic =====

  int rpmError = leftRPM - rightRPM;

  // update counters
  totalCycles++;
  if (rpmError == 0) zeroCount++;

  // then your existing prints
  Serial.print("  ");
  Serial.println(rpmError);
     

  // Apply a simple proportional correction
  int correction = rpmError * 0.5;  // adjust '1' to a lower value (like 0.5) for smoother changes

  desired_speed_motor1 -= correction;
  desired_speed_motor2 += correction;

  // Constrain values to safe PWM range
  desired_speed_motor1 = constrain(desired_speed_motor1, 0, 255);
  desired_speed_motor2 = constrain(desired_speed_motor2, 0, 255);
}

void LeftmotorISR() { 
  Lcount++; 
}

void RightmotorISR() { 
  Rcount++; 
}
