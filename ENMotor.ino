// Motor.ino

// Motor A (left motor)
int enA = 9;   // PWM speed control
int in1 = 5;   // Direction
int in2 = 4;   // Direction

// Motor B (right motor)
int enB = 10;  // PWM speed control
int in3 = 2;   // Direction
int in4 = 3;   // Direction

// Blinker LEDs (moved from pins 2 and 3, which the speed sensors need)
int leftBlinker = 11;
int rightBlinker = 6;

// IR speed sensors (the D0 pin of each module)
// These must be on pins 2 and 3, the interrupt pins on an Arduino Uno
int leftSensor = 13;   // Left wheel
int rightSensor = 12;  // Right wheel

// Blinker state
const int BLINK_OFF = 0;
const int BLINK_LEFT = 1;
const int BLINK_RIGHT = 2;
const int BOTH_ON = 3;  // Both LEDs solid (reversing)

int blinkMode = BLINK_OFF;                 // Which blinker is active
bool blinkOn = false;                      // Is the active LED currently lit
unsigned long lastBlinkTime = 0;           // When the LED last toggled
const unsigned long BLINK_INTERVAL = 300;  // Milliseconds between toggles

// Curving: the inside wheel runs at this percentage of the outside wheel's speed.
// Lower = tighter curve, higher = gentler curve.
const int CURVE_PERCENT = 50;

// Speed sensor settings
const int SLOTS_PER_TURN = 20;                 // Slots in each encoder disc
const unsigned long SPEED_INTERVAL = 500;      // Milliseconds between speed reports
const unsigned long SENSOR_DEBOUNCE = 2000;    // Microseconds to ignore flicker after an edge

// Speed sensor state (volatile because the interrupt functions change it)
volatile unsigned long leftPulses = 0;         // Slots counted since the last report
volatile unsigned long rightPulses = 0;
volatile unsigned long leftLastChange = 0;     // When each sensor last changed (micros)
volatile unsigned long rightLastChange = 0;
volatile bool leftSensorState = false;         // Last accepted reading of each sensor
volatile bool rightSensorState = false;

unsigned long lastSpeedTime = 0;               // When the last speed report was sent


// -------------------------
// Setup
// -------------------------

void setup() {
  // Motor A
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Motor B
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Blinkers
  pinMode(leftBlinker, OUTPUT);
  pinMode(rightBlinker, OUTPUT);

  // Speed sensors: run a function every time a sensor's output changes
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);
  leftSensorState = digitalRead(leftSensor);
  rightSensorState = digitalRead(rightSensor);
  attachInterrupt(digitalPinToInterrupt(leftSensor), leftSensorChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightSensor), rightSensorChanged, CHANGE);

  // Start stopped
  stop();

  // Serial connection for keyboard control
  Serial.begin(9600);
  Serial.println("WASD control ready");
  Serial.println("w = forward, s = backward, a = left, d = right, r = rotate");
  Serial.println("q = curve left, e = curve right, x or space = stop");
}


// -------------------------
// Motor A
// -------------------------

void motorAForward(int speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed);
}

void motorABackward(int speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, speed);
}


// -------------------------
// Motor B
// -------------------------

void motorBForward(int speed) {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, speed);
}

void motorBBackward(int speed) {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, speed);
}


// -------------------------
// Blinkers
// -------------------------

// Set the lights: BLINK_OFF, BLINK_LEFT, BLINK_RIGHT or BOTH_ON
void setBlinker(int mode) {
  if (mode == blinkMode) return;

  blinkMode = mode;
  blinkOn = true;  // Light up right away when a turn starts
  lastBlinkTime = millis();

  bool leftLit = (mode == BLINK_LEFT || mode == BOTH_ON);
  bool rightLit = (mode == BLINK_RIGHT || mode == BOTH_ON);
  digitalWrite(leftBlinker, leftLit ? HIGH : LOW);
  digitalWrite(rightBlinker, rightLit ? HIGH : LOW);
}

// Call this every loop so the active blinker keeps flashing
void updateBlinkers() {
  // Only the turn modes flash; off and both-on stay as they are
  if (blinkMode != BLINK_LEFT && blinkMode != BLINK_RIGHT) return;

  if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = millis();
    blinkOn = !blinkOn;

    int pin = (blinkMode == BLINK_LEFT) ? leftBlinker : rightBlinker;
    digitalWrite(pin, blinkOn ? HIGH : LOW);
  }
}


// -------------------------
// Speed sensors
// -------------------------

// Runs automatically whenever the left sensor's output changes
void leftSensorChanged() {
  unsigned long now = micros();
  if (now - leftLastChange < SENSOR_DEBOUNCE) return;  // Ignore flicker right after an edge

  bool state = digitalRead(leftSensor);
  if (state != leftSensorState) {
    leftSensorState = state;
    leftLastChange = now;
    if (state) leftPulses++;  // Count once per slot
  }
}

// Runs automatically whenever the right sensor's output changes
void rightSensorChanged() {
  unsigned long now = micros();
  if (now - rightLastChange < SENSOR_DEBOUNCE) return;  // Ignore flicker right after an edge

  bool state = digitalRead(rightSensor);
  if (state != rightSensorState) {
    rightSensorState = state;
    rightLastChange = now;
    if (state) rightPulses++;  // Count once per slot
  }
}

// Call this every loop. Every SPEED_INTERVAL it sends "SPEED <left RPM> <right RPM>"
void reportSpeed() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastSpeedTime;
  if (elapsed < SPEED_INTERVAL) return;
  lastSpeedTime = now;

  // Copy and reset the counts with interrupts paused so they can't change mid-read
  noInterrupts();
  unsigned long left = leftPulses;
  unsigned long right = rightPulses;
  leftPulses = 0;
  rightPulses = 0;
  interrupts();

  // pulses / slots = turns, and turns per elapsed time scaled up to one minute = RPM
  unsigned long leftRPM = left * 60000UL / (SLOTS_PER_TURN * elapsed);
  unsigned long rightRPM = right * 60000UL / (SLOTS_PER_TURN * elapsed);

  Serial.print("SPEED ");
  Serial.print(leftRPM);
  Serial.print(" ");
  Serial.println(rightRPM);
}


// -------------------------
// Both motors
// -------------------------

// Cut power to both motors (they coast to a stop)
void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  setBlinker(BLINK_OFF);
}

void forward(int speed) {
  motorAForward(speed);
  motorBForward(speed);
  setBlinker(BLINK_OFF);
}

void backward(int speed) {
  motorABackward(speed);
  motorBBackward(speed);
  setBlinker(BOTH_ON);
}

// Spin in place: left motor backward, right motor forward
void turnLeft(int speed) {
  motorABackward(speed);
  motorBForward(speed);
  setBlinker(BLINK_LEFT);
}

// Spin in place: left motor forward, right motor backward
void turnRight(int speed) {
  motorAForward(speed);
  motorBBackward(speed);
  setBlinker(BLINK_RIGHT);
}

// Drive forward while curving left: left motor slowed, right motor full speed
void curveLeft(int speed) {
  motorAForward(speed * CURVE_PERCENT / 100);
  motorBForward(speed);
  setBlinker(BLINK_LEFT);
}

// Drive forward while curving right: left motor full speed, right motor slowed
void curveRight(int speed) {
  motorAForward(speed);
  motorBForward(speed * CURVE_PERCENT / 100);
  setBlinker(BLINK_RIGHT);
}

// Rotate on the spot, clockwise: left motor forward, right motor backward
void rotate(int speed) {
  motorAForward(speed);
  motorBBackward(speed);
  setBlinker(BLINK_RIGHT);
}


// -------------------------
// Main loop (WASD control)
// -------------------------

int driveSpeed = 200;  // 0 (off) to 255 (full speed)

void loop() {
  // Keep the active blinker flashing
  updateBlinkers();

  // Send the wheel speeds to the computer
  reportSpeed();

  // Only act when a key has arrived over serial
  if (Serial.available() > 0) {
    char key = Serial.read();

    switch (key) {
      case 'w':
      case 'W':
        forward(driveSpeed);
        Serial.println("Forward");
        break;

      case 's':
      case 'S':
        backward(driveSpeed);
        Serial.println("Backward");
        break;

      case 'a':
      case 'A':
        turnLeft(driveSpeed);
        Serial.println("Turn left");
        break;

      case 'd':
      case 'D':
        turnRight(driveSpeed);
        Serial.println("Turn right");
        break;

      // Sent by the Processing sketch when W and A are held together
      case 'q':
      case 'Q':
        curveLeft(driveSpeed);
        Serial.println("Curve left");
        break;

      // Sent by the Processing sketch when W and D are held together
      case 'e':
      case 'E':
        curveRight(driveSpeed);
        Serial.println("Curve right");
        break;

      case 'r':
      case 'R':
        rotate(driveSpeed);
        Serial.println("Rotate");
        break;

      case 'x':
      case 'X':
      case ' ':
        stop();
        Serial.println("Stop");
        break;

      default:
        // Ignore anything else (including the newline the Serial Monitor sends)
        break;
    }
  }
}
