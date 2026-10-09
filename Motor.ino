// Motor.ino



// Motor A (left motor)
int enA = 9;   // PWM speed control
int in1 = 5;   // Direction
int in2 = 4;   // Direction

// Motor B (right motor)
int enB = 10;  // PWM speed control
int in3 = 2;   // Direction
int in4 = 3;   // Direction


// Blinker LEDs
int leftBlinker = 11;
int rightBlinker = 6;

// Blinker state
const int BLINK_OFF = 0;
const int BLINK_LEFT = 1;
const int BLINK_RIGHT = 2;
const int BOTH_ON = 3;  // Both LEDs solid (reversing)

int blinkMode = BLINK_OFF;                 // Which blinker is active
bool blinkOn = false;                      // Is the active LED currently lit
unsigned long lastBlinkTime = 0;           // When the LED last toggled
const unsigned long BLINK_INTERVAL = 300;  // Milliseconds between toggles


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

  // Start stopped
  stop();

  // Serial connection for keyboard control
  Serial.begin(9600);
  Serial.println("WASD control ready");
  Serial.println("w = forward, s = backward, a = left, d = right, x or space = stop");
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
  motorABackward(0);
  motorBForward(speed);
  setBlinker(BLINK_LEFT);
}

// Spin in place: left motor forward, right motor backward
void turnRight(int speed) {
  motorAForward(speed);
  motorBBackward(0);
  setBlinker(BLINK_RIGHT);
}


// -------------------------
// Main loop (WASD control)
// -------------------------

int driveSpeed = 200;  // 0 (off) to 255 (full speed)

void loop() {
  // Keep the active blinker flashing
  updateBlinkers();

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
