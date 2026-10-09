// Motor.ino

// Motor A (left motor)
int enA = 9;   // PWM speed control
int in1 = 5;   // Direction
int in2 = 4;   // Direction

// Motor B (right motor)
int enB = 10;  // PWM speed control
int in3 = 2;   // Direction
int in4 = 3;   // Direction


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
}

void forward(int speed) {
  motorAForward(speed);
  motorBForward(speed);
}

void backward(int speed) {
  motorABackward(speed);
  motorBBackward(speed);
}

// Spin in place: left motor backward, right motor forward
void turnLeft(int speed) {
  motorABackward(speed);
  motorBForward(speed);
}

// Spin in place: left motor forward, right motor backward
void turnRight(int speed) {
  motorAForward(speed);
  motorBBackward(speed);
}


// -------------------------
// Main loop (WASD control)
// -------------------------

int driveSpeed = 255;  // 0 (off) to 255 (full speed)

void loop() {
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
