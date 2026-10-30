// Left-wall-following robot with PID.
// - When switched on, it keeps 15 cm from the left wall and keeps that distance.
// - Gradual corners: the PID steering follows the curve (both wheels keep driving).
// - Walls ahead: the robot slows down as it gets closer. At FRONT_LIMIT it spins right
//   in place until that wall is beside the left sensor (about as far away as it was
//   in front) and the front is clear. No timed turns.
// - Outside corners: if the left wall disappears, it arcs left until it sees the wall again.
// It starts on its own when powered, so it runs on a battery pack with no computer attached.

// Motor A (left motor)
int enA = 10;  // PWM speed control
int in1 = 3;   // Direction
int in2 = 2;   // Direction

// Motor B (right motor)
int enB = 9;   // PWM speed control
int in3 = 4;   // Direction
int in4 = 5;   // Direction

// Front ultrasonic sensor (HC-SR04), facing forward
int frontTrig = 7;
int frontEcho = 8;

// Left ultrasonic sensor (HC-SR04), facing left toward the wall
int leftTrig = 11;
int leftEcho = 12;

// Distances in cm (placeholders, tune these)
const float SET_POINT = 15;     // distance to keep from the left wall
const float FRONT_LIMIT = 15;   // spin right when the front wall is this close
const float SLOW_START = 40;    // start slowing down when the front wall is this close
const float LOST_WALL = 50;     // left reading above this means the wall disappeared
const float TOLERANCE = 7;      // how close "about as far away" has to be

// Speeds 0-255 (placeholders, tune these)
const int BASE_SPEED = 150;     // normal driving speed
const int MIN_SPEED = 80;       // slowest speed while approaching a front wall
const int TURN_SPEED = 150;     // spin speed for sharp right turns
const int ARC_LEFT_SPEED = 60;  // left wheel during the outside-corner arc
const int ARC_RIGHT_SPEED = 160;// right wheel during the outside-corner arc

// PID gains (starter values, tune these)
const float Kp = 8;
const float Ki = 0.2;
const float Kd = 2;

// Variables to remember between loops
float integral = 0;
float prevError = 0;
unsigned long lastTime = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);

  lastTime = millis();
}

void loop() {
  float front = readDistance(frontTrig, frontEcho);
  float left = readDistance(leftTrig, leftEcho);

  // Wall ahead is too close: spin right until that wall is beside us
  if (front <= FRONT_LIMIT) {
    float target = front;  // remember how far the wall was in front
    setMotors(TURN_SPEED, -TURN_SPEED);
    while (!(abs(left - target) <= TOLERANCE && front > FRONT_LIMIT)) {
      front = readDistance(frontTrig, frontEcho);
      left = readDistance(leftTrig, leftEcho);
    }
    resetPID();
    return;
  }

  // Left wall disappeared (outside corner): arc left until we see it again
  if (left > LOST_WALL) {
    setMotors(ARC_LEFT_SPEED, ARC_RIGHT_SPEED);
    while (left > LOST_WALL) {
      left = readDistance(leftTrig, leftEcho);
    }
    resetPID();
    return;
  }

  // Slow down as the front wall gets closer
  int baseSpeed = BASE_SPEED;
  if (front < SLOW_START) {
    baseSpeed = map(front, FRONT_LIMIT, SLOW_START, MIN_SPEED, BASE_SPEED);
  }

  // Calculate time since the previous loop (in seconds)
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // PID
  float error = left - SET_POINT;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  prevError = error;

  float pid = Kp * error
              + Ki * integral
              + Kd * derivative;

  int leftSpeed = constrain(baseSpeed - int(pid), -255, 255);
  int rightSpeed = constrain(baseSpeed + int(pid), -255, 255);
  setMotors(leftSpeed, rightSpeed);
}

// Read an HC-SR04 and return the distance in cm
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // give up after 30 ms
  delay(20);  // let the echo die out so the two sensors don't hear each other

  if (duration == 0) {
    return 400;  // no echo: treat as nothing in range
  }
  return duration * 0.0343 / 2;
}

// Positive speed = forward, negative speed = backward
void setMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA, abs(leftSpeed));

  if (rightSpeed >= 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  analogWrite(enB, abs(rightSpeed));
}

// Start PID fresh after a turn so old errors don't cause a jerk
void resetPID() {
  integral = 0;
  prevError = 0;
  lastTime = millis();
}
