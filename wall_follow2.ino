// Left-wall-following robot with PID.
// - When switched on, it keeps 15 cm from the left wall and keeps that distance.
// - Gradual corners: the PID steering follows the curve.
// - Walls ahead: the robot slows down as it gets closer. At FRONT_LIMIT it turns right until
//   that wall is beside the left sensor (about as far away as it was in front) and the front is clear.
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

// PID gains (tune these)
const float Kp_side = 8;
const float Ki_side = 0;
const float Kd_side = 2;

// Variables to remember previous errors
float integral_side = 0;
float prevError_side = 0;
unsigned long lastTime = 0;
unsigned long lastPrint = 0;   // when we last printed to the Serial Monitor

const int set_point = 15;     // cm from the left wall

// Speeds and distances (tune these)
const int baseSpeed = 150;    // normal driving speed
const int minSpeed = 80;      // slowest speed when a wall is ahead
const int turnSpeed = 120;    // speed while turning right
const int SLOW_DIST = 40;     // cm: start slowing down
const int FRONT_LIMIT = 15;   // cm: stop going forward and turn right
const int TOLERANCE = 3;      // cm: how close "about as far away" has to be

const int distanceError = 0;      // cm: How off the ultrasonic Sensors are

void setup() {
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(frontTrig, OUTPUT); pinMode(frontEcho, INPUT);
  pinMode(leftTrig, OUTPUT);  pinMode(leftEcho, INPUT);
  Serial.begin(9600);
  lastTime = millis();
}

void loop() {
  float front = readDistance(frontTrig, frontEcho);

  // Wall ahead: turn right until the wall is beside the left sensor and the front is clear
  if (front <= FRONT_LIMIT) {
    float wallDist = front;   // how far the wall was in front
    setMotors(turnSpeed, -turnSpeed);   // spin right
    while (true) {
      float left = readDistance(leftTrig, leftEcho);
      front = readDistance(frontTrig, frontEcho);
      if (abs(left - wallDist) <= TOLERANCE && front > SLOW_DIST) break;
    }
    setMotors(0, 0);
    // Start PID fresh after the turn
    integral_side = 0;
    prevError_side = 0;
    lastTime = millis();
    return;
  }

  // Slow down as the front wall gets closer
  int speed = baseSpeed;
  if (front < SLOW_DIST) {
    speed = map(front, FRONT_LIMIT, SLOW_DIST, minSpeed, baseSpeed);
  }

  float left = readDistance(leftTrig, leftEcho);

  // Calculate time since the previous loop (in seconds)
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // PID
  float error = left - set_point;
  integral_side += error * dt;
  float derivative_side = (error - prevError_side) / dt;
  prevError_side = error;

  float pid = Kp_side * error + Ki_side * integral_side + Kd_side * derivative_side;

  // Too far from the wall (positive error) -> steer left
  int leftSpeed = constrain(speed - int(pid), -255, 255);
  int rightSpeed = constrain(speed + int(pid), -255, 255);
  setMotors(leftSpeed, rightSpeed);

  // Print values 4 times a second
  if (millis() - lastPrint >= 250) {
    Serial.print("front: ");  Serial.print(front, 1);
    Serial.print("  left: "); Serial.print(left, 1);
    Serial.print("  error: "); Serial.print(error, 1);
    Serial.print("  pid: ");  Serial.print(int(pid));
    Serial.print("  L: ");    Serial.print(leftSpeed);
    Serial.print("  R: ");    Serial.println(rightSpeed);
    lastPrint = millis();
  }
}

// Returns distance in cm
float readDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000);   // give up after 30 ms
  if (duration == 0) return 500;                // nothing heard = far away
  return duration * 0.0343 / 2 + distanceError;
}

// Positive = forward, negative = backward
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(in1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(in2, leftSpeed >= 0 ? LOW : HIGH);
  analogWrite(enA, abs(leftSpeed));

  digitalWrite(in3, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(in4, rightSpeed >= 0 ? LOW : HIGH);
  analogWrite(enB, abs(rightSpeed));
}
