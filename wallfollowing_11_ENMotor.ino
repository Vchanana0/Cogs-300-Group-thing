// Motor.ino
// Left-wall-following robot with PID.
// - When switched on, it measures how far the left wall is and keeps that distance.
// - Gradual corners: the PID steering follows the curve.
// - Walls ahead: the robot slows down as it gets closer. At FRONT_LIMIT it spins right until
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

// Wall-following settings
float set_point = 15;              // Distance to keep from the left wall (cm). Replaced at startup
                                   // by whatever distance the robot is placed at.
const float MIN_SET_POINT = 4;     // Closest the sensor can measure reliably (cm)
const float MAX_SET_POINT = 500;    // Farthest starting distance it will accept (cm)
const int BASE_SPEED = 80;        // Forward speed while following (0-255). Slower = more time to react
const int MAX_CORRECTION = 40;     // Most the PID can add to one wheel and take from the other

// Slowing down for a wall ahead
const int SLOW_DISTANCE = 100;      // Start slowing down when a wall ahead is closer than this (cm)
const int CRAWL_SPEED = 80;       // Speed just before the turn (raise it if the wheels hum but don't move)
const int TOO_CLOSE_TO_READ = 4;  // If the front sensor loses the echo after reading closer than this (cm),
                                   // the wall is too close to measure, so treat it as right in front

// PID gains (tune Kp first, then Kd. Ki is optional)
const float Kp = 10.0;
const float Ki = 0.0;
const float Kd = 2.5;
const float INTEGRAL_LIMIT = 50;   // Stops the I term from building up too far

// Corner settings
const int FRONT_LIMIT = 35;         // Wall ahead closer than this (cm) = inside corner, turn right
const int FRONT_CONFIRM = 2;       // Readings in a row needed before turning at an inside corner
const int TURN_SPEED = 170;        // Motor speed for turning in place

// Right turn at a wall ahead
const float TURN_MATCH = 10;       // Stop turning when the left sensor reads within this many cm of
                                   // how far away the wall was in front
const int TURN_SLOW_SPEED = 60;   // Spin speed for the end of the turn, once it sees the wall that was in front.
                                   // Lower = stops closer to 90 degrees. Raise it if the robot stops spinning

// PID state
float integral = 0;
float prevError = 0;
unsigned long lastTime = 0;
bool pidJustReset = true;          // Skip the D term on the first reading after a reset or turn

// Corner detection state
int frontCount = 0;                // Readings in a row with a wall close ahead
float lastFront = -1;              // Last good front reading (cm), -1 if none since the last turn
int frontMisses = 0;               // Front readings in a row with no echo


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

  // Ultrasonic sensors
  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  digitalWrite(frontTrig, LOW);
  digitalWrite(leftTrig, LOW);

  // Start stopped
  stop();

  // Serial is only for watching the numbers while tuning over USB.
  // The robot runs the same with nothing plugged in.
  Serial.begin(115200);

  // Two seconds to set the robot down next to the wall after switching it on
  delay(2000);

  // Measure how far the wall is right now, and keep that distance from here on
  captureSetPoint();

  resetPID();
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

// Stop the motors hard instead of letting them coast: both direction pins HIGH with the
// enable pin on shorts each motor through the driver, which stops it quickly
void brake() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  delay(60);
  stop();
}

// Set each wheel separately: positive = forward, negative = backward
void drive(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    motorAForward(leftSpeed);
  } else {
    motorABackward(-leftSpeed);
  }

  if (rightSpeed >= 0) {
    motorBForward(rightSpeed);
  } else {
    motorBBackward(-rightSpeed);
  }
}

// Turn right by driving only the left wheel forward. The right wheel stays still,
// so the robot pivots around it
void spinRight(int speed) {
  motorAForward(speed);

  // Right motor off
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}


// -------------------------
// Ultrasonic sensors
// -------------------------

// Returns the distance in cm seen by one sensor, or -1 if no echo came back
float readDistance(int trigPin, int echoPin) {
  // Send a 10 microsecond pulse to start a measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Time how long the echo pin stays high. Give up after 20 ms (about 3.4 m)
  unsigned long duration = pulseIn(echoPin, HIGH, 20000UL);
  if (duration == 0) return -1;

  // Sound travels 0.0343 cm per microsecond, and the pulse goes there and back
  return duration * 0.0343 / 2;
}

// Average 10 readings of the left sensor and use that as the distance to hold
void captureSetPoint() {
  float total = 0;
  int good = 0;

  for (int i = 0; i < 10; i++) {
    float d = readDistance(leftTrig, leftEcho);
    if (d > 0 && d < 100) {  // Skip missed echoes and stray far readings
      total += d;
      good++;
    }
    delay(30);
  }

  if (good > 0) {
    set_point = constrain(total / good, MIN_SET_POINT, MAX_SET_POINT);
  }
  // If the wall wasn't seen at all, set_point stays at its default of 15 cm

  Serial.print("Holding this distance from the wall: ");
  Serial.print(set_point);
  Serial.println(" cm");
}


// -------------------------
// PID
// -------------------------

// Start the PID fresh, used at startup and after every corner turn
void resetPID() {
  integral = 0;
  pidJustReset = true;
  frontCount = 0;
  lastFront = -1;
  frontMisses = 0;
  lastTime = millis();
}

// Forward speed for how close the wall ahead is.
// Like the Follow Me P-controller: full speed when far, slower the closer it gets.
int speedForFront(float frontDistance) {
  if (frontDistance < 0 || frontDistance >= SLOW_DISTANCE) {
    return BASE_SPEED;  // Nothing close ahead
  }

  // 0 at FRONT_LIMIT, 1 at SLOW_DISTANCE
  float fraction = (frontDistance - FRONT_LIMIT) / (SLOW_DISTANCE - FRONT_LIMIT);
  fraction = constrain(fraction, 0.0, 1.0);

  return CRAWL_SPEED + int(fraction * (BASE_SPEED - CRAWL_SPEED));
}

// Steer to hold set_point from the left wall, driving forward at the given speed
void followWall(float leftDistance, float frontDistance, int speed) {
  // Time since the previous PID update (in seconds)
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // P: positive error = too far from the wall, negative = too close
  float error = leftDistance - set_point;

  // I: add up the error over time
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // D: how quickly the error is changing
  float derivative = 0;
  if (!pidJustReset && dt > 0) {
    derivative = (error - prevError) / dt;
  }
  prevError = error;
  pidJustReset = false;

  float pid = Kp * error
            + Ki * integral
            + Kd * derivative;
  pid = constrain(pid, -MAX_CORRECTION, MAX_CORRECTION);

  // Shrink the steering along with the speed, so slowing down doesn't make it turn harder
  int correction = int(pid * speed / BASE_SPEED);

  // Positive pid steers left, toward the wall: left wheel slower, right wheel faster
  int leftSpeed = constrain(speed - correction, -255, 255);
  int rightSpeed = constrain(speed + correction, -255, 255);
  drive(leftSpeed, rightSpeed);

  // Open the Serial Monitor at 115200 over USB to watch these while tuning
  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print("  Left: ");
  Serial.print(leftDistance);
  Serial.print("  Error: ");
  Serial.print(error);
  Serial.print("  PID: ");
  Serial.print(pid);
  Serial.print("  L: ");
  Serial.print(leftSpeed);
  Serial.print("  R: ");
  Serial.println(rightSpeed);
}


// -------------------------
// Corners
// -------------------------

// Inside corner: a wall straight ahead.
// Spin right until the left sensor sees that wall and the front sensor is clear.
void insideCorner() {
  Serial.println("Inside corner: turning right");

  stop();
  delay(100);

  // How far away the wall ahead is. After the turn, the left sensor should see it about this far away.
  float wallDistance = readDistance(frontTrig, frontEcho);
  if (wallDistance < 0) wallDistance = FRONT_LIMIT;

  spinRight(TURN_SPEED);
  bool frontClear = false;
  bool onNewWall = false;   // Is the left sensor on the wall that was in front yet?
  float farthest = -1;      // Farthest left reading so far in this turn
  float lastGood = -1;      // Last left reading that got an echo
  float prevLeft = -1;      // The reading just before this one, on the same wall (-1 if none)
  int misses = 0;           // Left readings in a row with no echo

  // Keep turning until the sensors say it's done
  while (true) {
    // Nearly there once it sees the wall that was in front: slow down so it doesn't overshoot
    if (onNewWall) spinRight(TURN_SLOW_SPEED);

    // Check the front until it's clear. Once it is, it stays clear for the rest of the turn,
    // so after that only the left sensor is read, which checks twice as often
    if (!frontClear) {
      float front = readDistance(frontTrig, frontEcho);
      frontClear = front < 0 || front > FRONT_LIMIT;
      delay(20);
    }

    float left = readDistance(leftTrig, leftEcho);
    delay(20);

    Serial.print("Turning  front clear: ");
    Serial.print(frontClear);
    Serial.print("  left: ");
    Serial.println(left);

    if (left < 0) {
      misses++;
      if (misses >= 2) onNewWall = true;   // Lost sight of the old left wall
      prevLeft = -1;
      continue;
    }
    misses = 0;

    // While spinning right, the old left wall's reading grows smoothly. The wall that was in front
    // shows up as a sudden jump to a farther reading, or as the reading coming back down.
    if (lastGood > 0 && left > lastGood + TURN_MATCH) {
      onNewWall = true;
      prevLeft = -1;   // Different wall, so don't compare with the old one
    }
    if (left < farthest - 2) onNewWall = true;
    if (left > farthest) farthest = left;
    lastGood = left;

    bool leftOnWall = fabs(left - wallDistance) <= TURN_MATCH;   // About as far as it was in front

    // Stop once the reading stops getting smaller: that's when the sensor points straight at the wall
    bool stoppedGettingCloser = prevLeft > 0 && left >= prevLeft - 1;

    if (onNewWall && frontClear && leftOnWall && stoppedGettingCloser) break;

    prevLeft = left;
  }

  // Brake instead of coasting, so it doesn't keep spinning past the wall
  brake();
  delay(100);

  Serial.println("Turn done");

  resetPID();
}

// -------------------------
// Main loop
// -------------------------

void loop() {
  // Read one sensor at a time, with a short gap so they don't hear each other's echoes
  float front = readDistance(frontTrig, frontEcho);
  delay(10);
  float left = readDistance(leftTrig, leftEcho);
  delay(10);

  // When the front sensor misses an echo:
  if (front > 0) {
    lastFront = front;
    frontMisses = 0;
  } else {
    frontMisses++;

    if (lastFront > 0 && lastFront < TOO_CLOSE_TO_READ) {
      // Right up against a wall the sensor can't measure (under about 3 cm).
      // The last reading was already close, so the wall is still there.
      front = 1;
    } else if (lastFront > 0 && frontMisses <= 2) {
      // Probably just one missed echo: keep using the last reading so it doesn't speed back up
      front = lastFront;
    }
  }

  // Inside corner: a wall close ahead for a few readings in a row
  if (front > 0 && front < FRONT_LIMIT) {
    frontCount++;
  } else {
    frontCount = 0;
  }

  if (frontCount >= FRONT_CONFIRM) {
    insideCorner();
    return;
  }

  // No echo from the left sensor: keep driving as before until it gets a reading
  if (left < 0) {
    return;
  }

  // Slow down as a wall ahead gets closer
  followWall(left, front, speedForFront(front));
}
