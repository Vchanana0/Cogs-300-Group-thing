// LineFollower.ino
// Line-following robot using the 3 analog IR sensors underneath (A0 left, A1 middle, A2 right).
// Built from Motor.ino: same pins and motor functions, with the wall following taken out.
//
// The line reads LOWER than the floor on these sensors. A reading below LINE_THRESHOLD (halfway between the
// floor and line readings) = over the line.
// When the robot is centred, only the middle sensor is over the line.
//
// Every turn spins in place (one wheel forward, the other backward) and stops when the middle sensor finds
// the line. Timing is only a safety limit: a search that has spun about 160° without finding a line stops
// (the line it came along is right behind it at 180°), and when the line slips out from under the middle
// sensor on its own, it looks about 30° each way. Both are based on FULL_SPIN_MS, which you measure.
//
// Following the line uses the PID from the wall follower. The line's position under the robot (0 = under the
// middle sensor) is the PID's error. That position comes from a Bayes filter: it keeps a probability for each
// place the line could be, and on every PID update applies Bayes' theorem,
//   new belief = prior (last belief, blurred a little because the line may have moved) x likelihood (how well
//   the three readings fit a line at that place), divided by the total so it adds up to 1.
// The PID steers toward the average of the belief. Noise on one reading only nudges the belief, and when the
// line is between two sensors the belief keeps its last estimate instead of jumping.
// At junctions it follows the right-hand rule: take a way to the right if there is one, else go straight,
// else go left.
//
// Ways off the line (junctions, corners, sharp turns):
// A "way" is a side sensor going squarely onto a line while the middle sensor is still on the line
// (just touching the edge of the line is the robot wobbling, not a way).
// - A way right, found while running steadily along the line: once the robot is past it, if the middle is
//   still on a line (a crossing, or the line carries on straight too), spin right onto the new line.
// - If the line ends just after a way, it's a corner or sharp turn: spin toward that way.
//   Ways found on both sides at once mean a T-junction: spin right.
//
// When the line is lost:
// - A way was found just before (a corner, sharp turn or T-junction): spin toward it.
// - Otherwise, if a side sensor was among the last to see the line, spin toward that side (right first).
// - Otherwise, if it was part way round a turn, keep turning the same way.
// - Otherwise look about 30° each way, in case the line just slipped out from under the middle sensor.
// In each case it spins until the middle sensor finds the line. If there's no line within about 160°
// (or within the 30° look), the line has ended: that's the finish, so the robot stops.
//
// Before running it, set the numbers marked "Measure": FLOOR_READING, LINE_READING and FULL_SPIN_MS.
//
// The two ultrasonic sensors are still wired and set up, but the line follower doesn't use them.
// It starts on its own when powered, so it runs on a battery pack with no computer attached.

// Motor A (left motor)
int enA = 10;  // PWM speed control
int in1 = 3;   // Direction
int in2 = 2;   // Direction

// Motor B (right motor)
int enB = 9;   // PWM speed control
int in3 = 4;   // Direction
int in4 = 5;   // Direction

// Front ultrasonic sensor (HC-SR04), facing forward. Wired but not used by the line follower
int frontTrig = 7;
int frontEcho = 8;

// Left ultrasonic sensor (HC-SR04), facing left. Wired but not used by the line follower
int leftTrig = 11;
int leftEcho = 12;

// Left, middle and right IR sensors, read as analog values (0-1023)
const int leftIR_PIN = A0;
const int midIR_PIN = A1;
const int rigIR_PIN = A2;

// Line detection. Measure: hold the robot with the sensors over the floor, then over the middle of the line,
// and read the numbers in the Serial Monitor (115200)
const int FLOOR_READING = 1000;     // What the IR sensors read over the floor
const int LINE_READING = 45;      // What they read right over the middle of the line (lower than the floor)
const int LINE_THRESHOLD = (FLOOR_READING + LINE_READING) / 2;   // Below this = that sensor is over the line
const int SQUARE_THRESHOLD = LINE_READING + (FLOOR_READING - LINE_READING) / 4;
                                   // Below this = squarely over a line, not just its edge. A side sensor squarely
                                   // on a line while the middle is on the line means two lines: a junction

// Motor trim. Measure: no two motors are exactly alike. With Kp = 0, let it drive across the floor:
// if it curves left, the right motor is faster, so lower RIGHT_MOTOR_TRIM (0.95, 0.92, ...);
// if it curves right, lower LEFT_MOTOR_TRIM instead. Keep both at 1.0 or below
const float LEFT_MOTOR_TRIM = 1.0;
const float RIGHT_MOTOR_TRIM = 0.90;

// Speeds (0-255)
const int BASE_SPEED = 150;         // Forward speed on the line. Slower = more time to react at corners
const int TURN_SPEED = 150;         // Speed of both wheels when spinning in place
                                   // (raise it if the wheels hum but don't move, lower it if the robot
                                   // spins past the line)

// PID for following the line, from the wall follower (tune Kp first, then Kd. Ki is optional)
const float Kp = 8;
const float Ki = 0;
const float Kd = 0.02;              // Was 2.0 in the wall follower. The line position changes much faster than the
                                   // wall distance did, so 2.0 made the D term max out the steering on every
                                   // update and the robot lurched. 0.3 did best in testing
const float INTEGRAL_LIMIT = 50;   // Stops the I term from building up too far
const int MAX_CORRECTION = 120;     // Most the PID can add to one wheel and take from the other
const unsigned long PID_INTERVAL_MS = 2;  // How often the PID updates (ms). The wall follower's loop took tens of
                                           // ms because of the ultrasonic readings; the IR sensors are much faster,
                                           // and updating on every reading makes the D term jumpy
const float LINE_POSITION_RANGE = 10;     // The error the PID sees: 0 = line under the middle sensor,
                                           // +10 = under the left sensor only, -10 = under the right sensor only

// Bayes filter for the line position
const float SENSOR_SPACING_MM = 16.0;      // Measure: distance between the middle of neighbouring IR sensors (mm)
const float SENSOR_SEES_MM = 8.0;          // How far from the line (mm) a sensor still reads it about half way.
                                           // About 8 for 19 mm tape; scale it with the width of your line
const float READING_NOISE = 0.2;           // How much a reading wobbles, on the 0..1 scale of onLine()
const float LINE_DRIFT = 0.15;             // Chance the line moves one place left or right between PID updates
const float NEVER_RULED_OUT = 0.001;       // Small chance kept for every place, so the belief can always recover
const int NUM_PLACES = 13;                 // Places the line could be: -1.5 .. +1.5 sensor spacings, 0.25 apart
                                           // (-1 = under the left sensor, 0 = middle, +1 = under the right)

// Safety limits. Turns stop when the sensors find the line; these only stop a search that has gone too far
const unsigned long FULL_SPIN_MS = 1600;    // Measure: time to spin in place one full circle at TURN_SPEED (ms).
                                            // Set SPIN_TEST to true, upload, and adjust this until the robot
                                            // ends up facing exactly where it started
const bool SPIN_TEST = false;               // true = just spin for FULL_SPIN_MS and stop (for measuring it)
const unsigned long SEARCH_LIMIT_MS = FULL_SPIN_MS * 160 / 360;  // About 160°: the most it will spin looking
                                            // for a line. The line it came along is right behind it, at 180°,
                                            // so if there's no line by then, the line has ended. If the robot
                                            // turns round and drives back at the finish, lower the 160 to 150
const unsigned long LOOK_MS = FULL_SPIN_MS * 30 / 360;           // About 30°: how far it looks each way when
                                            // the line slips out from under the middle sensor on its own

// Reading the sensors. These only help tell what the sensors are seeing; they don't decide how far to turn
const int CONFIRM = 50;              // Readings in a row that must agree, so one bad reading doesn't count.
                                   // Keep it small (3-10): it counts readings, not time. Each reading is
                                   // ~0.35 ms while spinning and ~5 ms while printing, so 1000 meant seconds
                                   // of driving blind after losing the line, and a spin could never stay on
                                   // the line long enough to stop
const unsigned long LOST_MS = 80;           // No sensor seeing the line for this long (ms) = the line is lost.
                                            // Shorter gaps are the line slipping between two sensors; the
                                            // robot keeps steering from the Bayes filter's last belief.
                                            // Raise it if it still stops to look around on straight lines,
                                            // lower it if it runs far past corners before spinning
const unsigned long WAY_MEMORY_MS = 400;    // A way off the line seen this recently (ms) before the line ends
                                            // means a corner or sharp turn that way
const unsigned long TOGETHER_MS = 150;      // Ways found on both sides within this long (ms) of each other were
                                            // found together: a T-junction, so go right
const unsigned long STEADY_MS = 150;        // The robot only turns off the line at a way right if the middle sensor
                                            // had been on the line, with the right sensor not squarely on a line,
                                            // for this long (ms) before. Coming out of a turn at an angle, the line can lie
                                            // across two sensors at once and look like a junction

// Print the sensor readings to the Serial Monitor (115200) while tuning
const bool PRINT_READINGS = false;

// Which way to spin
const int SIDE_LEFT = 1;
const int SIDE_RIGHT = 0;

// Latest IR readings
int rawLeft = 0;
int rawMid = 0;
int rawRight = 0;
bool seeLeft = false;              // Over the line
bool seeMid = false;
bool seeRight = false;
bool squareLeft = false;           // Squarely over a line (after CONFIRM readings in a row agree)
bool squareRight = false;
int leftSquareCount = 0;           // Readings in a row disagreeing with squareLeft / squareRight
int rightSquareCount = 0;

// The reading before the latest one
bool prevLeft = false;
bool prevMid = false;
bool prevRight = false;
bool prevSquareLeft = false;
bool prevSquareRight = false;

// When the left / right sensor last saw a line (ms, 0 = not since the last turn). At a 90° corner the side
// sensor crosses the new line just before the line ends, often together with the middle, so "was it seen in
// the last WAY_MEMORY_MS" is more reliable than "was it among the very last readings"
unsigned long leftSeenAt = 0;
unsigned long rightSeenAt = 0;

unsigned long midOnSince = 0;      // When the middle sensor last came onto a line (ms)
unsigned long lastRightTime = 0;   // When the right sensor was last squarely over a line (ms), 0 if never

// A way right the robot may turn onto
bool rightBranch = false;          // The right sensor went squarely onto a line while running steadily along the line
unsigned long rightOnAt = 0;       // When the right sensor found it
unsigned long rightOffAt = 0;      // When the right sensor went past it with the middle still on a line
                                   // (0 = not yet)

// A "way" is a side sensor going squarely onto a line while the middle is already on the line: maybe a way
// off to that side. If the line ends soon after, the newest way says which way the line went
bool leftWay = false;              // The left / right sensor is on a way right now
bool rightWay = false;
unsigned long leftWayStart = 0;    // When the left / right sensor found its latest way (ms)
unsigned long rightWayStart = 0;
unsigned long leftWaySeen = 0;     // Last time the left / right sensor was on that way (ms, 0 = none)
unsigned long rightWaySeen = 0;

int turningSide = -1;              // The way it's turning (a corner, curve or junction) until it's running
                                   // steadily along the line again, -1 = not turning. If the line is lost
                                   // part way round, it keeps turning this way

// PID state
float integral = 0;
float prevError = 0;
unsigned long lastTime = 0;
bool pidJustReset = true;          // Skip the D term on the first reading after a reset or turn

// Bayes filter state
float placePosition[NUM_PLACES];          // Where each place is, in sensor spacings (-1 = left sensor)
float expectedOnLine[NUM_PLACES][3];      // What onLine() should read on the left, middle and right sensors
                                          // if the line is at that place
float belief[NUM_PLACES];                 // Probability the line is at each place

unsigned long lostSince = 0;       // When no sensor has seen the line since (ms), 0 = a sensor sees it
bool finished = false;             // Set at the finish, keeps the robot stopped


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

  // Ultrasonic sensors (still set up, just not used)
  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  digitalWrite(frontTrig, LOW);
  digitalWrite(leftTrig, LOW);

  // The IR sensors are read with analogRead, which needs no pinMode

  // Start stopped
  stop();

  // Serial is only for watching the numbers while tuning over USB.
  // The robot runs the same with nothing plugged in.
  Serial.begin(115200);

  // Two seconds to set the robot down on the line after switching it on
  delay(2000);

  // Measuring FULL_SPIN_MS: spin for that long, then stay stopped
  if (SPIN_TEST) {
    spin(SIDE_RIGHT);
    delay(FULL_SPIN_MS);
    brake();
    finished = true;
  }

  setupBayes();
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
  // Even out the two motors so equal speeds drive straight
  leftSpeed = constrain((int)(leftSpeed * LEFT_MOTOR_TRIM), -255, 255);
  rightSpeed = constrain((int)(rightSpeed * RIGHT_MOTOR_TRIM), -255, 255);

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

// Spin in place around the middle of the wheel axle: one wheel forward, the other backward
void spin(int side) {
  if (side == SIDE_RIGHT) {
    drive(TURN_SPEED, -TURN_SPEED);   // Left wheel forward, right wheel backward
  } else {
    drive(-TURN_SPEED, TURN_SPEED);   // Left wheel backward, right wheel forward
  }
}

int otherSide(int side) {
  return (side == SIDE_RIGHT) ? SIDE_LEFT : SIDE_RIGHT;
}


// -------------------------
// Ultrasonic sensors (not used by the line follower)
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


// -------------------------
// IR line sensors
// -------------------------

// Change `state` to `reading` only once CONFIRM readings in a row agree. `count` keeps track between calls
bool settle(bool reading, bool state, int &count) {
  if (reading == state) {
    count = 0;
    return state;
  }
  count++;
  if (count >= CONFIRM) {
    count = 0;
    return reading;
  }
  return state;
}

// Read all three IR sensors. The line reads lower than the floor
void readLineSensors() {
  bool midBefore = seeMid;

  rawLeft = analogRead(leftIR_PIN);
  rawMid = analogRead(midIR_PIN);
  rawRight = analogRead(rigIR_PIN);

  seeLeft = rawLeft < LINE_THRESHOLD;
  seeMid = rawMid < LINE_THRESHOLD;
  seeRight = rawRight < LINE_THRESHOLD;
  // Squarely on a line only counts once CONFIRM readings in a row agree, and so does coming off it,
  // so a reading that flickers around SQUARE_THRESHOLD doesn't look like a junction
  squareLeft = settle(rawLeft < SQUARE_THRESHOLD, squareLeft, leftSquareCount);
  squareRight = settle(rawRight < SQUARE_THRESHOLD, squareRight, rightSquareCount);

  if (seeMid && !midBefore) midOnSince = millis();
}

bool anySeesLine() {
  return seeLeft || seeMid || seeRight;
}

// Was `seenTime` within WAY_MEMORY_MS of `now`? (0 = never)
bool sawRecently(unsigned long seenTime, unsigned long now) {
  return seenTime != 0 && now - seenTime <= WAY_MEMORY_MS;
}

// Open the Serial Monitor at 115200 over USB to watch these while tuning
void printStatus(const char* action) {
  if (!PRINT_READINGS) return;

  Serial.print("L: ");
  Serial.print(rawLeft);
  Serial.print("  M: ");
  Serial.print(rawMid);
  Serial.print("  R: ");
  Serial.print(rawRight);
  Serial.print("  Sees: ");
  Serial.print(seeLeft ? "L" : "-");
  Serial.print(seeMid ? "M" : "-");
  Serial.print(seeRight ? "R" : "-");
  Serial.print("  ");
  Serial.println(action);
}


// -------------------------
// PID
// -------------------------

// The Bayes filter knows nothing yet: every place is equally likely
void resetBelief() {
  for (int k = 0; k < NUM_PLACES; k++) belief[k] = 1.0 / NUM_PLACES;
}

// Start the PID fresh: at startup and whenever a spin turn or search starts
void resetPID() {
  integral = 0;
  pidJustReset = true;
  lastTime = millis();
  resetBelief();   // After a turn the old belief is about the old line
}

// How much one sensor is over the line: 0 = floor, 1 = right over the middle of the line
float onLine(int raw) {
  float amount = (float)(FLOOR_READING - raw) / (FLOOR_READING - LINE_READING);
  return constrain(amount, 0.0, 1.0);
}

// Work out once what each sensor should read for each place the line could be.
// A sensor reads 1 right over the line, falling off smoothly (a bell curve) as the line moves away from it
void setupBayes() {
  for (int k = 0; k < NUM_PLACES; k++) {
    placePosition[k] = -1.5 + k * 0.25;
    for (int i = 0; i < 3; i++) {
      float awayMm = (placePosition[k] - (i - 1)) * SENSOR_SPACING_MM;   // Sensors at -1, 0, +1
      expectedOnLine[k][i] = exp(-awayMm * awayMm / (2 * SENSOR_SEES_MM * SENSOR_SEES_MM));
    }
  }
  resetBelief();
}

// Bayes' theorem with the latest readings: new belief = prior x likelihood, divided by the total
void updateBelief(float left, float mid, float right) {
  float reading[3] = {left, mid, right};

  // Prior: the last belief, blurred one place each way (the line may have moved since the last update)
  float prior[NUM_PLACES];
  for (int k = 0; k < NUM_PLACES; k++) {
    float before = belief[k > 0 ? k - 1 : k];
    float after = belief[k < NUM_PLACES - 1 ? k + 1 : k];
    prior[k] = (1 - 2 * LINE_DRIFT) * belief[k] + LINE_DRIFT * before + LINE_DRIFT * after + NEVER_RULED_OUT;
  }

  // Likelihood: how well the readings fit a line at each place (bell-curve noise on each reading),
  // kept as an exponent. Subtracting the smallest one stops exp() rounding every place down to 0
  float misfit[NUM_PLACES];
  float smallest = 1e30;
  for (int k = 0; k < NUM_PLACES; k++) {
    misfit[k] = 0;
    for (int i = 0; i < 3; i++) {
      float diff = reading[i] - expectedOnLine[k][i];
      misfit[k] += diff * diff / (2 * READING_NOISE * READING_NOISE);
    }
    if (misfit[k] < smallest) smallest = misfit[k];
  }

  float total = 0;
  for (int k = 0; k < NUM_PLACES; k++) {
    belief[k] = prior[k] * exp(-(misfit[k] - smallest));
    total += belief[k];
  }
  for (int k = 0; k < NUM_PLACES; k++) belief[k] /= total;
}

// Where the line is under the robot, from the Bayes filter:
// 0 = under the middle sensor, positive = to the left, negative = to the right (up to LINE_POSITION_RANGE)
float linePosition() {
  float left = onLine(rawLeft);
  float mid = onLine(rawMid);
  float right = onLine(rawRight);

  // A side sensor squarely on a line while the middle is also on the line is seeing a second line (a junction
  // or the start of a sharp turn), not where this line is. Leave it out so it doesn't pull the robot off course
  if (seeMid && squareLeft) left = 0;
  if (seeMid && squareRight) right = 0;

  updateBelief(left, mid, right);

  // The average of the belief: the best guess of where the line is
  float average = 0;
  for (int k = 0; k < NUM_PLACES; k++) average += belief[k] * placePosition[k];

  return -LINE_POSITION_RANGE * average;   // Places count left as negative; the PID wants left positive
}

// Steer to keep the line under the middle sensor, driving forward at BASE_SPEED.
// The same PID as the wall follower, with the line position as the error instead of the wall distance
void followLine() {
  unsigned long now = millis();

  // Keep the current wheel speeds until it's time for the next update
  if (!pidJustReset && now - lastTime < PID_INTERVAL_MS) return;

  // Time since the previous PID update (in seconds)
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // P: positive error = line is to the left, negative = to the right
  float error = linePosition();

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

  int correction = int(pid);

  // Positive pid steers left, toward the line: left wheel slower, right wheel faster
  int leftSpeed = constrain(BASE_SPEED - correction, -255, 255);
  int rightSpeed = constrain(BASE_SPEED + correction, -255, 255);
  drive(leftSpeed, rightSpeed);

  // Open the Serial Monitor at 115200 over USB to watch these while tuning
  if (PRINT_READINGS) {
    Serial.print("L: ");
    Serial.print(rawLeft);
    Serial.print("  M: ");
    Serial.print(rawMid);
    Serial.print("  R: ");
    Serial.print(rawRight);
    Serial.print("  Error: ");
    Serial.print(error);
    Serial.print("  PID: ");
    Serial.print(pid);
    Serial.print("  L: ");
    Serial.print(leftSpeed);
    Serial.print("  R: ");
    Serial.println(rightSpeed);
  }
}


// -------------------------
// Turning
// -------------------------

// Spin toward `side` until the middle sensor finds a line, then hand back to the main loop.
// If the middle sensor is on a line when the turn starts (turning off the line at a junction),
// it first spins until the middle sensor is off that line, so it doesn't stop on the line it's leaving.
// The side sensors are ignored here: one of them can swing back over the line the robot came along.
// Returns false if there's no line within `maxMs` (the safety limit).
bool spinOntoLine(int side, unsigned long maxMs) {
  Serial.println(side == SIDE_RIGHT ? "Spinning right onto the line" : "Spinning left onto the line");

  // Start fresh: anything seen before this turn was about the old line
  rightBranch = false;
  leftWay = false;
  rightWay = false;
  leftWaySeen = 0;
  rightWaySeen = 0;
  turningSide = side;
  resetPID();

  bool offOldLine = !seeMid;
  int count = 0;   // Readings in a row that agree
  unsigned long start = millis();

  spin(side);
  while (millis() - start < maxMs) {
    readLineSensors();

    if (!offOldLine) {
      // Get off the line it started on
      count = seeMid ? 0 : count + 1;
      if (count >= CONFIRM) {
        offOldLine = true;
        count = 0;
      }
    } else {
      // Find the new line
      count = seeMid ? count + 1 : 0;
      if (count >= CONFIRM) return true;
    }
  }
  return false;
}

// Spin toward `side` for up to `maxMs`, stopping as soon as any sensor sees a line.
// Returns true if it found one
bool spinUntilAnySensor(int side, unsigned long maxMs) {
  resetPID();
  unsigned long start = millis();
  spin(side);
  while (millis() - start < maxMs) {
    readLineSensors();
    if (anySeesLine()) return true;
  }
  return false;
}

// The line slipped out from under the middle sensor with nothing else seeing it: either it's just beside
// the robot (between two sensors) or it has ended. Look about 30° each way, starting on the side whose
// reading is closer to the line's (lower), coming back to the start after each look.
// Returns true if it found the line
bool lookEachWay() {
  Serial.println("Lost line: looking a little each way");
  int first = (rawRight <= rawLeft) ? SIDE_RIGHT : SIDE_LEFT;

  if (spinUntilAnySensor(first, LOOK_MS)) return true;
  if (spinUntilAnySensor(otherSide(first), LOOK_MS)) return true;        // Back to the start
  if (spinUntilAnySensor(otherSide(first), LOOK_MS)) return true;
  if (spinUntilAnySensor(first, LOOK_MS)) return true;                   // Back to the start
  return false;
}

// The line has ended: stop for good (until the robot is switched off and on again)
void finish() {
  brake();
  finished = true;
  Serial.println("Line ended: finish, stopped");
}

// Which side did the newest way off the line go, if one was seen just before now?
// Returns SIDE_LEFT, SIDE_RIGHT, or -1 for none
int newestWay(unsigned long now) {
  bool leftRecent = sawRecently(leftWaySeen, now);
  bool rightRecent = sawRecently(rightWaySeen, now);

  if (leftRecent && rightRecent) {
    unsigned long apart = (leftWayStart > rightWayStart) ? leftWayStart - rightWayStart
                                                         : rightWayStart - leftWayStart;
    if (apart <= TOGETHER_MS) return SIDE_RIGHT;   // Found together: a T-junction. Right-hand rule
    return (leftWayStart > rightWayStart) ? SIDE_LEFT : SIDE_RIGHT;
  }
  if (leftRecent) return SIDE_LEFT;
  if (rightRecent) return SIDE_RIGHT;
  return -1;
}

// No sensor sees the line any more
void lineLost() {
  unsigned long now = millis();
  int way = newestWay(now);
  int side = -1;

  if (way >= 0) {
    // A side sensor found a way off the line just before: a corner (90° or sharp) or a T-junction
    side = way;
  } else if (sawRecently(leftSeenAt, now) || sawRecently(rightSeenAt, now)) {
    // A side sensor saw the line just before it was lost: it went off that way (a 90° corner, or the line slid
    // out past that side on a curve). If both did, the one that saw it most recently. A tie goes right,
    // for the right-hand rule
    if (!sawRecently(leftSeenAt, now)) side = SIDE_RIGHT;
    else if (!sawRecently(rightSeenAt, now)) side = SIDE_LEFT;
    else side = (leftSeenAt > rightSeenAt) ? SIDE_LEFT : SIDE_RIGHT;
  } else if (turningSide >= 0) {
    // It was part way round a turn and ran off the line before it was lined up: keep turning the same way
    side = turningSide;
  }

  if (side >= 0) {
    // Spin that way until the middle sensor finds the line. Nothing within the safety limit: the line has ended
    if (!spinOntoLine(side, SEARCH_LIMIT_MS)) finish();
    return;
  }

  // Only the middle sensor saw it last, and it wasn't turning: the line has slipped out from under it, or ended
  if (lookEachWay()) return;
  finish();
}


// -------------------------
// Main loop
// -------------------------

void loop() {
  // Stay stopped after the finish
  if (finished) {
    stop();
    return;
  }

  prevLeft = seeLeft;
  prevMid = seeMid;
  prevRight = seeRight;
  prevSquareLeft = squareLeft;
  prevSquareRight = squareRight;
  unsigned long rightSeenBefore = lastRightTime;

  readLineSensors();
  unsigned long now = millis();

  // Remember what the sensors saw while the line is in view
  if (anySeesLine()) {
    lostSince = 0;
    if (seeLeft) leftSeenAt = now;
    if (seeRight) rightSeenAt = now;
    if (squareRight) lastRightTime = now;
  }

  // Running steadily along the line: the middle has been on it for a while, and the right sensor hasn't been
  // squarely over a line
  bool midSteady = prevMid && now - midOnSince >= STEADY_MS;
  bool rightQuiet = rightSeenBefore == 0 || now - rightSeenBefore >= STEADY_MS;

  // Once it's running steadily along the line again, any turn is over
  if (midSteady && seeMid) turningSide = -1;

  // The right sensor has just gone squarely onto a line while running steadily along the middle of the line:
  // a way right. (Just touching the edge of the line is the robot wobbling, not a junction)
  if (squareRight && !prevSquareRight && seeMid && midSteady && rightQuiet) {
    rightBranch = true;
    rightOnAt = now;
    rightOffAt = 0;
  }

  // A side sensor has just gone squarely onto a line while the middle is on the line: maybe a way off to that side.
  // Used if the line ends soon after (a corner or sharp turn)
  if (squareLeft && !prevSquareLeft && (seeMid || prevMid)) {
    leftWay = true;
    leftWayStart = now;
  }
  if (squareRight && !prevSquareRight && (seeMid || prevMid)) {
    rightWay = true;
    rightWayStart = now;
  }

  // Only the right sensor sees the line: it may just have drifted over there, so don't turn off the line yet.
  // (If the line ends here instead, lineLost() still turns right)
  if (seeRight && !seeMid && !seeLeft) {
    rightBranch = false;
  }

  // Note when each side sensor last saw a way off to its side
  if (leftWay) {
    if (seeLeft) {
      leftWaySeen = now;
    } else {
      leftWay = false;
    }
  }
  if (rightWay) {
    if (seeRight) {
      rightWaySeen = now;
    } else {
      rightWay = false;
    }
  }

  // The right sensor has gone past the way right (no longer squarely on it), and the middle is still on a line
  if (rightBranch && rightOffAt == 0 && prevSquareRight && !squareRight && seeMid) {
    rightOffAt = now;
  }

  // Wait until the robot is as far past the way right as the way was wide, then check the middle is still on a
  // line. That tells a crossing (line carries on) from a corner or sharp turn (line ends just after)
  bool pastRightWay = rightBranch && rightOffAt != 0 && seeMid && !squareRight
                      && now - rightOffAt >= rightOffAt - rightOnAt;

  if (pastRightWay) {
    // Right-hand rule: take the way right
    if (!spinOntoLine(SIDE_RIGHT, SEARCH_LIMIT_MS)) {
      // No line within the safety limit, so it wasn't a way right after all: spin back onto the line
      Serial.println("No way right here: turning back to the line");
      if (spinOntoLine(SIDE_LEFT, SEARCH_LIMIT_MS + LOOK_MS)) {
        // Back on the line. If it ends just ahead, it's a sharp right turn, so remember the way right
        turningSide = -1;
        rightWayStart = millis();
        rightWaySeen = rightWayStart;
      } else if (!spinOntoLine(SIDE_RIGHT, SEARCH_LIMIT_MS + LOOK_MS)) {
        // The line it was on isn't there either, so it was already at the tip of a sharp right turn:
        // it spins right again to find the new line. If there's nothing there either, the line has ended
        finish();
      }
    }

  } else if (anySeesLine()) {
    // On the line: the PID steers to keep it under the middle sensor
    followLine();

  } else {
    // No sensor sees the line. Usually it has just slipped between two sensors while wobbling, so for
    // LOST_MS keep steering toward where the Bayes filter last believed it was, and only then treat it as lost
    if (lostSince == 0) lostSince = now;

    if (now - lostSince < LOST_MS) {
      followLine();
    } else {
      printStatus("no line");
      lineLost();
      lostSince = 0;
    }
  }
}
