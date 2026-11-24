// ObjectFinder.ino
// Scans all the way round, picks the WIDEST object, drives to it and stops facing it with the front
// ultrasonic sensor about STOP_DISTANCE_CM (5 cm) from it. Stays inside the white lines on the way.
//
// HOW IT WORKS
// 1. SCAN. The servo sweeps the front ultrasonic sensor from the robot's left to its right (180°), taking a
//    reading every SCAN_STEP_DEG. The servo only covers the front half, so the robot then spins in place
//    (angle counted by the wheel speed sensors) and sweeps again, SWEEPS times in all, to cover 360°.
//    That is what lets it find the object whichever way it's facing when it starts.
// 2. FIND THE THINGS IN EACH SWEEP. Readings next to each other that are about the same distance
//    (within EDGE_JUMP_CM) belong to the same thing. For each thing it works out:
//      - the angle to its middle (this is "the angle at which the object lies"),
//      - its distance (the nearest reading on it),
//      - its real width in cm = 2 x distance x tan(half the angle it covers). Using cm, not degrees, is what
//        stops a small thing close up from beating a big thing far away.
//    Walls: anything wider than MAX_OBJECT_WIDTH_CM is a wall and is ignored, and so is anything farther than
//    MAX_RANGE_CM. Everything it finds is printed to the Serial Monitor (115200) so you can tune these.
// 3. TURN to face the widest thing (spin in place, counted by the speed sensors).
// 4. APPROACH. Servo straight ahead, drive straight, reading the front sensor all the way. It slows down close
//    up and brakes at STOP_DISTANCE_CM, then nudges forward / backward until it reads about 5 cm.
//    If it loses the object on the way (aimed a bit off), it stops, does a small sweep (REACQUIRE_DEG each
//    side), turns to the thing nearest straight ahead and carries on.
// 5. WHITE LINE. While driving forward the 3 IR sensors watch for the white boundary line. If they see it, the
//    robot brakes, backs up BACKUP_CM and scans again, this time ignoring whatever is past that line.
// When it's standing at the object the "L" LED comes on and it stays stopped until it's switched off and on.
//
// THE SERVO AND THE MOTORS
// This sketch does NOT use the Servo library. On an Uno/Nano the Servo library takes over Timer1, which
// switches off analogWrite on pins 9 and 10, and those are this robot's two motor speed pins (enA, enB).
// Instead the servo pulses are sent by hand (servoPulse), which needs no timer.
//
// BEFORE RUNNING IT
// - Measure SENSOR_AHEAD_CM (below).
// - Set SERVO_TEST to true once and check the sensor points left, straight ahead, right, in that order.
// - Set SCAN_TEST to true to watch what it finds without it driving anywhere, and tune MAX_OBJECT_WIDTH_CM,
//   MAX_RANGE_CM and BEAM_WIDTH_DEG from what it prints.
// - Start it INSIDE the arena, not sitting on a white line.

// Motor A (left motor)
const int enA = 10;  // PWM speed control
const int in1 = 4;   // Direction
const int in2 = 5;   // Direction

// Motor B (right motor)
const int enB = 9;   // PWM speed control
const int in3 = A3;  // Direction
const int in4 = A4;  // Direction

// Front ultrasonic sensor (HC-SR04), mounted on the servo
const int frontTrig = 7;
const int frontEcho = 8;

// Left ultrasonic sensor (HC-SR04). Not used in this lab.
const int leftTrig = 11;
const int leftEcho = 12;

// IR sensors (0-1023)
const int leftIR_PIN = A0;
const int midIR_PIN = A1;
const int rigIR_PIN = A2;

// Servo under the front ultrasonic sensor
const int servoPin = A5;

// Wheel encoders (2 and 3 are the Uno's interrupt pins)
const int leftEncoder_PIN = 2;
const int rightEncoder_PIN = 3;

// The board's built-in "L" LED: on once it's standing at the object
const int DONE_LED_PIN = LED_BUILTIN;


// -------------------------
// Settings
// -------------------------

// Tests (set one to true, upload, watch the Serial Monitor at 115200)
const bool SERVO_TEST = false;  // true = no driving: the sensor points left, straight ahead, right, over and
                                // over. If left and right are swapped, flip SERVO_BIGGER_IS_RIGHT. If "ahead"
                                // isn't straight ahead, change SERVO_AHEAD
const bool SCAN_TEST = false;   // true = scan, print everything found, turn to face the widest object, wait
                                // 3 s, and do it again. It never drives toward the object
const bool PRINT_SCAN = true;   // Print every thing found in each sweep (harmless with no computer attached)

// Servo
const int SERVO_AHEAD = 90;                // Servo angle that points the sensor straight ahead
const bool SERVO_BIGGER_IS_RIGHT = true;   // Bigger servo angles turn the sensor to the robot's right
const int SERVO_MIN_US = 544;              // Pulse length for 0° and 180° (µs). These are the Servo library's
const int SERVO_MAX_US = 2400;             // own numbers, so angles here mean the same as with Servo.write()
const unsigned long SERVO_SWING_MS = 500;  // Time allowed for a big move (to the start of a sweep, or back
                                           // to straight ahead)
const unsigned long SERVO_STEP_MS = 40;    // Time allowed for one SCAN_STEP_DEG step before reading

// Measure: how far the front of the ultrasonic sensor is ahead of the middle of the wheel axle (cm), with the
// sensor pointing straight ahead. The robot spins around the middle of the axle but measures from the sensor,
// so this is needed to turn the right amount toward an object off to one side
const float SENSOR_AHEAD_CM = 8.0;

// Scanning
const int SCAN_STEP_DEG = 3;           // Degrees between readings in a sweep. Must divide 180
const int SWEEPS = 3;                  // Sweeps per scan, the robot spinning 360 / SWEEPS degrees between them.
                                       // 3 = every direction is covered twice, so an object is never cut in
                                       // half by the edge of a sweep. 2 is faster (a third less time), but an
                                       // object exactly to the robot's side is seen as two halves
const float MAX_RANGE_CM = 100;        // Anything farther than this is ignored (outside the arena)
const float MAX_OBJECT_WIDTH_CM = 30;  // Anything that measures wider than this is a wall, not an object
const float EDGE_JUMP_CM = 8;          // Neighbouring readings farther apart than this are different things
const float BEAM_WIDTH_DEG = 15;       // The sensor's beam is wide, so everything looks this many degrees
                                       // wider than it is. To tune: stand a thin pole (a marker pen) 40 cm
                                       // away, run SCAN_TEST, and put the "seen over" angle it prints here
const int MIN_POINTS = 2;              // Things seen in fewer readings than this are noise
const float AGREE_CM = 3;              // Two readings this close together agree; otherwise a third is taken
const unsigned long PING_GAP_MS = 15;  // Wait between readings, so the last one's echoes have died away

// Driving to the object
const float STOP_DISTANCE_CM = 5;     // Where to stand: the front sensor's reading to the object
const float STOP_TOLERANCE_CM = 1.0;  // Close enough to STOP_DISTANCE_CM (one encoder tick is about 0.55 cm)
const float STOP_EARLY_CM = 1.0;      // Brake this much before STOP_DISTANCE_CM, to allow for stopping.
                                      // Raise it if it ends up too close and has to back up every time
const float SLOW_CM = 20;             // Slow to CRAWL_SPEED when the object is closer than this
const int LOST_COUNT = 4;             // Readings in a row without the object = lost it: stop and look again
const int MAX_REACQUIRES = 3;         // Times it will look again on one approach before doing a full scan
const int REACQUIRE_DEG = 45;         // How far each side of straight ahead it looks when it has lost the object
const float TURN_MIN_DEG = 3;         // Turns smaller than this aren't worth making (one tick is about 2°)

// Staying inside the white lines
const bool STAY_INSIDE = true;   // false = ignore the IR sensors
const float BACKUP_CM = 10;      // How far it backs away from a white line before scanning again
const float BLOCK_DEG = 40;      // After backing off a line, things within this many degrees of straight
                                 // ahead and beyond the line are outside the arena and ignored for one scan
const int LINE_CONFIRM = 3;      // IR readings in a row (taken back to back) that must all see the line

// IR line sensors: the same numbers as the line follower. These sensors read LOW on the white line, so
// every reading is flipped (1023 - reading) to make the line read HIGH
const bool INVERT_READINGS = true;
const int FLOOR_READING = 23;
const int LINE_READING = 978;
const int LINE_THRESHOLD = (FLOOR_READING + LINE_READING) / 2;  // Above this = over the white line

// Speeds (0-255, where 255 = TOP_SPEED_MM_S), as in the line / wall follower
const int APPROACH_SPEED = 60;  // Driving toward the object
const int CRAWL_SPEED = 55;     // The last SLOW_CM, backing up and nudging (raise it if the wheels hum but
                                // don't move)
const int TURN_SPEED = 55;      // Both wheels when spinning in place

// Speed sensors and wheels (from the line follower)
const int SLOTS_PER_TURN = 20;
const float WHEEL_DIAMETER_MM = 70.0;
const float WHEEL_SPACING_MM = 140.0;  // If turns go too far, lower it a little; not far enough, raise it
const float MM_PER_TICK = PI * WHEEL_DIAMETER_MM / (2 * SLOTS_PER_TURN);
const float DEG_PER_TICK = 360.0 * MM_PER_TICK / (PI * WHEEL_SPACING_MM);
const unsigned long TICK_DEBOUNCE_US = 1000;

// Holding each wheel at the speed it's asked for (from the line follower)
const float TOP_SPEED_MM_S = 854.27;
const float SPEED_KP = 0.1;
const float SPEED_KI = 1.0;
const float SPEED_I_LIMIT = 40;
const unsigned long SPEED_INTERVAL_MS = 10;
const unsigned long STOPPED_US = 200000;
const unsigned long STALL_MS = 1500;  // No ticks for this long while it should be moving = stuck: give up

// Which way to spin
const int SIDE_LEFT = 1;
const int SIDE_RIGHT = 0;

// How an approach ended
const int ARRIVED = 0;
const int LOST_IT = 1;
const int HIT_LINE = 2;

// Which thing to pick out of a sweep
const int RULE_WIDEST = 0;  // The widest (the scan)
const int RULE_AHEAD = 1;   // The one nearest straight ahead (finding the object again on the way to it)

// The echo can take this long to come back from MAX_RANGE_CM (58.3 µs per cm, there and back), plus a margin
const unsigned long ECHO_TIMEOUT_US = (unsigned long)((MAX_RANGE_CM + 30) * 58.3) + 1000;


// -------------------------
// State
// -------------------------

// One sweep's readings: scanCm[i] is the distance (cm) at servo angle i x SCAN_STEP_DEG, 0 = nothing there
const int SCAN_POINTS = 180 / SCAN_STEP_DEG + 1;
float scanCm[SCAN_POINTS];

// Which way the robot is facing (degrees, clockwise = right is positive), measured from the way it faced
// when the current scan started
float heading = 0;

// The best thing found so far. Bearing and distance are from the middle of the wheel axle
bool bestFound = false;
float bestBearing = 0;  // Degrees, on the same scale as `heading`
float bestDistance = 0;  // cm
float bestWidth = 0;     // cm
float bestScore = 0;     // What it's being picked on (width, or closeness to straight ahead)

// After backing off a white line: ignore things straight ahead beyond blockBeyondCm for the next scan
bool blockActive = false;
float blockBeyondCm = 0;

// Servo
int servoAngle = SERVO_AHEAD;        // Where the servo is being held
unsigned long lastServoPulse = 0;

// White line watch (only switched on while driving forward)
bool watchLine = false;
bool lineSeen = false;

bool finished = false;  // Standing at the object

// Speed sensor state, changed by the interrupts
volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;
volatile unsigned long leftTickAt[3] = { 0, 0, 0 };
volatile unsigned long rightTickAt[3] = { 0, 0, 0 };

// Speed control
int leftTarget = 0;
int rightTarget = 0;
float leftBoost[2] = { 0, 0 };  // [0] forward, [1] backward
float rightBoost[2] = { 0, 0 };
float leftError = 0;
float rightError = 0;
unsigned long lastSpeedUpdate = 0;

// Counting how far a spin or a straight drive has gone
unsigned long moveStartLeft = 0;
unsigned long moveStartRight = 0;
unsigned long moveLastTotal = 0;
unsigned long moveLastTickMs = 0;


// -------------------------
// Setup
// -------------------------

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  digitalWrite(frontTrig, LOW);

  pinMode(servoPin, OUTPUT);
  digitalWrite(servoPin, LOW);

  pinMode(DONE_LED_PIN, OUTPUT);
  digitalWrite(DONE_LED_PIN, LOW);

  pinMode(leftEncoder_PIN, INPUT);
  pinMode(rightEncoder_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftEncoder_PIN), leftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder_PIN), rightTick, CHANGE);

  stop();
  Serial.begin(115200);

  // Two seconds to set the robot down, with the sensor moving to straight ahead
  pointServo(SERVO_AHEAD, 2000);
}


// -------------------------
// Motors (from the line follower)
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

// Cut power to both motors (they coast to a stop)
void stop() {
  leftTarget = 0;
  rightTarget = 0;
  leftError = 0;
  rightError = 0;
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

// Stop the motors hard instead of letting them coast
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

void setLeftMotor(int power) {
  if (power >= 0) motorAForward(power);
  else motorABackward(-power);
}

void setRightMotor(int power) {
  if (power >= 0) motorBForward(power);
  else motorBBackward(-power);
}

// The power (-255..255) to give a wheel: what it's asked for, plus the speed control's correction
int wheelPower(int target, float error, float boost) {
  if (target == 0) return 0;
  int power = constrain((int)(abs(target) + SPEED_KP * error + boost), 0, 255);
  return target > 0 ? power : -power;
}

void applyPower() {
  setLeftMotor(wheelPower(leftTarget, leftError, leftBoost[leftTarget < 0]));
  setRightMotor(wheelPower(rightTarget, rightError, rightBoost[rightTarget < 0]));
}

// Set each wheel (0-255 scale): positive = forward, negative = backward. The speed sensors hold it there
void drive(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if ((leftSpeed < 0) != (leftTarget < 0) || leftSpeed == 0) leftError = 0;
  if ((rightSpeed < 0) != (rightTarget < 0) || rightSpeed == 0) rightError = 0;

  leftTarget = leftSpeed;
  rightTarget = rightSpeed;
  applyPower();
  updateMotors();
}

void correctWheel(int target, float measured, float &error, float &boost, float dt) {
  if (target == 0) {
    error = 0;
    return;
  }
  error = abs(target) - measured;

  int power = abs(wheelPower(target, error, boost));
  bool stuckHigh = power >= 255 && error > 0;
  bool stuckLow = power <= 0 && error < 0;
  if (!stuckHigh && !stuckLow) {
    boost = constrain(boost + SPEED_KI * error * dt, -SPEED_I_LIMIT, SPEED_I_LIMIT);
  }
}

// Check both wheel speeds and correct them. Does nothing while the robot is stopped
void updateMotors() {
  if (leftTarget == 0 && rightTarget == 0) return;

  unsigned long now = millis();
  if (now - lastSpeedUpdate < SPEED_INTERVAL_MS) return;
  float dt = min((now - lastSpeedUpdate) / 1000.0, 0.05);
  lastSpeedUpdate = now;

  correctWheel(leftTarget, wheelSpeed(leftTickAt), leftError, leftBoost[leftTarget < 0], dt);
  correctWheel(rightTarget, wheelSpeed(rightTickAt), rightError, rightBoost[rightTarget < 0], dt);
  applyPower();
}


// -------------------------
// Speed sensors (from the line follower)
// -------------------------

void leftTick() {
  unsigned long t = micros();
  if (t - leftTickAt[0] < TICK_DEBOUNCE_US) return;
  leftTickAt[2] = leftTickAt[1];
  leftTickAt[1] = leftTickAt[0];
  leftTickAt[0] = t;
  leftTicks++;
}

void rightTick() {
  unsigned long t = micros();
  if (t - rightTickAt[0] < TICK_DEBOUNCE_US) return;
  rightTickAt[2] = rightTickAt[1];
  rightTickAt[1] = rightTickAt[0];
  rightTickAt[0] = t;
  rightTicks++;
}

void readTicks(unsigned long &left, unsigned long &right) {
  noInterrupts();
  left = leftTicks;
  right = rightTicks;
  interrupts();
}

// How fast one wheel is turning, on the 0-255 scale
float wheelSpeed(volatile unsigned long *tickAt) {
  noInterrupts();
  unsigned long newest = tickAt[0];
  unsigned long twoBefore = tickAt[2];
  interrupts();

  unsigned long now = micros();
  if (twoBefore == 0 || now - newest > STOPPED_US) return 0;

  unsigned long slotTime = newest - twoBefore;
  if (now - newest > slotTime) slotTime = now - newest;

  float mmPerSec = 2 * MM_PER_TICK * 1000000.0 / slotTime;
  return mmPerSec / TOP_SPEED_MM_S * 255;
}

// Start counting how far a move (a spin or a straight drive) goes
void startCounting() {
  readTicks(moveStartLeft, moveStartRight);
  moveLastTotal = 0;
  moveLastTickMs = millis();
}

// Ticks since startCounting(), averaged over the two wheels
float ticksMoved() {
  unsigned long left, right;
  readTicks(left, right);
  unsigned long total = (left - moveStartLeft) + (right - moveStartRight);
  if (total != moveLastTotal) {
    moveLastTotal = total;
    moveLastTickMs = millis();
  }
  return total / 2.0;
}

bool stalled() {
  if (millis() - moveLastTickMs <= STALL_MS) return false;
  Serial.println(F("Stalled: no ticks from the speed sensors"));
  return true;
}


// -------------------------
// Things that must keep happening while the robot waits for anything
// -------------------------

// Keeps the wheel speeds matched, the servo held where it is, and (while driving forward) watches for the
// white line. Every waiting loop in this sketch calls it
void background() {
  updateMotors();
  holdServo();
  if (watchLine && !lineSeen && onWhiteLine()) lineSeen = true;
}

void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) background();
}


// -------------------------
// Servo (pulses sent by hand: see the note at the top about the Servo library and pins 9 / 10)
// -------------------------

// One servo pulse: the length of the pulse tells the servo which angle to go to
void servoPulse() {
  int pulseUs = map(servoAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(servoPin, LOW);
  lastServoPulse = millis();
}

// A servo wants a pulse every 20 ms to hold its angle
void holdServo() {
  if (millis() - lastServoPulse >= 20) servoPulse();
}

// Point the sensor at a servo angle (0-180) and give it `ms` to get there
void pointServo(int angle, unsigned long ms) {
  servoAngle = constrain(angle, 0, 180);
  waitMs(ms);
}


// -------------------------
// Ultrasonic sensor
// -------------------------

// One reading: the distance in cm, or -1 if nothing is within MAX_RANGE_CM
float readDistance() {
  digitalWrite(frontTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(frontTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontTrig, LOW);

  unsigned long duration = pulseIn(frontEcho, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    // No echo in time. The HC-SR04 holds its echo pin HIGH until it gives up (about 40 ms): wait for
    // that, or the next reading would be wrong
    unsigned long start = millis();
    while (digitalRead(frontEcho) == HIGH && millis() - start < 45) background();
    return -1;
  }

  // Sound travels 0.0343 cm per microsecond, and the pulse goes there and back
  float cm = duration * 0.0343 / 2;
  if (cm > MAX_RANGE_CM) return -1;
  return cm;
}

// The middle one of three readings, with "nothing there" (-1) counting as very far
float middleOf(float a, float b, float c) {
  if (a < 0) a = 10000;
  if (b < 0) b = 10000;
  if (c < 0) c = 10000;
  float middle = max(min(a, b), min(max(a, b), c));
  return middle >= 10000 ? -1 : middle;
}

// A careful reading for the scan and for the final position: two readings, and if they don't agree, a
// third, taking the middle one. One stray echo or one missed echo doesn't count
float steadyDistance() {
  float a = readDistance();
  waitMs(PING_GAP_MS);
  float b = readDistance();
  waitMs(PING_GAP_MS);

  if (a < 0 && b < 0) return -1;
  if (a > 0 && b > 0 && fabs(a - b) <= AGREE_CM) return (a + b) / 2;

  float c = readDistance();
  waitMs(PING_GAP_MS);
  return middleOf(a, b, c);
}


// -------------------------
// IR line sensors
// -------------------------

int readIR(int pin) {
  int value = analogRead(pin);
  return INVERT_READINGS ? 1023 - value : value;
}

// Is any of the three IR sensors over the white line? LINE_CONFIRM readings back to back must all say so
bool onWhiteLine() {
  for (int i = 0; i < LINE_CONFIRM; i++) {
    bool any = readIR(leftIR_PIN) > LINE_THRESHOLD
               || readIR(midIR_PIN) > LINE_THRESHOLD
               || readIR(rigIR_PIN) > LINE_THRESHOLD;
    if (!any) return false;
  }
  return true;
}


// -------------------------
// Moving by the speed sensors
// -------------------------

// Bring an angle into the range -180..180
float wrap180(float angle) {
  while (angle > 180) angle -= 360;
  while (angle <= -180) angle += 360;
  return angle;
}

// Spin in place toward `side` by `degrees`. Returns how far it really turned (it coasts a little after
// the brake), so `heading` stays right
float spinDegrees(int side, float degrees) {
  startCounting();
  if (side == SIDE_RIGHT) drive(TURN_SPEED, -TURN_SPEED);
  else drive(-TURN_SPEED, TURN_SPEED);

  while (ticksMoved() * DEG_PER_TICK < degrees && !stalled()) background();

  brake();
  waitMs(150);  // Let it come to rest before counting
  return ticksMoved() * DEG_PER_TICK;
}

// Spin to face `bearing` (on the same scale as `heading`), the short way round
void turnTo(float bearing) {
  float turn = wrap180(bearing - heading);
  if (fabs(turn) < TURN_MIN_DEG) return;

  if (turn > 0) heading += spinDegrees(SIDE_RIGHT, turn);
  else heading -= spinDegrees(SIDE_LEFT, -turn);
}

// Drive straight by `cm`: positive = forward, negative = backward
void driveStraight(float cm) {
  int speed = cm > 0 ? CRAWL_SPEED : -CRAWL_SPEED;
  float ticksNeeded = fabs(cm) * 10 / MM_PER_TICK;

  startCounting();
  drive(speed, speed);
  while (ticksMoved() < ticksNeeded && !stalled()) background();
  brake();
}


// -------------------------
// Scanning
// -------------------------

// Sweep the sensor from servo angle `fromAngle` to `toAngle`, filling scanCm[]. Angles outside are left empty
void sweep(int fromAngle, int toAngle) {
  pointServo(fromAngle, SERVO_SWING_MS);

  for (int i = 0; i < SCAN_POINTS; i++) {
    int angle = i * SCAN_STEP_DEG;
    if (angle < fromAngle || angle > toAngle) {
      scanCm[i] = 0;
      continue;
    }
    pointServo(angle, SERVO_STEP_MS);
    float d = steadyDistance();
    scanCm[i] = d > 0 ? d : 0;
  }

  // One empty reading with about the same distance on both sides of it is a missed echo, not a gap
  for (int i = 1; i < SCAN_POINTS - 1; i++) {
    if (scanCm[i] == 0 && scanCm[i - 1] > 0 && scanCm[i + 1] > 0
        && fabs(scanCm[i - 1] - scanCm[i + 1]) <= EDGE_JUMP_CM) {
      scanCm[i] = (scanCm[i - 1] + scanCm[i + 1]) / 2;
    }
  }
}

// Forget the best thing found so far
void clearBest() {
  bestFound = false;
  bestScore = -1e9;
}

// One thing found in a sweep: readings `first`..`last` of scanCm[], the nearest of them `nearest` cm away.
// Work out where it is and how wide it is, and keep it if it's the best so far under `rule`
void judge(int first, int last, float nearest, int rule) {
  int points = last - first + 1;
  if (points < MIN_POINTS) return;  // Noise

  // The angle it covers. It was seen over `seenDeg`, but the wide beam adds BEAM_WIDTH_DEG to everything
  float seenDeg = points * SCAN_STEP_DEG;
  float realDeg = constrain(seenDeg - BEAM_WIDTH_DEG, (float)SCAN_STEP_DEG, 170.0);

  // Its real width: two right-angled triangles back to back, each with the distance as one side
  float width = 2 * nearest * tan(radians(realDeg / 2));

  // The angle to its middle, from the SENSOR: 0 = straight ahead, positive = to the robot's right
  float servoMiddle = (first + last) / 2.0 * SCAN_STEP_DEG;
  float fromSensor = (servoMiddle - SERVO_AHEAD) * (SERVO_BIGGER_IS_RIGHT ? 1 : -1);

  // The robot spins around the middle of its axle, and the sensor sits SENSOR_AHEAD_CM in front of that.
  // So work out where the thing is from the axle (how far ahead, how far to the right), and from that the
  // angle the robot has to turn and the distance from the axle
  float ahead = SENSOR_AHEAD_CM + nearest * cos(radians(fromSensor));
  float toRight = nearest * sin(radians(fromSensor));
  float fromAxle = degrees(atan2(toRight, ahead));
  float distance = sqrt(ahead * ahead + toRight * toRight);
  float bearing = wrap180(heading + fromAxle);

  bool isWall = width > MAX_OBJECT_WIDTH_CM;
  bool pastLine = blockActive && fabs(bearing) < BLOCK_DEG && distance > blockBeyondCm;

  if (PRINT_SCAN) {
    Serial.print(F("  Found: angle "));
    Serial.print(bearing, 0);
    Serial.print(F(" deg, "));
    Serial.print(nearest, 1);
    Serial.print(F(" cm away, seen over "));
    Serial.print(seenDeg, 0);
    Serial.print(F(" deg, width "));
    Serial.print(width, 1);
    Serial.print(F(" cm"));
    if (isWall) Serial.print(F("  -> wall, ignored"));
    else if (pastLine) Serial.print(F("  -> past the white line, ignored"));
    Serial.println();
  }

  if (isWall || pastLine) return;

  // The scan wants the widest; finding the object again on the way wants the one nearest straight ahead
  float score = (rule == RULE_WIDEST) ? width : -fabs(wrap180(bearing - heading));
  if (score > bestScore) {
    bestFound = true;
    bestScore = score;
    bestBearing = bearing;
    bestDistance = distance;
    bestWidth = width;
  }
}

// Split the sweep in scanCm[] into separate things and judge each one. Neighbouring readings within
// EDGE_JUMP_CM of each other are the same thing; an empty reading or a bigger jump is the edge of it
void lookThroughSweep(int rule) {
  int i = 0;
  while (i < SCAN_POINTS) {
    if (scanCm[i] <= 0) {
      i++;
      continue;
    }

    int first = i;
    float nearest = scanCm[i];
    while (i + 1 < SCAN_POINTS && scanCm[i + 1] > 0 && fabs(scanCm[i + 1] - scanCm[i]) <= EDGE_JUMP_CM) {
      i++;
      if (scanCm[i] < nearest) nearest = scanCm[i];
    }

    judge(first, i, nearest, rule);
    i++;
  }
}

// Scan all the way round: SWEEPS sweeps with a spin between them. Leaves the widest object in best...
// Returns false if it found nothing
bool fullScan() {
  Serial.println(F("Scanning"));
  heading = 0;
  clearBest();

  for (int s = 0; s < SWEEPS; s++) {
    if (PRINT_SCAN) {
      Serial.print(F(" Sweep "));
      Serial.print(s + 1);
      Serial.print(F(", facing "));
      Serial.println(heading, 0);
    }
    sweep(0, 180);
    lookThroughSweep(RULE_WIDEST);

    if (s < SWEEPS - 1) heading += spinDegrees(SIDE_RIGHT, 360.0 / SWEEPS);
  }

  pointServo(SERVO_AHEAD, SERVO_SWING_MS);

  if (bestFound) {
    Serial.print(F("Widest object: angle "));
    Serial.print(bestBearing, 0);
    Serial.print(F(" deg, "));
    Serial.print(bestDistance, 1);
    Serial.print(F(" cm from the axle, width "));
    Serial.print(bestWidth, 1);
    Serial.println(F(" cm"));
  } else {
    Serial.println(F("No object found"));
  }
  return bestFound;
}


// -------------------------
// Driving to the object
// -------------------------

// Drive straight at the object, which should be about `expectCm` in front of the sensor.
// Returns ARRIVED (braked at the stopping distance), LOST_IT (do a full scan again) or HIT_LINE
int approach(float expectCm) {
  Serial.println(F("Driving to the object"));

  float last = expectCm;  // The latest reading that was on the object
  int misses = 0;         // Readings in a row that weren't on the object
  int closeCount = 0;     // Readings in a row at the stopping distance
  int reacquires = 0;

  lineSeen = false;
  watchLine = STAY_INSIDE;

  while (true) {
    background();

    float d = readDistance();
    waitMs(PING_GAP_MS);

    if (lineSeen) {
      brake();
      watchLine = false;
      Serial.println(F("White line"));
      return HIT_LINE;
    }

    // On the object = an echo that isn't much farther than the last one (it should only be getting closer).
    // Anything else is the beam slipping off the object and seeing past it
    bool onObject = d > 0 && d <= last + EDGE_JUMP_CM;

    if (onObject) {
      misses = 0;
      last = d;
    } else {
      misses++;
    }

    // Close enough: brake. Two readings in a row, so one stray echo can't stop it early
    if (onObject && d <= STOP_DISTANCE_CM + STOP_EARLY_CM) closeCount++;
    else closeCount = 0;

    // The HC-SR04 can't read under about 3 cm. Losing the echo right at the stopping distance means the
    // object is closer than that, not gone
    bool tooCloseToRead = !onObject && misses >= 2 && last <= STOP_DISTANCE_CM + 3;

    if (closeCount >= 2 || tooCloseToRead) {
      brake();
      watchLine = false;
      return ARRIVED;
    }

    // Lost it: stop, sweep a little each side, and turn to whatever is nearest straight ahead
    if (misses >= LOST_COUNT) {
      brake();
      watchLine = false;
      reacquires++;
      Serial.println(F("Lost the object: looking again"));
      if (reacquires > MAX_REACQUIRES) return LOST_IT;

      clearBest();
      sweep(SERVO_AHEAD - REACQUIRE_DEG, SERVO_AHEAD + REACQUIRE_DEG);
      lookThroughSweep(RULE_AHEAD);
      pointServo(SERVO_AHEAD, SERVO_SWING_MS);
      if (!bestFound) return LOST_IT;

      turnTo(bestBearing);
      last = bestDistance - SENSOR_AHEAD_CM;
      misses = 0;
      closeCount = 0;
      lineSeen = false;
      watchLine = STAY_INSIDE;
      continue;
    }

    // Slow down for the last stretch
    int speed = (last < SLOW_CM) ? CRAWL_SPEED : APPROACH_SPEED;
    drive(speed, speed);
  }
}

// Stopped near the object: nudge forward or backward until the sensor reads STOP_DISTANCE_CM.
// Returns false if the object isn't in front any more
bool settle() {
  for (int tries = 0; tries < 5; tries++) {
    waitMs(150);  // Come to rest first
    float d = steadyDistance();

    if (d < 0) {
      // No echo: closer than the sensor can read. Back off a little and read again
      driveStraight(-2);
      continue;
    }
    if (d > STOP_DISTANCE_CM + 10) return false;

    float off = d - STOP_DISTANCE_CM;  // Positive = too far away
    Serial.print(F("Standing "));
    Serial.print(d, 1);
    Serial.println(F(" cm from the object"));

    if (fabs(off) <= STOP_TOLERANCE_CM) return true;
    driveStraight(off);
  }
  return true;
}


// -------------------------
// Main loop
// -------------------------

void loop() {
  if (SERVO_TEST) {
    Serial.println(F("Sensor should point to the robot's LEFT"));
    pointServo(SERVO_BIGGER_IS_RIGHT ? 0 : 180, 1500);
    Serial.println(F("Sensor should point STRAIGHT AHEAD"));
    pointServo(SERVO_AHEAD, 1500);
    Serial.println(F("Sensor should point to the robot's RIGHT"));
    pointServo(SERVO_BIGGER_IS_RIGHT ? 180 : 0, 1500);
    return;
  }

  // Standing at the object: stay there, with the sensor held straight ahead
  if (finished) {
    holdServo();
    return;
  }

  // Scan all the way round for the widest object. The "past the white line" rule only lasts one scan
  bool found = fullScan();
  blockActive = false;
  if (!found) return;  // Nothing there: scan again

  turnTo(bestBearing);

  if (SCAN_TEST) {
    Serial.println(F("Scan test: facing the widest object"));
    waitMs(3000);
    return;
  }

  int result = approach(bestDistance - SENSOR_AHEAD_CM);

  if (result == ARRIVED) {
    if (settle()) {
      finished = true;
      digitalWrite(DONE_LED_PIN, HIGH);
      Serial.println(F("Done: standing at the object"));
    }
    // If the object wasn't there after all, loop() scans again

  } else if (result == HIT_LINE) {
    // Whatever it was driving at is outside the arena. Back away from the line and scan again, ignoring
    // things straight ahead that are beyond the line. (This takes the IR sensors to be about as far
    // forward on the robot as the ultrasonic sensor)
    driveStraight(-BACKUP_CM);
    blockActive = true;
    blockBeyondCm = BACKUP_CM + SENSOR_AHEAD_CM;
  }
  // LOST_IT: loop() scans again
}
