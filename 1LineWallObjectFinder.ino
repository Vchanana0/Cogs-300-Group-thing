// 1LineWallObjectFinder.ino
// Starts as a line follower, switches to left-wall following, then switches to object finding.
// Each switch is for good: it never goes back to an earlier mode until it's switched off and on.
//
// SWITCHING TO OBJECT FINDING:
// - While wall following it keeps reading the three IR sensors underneath, whatever it's doing: driving along
//   the wall, spinning at a corner or backing up. Once all three have seen the line at the same time for
//   ALL_IR_HOLD_MS (0.3 s) without a break, it stops what it's doing and brakes. Then it drives straight ahead
//   for SWITCH_STRAIGHT_MS (2 s), with the right wheel 5% faster than the left, brakes and runs the Object Finding code (see
//   OBJECT FINDING near the end): it scans with the front sensor on its servo, picks the widest object, turns
//   to it, drives up to it and touches it.
// - The object finding area has a line round it (the perimeter). Whenever it drives forward (the 2 s drive,
//   moving to a better view, the approach and the final touch), any IR sensor seeing a line for
//   PERIMETER_CONFIRM readings means it is leaving the area: it brakes, backs up PERIMETER_BACK_CM, turns
//   round 180° and scans again. The 2 s drive starts on the line, so there it only counts once it's off it.
// - The front ultrasonic sensor sits on the servo (pin A5). Until object finding starts, the servo is held
//   pointing straight ahead, so the line and wall code see straight ahead with it.
//
// SWITCHING TO WALL FOLLOWING (it never switches back until it's switched off and on):
// - While driving along the line (not during spins), every WALL_CHECK_MS it reads one ultrasonic sensor.
//   It switches when the FRONT sensor sees a wall close ahead and the LEFT sensor sees a wall close beside it:
//   SWITCH_COUNT front readings in a row must each be under SWITCH_FRONT_CM, and the left sensor's latest
//   reading must be under SWITCH_DISTANCE_CM. Until something is within SWITCH_FRONT_CM ahead, only the front
//   sensor is read. A reading with no echo is skipped.
// - It never switches because the line was lost or ended. If a search finds no line, it searches again
//   until it finds one (see "When the line is lost" below).
//
// WALL FOLLOWING (from the wall-following code, using these pins and the speed sensors):
// - Holds WALL_SET_POINT cm from the left wall with its own PID (the old wall gains).
// - Slows down as a wall ahead gets closer. At FRONT_LIMIT it spins in place to the right until that wall
//   is beside the left sensor (about as far away as it was in front) and the front is clear, or until it has
//   turned WALL_TURN_MAX_DEG (counted by the speed sensors), whichever comes first. The ultrasonic sensors can
//   only end the turn once it has turned at least WALL_TURN_MIN_DEG.
// - If the wheels aren't turning (fewer than STUCK_TICKS speed sensor ticks in STUCK_MS while it's driving or
//   turning), it backs up BACK_UP_CM and tries the turn again. It does the same if the front of the
//   robot is touching the wall ahead (a front reading under TOUCH_CM, or no echo just after one).
//
// LINE FOLLOWING (unchanged from finalfinalLineFollower):
// Line-following robot using the 3 analog IR sensors underneath (A0 left, A1 middle, A2 right).
//
// The line reads HIGHER than the floor. A reading above LINE_THRESHOLD (halfway between the
// floor and line readings) = over the line. (These sensors physically read LOW on the white line, so
// readLineSensors() flips every reading with INVERT_READINGS to make the line read high.)
// When the robot is centred, only the middle sensor is over the line.
//
// Every turn spins in place (one wheel forward, the other backward) and stops when the middle sensor finds
// the line. How far it has turned is measured by the two LM393 speed sensors on the wheels (not by timing):
// a search that has spun about 160° without finding a line stops (the line it came along is right behind it
// at 180°), and when the line slips out from under the middle sensor on its own, it looks about 30° each way.
//
// Wheel speeds: the speed sensors also hold each wheel at the speed it's asked for, so equal speeds really
// are equal (going straight, and both wheels when spinning). This replaces the old motor trim. Speeds are
// still asked for on the 0-255 scale, where 255 = TOP_SPEED_MM_S, the robot's top speed, which you measure.
// A wheel that's asked to move but isn't turning gets a short kick of START_POWER until it ticks, so slow
// speeds (spins, the crawl before a corner, backing up) don't just make the motors hum.
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
// (or within the 30° look), it doesn't give up: it drives on for LOST_MS and searches again. After a failed
// 160° spin it spins back past where it started and looks 160° the other way, in case it picked the wrong side
// at a corner. Only if that fails too (the line has ended) does the next search carry on the same way, so it
// finds the line it came along and turns round. After a failed look each way it inches forward and looks again. Losing the line never switches it
// to wall following.
//
// To check that every motor and sensor is wired up and working, upload the separate HardwareTest sketch and
// follow the steps it gives in the Serial Monitor.
//
// Before running it, set the numbers marked "Measure": FLOOR_READING, LINE_READING and TOP_SPEED_MM_S.
//
// It starts on its own when powered, so it runs on a battery pack with no computer attached.

// Motor A (left motor)
int enA = 10;  // PWM speed control
int in1 = 4;   // Direction
int in2 = 5;   // Direction

// Motor B (right motor)
int enB = 9;   // PWM speed control
int in3 = A3;  // Direction
int in4 = A4;  // Direction

// Front ultrasonic sensor (HC-SR04), facing forward. Watched while line following, used in wall following.
// It sits on the servo, which holds it straight ahead until object finding turns it to scan
int frontTrig = 7;
int frontEcho = 8;
const int servoPin = A5;  // The servo the front sensor sits on (pulsed by hand: see servoPulse())

// Left ultrasonic sensor (HC-SR04), facing left. Used in wall following
int leftTrig = 11;
int leftEcho = 12;

// The board's built-in "L" LED (pin 13 on an Uno/Nano). Off while line following, on while wall following,
// off again during object finding, and on once object finding has made contact
const int WALL_LED_PIN = LED_BUILTIN;

// Left, middle and right IR sensors, read as analog values (0-1023)
const int leftIR_PIN = A0;
const int midIR_PIN = A1;
const int rigIR_PIN = A2;

// Line detection. Measure: hold the robot with the sensors over the floor, then over the middle of the line,
// and read the numbers in the Serial Monitor (115200)
// Which way the raw sensor reads: true = the sensor reads LOW over the white line (yours does), so every
// reading is flipped (1023 - reading) and the numbers below and in the Serial Monitor go HIGH on the line.
// Set it to false if your sensors already read high over the line.
const bool INVERT_READINGS = true;

const int FLOOR_READING = 23;                                   // What the IR sensors read over the floor (after the flip: low)
                                                                // Re-measure: these are your old 1000 / 45 flipped (1023 - x)
const int LINE_READING = 978;                                   // What they read right over the middle of the line (higher than the floor)
const int LINE_THRESHOLD = (FLOOR_READING + LINE_READING) / 2;  // Above this = that sensor is over the line
const int SQUARE_THRESHOLD = LINE_READING - (LINE_READING - FLOOR_READING) / 4;
// Above this = squarely over a line, not just its edge. A side sensor squarely
// on a line while the middle is on the line means two lines: a junction

// LM393 speed sensors (one slotted encoder disc on each motor). D0 goes to an interrupt pin so no pulse
// is missed. Both edges of every slot are counted (a "tick"), so one wheel turn = 2 x SLOTS_PER_TURN ticks
const int leftEncoder_PIN = 2;
const int rightEncoder_PIN = 3;
const int SLOTS_PER_TURN = 20;  // Slots in each encoder disc
const float WHEEL_DIAMETER_MM = 60.0;
const float WHEEL_SPACING_MM = 140.0;                                     // From the middle of one tyre to the middle of the other. If spins
                                                                          // turn too far, lower it a little; if they don't turn far enough,
                                                                          // raise it (the tyres slip a bit on some floors)
const float MM_PER_TICK = PI * WHEEL_DIAMETER_MM / (2 * SLOTS_PER_TURN);  // About 5.5 mm
const unsigned long TICK_DEBOUNCE_US = 1000;                              // Ignore edges closer together than this (the LM393 can
                                                                          // flicker on an edge). The fastest real ticks are ~7 ms apart

// Holding each wheel at the speed it's asked for
const float TOP_SPEED_MM_S = 1367.04;         // Measure: top speed of the slower wheel (mm/s), at full power
const float SPEED_KP = 0.1;                  // How hard a wheel that's too slow / too fast is pushed back right away
const float SPEED_KI = 1.0;                  // How quickly the difference is built up and held (this is what
                                             // makes the two wheels match)
const float SPEED_I_LIMIT = 50;              // Most the speed control can add to or take from one wheel's power
const int START_POWER = 150;                 // A wheel that's asked to move but isn't turning gets at least this
                                             // much power (0-255) until its speed sensor ticks. Getting a wheel
                                             // moving takes more power than keeping it going: at 45 the most it
                                             // got was about 100 (45 + SPEED_I_LIMIT), and only after a second,
                                             // so it hummed without moving. Raise it if the wheels still hum,
                                             // lower it if the robot jerks when it starts
const unsigned long SPEED_INTERVAL_MS = 10;  // How often the wheel speeds are checked and corrected (ms)
const unsigned long STOPPED_US = 200000;     // No tick for this long (µs) = the wheel has stopped

// Speeds (0-255, where 255 = TOP_SPEED_MM_S)
const int BASE_SPEED = 50;  // Forward speed on the line. Slower = more time to react at corners
const int TURN_SPEED = 45;  // Speed of both wheels when spinning in place
                            // (raise it if the wheels hum but don't move, lower it if the robot
                            // spins past the line)

// PID for following the line, from the wall follower (tune Kp first, then Kd. Ki is optional)
const float Kp = 2;
const float Ki = 0;
const float Kd = 0.05;                     // Was 2.0 in the wall follower. The line position changes much faster than the
                                          // wall distance did, so 2.0 made the D term max out the steering on every
                                          // update and the robot lurched. 0.3 did best in testing
const float INTEGRAL_LIMIT = 50;          // Stops the I term from building up too far
const int MAX_CORRECTION = 50;            // Most the PID can add to one wheel and take from the other
const unsigned long PID_INTERVAL_MS = 10;  // How often the PID updates (ms). The wall follower's loop took tens of
                                          // ms because of the ultrasonic readings; the IR sensors are much faster,
                                          // and updating on every reading makes the D term jumpy
const float LINE_POSITION_RANGE = 10;     // The error the PID sees: 0 = line under the middle sensor,
                                          // +10 = under the left sensor only, -10 = under the right sensor only

// Bayes filter for the line position
const float SENSOR_SPACING_MM = 35.0;  // Measure: distance between the middle of neighbouring IR sensors (mm)
const float SENSOR_SEES_MM = 8.0;      // How far from the line (mm) a sensor still reads it about half way.
                                       // About 8 for 19 mm tape; scale it with the width of your line
const float READING_NOISE = 0.2;       // How much a reading wobbles, on the 0..1 scale of onLine()
const float LINE_DRIFT = 0.15;         // Chance the line moves one place left or right between PID updates
const float NEVER_RULED_OUT = 0.001;   // Small chance kept for every place, so the belief can always recover
const int NUM_PLACES = 13;             // Places the line could be: -1.5 .. +1.5 sensor spacings, 0.25 apart
                                       // (-1 = under the left sensor, 0 = middle, +1 = under the right)

// Safety limits. Turns stop when the sensors find the line; these only stop a search that has gone too far.
// The angle turned is measured by the speed sensors. One tick is about 4.5° of spin
const float SEARCH_LIMIT_DEG = 160;   // The most one search spins looking for a line. The line it came along is
                                      // right behind it, at 180°, so stopping here keeps the first search from
                                      // just finding that line again. If there's no line by then, it spins back
                                      // and looks this far the other way. If it still turns round at a corner,
                                      // lower it to 150 (or check WHEEL_DIAMETER_MM: if it's too small, every
                                      // spin really turns further than it counts)
const float LOOK_DEG = 30;            // How far it looks each way when the line slips out from under the
                                      // middle sensor on its own
const unsigned long STALL_MS = 1500;  // A spin where neither wheel ticks for this long has stalled (stuck,
                                      // or a speed sensor unplugged): give up on that spin

const float servoStraight = 85;
// Reading the sensors. These only help tell what the sensors are seeing; they don't decide how far to turn
const int CONFIRM = 5;                   // Readings in a row that must agree, so one bad reading doesn't count.
                                          // Keep it small (3-10): it counts readings, not time. Each reading is
                                          // ~0.35 ms while spinning and ~5 ms while printing, so 1000 meant seconds
                                          // of driving blind after losing the line, and a spin could never stay on
                                          // the line long enough to stop
const unsigned long LOST_MS = 50;         // No sensor seeing the line for this long (ms) = the line is lost.
                                          // Shorter gaps are the line slipping between two sensors; the
                                          // robot keeps steering from the Bayes filter's last belief.
                                          // Raise it if it still stops to look around on straight lines,
                                          // lower it if it runs far past corners before spinning
const unsigned long WAY_MEMORY_MS = 400;  // A way off the line seen this recently (ms) before the line ends
                                          // means a corner or sharp turn that way
const unsigned long TOGETHER_MS = 150;    // Ways found on both sides within this long (ms) of each other were
                                          // found together: a T-junction, so go right
const unsigned long STEADY_MS = 150;      // The robot only turns off the line at a way right if the middle sensor
                                          // had been on the line, with the right sensor not squarely on a line,
                                          // for this long (ms) before. Coming out of a turn at an angle, the line can lie
                                          // across two sensors at once and look like a junction

// Switching from line following to wall following
const float SWITCH_FRONT_CM = 35;        // Switch once the front sensor reads closer than this (cm). Keep it above
                                         // FRONT_LIMIT, or it turns at the wall ahead the moment it switches
const int SWITCH_COUNT = 2;              // Front readings in a row under SWITCH_FRONT_CM needed to switch, so one bad
                                         // reading can't switch it (it never switches back). They're about 60 ms apart,
                                         // so at BASE_SPEED it switches about 3.5 cm after the first one: roughly 20 cm
                                         // from the wall with SWITCH_FRONT_CM = 25. 1 = switch on the first reading
const float SWITCH_DISTANCE_CM = 25;     // The left sensor must read closer than this (cm): a wall beside the robot
const unsigned long WALL_CHECK_MS = 15;  // How often an ultrasonic sensor is read while line following (ms). While a
                                         // wall ahead is within SWITCH_FRONT_CM it takes turns (front, left, front,
                                         // left), so each is read every 60 ms and they can't hear each other's echoes.
                                         // Each reading can take up to 20 ms, which pauses the line steering

// Switching from wall following to object finding
const unsigned long ALL_IR_HOLD_MS = 50;  // All three IR sensors must see the line together for this long (ms)
                                           // without a break. One reading with any of them off the line starts
                                           // the count again. At WALL_BASE_SPEED the robot drives about 9 cm in
                                           // 0.3 s, so a line it only drives across has to be at least that long
                                           // in the direction it's going
const unsigned long SWITCH_STRAIGHT_MS = 2000;  // Once it switches, it drives straight ahead for this long (ms)
                                                // before object finding starts
const int SWITCH_STRAIGHT_SPEED = 70;           // Speed of both wheels for that drive (0-255 scale). 55 = about
                                                // 30 cm/s, so about 60 cm in 2 s
const float SWITCH_STRAIGHT_KP = 2.0;           // Speed change per tick one wheel is ahead of the other, to keep
                                                // both wheels' distance the same (so it doesn't drift to one side)
const int SWITCH_STRAIGHT_MAX_FIX = 10;         // Largest speed change it can make for that
const float SWITCH_RIGHT_FASTER = 1.05;         // During that drive the right wheel goes this many times as fast
                                                // (and as far) as the left: 1.05 = 5% faster, 1.0 = the same

// The perimeter: the object finding area has a line round it. While object finding (and during the 2 s
// drive), any IR sensor seeing a line while it drives forward means it is leaving the area
const int PERIMETER_CONFIRM = 1;            // IR readings in a row that must see the line (so one bad reading
                                            // doesn't count). Each reading is about 0.35 ms
const unsigned long PERIMETER_START_OFF_MS = 100;  // The 2 s drive starts on the line: a line only counts as
                                                   // the perimeter once no IR sensor has seen one for this long
const float PERIMETER_BACK_CM = 15.0;       // How far it backs up after seeing the line (cm), so the IR sensors
                                            // are back inside before it turns round
const int PERIMETER_BACK_SPEED = 130;       // Motor power backing up (0-255, like the object finding speeds)
const unsigned long PERIMETER_BACK_MS = 3000;  // Stop backing up after this long (ms), even if it hasn't gone
                                               // PERIMETER_BACK_CM (the wheels are stuck)
const float PERIMETER_OFF_CM = 15.0;        // Once the object it's driving to is this close (cm), the perimeter
                                            // isn't watched for the rest of the approach and the final touch.
                                            // Close to the object, IR bouncing off it can read like the line

// Wall-following settings (the old wall code's, with speeds on the speed sensors' 0-255 scale)
const float WALL_SET_POINT = 15;       // Distance to keep from the left wall (cm)
const float WALL_FOUND_DISTANCE = 35;  // After switching, it drives straight until the left sensor reads closer
                                       // than this (cm), which counts as a wall to follow. It normally finds one
                                       // right away, since switching needs a left reading under
                                       // SWITCH_DISTANCE_CM. (100 cm is where the old code threw away stray far
                                       // readings)
const int WALL_BASE_SPEED = 60;        // Forward speed while wall following
const int WALL_MAX_CORRECTION = 35;    // Most the wall PID can add to one wheel and take from the other. Was 40,
                                       // which let it turn so sharply (wheels at 15 and 95) that it overshot and
                                       // the left sensor lost the wall

// Slowing down for a wall ahead
const int SLOW_DISTANCE = 55;     // Start slowing down when a wall ahead is closer than this (cm)
const int WALL_CRAWL_SPEED = 45;  // Speed just before the turn (raise it if the wheels hum but don't move)
const int TOO_CLOSE_TO_READ = 4;  // If the front sensor loses the echo after reading closer than this (cm),
                                  // the wall is too close to measure, so treat it as right in front

// Wall PID gains (tune Kp first, then Kd. Ki is optional). Separate from the line follower's
const float WALL_Kp = 3.0;  // Was 3.0, which overshot: 5 cm off already gave 15 of steering
const float WALL_Ki = 0.0;
const float WALL_Kd = 1.5;
const float WALL_INTEGRAL_LIMIT = 50;  // Stops the I term from building up too far

// Corner settings
const int FRONT_LIMIT = 13;      // Wall ahead closer than this (cm) = inside corner, turn right
const int FRONT_CONFIRM = 2;     // Readings in a row needed before turning at an inside corner
const int WALL_TURN_SPEED = 40;  // Speed of both wheels spinning in place at a corner

// Right turn at a wall ahead
const float TURN_MATCH = 12;          // Stop turning when the left sensor reads within this many cm of
                                      // how far away the wall was in front
const int WALL_TURN_SLOW_SPEED = 45;  // Spin speed for the end of the turn, once it sees the wall that was in
                                      // front. Lower = stops closer to 90 degrees. Raise it if it stops spinning
const float WALL_TURN_MAX_DEG = 90;   // The most one corner turn spins (degrees, counted by the speed sensors,
                                      // over all its tries, and checked all the time while it spins). In a tight
                                      // spot the ultrasonic sensors can miss the wall and it would spin on past the
                                      // opening. It coasts a few degrees after braking: lower it (85) if it still
                                      // turns too far, raise it if it stops short
const float WALL_TURN_MIN_DEG = 70;   // The ultrasonic sensors can't end a corner turn before it has spun this far
                                      // (degrees). In a tight spot they can think it's lined up with the new wall
                                      // part way round. Stopping there left it at an angle, and it then turned
                                      // again at the same corner, turning far too much in all

// Backing up when it gets too close to the wall ahead to turn
const float TOUCH_CM = 3;            // Measure: a front reading under this (cm) = too close to the wall ahead to spin
                                     // without hitting it, so it backs up and tries again. Put the robot facing a
                                     // wall, move it closer until it can't spin in place without touching, and set
                                     // this a little above the front reading there. Keep it above TOO_CLOSE_TO_READ
const float BACK_UP_CM = 1;          // How far it backs up (cm, counted by the speed sensors) before trying again
const int BACK_UP_SPEED = 55;        // Speed backing up (raise it if the wheels hum but don't move)

// Noticing the wheels aren't turning, while wall following (driving, turning or backing up)
const unsigned long STUCK_MS = 500;  // How long it counts speed sensor ticks for each check (ms)
const unsigned long STUCK_TICKS = 10;  // Fewer ticks than this in STUCK_MS (both wheels added) = the wheels aren't
                                       // turning: back up and try the turn again. Normal driving and turning make
                                       // about 50-60. A stalled motor can still twitch out the odd tick, so it
                                       // counts ticks instead of waiting for none at all

// Print the sensor readings to the Serial Monitor (115200) while tuning (both modes)
const bool PRINT_READINGS = false;

// Which way to spin
const int SIDE_LEFT = 1;
const int SIDE_RIGHT = 0;

// Latest IR readings
int rawLeft = 0;
int rawMid = 0;
int rawRight = 0;
bool seeLeft = false;  // Over the line
bool seeMid = false;
bool seeRight = false;
bool squareLeft = false;  // Squarely over a line (after CONFIRM readings in a row agree)
bool squareRight = false;
int leftSquareCount = 0;  // Readings in a row disagreeing with squareLeft / squareRight
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

unsigned long midOnSince = 0;     // When the middle sensor last came onto a line (ms)
unsigned long lastRightTime = 0;  // When the right sensor was last squarely over a line (ms), 0 if never

// A way right the robot may turn onto
bool rightBranch = false;      // The right sensor went squarely onto a line while running steadily along the line
unsigned long rightOnAt = 0;   // When the right sensor found it
unsigned long rightOffAt = 0;  // When the right sensor went past it with the middle still on a line
                               // (0 = not yet)

// A "way" is a side sensor going squarely onto a line while the middle is already on the line: maybe a way
// off to that side. If the line ends soon after, the newest way says which way the line went
bool leftWay = false;  // The left / right sensor is on a way right now
bool rightWay = false;
unsigned long leftWayStart = 0;  // When the left / right sensor found its latest way (ms)
unsigned long rightWayStart = 0;
unsigned long leftWaySeen = 0;  // Last time the left / right sensor was on that way (ms, 0 = none)
unsigned long rightWaySeen = 0;

int turningSide = -1;  // The way it's turning (a corner, curve or junction) until it's running
                       // steadily along the line again, -1 = not turning. If the line is lost
                       // part way round, it keeps turning this way

// PID state
float integral = 0;
float prevError = 0;
unsigned long lastTime = 0;
bool pidJustReset = true;  // Skip the D term on the first reading after a reset or turn

// Bayes filter state
float placePosition[NUM_PLACES];      // Where each place is, in sensor spacings (-1 = left sensor)
float expectedOnLine[NUM_PLACES][3];  // What onLine() should read on the left, middle and right sensors
                                      // if the line is at that place
float belief[NUM_PLACES];             // Probability the line is at each place

unsigned long lostSince = 0;  // When no sensor has seen the line since (ms), 0 = a sensor sees it

// Mode
bool wallMode = false;            // false = line following, true = wall following (for good)
bool objectMode = false;          // true = object finding (for good), after wall following
bool perimeterHit = false;        // An IR sensor saw the perimeter line: back up, turn round and scan again
int perimeterCount = 0;           // IR readings in a row that saw the line while driving forward
bool nearTarget = false;          // Within PERIMETER_OFF_CM of the object it's driving to: perimeter not watched
int objLeftPower = 0;             // Motor power object finding last gave each wheel (see objDrive())
int objRightPower = 0;
unsigned long allIRSince = 0;     // When all three IR sensors started seeing the line together while wall
                                  // following (ms), 0 = they don't right now
bool allIRSeen = false;           // All three IR sensors have seen the line together for ALL_IR_HOLD_MS while wall
                                  // following (driving, turning or backing up): switch to object finding
unsigned long lastWallCheck = 0;  // When an ultrasonic sensor was last read while line following (ms)
int wallCount = 0;                // Front readings in a row under SWITCH_FRONT_CM
bool leftWallClose = false;       // The left sensor's latest reading was under SWITCH_DISTANCE_CM
bool readLeftNext = false;        // The next check reads the left sensor instead of the front

// Wall PID state
float wallIntegral = 0;
float wallPrevError = 0;
unsigned long wallLastTime = 0;
bool wallPidJustReset = true;  // Skip the D term on the first reading after a reset or turn
bool wallSeen = false;         // Has the left sensor found a wall to follow yet? Until then, drive straight
unsigned long stuckCheckAt = 0;     // When the current STUCK_MS of tick counting started (ms)
unsigned long stuckCheckTicks = 0;  // Ticks (both wheels added) when it started

// Corner detection state
int frontCount = 0;    // Readings in a row with a wall close ahead
float lastFront = -1;  // Last good front reading (cm), -1 if none since the last turn
int frontMisses = 0;   // Front readings in a row with no echo

// Corner turn state: how far the current corner turn has spun, counted by the speed sensors
float cornerTurnedBefore = 0;       // Degrees spun in earlier tries at this corner (before backing up)
unsigned long cornerStartLeft = 0;  // Tick counts when this try started
unsigned long cornerStartRight = 0;

// Speed sensor state, changed by the interrupts (volatile). Read them with readTicks()
volatile unsigned long leftTicks = 0;                // Ticks counted since power on (they only go up: the sensor can't
volatile unsigned long rightTicks = 0;               // tell forward from backward)
volatile unsigned long leftTickAt[3] = { 0, 0, 0 };  // Times (µs) of the latest three ticks, newest first
volatile unsigned long rightTickAt[3] = { 0, 0, 0 };

// Speed control: what each wheel is asked for (-255..255) and the correction built up for it
int leftTarget = 0;
int rightTarget = 0;
float leftBoost[2] = { 0, 0 };  // Correction built up for each wheel, [0] forward, [1] backward
float rightBoost[2] = { 0, 0 };
float leftError = 0;  // Latest speed error (asked for - measured, 0-255 scale)
float rightError = 0;
unsigned long lastSpeedUpdate = 0;
float leftMeasured = 0;  // Latest measured speeds (0-255 scale), for printing
float rightMeasured = 0;

// Counting how far a spin has turned
unsigned long spinStartLeft = 0;  // Tick counts when the spin started
unsigned long spinStartRight = 0;
unsigned long spinLastTotal = 0;   // Ticks so far (both wheels added), to notice a stall
unsigned long spinLastTickMs = 0;  // When the last of those ticks came


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

  // "L" LED: off until it switches to wall following
  pinMode(WALL_LED_PIN, OUTPUT);
  digitalWrite(WALL_LED_PIN, LOW);

  // The IR sensors are read with analogRead, which needs no pinMode

  // Speed sensors: count every edge on D0
  pinMode(leftEncoder_PIN, INPUT);
  pinMode(rightEncoder_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftEncoder_PIN), leftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder_PIN), rightTick, CHANGE);

  // Start stopped
  stop();

  // Serial is only for watching the numbers while tuning over USB.
  // The robot runs the same with nothing plugged in.
  Serial.begin(115200);

  // Two seconds to set the robot down on the line after switching it on. Meanwhile the servo turns the front
  // sensor straight ahead, and updateMotors() keeps it there until object finding
  pinMode(servoPin, OUTPUT);
  digitalWrite(servoPin, LOW);
  pointServo(servoStraight, 2000);

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

// Power one motor: positive = forward, negative = backward
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

// A wheel being given power but not turning (its speed sensor hasn't ticked for STOPPED_US, or it hasn't moved
// yet) gets at least START_POWER, the same way it's going, so it breaks free instead of humming. As soon as it
// ticks, it's back to the normal power
int withKick(int power, float measured) {
  if (power == 0 || measured > 0) return power;
  return power > 0 ? max(power, START_POWER) : min(power, -START_POWER);
}

void applyPower() {
  setLeftMotor(withKick(wheelPower(leftTarget, leftError, leftBoost[leftTarget < 0]), leftMeasured));
  setRightMotor(withKick(wheelPower(rightTarget, rightError, rightBoost[rightTarget < 0]), rightMeasured));
}

// Set each wheel separately (0-255 scale, 255 = TOP_SPEED_MM_S): positive = forward, negative = backward.
// The speed sensors then hold each wheel at that speed (see updateMotors), so equal speeds are really equal
void drive(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // A wheel changing direction: the last speed error was about the other direction
  if ((leftSpeed < 0) != (leftTarget < 0) || leftSpeed == 0) leftError = 0;
  if ((rightSpeed < 0) != (rightTarget < 0) || rightSpeed == 0) rightError = 0;

  leftTarget = leftSpeed;
  rightTarget = rightSpeed;
  applyPower();
  updateMotors();
}

// One wheel's speed control step: compare the measured speed with the target, and build up the correction
void correctWheel(int target, float measured, float &error, float &boost, float dt) {
  if (target == 0) {
    error = 0;
    return;
  }
  error = abs(target) - measured;

  // Build up the correction, but not while the motor is already at full (or no) power and can't follow it
  int power = abs(wheelPower(target, error, boost));
  bool stuckHigh = power >= 255 && error > 0;
  bool stuckLow = power <= 0 && error < 0;
  if (!stuckHigh && !stuckLow) {
    boost = constrain(boost + SPEED_KI * error * dt, -SPEED_I_LIMIT, SPEED_I_LIMIT);
  }
}

// Check both wheel speeds and correct them. Call this often: it does the work every SPEED_INTERVAL_MS.
// It also keeps the servo holding the front sensor straight ahead (a pulse every 20 ms), and while wall
// following it reads the IR sensors on every call (see watchIR()), so they're watched while driving, turning
// and backing up
void updateMotors() {
  holdServo();
  if (objectMode) return;  // Object finding drives the motors its own way
  if (wallMode) watchIR();  // All three IR sensors on the line = time to switch to object finding

  unsigned long now = millis();
  if (now - lastSpeedUpdate < SPEED_INTERVAL_MS) return;
  float dt = min((now - lastSpeedUpdate) / 1000.0, 0.05);  // After a pause, don't build up a big jump
  lastSpeedUpdate = now;

  leftMeasured = wheelSpeed(leftTickAt);
  rightMeasured = wheelSpeed(rightTickAt);

  // Each wheel keeps a separate correction for forward and backward (index 0 / 1): a motor isn't the
  // same both ways, and this way what it learned going forward is still there after a spin
  correctWheel(leftTarget, leftMeasured, leftError, leftBoost[leftTarget < 0], dt);
  correctWheel(rightTarget, rightMeasured, rightError, rightBoost[rightTarget < 0], dt);
  applyPower();
}

// Spin in place around the middle of the wheel axle: one wheel forward, the other backward
void spin(int side) {
  if (side == SIDE_RIGHT) {
    drive(TURN_SPEED, -TURN_SPEED);  // Left wheel forward, right wheel backward
  } else {
    drive(-TURN_SPEED, TURN_SPEED);  // Left wheel backward, right wheel forward
  }
}

int otherSide(int side) {
  return (side == SIDE_RIGHT) ? SIDE_LEFT : SIDE_RIGHT;
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


// -------------------------
// Speed sensors (LM393 encoders)
// -------------------------

// Spinning in place, each wheel rolls round a circle WHEEL_SPACING_MM across, so one tick turns the robot
// this many degrees (about 4.5° with 20 slots, 70 mm wheels 140 mm apart)
const float DEG_PER_TICK = 360.0 * MM_PER_TICK / (PI * WHEEL_SPACING_MM);

// Interrupts: run on every edge of each sensor's D0. Edges too close to the last one are flicker
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

// Copy both tick counts (with the interrupts paused, so they can't change half way through)
void readTicks(unsigned long &left, unsigned long &right) {
  noInterrupts();
  left = leftTicks;
  right = rightTicks;
  interrupts();
}

// How fast one wheel is turning, on the 0-255 scale (255 = TOP_SPEED_MM_S).
// Uses the time across the last whole slot (two ticks), which is steadier than counting ticks
float wheelSpeed(volatile unsigned long *tickAt) {
  noInterrupts();
  unsigned long newest = tickAt[0];
  unsigned long twoBefore = tickAt[2];
  interrupts();

  unsigned long now = micros();
  if (twoBefore == 0 || now - newest > STOPPED_US) return 0;  // Stopped, or not moved yet

  unsigned long slotTime = newest - twoBefore;
  if (now - newest > slotTime) slotTime = now - newest;  // Slowing down: no tick for over a slot

  float mmPerSec = 2 * MM_PER_TICK * 1000000.0 / slotTime;
  return mmPerSec / TOP_SPEED_MM_S * 255;
}

// Start spinning toward `side` and start counting how far it turns
void startSpin(int side) {
  resetWallCheck();  // A turn changes what's ahead, so the wall-ahead check starts over
  readTicks(spinStartLeft, spinStartRight);
  spinLastTotal = 0;
  spinLastTickMs = millis();
  spin(side);
}

// How far the spin has turned (degrees), from the average of the two wheels' ticks
float spunDegrees() {
  unsigned long left, right;
  readTicks(left, right);
  unsigned long total = (left - spinStartLeft) + (right - spinStartRight);
  if (total != spinLastTotal) {
    spinLastTotal = total;
    spinLastTickMs = millis();
  }
  return total / 2.0 * DEG_PER_TICK;
}

// Call in a spin's loop: keeps the wheel speeds matched, and returns false once the spin has turned
// `maxDegrees` (or has stalled)
bool keepSpinning(float maxDegrees) {
  updateMotors();
  if (spunDegrees() >= maxDegrees) return false;
  if (millis() - spinLastTickMs > STALL_MS) {
    Serial.println(F("Spin stalled: no ticks from the speed sensors"));
    return false;
  }
  return true;
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

// Read one IR sensor, flipped if needed so the line always reads HIGH
int readIR(int pin) {
  int value = analogRead(pin);
  return INVERT_READINGS ? 1023 - value : value;
}

// Read all three IR sensors. The line reads higher than the floor
void readLineSensors() {
  bool midBefore = seeMid;

  rawLeft = readIR(leftIR_PIN);
  rawMid = readIR(midIR_PIN);
  rawRight = readIR(rigIR_PIN);

  seeLeft = rawLeft > LINE_THRESHOLD;
  seeMid = rawMid > LINE_THRESHOLD;
  seeRight = rawRight > LINE_THRESHOLD;
  // Squarely on a line only counts once CONFIRM readings in a row agree, and so does coming off it,
  // so a reading that flickers around SQUARE_THRESHOLD doesn't look like a junction
  squareLeft = settle(rawLeft > SQUARE_THRESHOLD, squareLeft, leftSquareCount);
  squareRight = settle(rawRight > SQUARE_THRESHOLD, squareRight, rightSquareCount);

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
void printStatus(const __FlashStringHelper *action) {
  if (!PRINT_READINGS) return;

  Serial.print(F("L: "));
  Serial.print(rawLeft);
  Serial.print(F("  M: "));
  Serial.print(rawMid);
  Serial.print(F("  R: "));
  Serial.print(rawRight);
  Serial.print(F("  Sees: "));
  Serial.print(seeLeft ? 'L' : '-');
  Serial.print(seeMid ? 'M' : '-');
  Serial.print(seeRight ? 'R' : '-');
  Serial.print(F("  "));
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
  resetBelief();  // After a turn the old belief is about the old line
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
      float awayMm = (placePosition[k] - (i - 1)) * SENSOR_SPACING_MM;  // Sensors at -1, 0, +1
      expectedOnLine[k][i] = exp(-awayMm * awayMm / (2 * SENSOR_SEES_MM * SENSOR_SEES_MM));
    }
  }
  resetBelief();
}

// Bayes' theorem with the latest readings: new belief = prior x likelihood, divided by the total
void updateBelief(float left, float mid, float right) {
  float reading[3] = { left, mid, right };

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

  return -LINE_POSITION_RANGE * average;  // Places count left as negative; the PID wants left positive
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
    Serial.print(F("L: "));
    Serial.print(rawLeft);
    Serial.print(F("  M: "));
    Serial.print(rawMid);
    Serial.print(F("  R: "));
    Serial.print(rawRight);
    Serial.print(F("  Error: "));
    Serial.print(error);
    Serial.print(F("  PID: "));
    Serial.print(pid);
    Serial.print(F("  L: "));
    Serial.print(leftSpeed);
    Serial.print(F("  R: "));
    Serial.print(rightSpeed);
    Serial.print(F("  Measured L: "));  // What the speed sensors say the wheels are doing (0-255 scale)
    Serial.print(leftMeasured);
    Serial.print(F("  R: "));
    Serial.println(rightMeasured);
  }
}


// -------------------------
// Turning
// -------------------------

// Spin toward `side` until the middle sensor finds a line, then hand back to the main loop.
// If the middle sensor is on a line when the turn starts (turning off the line at a junction),
// it first spins until the middle sensor is off that line, so it doesn't stop on the line it's leaving.
// The side sensors are ignored here: one of them can swing back over the line the robot came along.
// Returns false if there's no line within `maxDegrees` (the safety limit, counted by the speed sensors).
bool spinOntoLine(int side, float maxDegrees) {
  Serial.println(side == SIDE_RIGHT ? F("Spinning right onto the line") : F("Spinning left onto the line"));

  // Start fresh: anything seen before this turn was about the old line
  rightBranch = false;
  leftWay = false;
  rightWay = false;
  leftWaySeen = 0;
  rightWaySeen = 0;
  turningSide = side;
  resetPID();

  bool offOldLine = !seeMid;
  int count = 0;  // Readings in a row that agree

  startSpin(side);
  while (keepSpinning(maxDegrees)) {
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

// Spin toward `side` for up to `maxDegrees` (counted by the speed sensors), stopping as soon as any
// sensor sees a line. Returns true if it found one
bool spinUntilAnySensor(int side, float maxDegrees) {
  resetPID();
  startSpin(side);
  while (keepSpinning(maxDegrees)) {
    readLineSensors();
    if (anySeesLine()) return true;
  }
  return false;
}

// The line slipped out from under the middle sensor with nothing else seeing it: either it's just beside
// the robot (between two sensors) or it has ended. Look about 30° each way, starting on the side whose
// reading is closer to the line's (higher), coming back to the start after each look.
// Returns true if it found the line
bool lookEachWay() {
  Serial.println(F("Lost line: looking a little each way"));
  int first = (rawRight >= rawLeft) ? SIDE_RIGHT : SIDE_LEFT;

  if (spinUntilAnySensor(first, LOOK_DEG)) return true;
  if (spinUntilAnySensor(otherSide(first), LOOK_DEG)) return true;  // Back to the start
  if (spinUntilAnySensor(otherSide(first), LOOK_DEG)) return true;
  if (spinUntilAnySensor(first, LOOK_DEG)) return true;  // Back to the start
  return false;
}

// No line within the search: don't give up, and don't switch to wall following (only checkForWall() does
// that). The main loop drives on for LOST_MS, then lineLost() searches again
void lineNotFound() {
  Serial.println(F("No line found: searching again"));
}

// Which side did the newest way off the line go, if one was seen just before now?
// Returns SIDE_LEFT, SIDE_RIGHT, or -1 for none
int newestWay(unsigned long now) {
  bool leftRecent = sawRecently(leftWaySeen, now);
  bool rightRecent = sawRecently(rightWaySeen, now);

  if (leftRecent && rightRecent) {
    unsigned long apart = (leftWayStart > rightWayStart) ? leftWayStart - rightWayStart
                                                         : rightWayStart - leftWayStart;
    if (apart <= TOGETHER_MS) return SIDE_RIGHT;  // Found together: a T-junction. Right-hand rule
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
    // Spin that way until the middle sensor finds the line
    if (spinOntoLine(side, SEARCH_LIMIT_DEG)) return;

    // Nothing that way within the safety limit. At a corner it may have picked the wrong side, so spin back past
    // where it started and look as far the other way. (Searching on the same way instead swings it round onto
    // the line it came along, at 180°, and it drives back.) Nothing there either: the line has ended, so search
    // again, which turns it round
    float turned = spunDegrees();
    brake();
    Serial.println(F("No line that way: looking the other way"));
    if (!spinOntoLine(otherSide(side), turned + SEARCH_LIMIT_DEG)) lineNotFound();
    return;
  }

  // Only the middle sensor saw it last, and it wasn't turning: the line has slipped out from under it, or ended
  if (lookEachWay()) return;
  lineNotFound();
}


// -------------------------
// Switching to wall following
// -------------------------

// Start the wall-ahead check over: nothing within SWITCH_FRONT_CM ahead, or the robot has turned
void resetWallCheck() {
  wallCount = 0;
  leftWallClose = false;
  readLeftNext = false;
}

// While line following: every WALL_CHECK_MS, read one ultrasonic sensor. It switches to wall following when
// the front sensor reads under SWITCH_FRONT_CM (SWITCH_COUNT front readings in a row) and the left sensor's
// latest reading is under SWITCH_DISTANCE_CM. No echo = skipped. Returns true if it switched
bool checkForWall() {
  if (millis() - lastWallCheck < WALL_CHECK_MS) return false;
  lastWallCheck = millis();

  if (readLeftNext) {
    // A wall ahead is within SWITCH_FRONT_CM: is there a wall close on the left too?
    readLeftNext = false;
    float left = readDistance(leftTrig, leftEcho);
    if (left > 0) leftWallClose = left < SWITCH_DISTANCE_CM;
  } else {
    float front = readDistance(frontTrig, frontEcho);
    if (front > 0) {
      if (front < SWITCH_FRONT_CM) wallCount++;  // A wall close ahead
      else resetWallCheck();                     // Nothing close ahead: start the count again
    }
    // While a wall ahead is within SWITCH_FRONT_CM, the next check reads the left sensor
    readLeftNext = wallCount > 0;
  }

  if (wallCount >= SWITCH_COUNT && leftWallClose) {
    Serial.println(F("Wall close ahead, wall on the left"));
    startWallMode();
    return true;
  }
  return false;
}

// Stop line following and start wall following, for good
void startWallMode() {
  brake();
  wallMode = true;
  digitalWrite(WALL_LED_PIN, HIGH);  // "L" LED on: wall following from here on
  wallSeen = false;
  resetWallPID();
  Serial.println(F("Switching to wall following"));
}


// -------------------------
// Wall following
// -------------------------

// Wait, keeping the wheel speeds matched (instead of delay(), which would stop the speed control)
void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) updateMotors();
}

// Start the wall PID fresh: when wall following starts and after every corner turn
void resetWallPID() {
  wallIntegral = 0;
  wallPidJustReset = true;
  frontCount = 0;
  lastFront = -1;
  frontMisses = 0;
  wallLastTime = millis();
  resetStuckCheck();  // Count the wheels' ticks afresh
}

// Forward speed for how close the wall ahead is: full speed when far, slower the closer it gets
int speedForFront(float frontDistance) {
  if (frontDistance < 0 || frontDistance >= SLOW_DISTANCE) {
    return WALL_BASE_SPEED;  // Nothing close ahead
  }

  // 0 at FRONT_LIMIT, 1 at SLOW_DISTANCE
  float fraction = (frontDistance - FRONT_LIMIT) / (SLOW_DISTANCE - FRONT_LIMIT);
  fraction = constrain(fraction, 0.0, 1.0);

  return WALL_CRAWL_SPEED + int(fraction * (WALL_BASE_SPEED - WALL_CRAWL_SPEED));
}

// Steer to hold WALL_SET_POINT from the left wall, driving forward at the given speed
void followWall(float leftDistance, float frontDistance, int speed) {
  // Time since the previous PID update (in seconds)
  unsigned long now = millis();
  float dt = (now - wallLastTime) / 1000.0;
  wallLastTime = now;

  // P: positive error = too far from the wall, negative = too close
  float error = leftDistance - WALL_SET_POINT;

  // I: add up the error over time
  wallIntegral += error * dt;
  wallIntegral = constrain(wallIntegral, -WALL_INTEGRAL_LIMIT, WALL_INTEGRAL_LIMIT);

  // D: how quickly the error is changing
  float derivative = 0;
  if (!wallPidJustReset && dt > 0) {
    derivative = (error - wallPrevError) / dt;
  }
  wallPrevError = error;
  wallPidJustReset = false;

  float pid = WALL_Kp * error
              + WALL_Ki * wallIntegral
              + WALL_Kd * derivative;
  pid = constrain(pid, -WALL_MAX_CORRECTION, WALL_MAX_CORRECTION);

  // Shrink the steering along with the speed, so slowing down doesn't make it turn harder
  int correction = int(pid * speed / WALL_BASE_SPEED);

  // Positive pid steers left, toward the wall: left wheel slower, right wheel faster
  int leftSpeed = constrain(speed - correction, -255, 255);
  int rightSpeed = constrain(speed + correction, -255, 255);
  drive(leftSpeed, rightSpeed);

  // Open the Serial Monitor at 115200 over USB to watch these while tuning
  if (PRINT_READINGS) {
    Serial.print(F("Front: "));
    Serial.print(frontDistance);
    Serial.print(F("  Left: "));
    Serial.print(leftDistance);
    Serial.print(F("  Error: "));
    Serial.print(error);
    Serial.print(F("  PID: "));
    Serial.print(pid);
    Serial.print(F("  L: "));
    Serial.print(leftSpeed);
    Serial.print(F("  R: "));
    Serial.print(rightSpeed);
    Serial.print(F("  Measured L: "));
    Serial.print(leftMeasured);
    Serial.print(F("  R: "));
    Serial.println(rightMeasured);
  }
}

// Both wheels' ticks added together, for noticing when the wheels stop turning
unsigned long totalTicks() {
  unsigned long left, right;
  readTicks(left, right);
  return left + right;
}

// Is the front of the robot touching (or nearly touching) the wall ahead? `front` is the latest front reading
// (-1 = no echo) and `before` the last one that got an echo (-1 = none). Right up against a wall the sensor
// often gets no echo at all, so no echo just after a reading under TOUCH_CM counts too
bool touchingAhead(float front, float before) {
  if (front > 0) return front < TOUCH_CM;
  return before > 0 && before < TOUCH_CM;
}

// Start counting the wheels' ticks afresh for wheelsStuck()
void resetStuckCheck() {
  stuckCheckAt = millis();
  stuckCheckTicks = totalTicks();
}

// Call often while the wheels are being driven. Every STUCK_MS it checks how many ticks the speed sensors
// counted: fewer than STUCK_TICKS (both wheels added) means the wheels aren't turning, and it returns true
bool wheelsStuck() {
  if (millis() - stuckCheckAt < STUCK_MS) return false;
  bool stuck = totalTicks() - stuckCheckTicks < STUCK_TICKS;
  resetStuckCheck();
  return stuck;
}

// Drive straight back about `cm` (counted by the speed sensors), then brake. Stops early if the wheels
// aren't turning (something behind it)
void backUp(float cm) {
  Serial.println(F("Backing up"));
  unsigned long start = totalTicks();
  unsigned long needed = (unsigned long)(2 * cm * 10 / MM_PER_TICK + 0.5);  // Both wheels' ticks added together

  drive(-BACK_UP_SPEED, -BACK_UP_SPEED);
  resetStuckCheck();
  while (totalTicks() - start < needed) {
    updateMotors();
    if (allIRSeen) return;  // All three IR sensors see the line: loop() brakes and switches to object finding
    if (wheelsStuck()) {
      Serial.println(F("Backing up stalled"));
      break;
    }
  }
  brake();
  delay(100);
}

// Inside corner: a wall straight ahead.
// Spin right in place until the left sensor sees that wall and the front sensor is clear (only once it has
// turned WALL_TURN_MIN_DEG), or it has turned WALL_TURN_MAX_DEG. If the front of the robot is touching the wall
// (too close to spin), or it gets stuck part way round, back up a little and try again (spinning only the rest
// of the way)
void insideCorner() {
  Serial.println(F("Inside corner: turning right"));

  stop();
  waitMs(100);  // Not delay(), so the IR sensors are still watched while it coasts to a stop

  cornerTurnedBefore = 0;
  while (!tryRightTurn()) {
    if (allIRSeen) return;  // All three IR sensors see the line: loop() switches to object finding
    cornerTurnedBefore = cornerTurned();  // Keep what this try turned (backing up makes ticks too)
    Serial.println(F("Stuck or touching the wall ahead: backing up to try the turn again"));
    backUp(BACK_UP_CM);
    lastFront = -1;  // It has moved, so the old front reading no longer counts
  }
  if (allIRSeen) return;  // They saw it on the last step of the turn

  // Brake instead of coasting, so it doesn't keep spinning past the wall
  brake();
  delay(100);

  // How far it really turned, coasting included. Open the Serial Monitor (115200) to see it
  Serial.print(F("Turn done: "));
  Serial.print(cornerTurned());
  Serial.println(F(" degrees"));

  resetWallPID();
}

// How far the current corner turn has spun (degrees): earlier tries plus this one, from the speed sensors
float cornerTurned() {
  unsigned long left, right;
  readTicks(left, right);
  return cornerTurnedBefore + ((left - cornerStartLeft) + (right - cornerStartRight)) / 2.0 * DEG_PER_TICK;
}

// Wait like waitMs() while spinning at a corner, but stop as soon as the turn reaches WALL_TURN_MAX_DEG, so it
// doesn't spin on past it while waiting between sensor readings. Returns true if it has
bool waitTurning(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateMotors();
    if (cornerTurned() >= WALL_TURN_MAX_DEG) return true;
  }
  return false;
}

// One try at the right turn. Returns true once the turn is done. Returns false, stopped, if the front of the
// robot is touching the wall ahead (before or during the spin) or the wheels get stuck, so insideCorner()
// can back up and try again. Also returns false as soon as all three IR sensors see the line (allIRSeen).
// It adds to how far earlier tries turned (cornerTurnedBefore), and stops once it reaches WALL_TURN_MAX_DEG
bool tryRightTurn() {
  readTicks(cornerStartLeft, cornerStartRight);  // Count this try's spin from here
  if (allIRSeen) return false;
  if (cornerTurned() >= WALL_TURN_MAX_DEG) return true;  // Earlier tries already turned far enough

  // How far away the wall ahead is. After the turn, the left sensor should see it about this far away.
  float wallDistance = readDistance(frontTrig, frontEcho);
  if (touchingAhead(wallDistance, lastFront)) return false;  // Too close to spin: back up first
  if (wallDistance > 0) lastFront = wallDistance;
  if (wallDistance < 0) wallDistance = FRONT_LIMIT;

  drive(WALL_TURN_SPEED, -WALL_TURN_SPEED);  // Spin right: left wheel forward, right wheel backward
  bool frontClear = false;
  bool onNewWall = false;  // Is the left sensor on the wall that was in front yet?
  float farthest = -1;     // Farthest left reading so far in this turn
  float lastGood = -1;     // Last left reading that got an echo
  float prevLeftCm = -1;   // The reading just before this one, on the same wall (-1 if none)
  int misses = 0;          // Left readings in a row with no echo
  resetStuckCheck();

  // Keep turning until the sensors say it's done
  while (true) {
    if (allIRSeen) return false;  // All three IR sensors see the line: stop turning

    // Turned as far as a corner needs: done, even if the ultrasonic sensors haven't said so yet
    float turned = cornerTurned();
    if (turned >= WALL_TURN_MAX_DEG) return true;

    // Nearly there once it sees the wall that was in front, or once it's past WALL_TURN_MIN_DEG: slow down so
    // it doesn't overshoot
    if (onNewWall || turned >= WALL_TURN_MIN_DEG) drive(WALL_TURN_SLOW_SPEED, -WALL_TURN_SLOW_SPEED);

    // Check the front until it's clear. Once it is, it stays clear for the rest of the turn,
    // so after that only the left sensor is read, which checks twice as often
    if (!frontClear) {
      float front = readDistance(frontTrig, frontEcho);
      if (touchingAhead(front, lastFront)) {
        brake();
        return false;  // The front has come up against the wall while spinning
      }
      if (front > 0) lastFront = front;
      frontClear = front < 0 || front > FRONT_LIMIT;
      if (waitTurning(20)) return true;
      if (allIRSeen) return false;
    }

    // The wheels aren't turning: it's jammed against the wall
    if (wheelsStuck()) {
      brake();
      return false;
    }

    float left = readDistance(leftTrig, leftEcho);
    if (waitTurning(20)) return true;
    if (allIRSeen) return false;

    if (PRINT_READINGS) {
      Serial.print(F("Turning  front clear: "));
      Serial.print(frontClear);
      Serial.print(F("  left: "));
      Serial.println(left);
    }

    if (left < 0) {
      misses++;
      if (misses >= 2) onNewWall = true;  // Lost sight of the old left wall
      prevLeftCm = -1;
      continue;
    }
    misses = 0;

    // While spinning right, the old left wall's reading grows smoothly. The wall that was in front
    // shows up as a sudden jump to a farther reading, or as the reading coming back down.
    if (lastGood > 0 && left > lastGood + TURN_MATCH) {
      onNewWall = true;
      prevLeftCm = -1;  // Different wall, so don't compare with the old one
    }
    if (left < farthest - 2) onNewWall = true;
    if (left > farthest) farthest = left;
    lastGood = left;

    bool leftOnWall = fabs(left - wallDistance) <= TURN_MATCH;  // About as far as it was in front

    // Stop once the reading stops getting smaller: that's when the sensor points straight at the wall
    bool stoppedGettingCloser = prevLeftCm > 0 && left >= prevLeftCm - 1;

    // The ultrasonic sensors say it's lined up with the new wall. Only trusted once it has turned
    // WALL_TURN_MIN_DEG: before that, in a tight spot, they can be fooled part way round
    bool farEnough = cornerTurned() >= WALL_TURN_MIN_DEG;
    if (farEnough && onNewWall && frontClear && leftOnWall && stoppedGettingCloser) return true;

    prevLeftCm = left;
  }
}

// One pass of wall following (called from loop() once wallMode is on)
void wallLoop() {
  updateMotors();

  // Read one sensor at a time, with a short gap so they don't hear each other's echoes. The IR sensors are
  // read the whole time in the gaps (by updateMotors()). As soon as all three see the line, stop here:
  // loop() switches to object finding
  float front = readDistance(frontTrig, frontEcho);
  waitMs(10);
  if (allIRSeen) return;
  float left = readDistance(leftTrig, leftEcho);
  waitMs(10);
  if (allIRSeen) return;

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

  // The wheels aren't turning: it has run into a wall the front sensor didn't see (at an angle, or right up
  // against it, it can lose the echo). Back up 4 cm and turn
  if (wheelsStuck()) {
    Serial.println(F("Stuck against the wall ahead"));
    brake();
    backUp(BACK_UP_CM);
    if (allIRSeen) return;  // All three IR sensors saw the line while backing up
    lastFront = -1;  // It has moved, so the old front reading no longer counts
    insideCorner();
    return;
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

  // No wall found on the left yet (the left sensor missed its echo just after switching): drive straight
  // until there is one
  if (!wallSeen) {
    if (left > 0 && left < WALL_FOUND_DISTANCE) {
      wallSeen = true;
      Serial.println(F("Found the wall: following it"));
    } else {
      drive(speedForFront(front), speedForFront(front));
      wallLastTime = millis();
      return;
    }
  }

  // No echo from the left sensor: drive straight until it gets a reading. It gets no echo when it's at a steep
  // angle to the wall, so keeping the wheel speeds it had (as it used to) meant that if it lost the wall part
  // way through a sharp turn, it kept turning until it faced back the way it came
  if (left < 0) {
    drive(speedForFront(front), speedForFront(front));
    return;
  }

  // Slow down as a wall ahead gets closer
  followWall(left, front, speedForFront(front));
}


// -------------------------
// Switching to object finding
// -------------------------

// Read the IR sensors and note if all three see the line at the same time. Called from updateMotors() on
// every call while wall following, so it's watched whatever the robot is doing (driving, turning at a corner
// or backing up). Once all three have been on the line for ALL_IR_HOLD_MS, with no reading in between where
// one of them was off, it switches: the wall code stops what it's doing and loop() calls startObjectMode().
// A pause with no readings (while an ultrasonic sensor is read) doesn't break the count
void watchIR() {
  readLineSensors();
  if (!(seeLeft && seeMid && seeRight)) {
    allIRSince = 0;  // One of them is off the line: start the count again
    return;
  }
  unsigned long now = millis();
  if (allIRSince == 0) allIRSince = now;
  if (now - allIRSince >= ALL_IR_HOLD_MS) allIRSeen = true;
}

// Drive ahead for SWITCH_STRAIGHT_MS, with the left wheel at SWITCH_STRAIGHT_SPEED and the right wheel
// SWITCH_RIGHT_FASTER times that (5% faster), then brake. The speed sensors hold each wheel at its speed (see
// updateMotors()). On top of that, the ticks are checked: the right wheel should have gone SWITCH_RIGHT_FASTER
// times as many as the left. If either wheel is ahead of that, it's slowed down and the other sped up a little
void driveStraightAfterSwitch() {
  unsigned long startLeft, startRight;
  readTicks(startLeft, startRight);
  int rightSpeed = (int)(SWITCH_STRAIGHT_SPEED * SWITCH_RIGHT_FASTER + 0.5);  // 55 x 1.05 = 58
  drive(SWITCH_STRAIGHT_SPEED, rightSpeed);

  unsigned long start = millis();
  unsigned long lastFix = start;
  unsigned long offSince = 0;  // When the IR sensors went off the line (ms), 0 = a sensor sees a line now
  bool offStartLine = false;   // The IR sensors have been off the line it switched on for PERIMETER_START_OFF_MS
  int lineCount = 0;
  while (millis() - start < SWITCH_STRAIGHT_MS) {
    updateMotors();  // This also reads the IR sensors (watchIR(), as wallMode is still on)

    // It starts on the perimeter line. Once off it, seeing a line again = the far side of the perimeter
    if (!anySeesLine()) {
      if (offSince == 0) offSince = millis();
      if (millis() - offSince >= PERIMETER_START_OFF_MS) offStartLine = true;
      lineCount = 0;
    } else {
      offSince = 0;
      if (offStartLine && ++lineCount >= PERIMETER_CONFIRM) {
        brake();
        perimeterHit = true;  // Object finding backs up and turns round before it scans
        Serial.println(F("IR sees the perimeter: stopped the 2 s drive"));
        return;
      }
    }

    if (millis() - lastFix < SPEED_INTERVAL_MS) continue;
    lastFix = millis();

    unsigned long left, right;
    readTicks(left, right);
    // + = left wheel ahead (the right wheel should have gone SWITCH_RIGHT_FASTER times as many ticks)
    float ahead = (float)(left - startLeft) * SWITCH_RIGHT_FASTER - (float)(right - startRight);
    int fix = constrain((int)(ahead * SWITCH_STRAIGHT_KP), -SWITCH_STRAIGHT_MAX_FIX, SWITCH_STRAIGHT_MAX_FIX);
    drive(SWITCH_STRAIGHT_SPEED - fix, rightSpeed + fix);
  }
  brake();
}

// Stop wall following and start object finding, for good
void startObjectMode() {
  brake();
  Serial.println(F("All three IR sensors see the line: switching to object finding"));
  delay(100);
  Serial.println(F("Driving straight for 2 s"));
  driveStraightAfterSwitch();  // Before objectMode is on, so the speed control still works
  objectMode = true;
  startObjectFinding();
}


// =====================================================================================================
// OBJECT FINDING (the "Object Finding - Latest" code)
// =====================================================================================================
// Runs once all three IR sensors see the line together while wall following (see startObjectMode()).
// It is the same code, with these changes so it fits in one sketch with the line and wall code:
// - The pins and the speed sensor interrupts are the ones at the top of this sketch (same pin numbers).
// - Names the line and wall code already use have OBJ_ / obj in front: OBJ_TURN_SPEED, OBJ_SLOW_DISTANCE,
//   OBJ_WHEEL_DIAMETER_MM, OBJ_WHEEL_SPACING_MM, OBJ_MM_PER_TICK, OBJ_DEG_PER_TICK, objFinished,
//   objDrive(), objWaitMs() and objReadDistance().
// - Its setup() is startObjectFinding() and its loop() is objectFindingLoop().
// It drives the motors directly, at fixed power, without the line and wall code's speed control.

// ==========================================
// OBJECT DETECTION SETTINGS
// ==========================================

const int SCAN_MIN = 30;
const int SCAN_MAX = 150;
const int SCAN_STEP = 2;

const int SCAN_POINTS =
  (SCAN_MAX - SCAN_MIN) / SCAN_STEP + 1;

const float MAX_RANGE = 300.0;

const float MIN_OBJECT_WIDTH = 5.0;
const float MAX_OBJECT_WIDTH = 120.0;

const float MIN_SEARCH_DISTANCE = 8.0;

const float EDGE_JUMP = 5.0;
const float MAX_GROUP_DEPTH = 20.0;

const float BEAM_CORRECTION = 10.0;
const float SWEEP_AGREEMENT = 10.0;

const int MIN_POINTS = 3;

const unsigned long PING_GAP = 60;
const unsigned long ECHO_TIMEOUT = 22000;

// ==========================================
// MOTOR SETTINGS
// ==========================================

const int OBJ_TURN_SPEED = 150;
const int DRIVE_SPEED = 145;
const int SLOW_SPEED = 120;
const int START_SPEED = 160;

const int CONTACT_SPEED = 110;

const int LEFT_MOTOR_DIRECTION = 1;
const int RIGHT_MOTOR_DIRECTION = 1;
const int TURN_DIRECTION = 1;

const int LEFT_TRIM = 0;
const int RIGHT_TRIM = 0;

// ==========================================
// FINAL APPROACH AND CONTACT
// ==========================================

const float STOP_DISTANCE = 5.0;
const float OBJ_SLOW_DISTANCE = 50.0;

const unsigned long CONTACT_TIME = 350;
const float CONTACT_DISTANCE = 5.0;

// ==========================================
// SAFETY TIMEOUTS
// ==========================================

const unsigned long STALL_TIMEOUT = 2000;
const unsigned long TURN_TIMEOUT = 10000;
const unsigned long APPROACH_TIMEOUT = 180000;
const unsigned long TOTAL_TIMEOUT = 600000;

// ==========================================
// SEARCH AND REPOSITIONING
// ==========================================

const int MAX_SCAN_RETRIES = 8;
const int MAX_RECOVERIES = 6;

const unsigned long SEARCH_FORWARD_TIME = 4000;
const float SEARCH_FORWARD_CM = 60.0;

const float SEARCH_CLEARANCE = 40.0;
const float SEARCH_STOP_DISTANCE = 30.0;

const float TRACK_TOLERANCE = 30.0;

const int MAX_MISSING_READINGS = 5;

int scanRetries = 0;

// ==========================================
// SERVO SETTINGS
// ==========================================

const int SERVO_MIN_US = 544;
const int SERVO_MAX_US = 2400;

const float SENSOR_OFFSET = 8.0;

int servoAngle = 90;

unsigned long lastServoPulse = 0;

// ==========================================
// WHEEL SETTINGS
// ==========================================

const float OBJ_WHEEL_DIAMETER_MM = 70.0;
const float OBJ_WHEEL_SPACING_MM = 140.0;

const int ENCODER_SLOTS = 20;

const float OBJ_MM_PER_TICK =
  PI * OBJ_WHEEL_DIAMETER_MM /
  (ENCODER_SLOTS * 2.0);

const float OBJ_DEG_PER_TICK =
  360.0 * OBJ_MM_PER_TICK /
  (PI * OBJ_WHEEL_SPACING_MM);

const float TURN_CALIBRATION = 1.0;

// ==========================================
// SCAN MEMORY
// ==========================================

float scanDistance[SCAN_POINTS];
float firstSweep[SCAN_POINTS];

char scanLabel[SCAN_POINTS];

byte confidence[SCAN_POINTS];

// ==========================================
// TARGET VARIABLES
// ==========================================

bool targetFound = false;
bool ambiguousTarget = false;
bool objFinished = false;
bool fatalError = false;

float targetWidth = 0;
float targetDistance = 0;
float targetAngle = 0;
float targetServoAngle = 90;

float targetScore = 0;
float secondScore = 0;
float widestUncertain = 0;

char targetName = '-';

unsigned long missionStart = 0;

// ==========================================
// ENCODERS
// ==========================================

// The ticks are counted by leftTick() / rightTick() in the speed sensor section above (the same
// interrupts as the line and wall code). resetEncoders() starts the counts from 0 again

void resetEncoders() {

  noInterrupts();

  leftTicks = 0;
  rightTicks = 0;

  interrupts();
}

void getEncoders(
  unsigned long &left,
  unsigned long &right
) {

  noInterrupts();

  left = leftTicks;
  right = rightTicks;

  interrupts();
}

void printEncoders() {

  unsigned long left, right;

  getEncoders(left, right);

  Serial.print(F("ENCODERS | Left: "));
  Serial.print(left);

  Serial.print(F(" | Right: "));
  Serial.println(right);
}

float travelledDistance() {

  unsigned long left, right;

  getEncoders(left, right);

  float averageTicks =
    (left + right) / 2.0f;

  return averageTicks * OBJ_MM_PER_TICK / 10.0f;
}

// ==========================================
// PROCESSING COMMUNICATION
// ==========================================

void status(const __FlashStringHelper *message) {

  Serial.print(F("@STATUS "));
  Serial.println(message);
}

void sendLiveDistance(float distance) {

  Serial.print(F("@D "));
  Serial.println(distance, 1);
}

void sendConfig() {

  Serial.print(F("@C "));
  Serial.print(MAX_RANGE);

  Serial.print(' ');
  Serial.print(SCAN_STEP);

  Serial.print(' ');
  Serial.println(MAX_OBJECT_WIDTH);
}

// ==========================================
// MOTOR CONTROL
// ==========================================

void leftMotor(int power) {

  power *= LEFT_MOTOR_DIRECTION;

  power = constrain(power, -255, 255);

  if (power > 0) {

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

  } else if (power < 0) {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

  } else {

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(enA, abs(power));
}

void rightMotor(int power) {

  power *= RIGHT_MOTOR_DIRECTION;

  power = constrain(power, -255, 255);

  if (power > 0) {

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

  } else if (power < 0) {

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

  } else {

    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  analogWrite(enB, abs(power));
}

void objDrive(int leftPower, int rightPower) {

  // After seeing the perimeter it can't drive forward until it has backed up and turned round
  if (perimeterHit && leftPower > 0 && rightPower > 0) {
    leftPower = 0;
    rightPower = 0;
  }

  objLeftPower = leftPower;
  objRightPower = rightPower;

  leftMotor(leftPower);
  rightMotor(rightPower);
}

// ==========================================
// PERIMETER
// ==========================================

// Called from holdServo(), which every object finding wait and drive loop calls. Only while both wheels are
// driving forward (not while it scans, turns or backs up): once any IR sensor has seen the line for
// PERIMETER_CONFIRM readings in a row, it brakes and sets perimeterHit. The drive loops then stop, and
// objectFindingLoop() calls leavePerimeter(). Not watched in the last PERIMETER_OFF_CM of the approach or the
// final touch (nearTarget)
void checkPerimeter() {

  if (perimeterHit || nearTarget || objLeftPower <= 0 || objRightPower <= 0) {

    perimeterCount = 0;

    return;
  }

  readLineSensors();

  perimeterCount = anySeesLine() ? perimeterCount + 1 : 0;

  if (perimeterCount < PERIMETER_CONFIRM) {

    return;
  }

  brake();

  objLeftPower = 0;
  objRightPower = 0;

  perimeterHit = true;

  status(F("IR sees the perimeter - leaving the area"));
}

// Back up PERIMETER_BACK_CM so the IR sensors are back inside, then turn round 180°.
// objectFindingLoop() then scans again
void leavePerimeter() {

  status(F("Perimeter - backing up"));

  objWaitMs(200);

  resetEncoders();

  unsigned long start = millis();

  objDrive(-PERIMETER_BACK_SPEED, -PERIMETER_BACK_SPEED);

  while (
    travelledDistance() < PERIMETER_BACK_CM &&
    millis() - start < PERIMETER_BACK_MS
  ) {

    holdServo();
  }

  stopMotors();

  objWaitMs(250);

  status(F("Perimeter - U-turn"));

  if (!turnRobot(180)) {

    return;  // Stalled: turnRobot() has already stopped the mission
  }

  perimeterHit = false;

  status(F("Perimeter - scanning again"));
}

void stopMotors() {

  objDrive(0, 0);
}

void fail(const __FlashStringHelper *message) {

  stopMotors();

  fatalError = true;

  status(message);
}

bool missionExpired() {

  return millis() - missionStart >= TOTAL_TIMEOUT;
}

// ==========================================
// MANUAL SERVO CONTROL
// ==========================================

// Avoid Servo library because motor PWM
// pins 9 and 10 use Timer1.

void servoPulse() {

  int pulse = map(
    servoAngle,
    0,
    180,
    SERVO_MIN_US,
    SERVO_MAX_US
  );

  digitalWrite(servoPin, HIGH);

  delayMicroseconds(pulse);

  digitalWrite(servoPin, LOW);

  lastServoPulse = millis();
}

void holdServo() {

  if (objectMode) {

    checkPerimeter();  // Watch the IR sensors for the perimeter line while driving forward
  }

  if (millis() - lastServoPulse >= 20) {

    servoPulse();
  }
}

void objWaitMs(unsigned long duration) {

  unsigned long start = millis();

  while (millis() - start < duration) {

    holdServo();
  }
}

void pointServo(
  int angle,
  unsigned long duration
) {

  servoAngle = constrain(angle, 0, 180);

  objWaitMs(duration);
}

// ==========================================
// ULTRASONIC SENSOR
// ==========================================

float objReadDistance() {

  holdServo();

  digitalWrite(frontTrig, LOW);
  delayMicroseconds(2);

  digitalWrite(frontTrig, HIGH);
  delayMicroseconds(10);

  digitalWrite(frontTrig, LOW);

  unsigned long duration = pulseIn(
    frontEcho,
    HIGH,
    ECHO_TIMEOUT
  );

  if (duration == 0) {

    return -1;
  }

  float distance =
    duration * 0.0343f / 2.0f;

  if (distance > MAX_RANGE) {

    return -1;
  }

  return max(distance, 2.0f);
}

// ==========================================
// DISTANCE FILTERING
// ==========================================

float filteredDistance() {

  float a = objReadDistance();

  objWaitMs(PING_GAP);

  float b = objReadDistance();

  objWaitMs(PING_GAP);

  if (
    a > 0 &&
    b > 0 &&
    fabs(a - b) <= 8.0f
  ) {

    return (a + b) / 2.0f;
  }

  float c = objReadDistance();

  objWaitMs(PING_GAP);

  if (
    a > 0 &&
    c > 0 &&
    fabs(a - c) <= 8.0f
  ) {

    return (a + c) / 2.0f;
  }

  if (
    b > 0 &&
    c > 0 &&
    fabs(b - c) <= 8.0f
  ) {

    return (b + c) / 2.0f;
  }

  if (a > 0 && b <= 0 && c <= 0) {

    return a;
  }

  if (b > 0 && a <= 0 && c <= 0) {

    return b;
  }

  if (c > 0 && a <= 0 && b <= 0) {

    return c;
  }

  return -1;
}

// ==========================================
// GEOMETRY
// ==========================================

int indexToAngle(int index) {

  return SCAN_MIN + index * SCAN_STEP;
}

float correctedBearing(
  float servoPosition,
  float distance
) {

  float bearing =
    radians(90.0f - servoPosition);

  float x = distance * sin(bearing);

  float y =
    SENSOR_OFFSET +
    distance * cos(bearing);

  return degrees(atan2(x, y));
}

float calculateWidth(
  float distance,
  float angularWidth
) {

  float correctedWidth = max(
    angularWidth - BEAM_CORRECTION,
    angularWidth * 0.35f
  );

  return
    2.0f * distance *
    sin(radians(correctedWidth / 2.0f));
}

// ==========================================
// PERFORM SWEEP
// ==========================================

void sweep(bool reverse) {

  stopMotors();

  Serial.println(F("@S"));

  int start =
    reverse ? SCAN_POINTS - 1 : 0;

  int end =
    reverse ? 0 : SCAN_POINTS - 1;

  int direction =
    reverse ? -1 : 1;

  pointServo(indexToAngle(start), 700);

  for (
    int i = start;
    direction > 0 ? i <= end : i >= end;
    i += direction
  ) {

    if (missionExpired()) {

      fail(F("ERROR: Mission timeout"));

      return;
    }

    int angle = indexToAngle(i);

    pointServo(angle, 60);

    float distance = filteredDistance();

    scanDistance[i] =
      distance > 0 ? distance : 0;

    scanLabel[i] = '-';

    Serial.print(F("@R "));
    Serial.print(angle);

    Serial.print(' ');
    Serial.println(scanDistance[i], 1);
  }
}

// ==========================================
// COMBINE BOTH SWEEPS
// ==========================================

void combineSweeps() {

  for (int i = 0; i < SCAN_POINTS; i++) {

    float a = firstSweep[i];
    float b = scanDistance[i];

    confidence[i] = 0;

    if (a > 0 && b > 0) {

      if (fabs(a - b) <= SWEEP_AGREEMENT) {

        scanDistance[i] =
          (a + b) / 2.0f;

        confidence[i] = 2;

      } else {

        scanDistance[i] = 0;
      }

    } else if (a > 0 || b > 0) {

      scanDistance[i] = max(a, b);

      confidence[i] = 1;

    } else {

      scanDistance[i] = 0;
    }
  }

  // Repair one missing point when both
  // neighboring readings agree.

  for (int i = 1; i < SCAN_POINTS - 1; i++) {

    if (scanDistance[i] > 0) {

      continue;
    }

    float left = scanDistance[i - 1];
    float right = scanDistance[i + 1];

    if (
      left > 0 &&
      right > 0 &&
      confidence[i - 1] == 2 &&
      confidence[i + 1] == 2 &&
      fabs(left - right) <= 6.0f
    ) {

      scanDistance[i] =
        (left + right) / 2.0f;

      confidence[i] = 0;
    }
  }
}

// ==========================================
// ANALYZE OBJECT
// ==========================================

void analyzeObject(
  int first,
  int last,
  char name
) {

  int count = last - first + 1;

  float total = 0;
  float nearest = MAX_RANGE;
  float farthest = 0;

  int confirmedPoints = 0;
  int measuredPoints = 0;

  for (int i = first; i <= last; i++) {

    float distance = scanDistance[i];

    total += distance;

    nearest = min(nearest, distance);
    farthest = max(farthest, distance);

    if (confidence[i] > 0) {

      measuredPoints++;
    }

    if (confidence[i] == 2) {

      confirmedPoints++;
    }
  }

  float average = total / count;

  int firstAngle = indexToAngle(first);
  int lastAngle = indexToAngle(last);

  float middle =
    (firstAngle + lastAngle) / 2.0f;

  float angularWidth =
    lastAngle - firstAngle + SCAN_STEP;

  float width = calculateWidth(
    average,
    angularWidth
  );

  float agreement =
    (float)confirmedPoints / count;

  bool touchesEdge =
    first == 0 ||
    last == SCAN_POINTS - 1;

  bool noise =
    measuredPoints < MIN_POINTS ||
    width < MIN_OBJECT_WIDTH;

  bool tooClose =
    nearest < MIN_SEARCH_DISTANCE;

  bool oversized =
    width > MAX_OBJECT_WIDTH;

  bool broadArc =
    average > 60.0f &&
    angularWidth > 55.0f &&
    farthest - nearest < 15.0f;

  bool wall =
    touchesEdge ||
    oversized ||
    tooClose;

  bool uncertain =
    broadArc ||
    agreement < 0.25f;

  char type = 'K';

  if (noise || uncertain) {

    type = 'N';
  }

  if (wall) {

    type = 'W';
  }

  // Send object information to Processing.

  Serial.print(F("@T "));
  Serial.print(name);

  Serial.print(' ');
  Serial.print(type);

  Serial.print(' ');
  Serial.print(middle, 1);

  Serial.print(' ');
  Serial.print(average, 1);

  Serial.print(' ');
  Serial.print(width, 1);

  Serial.print(' ');
  Serial.print(firstAngle);

  Serial.print(' ');
  Serial.println(lastAngle);

  Serial.print(F("Object "));
  Serial.print(name);

  Serial.print(F(" | Width: "));
  Serial.print(width, 1);

  Serial.print(F(" | Agreement: "));
  Serial.print(agreement * 100.0f, 0);

  Serial.print(F("% | Span: "));
  Serial.println(angularWidth, 1);

  for (int i = first; i <= last; i++) {

    scanLabel[i] =
      type == 'K'
      ? name
      : (char)(name - 'A' + 'a');
  }

  // Remember larger uncertain objects.

  if (
    !noise &&
    !tooClose &&
    !touchesEdge &&
    (uncertain || oversized)
  ) {

    widestUncertain = max(
      widestUncertain,
      width
    );
  }

  if (type != 'K') {

    return;
  }

  // Width remains the primary criterion.

  float score =
    width * (0.85f + 0.15f * agreement);

  if (!targetFound || score > targetScore) {

    secondScore = targetScore;

    targetFound = true;

    targetScore = score;
    targetWidth = width;

    targetName = name;
    targetDistance = average;
    targetServoAngle = middle;

    targetAngle = correctedBearing(
      middle,
      average
    );

  } else {

    secondScore = max(secondScore, score);
  }
}

// ==========================================
// GROUP SCAN POINTS
// ==========================================

void analyzeScan() {

  targetFound = false;
  ambiguousTarget = false;

  targetWidth = 0;
  targetScore = 0;
  secondScore = 0;
  widestUncertain = 0;

  targetName = '-';

  int i = 0;

  char name = 'A';

  while (i < SCAN_POINTS) {

    if (scanDistance[i] <= 0) {

      i++;

      continue;
    }

    int first = i;

    float nearest = scanDistance[i];
    float farthest = scanDistance[i];

    while (i + 1 < SCAN_POINTS) {

      float current = scanDistance[i];
      float next = scanDistance[i + 1];

      if (next <= 0) {

        break;
      }

      if (fabs(current - next) > EDGE_JUMP) {

        break;
      }

      float newNearest = min(nearest, next);
      float newFarthest = max(farthest, next);

      if (
        newFarthest - newNearest >
        MAX_GROUP_DEPTH
      ) {

        break;
      }

      nearest = newNearest;
      farthest = newFarthest;

      i++;
    }

    analyzeObject(first, i, name);

    if (name < 'Z') {

      name++;
    }

    i++;
  }

  if (
    targetFound &&
    widestUncertain > targetWidth * 1.25f &&
    scanRetries < 2
  ) {

    ambiguousTarget = true;
  }

  if (
    targetFound &&
    secondScore > targetScore * 0.92f &&
    scanRetries < 2
  ) {

    ambiguousTarget = true;
  }

  if (ambiguousTarget) {

    targetName = '-';
  }

  for (int k = 0; k < SCAN_POINTS; k++) {

    Serial.print(F("@P "));
    Serial.print(indexToAngle(k));

    Serial.print(' ');
    Serial.print(scanDistance[k], 1);

    Serial.print(' ');
    Serial.println(scanLabel[k]);
  }

  Serial.print(F("@B "));
  Serial.println(targetName);

  Serial.println(F("@E"));
}

// ==========================================
// VERIFY SELECTED TARGET
// ==========================================

bool verifyTarget() {

  stopMotors();

  status(F("Verifying selected object"));

  int center = (int)round(targetServoAngle);

  int matches = 0;

  float totalDistance = 0;
  float totalAngle = 0;

  for (
    int angle = center - 6;
    angle <= center + 6;
    angle += 3
  ) {

    if (missionExpired()) {

      fail(F("ERROR: Mission timeout"));

      return false;
    }

    int actualAngle =
      constrain(angle, SCAN_MIN, SCAN_MAX);

    pointServo(actualAngle, 180);

    float distance = filteredDistance();

    if (
      distance > 0 &&
      fabs(distance - targetDistance) <= 20.0f
    ) {

      matches++;

      totalDistance += distance;
      totalAngle += actualAngle;
    }
  }

  pointServo(90, 300);

  if (matches < 2) {

    return false;
  }

  targetDistance =
    totalDistance / matches;

  targetServoAngle =
    totalAngle / matches;

  targetAngle = correctedBearing(
    targetServoAngle,
    targetDistance
  );

  return true;
}

// ==========================================
// FIND DESTINATION
// ==========================================

bool findDestination() {

  stopMotors();

  status(F("Scanning forward"));

  sweep(false);

  if (fatalError) {

    return false;
  }

  for (int i = 0; i < SCAN_POINTS; i++) {

    firstSweep[i] = scanDistance[i];
  }

  // One sweep only (the backward sweep was removed), so combineSweeps() compares the sweep with its own copy:
  // every reading agrees with itself

  combineSweeps();

  status(F("Ranking detected objects"));

  analyzeScan();

  pointServo(90, 300);

  if (!targetFound) {

    status(F("No suitable destination found"));

    return false;
  }

  if (ambiguousTarget) {

    status(F("Ambiguous objects - repositioning"));

    return false;
  }

  if (!verifyTarget()) {

    if (fatalError) {

      return false;
    }

    if (!verifyTarget()) {

      status(F("Target verification failed"));

      return false;
    }
  }

  float bearing =
    radians(90.0f - targetServoAngle);

  float axleDistance = sqrt(
    targetDistance * targetDistance +
    SENSOR_OFFSET * SENSOR_OFFSET +
    2.0f * SENSOR_OFFSET *
    targetDistance * cos(bearing)
  );

  Serial.print(F("@TARGET "));
  Serial.print(targetAngle, 1);

  Serial.print(' ');
  Serial.print(axleDistance, 1);

  Serial.print(' ');
  Serial.println(targetWidth, 1);

  status(F("Widest destination selected"));

  return true;
}

// ==========================================
// STRAIGHT DRIVING
// ==========================================

void driveStraight(int speed) {

  unsigned long left, right;

  getEncoders(left, right);

  long difference =
    (long)left - (long)right;

  int correction = constrain(
    difference * 2L,
    -20L,
    20L
  );

  objDrive(
    speed - correction + LEFT_TRIM,
    speed + correction + RIGHT_TRIM
  );
}

// ==========================================
// CHECK FOR OBSTACLES
// ==========================================

bool confirmedClearance(float clearance) {

  int valid = 0;

  for (int attempt = 0; attempt < 5; attempt++) {

    float distance = objReadDistance();

    objWaitMs(PING_GAP);

    if (
      distance > 0 &&
      distance < clearance
    ) {

      return false;
    }

    if (distance >= clearance) {

      valid++;
    }
  }

  return valid >= 2;
}

// ==========================================
// MOVE FORWARD AND RESCAN
// ==========================================

bool moveForwardAndRescan() {

  stopMotors();

  status(F("Checking forward clearance"));

  pointServo(105, 300);

  bool leftClear =
    confirmedClearance(SEARCH_CLEARANCE);

  pointServo(75, 300);

  bool rightClear =
    confirmedClearance(SEARCH_CLEARANCE);

  pointServo(90, 300);

  bool centerClear =
    confirmedClearance(SEARCH_CLEARANCE);

  if (
    !leftClear ||
    !rightClear ||
    !centerClear
  ) {

    status(F("Clearance not confirmed"));

    return false;
  }

  status(F("Moving forward for better view"));

  resetEncoders();

  unsigned long start = millis();

  objDrive(START_SPEED, START_SPEED);

  while (
    millis() - start < SEARCH_FORWARD_TIME
  ) {

    if (perimeterHit) {

      return false;  // Reached the perimeter line
    }

    if (missionExpired()) {

      fail(F("ERROR: Mission timeout"));

      return false;
    }

    float distance = objReadDistance();

    sendLiveDistance(distance);

    if (
      distance > 0 &&
      distance < SEARCH_STOP_DISTANCE
    ) {

      stopMotors();

      status(F("Repositioning stopped for obstacle"));

      return false;
    }

    if (distance <= 0) {

      stopMotors();

      bool clearAgain = false;

      for (int retry = 0; retry < 3; retry++) {

        objWaitMs(PING_GAP);

        distance = objReadDistance();

        sendLiveDistance(distance);

        if (
          distance > 0 &&
          distance < SEARCH_STOP_DISTANCE
        ) {

          status(F("Obstacle detected"));

          return false;
        }

        if (distance >= SEARCH_STOP_DISTANCE) {

          clearAgain = true;

          break;
        }
      }

      if (!clearAgain) {

        status(F("Repositioning lost clearance"));

        return false;
      }

      if (millis() - start >= SEARCH_FORWARD_TIME) {

        break;
      }
    }

    // Stop after the configured repositioning distance.

    if (travelledDistance() >= SEARCH_FORWARD_CM) {

      break;
    }

    if (millis() - start >= 100) {

      driveStraight(DRIVE_SPEED);

    } else {

      objDrive(START_SPEED, START_SPEED);
    }

    objWaitMs(PING_GAP);
  }

  stopMotors();

  printEncoders();

  Serial.print(F("Repositioned distance: "));
  Serial.print(travelledDistance(), 1);
  Serial.println(F(" cm"));

  objWaitMs(500);

  status(F("Repositioning complete - rescanning"));

  return true;
}

// ==========================================
// TURN ROBOT
// ==========================================

bool turnRobot(float angle) {

  if (fabs(angle) < 3.0f) {

    return true;
  }

  stopMotors();

  pointServo(90, 200);

  resetEncoders();

  float requiredTicks =
    fabs(angle) /
    (OBJ_DEG_PER_TICK * TURN_CALIBRATION);

  int minimumWheelTicks = max(
    1,
    (int)(requiredTicks * 0.7f)
  );

  int direction =
    (angle > 0 ? 1 : -1) *
    TURN_DIRECTION;

  unsigned long start = millis();

  unsigned long lastLeftMovement = millis();
  unsigned long lastRightMovement = millis();

  unsigned long previousLeft = 0;
  unsigned long previousRight = 0;

  status(F("Turning towards destination"));

  objDrive(
    direction * OBJ_TURN_SPEED,
    -direction * OBJ_TURN_SPEED
  );

  while (true) {

    holdServo();

    unsigned long left, right;

    getEncoders(left, right);

    float averageTicks =
      (left + right) / 2.0f;

    if (
      averageTicks >= requiredTicks &&
      left >= (unsigned long)minimumWheelTicks &&
      right >= (unsigned long)minimumWheelTicks
    ) {

      break;
    }

    if (left != previousLeft) {

      previousLeft = left;
      lastLeftMovement = millis();
    }

    if (right != previousRight) {

      previousRight = right;
      lastRightMovement = millis();
    }

    if (
      millis() - lastLeftMovement >
      STALL_TIMEOUT ||
      millis() - lastRightMovement >
      STALL_TIMEOUT
    ) {

      fail(F("ERROR: Motor or encoder stalled"));

      return false;
    }

    if (
      millis() - start > TURN_TIMEOUT ||
      missionExpired()
    ) {

      fail(F("ERROR: Turn timeout"));

      return false;
    }
  }

  stopMotors();

  objWaitMs(250);

  printEncoders();

  Serial.print(F("@TURN "));
  Serial.println(angle, 1);

  return true;
}

// ==========================================
// TARGET MATCHING
// ==========================================

bool matchesTarget(
  float distance,
  float expected
) {

  return
    distance > 0 &&
    fabs(distance - expected) <=
    TRACK_TOLERANCE;
}

// ==========================================
// RECOVER LOST TARGET
// ==========================================

bool recoverTarget(
  float expected,
  int range,
  float &recovered
) {

  stopMotors();

  status(F("Searching for lost target"));

  int runCount = 0;

  float runDistance = 0;
  float runAngle = 0;

  int bestCount = 0;

  float bestDistance = 0;
  float bestAngle = 90;

  float bestScore = 100000;

  for (
    int angle = 90 - range;
    angle <= 90 + range;
    angle += 5
  ) {

    if (missionExpired()) {

      fail(F("ERROR: Mission timeout"));

      return false;
    }

    pointServo(angle, 180);

    float distance = filteredDistance();

    if (
      distance > STOP_DISTANCE &&
      matchesTarget(distance, expected)
    ) {

      runCount++;

      runDistance += distance;
      runAngle += angle;

      if (runCount >= 2) {

        float average =
          runDistance / runCount;

        float middle =
          runAngle / runCount;

        float score =
          fabs(average - expected) +
          fabs(middle - 90.0f) * 0.5f -
          runCount * 2.0f;

        if (score < bestScore) {

          bestScore = score;

          bestCount = runCount;

          bestDistance = average;
          bestAngle = middle;
        }
      }

    } else {

      runCount = 0;

      runDistance = 0;
      runAngle = 0;
    }
  }

  pointServo(90, 300);

  if (bestCount < 2) {

    return false;
  }

  float correction = correctedBearing(
    bestAngle,
    bestDistance
  );

  if (fabs(correction) > 5.0f) {

    if (!turnRobot(correction)) {

      return false;
    }
  }

  pointServo(90, 300);

  for (int attempt = 0; attempt < 3; attempt++) {

    float center = filteredDistance();

    if (matchesTarget(center, bestDistance)) {

      recovered = center;

      status(F("Target recovered"));

      return true;
    }
  }

  return false;
}

// ==========================================
// CONFIRM CLOSE DESTINATION
// ==========================================

bool confirmArrival(float expected) {

  stopMotors();

  int closeReadings = 0;

  for (int attempt = 0; attempt < 3; attempt++) {

    float distance = objReadDistance();

    sendLiveDistance(distance);

    if (
      distance > 0 &&
      distance <= STOP_DISTANCE + 3.0f
    ) {

      closeReadings++;
    }

    objWaitMs(PING_GAP);
  }

  return
    closeReadings >= 2 &&
    expected <= STOP_DISTANCE + 15.0f;
}

// ==========================================
// FINAL CONTACT MOVEMENT
// ==========================================

bool touchDestination() {

  stopMotors();

  pointServo(90, 200);

  nearTarget = true;  // Right at the object: don't watch the perimeter during the touch

  status(F("Making gentle contact"));

  resetEncoders();

  unsigned long start = millis();

  unsigned long lastMovement = millis();

  unsigned long previousLeft = 0;
  unsigned long previousRight = 0;

  objDrive(CONTACT_SPEED, CONTACT_SPEED);

  while (millis() - start < CONTACT_TIME) {

    holdServo();

    if (perimeterHit) {

      return false;  // Reached the perimeter line
    }

    if (missionExpired()) {

      fail(F("ERROR: Mission timeout"));

      return false;
    }

    unsigned long left, right;

    getEncoders(left, right);

    if (
      left != previousLeft ||
      right != previousRight
    ) {

      previousLeft = left;
      previousRight = right;

      lastMovement = millis();
    }

    // Limit final forward movement.

    if (travelledDistance() >= CONTACT_DISTANCE) {

      break;
    }

    // Avoid continuing to push if the
    // wheels stop turning.

    if (millis() - lastMovement >= 180) {

      break;
    }
  }

  stopMotors();

  objWaitMs(200);

  printEncoders();

  status(F("Contact attempt complete - verify touch"));

  return true;
}

// ==========================================
// APPROACH DESTINATION
// ==========================================

bool approachDestination() {

  stopMotors();

  pointServo(90, 300);

  float bearing =
    radians(90.0f - targetServoAngle);

  float expected = sqrt(
    targetDistance * targetDistance +
    SENSOR_OFFSET * SENSOR_OFFSET +
    2.0f * SENSOR_OFFSET *
    targetDistance * cos(bearing)
  ) - SENSOR_OFFSET;

  unsigned long start = millis();

  unsigned long lastLeftMovement = millis();
  unsigned long lastRightMovement = millis();

  unsigned long previousLeft = 0;
  unsigned long previousRight = 0;

  unsigned long movementStart = millis();

  int missingReadings = 0;
  int recoveryAttempts = 0;

  bool moving = false;

  float progressReference = expected;
  float progressTicks = 0;

  resetEncoders();

  nearTarget = false;  // Watch the perimeter until the object is within PERIMETER_OFF_CM

  status(F("Approaching destination"));

  while (millis() - start < APPROACH_TIMEOUT) {

    if (perimeterHit) {

      return false;  // Reached the perimeter line
    }

    if (missionExpired()) {

      fail(F("ERROR: Mission timeout"));

      return false;
    }

    float distance = objReadDistance();

    sendLiveDistance(distance);

    // ======================================
    // FINAL CONTACT
    // ======================================

    if (
      distance > 0 &&
      distance <= STOP_DISTANCE
    ) {

      stopMotors();

      objWaitMs(PING_GAP);

      if (confirmArrival(expected)) {

        return touchDestination();
      }

      fail(F("Stopped for unexpected obstacle"));

      return false;
    }

    // ======================================
    // MISSING OR INCONSISTENT READINGS
    // ======================================

    if (!matchesTarget(distance, expected)) {

      stopMotors();

      moving = false;

      missingReadings++;

      status(F("Retrying target reading"));

      objWaitMs(PING_GAP);

      if (
        missingReadings >= MAX_MISSING_READINGS
      ) {

        // Avoid turning around blindly when
        // already very close to the object.

        if (expected <= 15.0f) {

          status(F("Stopped near target - echo lost"));

          return false;
        }

        bool recovered = false;

        float recoveredDistance = 0;

        while (
          recoveryAttempts < MAX_RECOVERIES
        ) {

          recoveryAttempts++;

          int range =
            recoveryAttempts % 2 == 1
            ? 20
            : 40;

          if (
            recoverTarget(
              expected,
              range,
              recoveredDistance
            )
          ) {

            recovered = true;

            break;
          }

          if (fatalError) {

            return false;
          }
        }

        if (!recovered) {

          status(F("Target recovery exhausted"));

          return false;
        }

        expected = recoveredDistance;

        missingReadings = 0;
      }

      continue;
    }

    missingReadings = 0;

    expected = distance;

    // Close to the object: stop watching the perimeter for the rest of the approach (see PERIMETER_OFF_CM)

    if (distance <= PERIMETER_OFF_CM) {

      nearTarget = true;
    }

    // ======================================
    // START OR RESUME MOVEMENT
    // ======================================

    if (!moving) {

      resetEncoders();

      previousLeft = 0;
      previousRight = 0;

      lastLeftMovement = millis();
      lastRightMovement = millis();

      movementStart = millis();

      progressReference = distance;
      progressTicks = 0;

      moving = true;

      status(F("Driving forward"));
    }

    // ======================================
    // SPEED CONTROL
    // ======================================

    int speed;

    if (distance > OBJ_SLOW_DISTANCE) {

      speed = DRIVE_SPEED;

    } else if (distance > 15.0f) {

      speed = SLOW_SPEED;

    } else {

      speed = CONTACT_SPEED;
    }

    if (
      millis() - movementStart < 100 &&
      distance > OBJ_SLOW_DISTANCE
    ) {

      speed = START_SPEED;
    }

    driveStraight(speed);

    // ======================================
    // ENCODER PROTECTION
    // ======================================

    unsigned long left, right;

    getEncoders(left, right);

    if (left != previousLeft) {

      previousLeft = left;

      lastLeftMovement = millis();
    }

    if (right != previousRight) {

      previousRight = right;

      lastRightMovement = millis();
    }

    if (
      millis() - lastLeftMovement >
      STALL_TIMEOUT ||
      millis() - lastRightMovement >
      STALL_TIMEOUT
    ) {

      fail(F("ERROR: Motor or encoder stalled"));

      return false;
    }

    // ======================================
    // CHECK FOR FORWARD PROGRESS
    // ======================================

    float averageTicks =
      (left + right) / 2.0f;

    float travelled =
      (averageTicks - progressTicks) *
      OBJ_MM_PER_TICK / 10.0f;

    if (travelled >= 15.0f) {

      float improvement =
        progressReference - distance;

      if (improvement < 3.0f) {

        stopMotors();

        moving = false;

        if (distance <= 15.0f) {

          status(F("Stopped - no close-range progress"));

          return false;
        }

        status(F("Checking destination alignment"));

        if (
          recoveryAttempts >= MAX_RECOVERIES
        ) {

          return false;
        }

        recoveryAttempts++;

        float recovered;

        objWaitMs(PING_GAP);

        if (
          recoverTarget(
            expected,
            30,
            recovered
          )
        ) {

          expected = recovered;

        } else {

          return false;
        }

        continue;
      }

      progressReference = distance;
      progressTicks = averageTicks;
    }

    // Continuous slow movement near the
    // destination instead of 20 ms bursts.

    objWaitMs(PING_GAP);
  }

  fail(F("ERROR: Approach timeout"));

  return false;
}

// ==========================================
// START OBJECT FINDING
// ==========================================

// What the Object Finding setup() did. The pins, servo, speed sensors and Serial are already set up

void startObjectFinding() {

  stopMotors();

  digitalWrite(LED_BUILTIN, LOW);

  pointServo(90, 2000);

  sendConfig();

  status(F("Robot ready"));

  missionStart = millis();
}

// ==========================================
// OBJECT FINDING LOOP
// ==========================================

// What the Object Finding loop() did. Called from loop() once objectMode is on

void objectFindingLoop() {

  if (objFinished || fatalError) {

    stopMotors();

    holdServo();

    return;
  }

  if (missionExpired()) {

    fail(F("ERROR: Mission timeout"));

    return;
  }

  // ======================================
  // LEFT THE PERIMETER
  // ======================================

  // An IR sensor saw the perimeter line: back up, turn round, and the next loop scans again

  if (perimeterHit) {

    leavePerimeter();

    return;
  }

  // ======================================
  // SEARCH FOR WIDEST OBJECT
  // ======================================

  if (!findDestination()) {

    stopMotors();

    if (fatalError) {

      return;
    }

    if (scanRetries >= MAX_SCAN_RETRIES) {

      fail(F("ERROR: Search attempts exhausted"));

      return;
    }

    scanRetries++;

    Serial.print(F("Search retry: "));
    Serial.print(scanRetries);

    Serial.print(F(" / "));
    Serial.println(MAX_SCAN_RETRIES);

    // Move forward and obtain a better view.

    if (!moveForwardAndRescan()) {

      if (fatalError || perimeterHit) {

        return;  // At the perimeter: the next loop backs up, turns round and scans again
      }

      if (scanRetries >= 3) {
        fail(F("Unable to reposition or select target"));
        return;
      }

      status(F("Retrying scan from current position"));

      objWaitMs(500);
    }

    return;
  }

  // ======================================
  // TURN TOWARDS DESTINATION
  // ======================================

  if (!turnRobot(targetAngle)) {

    stopMotors();

    objFinished = true;

    return;
  }

  // ======================================
  // APPROACH AND TOUCH DESTINATION
  // ======================================

  bool completed = approachDestination();

  stopMotors();

  nearTarget = false;  // Watch the perimeter again from here on

  if (perimeterHit) {

    return;  // At the perimeter: the next loop backs up, turns round and scans again
  }

  if (completed) {

    digitalWrite(LED_BUILTIN, HIGH);

    status(F("Contact attempt complete - verify touch"));

  } else {

    if (!fatalError) {

      status(F("Destination not reached"));
    }
  }

  objFinished = true;
}


// -------------------------
// Main loop
// -------------------------

void loop() {
  // After wall following, it finds an object for good
  if (objectMode) {
    objectFindingLoop();
    return;
  }

  // Once it has switched, it wall-follows for good
  if (wallMode) {
    wallLoop();
    // All three IR sensors saw the line (driving, turning or backing up): switch to object finding
    if (allIRSeen) startObjectMode();
    return;
  }

  // Keep the wheel speeds matched between PID updates
  updateMotors();

  // A wall close ahead, with a wall close on the left: switch to wall following
  if (checkForWall()) return;

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
    if (!spinOntoLine(SIDE_RIGHT, SEARCH_LIMIT_DEG)) {
      // No line within the safety limit, so it wasn't a way right after all: spin back onto the line
      Serial.println(F("No way right here: turning back to the line"));
      if (spinOntoLine(SIDE_LEFT, SEARCH_LIMIT_DEG + LOOK_DEG)) {
        // Back on the line. If it ends just ahead, it's a sharp right turn, so remember the way right
        turningSide = -1;
        rightWayStart = millis();
        rightWaySeen = rightWayStart;
      } else if (!spinOntoLine(SIDE_RIGHT, SEARCH_LIMIT_DEG + LOOK_DEG)) {
        // The line it was on isn't there either, so it was already at the tip of a sharp right turn:
        // it spins right again to find the new line. If there's nothing there either, it searches again
        lineNotFound();
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
      printStatus(F("no line"));
      lineLost();
      lostSince = 0;
    }
  }
}
