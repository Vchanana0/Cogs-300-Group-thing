// MotorControl.pde
// Sends drive commands to the Arduino running Motor.ino.
// Hold a key to drive, release it to stop. Hold W with A or D to curve.
// Shows each wheel's speed as measured by the IR speed sensors.

import processing.serial.*;

// -------------------------
// Settings
// -------------------------

final int PORT_INDEX = 2;     // Which entry in the console's port list is the Arduino
final int BAUD_RATE  = 9600;  // Must match Serial.begin() in Motor.ino


// -------------------------
// State
// -------------------------

Serial port;
String status = "";
String arduinoReply = "(nothing yet)";

// Latest wheel speeds reported by the Arduino
int leftRPM = 0;
int rightRPM = 0;

// Last command sent to the Arduino:
// w, a, s, d, r, q (curve left), e (curve right) or x (stop)
char activeKey = 'x';

// Which keys are being held down right now
boolean wDown = false;
boolean aDown = false;
boolean sDown = false;
boolean dDown = false;
boolean rDown = false;
boolean stopDown = false;  // Space or X


// -------------------------
// Setup
// -------------------------

void setup() {
  size(420, 400);
  textAlign(CENTER, CENTER);
  rectMode(CENTER);

  String[] ports = Serial.list();
  println("Available serial ports:");
  printArray(ports);

  if (ports.length == 0) {
    status = "No serial ports found. Plug in the Arduino and restart.";
  } else if (PORT_INDEX >= ports.length) {
    status = "PORT_INDEX is out of range. Check the console list.";
  } else {
    String portName = ports[PORT_INDEX];
    try {
      port = new Serial(this, portName, BAUD_RATE);
      port.bufferUntil('\n');
      status = "Connected to " + portName;
    } catch (Exception e) {
      status = "Could not open " + portName + ". Is the Serial Monitor open?";
    }
  }
}


// -------------------------
// Drawing
// -------------------------

void draw() {
  background(30);

  fill(255);
  textSize(14);
  text(status, width / 2, 30);

  // A curve lights up both of the keys that make it
  drawKey("W", width / 2,       110, activeKey == 'w' || activeKey == 'q' || activeKey == 'e');
  drawKey("R", width / 2 + 140, 110, activeKey == 'r');
  drawKey("A", width / 2 - 70,  180, activeKey == 'a' || activeKey == 'q');
  drawKey("S", width / 2,       180, activeKey == 's');
  drawKey("D", width / 2 + 70,  180, activeKey == 'd' || activeKey == 'e');

  fill(200);
  textSize(13);
  text("Hold W A S D to drive or R to rotate. Release to stop.", width / 2, 243);
  text("Hold W with A or D to curve. Space or X also stops.", width / 2, 263);

  fill(255);
  textSize(14);
  text("Arduino says: " + arduinoReply, width / 2, 298);

  // Wheel speeds from the IR speed sensors
  stroke(80);
  line(40, 322, width - 40, 322);
  drawSpeed("Left wheel", leftRPM, width / 2 - 100);
  drawSpeed("Right wheel", rightRPM, width / 2 + 100);
}

void drawSpeed(String label, int rpm, float x) {
  fill(200);
  textSize(13);
  text(label, x, 343);

  fill(255);
  textSize(22);
  text(rpm + " RPM", x, 370);
}

void drawKey(String label, float x, float y, boolean active) {
  stroke(200);
  if (active) {
    fill(0, 170, 255);
  } else {
    fill(60);
  }
  rect(x, y, 60, 60, 8);

  fill(255);
  textSize(24);
  text(label, x, y);
}


// -------------------------
// Keyboard
// -------------------------

void keyPressed() {
  if (key == CODED) return;

  char k = Character.toLowerCase(key);

  // Space or X always sends a stop, even if one was already sent
  if (k == ' ' || k == 'x') {
    if (!stopDown) {
      stopDown = true;
      sendCommand('x');
    }
    return;
  }

  setKeyDown(k, true);
  updateCommand();
}

void keyReleased() {
  if (key == CODED) return;

  char k = Character.toLowerCase(key);
  setKeyDown(k, false);
  updateCommand();
}

// Remember whether a key is held down
void setKeyDown(char k, boolean down) {
  if (k == 'w') wDown = down;
  if (k == 'a') aDown = down;
  if (k == 's') sDown = down;
  if (k == 'd') dDown = down;
  if (k == 'r') rDown = down;
  if (k == ' ' || k == 'x') stopDown = down;
}

// Work out the command from the keys that are held down
char currentCommand() {
  if (stopDown) return 'x';

  if (wDown && dDown && !aDown) return 'e';  // W + D = curve right
  if (wDown && aDown && !dDown) return 'q';  // W + A = curve left

  if (wDown) return 'w';
  if (sDown) return 's';
  if (aDown) return 'a';
  if (dDown) return 'd';
  if (rDown) return 'r';

  return 'x';  // Nothing held, so stop
}

// Send the command only when it has changed
void updateCommand() {
  char command = currentCommand();
  if (command != activeKey) {
    sendCommand(command);
  }
}

// If the window loses focus, key releases never arrive, so let go of everything
void focusLost() {
  super.focusLost();

  wDown = false;
  aDown = false;
  sDown = false;
  dDown = false;
  rDown = false;
  stopDown = false;
  updateCommand();
}


// -------------------------
// Serial
// -------------------------

void sendCommand(char k) {
  activeKey = k;
  if (port != null) {
    port.write(k);
  }
}

// Called whenever the Arduino sends a full line back
void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;
  line = trim(line);

  if (line.startsWith("SPEED")) {
    // A speed report looks like: SPEED 132 128 (left RPM, right RPM)
    String[] parts = splitTokens(line);
    if (parts.length == 3) {
      leftRPM = parseInt(parts[1]);
      rightRPM = parseInt(parts[2]);
    }
  } else if (line.length() > 0) {
    arduinoReply = line;
  }
}

// Stop the robot when the sketch window closes
void exit() {
  if (port != null) {
    port.write('x');
  }
  super.exit();
}
