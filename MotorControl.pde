// MotorControl.pde
// Sends WASD drive commands to the Arduino running Motor.ino.
// Hold a key to drive, release it to stop.

import processing.serial.*;

// -------------------------
// Settings
// ------------------------

final int PORT_INDEX = 2;     // Which entry in the console's port list is the Arduino
final int BAUD_RATE  = 9600;  // Must match Serial.begin() in Motor.ino


// -------------------------
// State
// -------------------------

Serial port;
String status = "";
String arduinoReply = "(nothing yet)";
char activeKey = 'x';  // Last command sent: w, a, s, d or x (stop)


// -------------------------
// Setup
// -------------------------

void setup() {
  size(420, 340);
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

  drawKey("W", width / 2,      110, activeKey == 'w');
  drawKey("A", width / 2 - 70, 180, activeKey == 'a');
  drawKey("S", width / 2,      180, activeKey == 's');
  drawKey("D", width / 2 + 70, 180, activeKey == 'd');

  fill(200);
  textSize(13);
  text("Hold W A S D to drive, release to stop. Space or X also stops.", width / 2, 250);

  fill(255);
  textSize(14);
  text("Arduino says: " + arduinoReply, width / 2, 295);
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
  if (k == ' ') k = 'x';

  if (k == 'w' || k == 'a' || k == 's' || k == 'd' || k == 'x') {
    sendCommand(k);
  }
}

void keyReleased() {
  if (key == CODED) return;

  // Stop only when the key that is currently driving the robot is let go
  char k = Character.toLowerCase(key);
  if (k == activeKey && k != 'x') {
    sendCommand('x');
  }
}


// -------------------------
// Serial
// -------------------------

void sendCommand(char k) {
  // A held key repeats, so skip commands the robot is already doing
  if (k == activeKey && k != 'x') return;

  activeKey = k;
  if (port != null) {
    port.write(k);
  }
}

// Called whenever the Arduino sends a full line back
void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line != null) {
    arduinoReply = trim(line);
  }
}

// Stop the robot when the sketch window closes
void exit() {
  if (port != null) {
    port.write('x');
  }
  super.exit();
}
