/*
 * Line-Following Maze Car
 * Board  : Arduino Mega 2560
 * Driver : one dual H-bridge (e.g. L298N), 2 motors in parallel on each side
 * Sensors: 5 digital IR sensors at the front of the car
 *
 * Behaviour
 *   - Follows a black line on a white floor.
 *   - When the far-left / far-right sensor sees black, or the line is lost
 *     completely, the car is at a 90-degree turn or junction:
 *       1. Creep forward so the wheels sit over the corner, then stop.
 *       2. Rotate up to 90 degrees anti-clockwise (left) looking for a line.
 *       3. If no line: rotate up to 180 degrees clockwise to check the
 *          other side (passing straight-ahead on the way).
 *       4. If still no line it is a dead end: keep rotating clockwise
 *          until the line we came from is found (U-turn).
 */

// Global Variables - Motor Control Pins
const int ENA = 13; // PWM pin Left Motor
const int IN1 = 12; // Direction Left Motor
const int IN2 = 11; // Direction Left Motor
const int IN3 = 10; // Direction Right Motor
const int IN4 = 9;  // Direction Right Motor
const int ENB = 8;  // PWM pin Right Motor

// 5-Sensor Pin Layout
const int SENSOR_FAR_LEFT  = 29;
const int SENSOR_LEFT      = 31;
const int SENSOR_CENTER    = 35;
const int SENSOR_RIGHT     = 39;
const int SENSOR_FAR_RIGHT = 41;

// ---------------------------------------------------------------------------
// Tuning - adjust these on the real car
// ---------------------------------------------------------------------------

// Most IR modules (TCRT5000) output HIGH over black and LOW over white.
// If yours is the opposite, change this to LOW.
const int LINE_DETECTED = HIGH;

// Set to true if a side drives backwards when it should go forwards.
const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;

// Speeds (0 - 255)
const int BASE_SPEED   = 140; // straight-line speed
const int MAX_SPEED    = 200;
const int TURN_SPEED   = 130; // in-place rotation speed
const int CREEP_SPEED  = 110; // speed while moving onto the junction

// Line-following PD gains (error is in the range -2 .. +2)
const float KP = 45.0;
const float KD = 25.0;

// Timings (ms). TURN_90_MS is the most important one: measure how long the
// car takes to spin 90 degrees at TURN_SPEED (see CALIBRATE_TURN below).
const unsigned long TURN_90_MS       = 650;
const unsigned long CREEP_MS         = 180; // drive forward so wheels are over the corner
const unsigned long STOP_PAUSE_MS    = 150; // settle time after stopping
const unsigned long LEAVE_LINE_MS    = 150; // min. rotation before accepting a new line
const unsigned long TURN_COOLDOWN_MS = 300; // ignore junctions right after a turn
const int           CONFIRM_READS    = 2;   // consecutive reads to confirm a junction

// Set to true to make the car spin exactly TURN_90_MS anti-clockwise once
// every 3 s. Adjust TURN_90_MS until it turns exactly 90 degrees.
const bool CALIBRATE_TURN = false;

// ---------------------------------------------------------------------------

enum Direction { ANTI_CLOCKWISE, CLOCKWISE };

struct Sensors {
  bool farLeft, left, center, right, farRight;
};

float lastError = 0;
int junctionCount = 0;
unsigned long lastTurnTime = 0;

// ---------------------------------------------------------------------------
// Low-level helpers
// ---------------------------------------------------------------------------

Sensors readSensors() {
  Sensors s;
  s.farLeft  = digitalRead(SENSOR_FAR_LEFT)  == LINE_DETECTED;
  s.left     = digitalRead(SENSOR_LEFT)      == LINE_DETECTED;
  s.center   = digitalRead(SENSOR_CENTER)    == LINE_DETECTED;
  s.right    = digitalRead(SENSOR_RIGHT)     == LINE_DETECTED;
  s.farRight = digitalRead(SENSOR_FAR_RIGHT) == LINE_DETECTED;
  return s;
}

bool anyLine(const Sensors &s) {
  return s.farLeft || s.left || s.center || s.right || s.farRight;
}

// Drive one side. speed: -255 (full reverse) .. 255 (full forward)
void driveSide(int pwmPin, int inA, int inB, int speed, bool invert) {
  if (invert) speed = -speed;
  if (speed >= 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    speed = -speed;
  }
  analogWrite(pwmPin, constrain(speed, 0, 255));
}

void setMotors(int leftSpeed, int rightSpeed) {
  driveSide(ENA, IN1, IN2, leftSpeed,  INVERT_LEFT_MOTOR);
  driveSide(ENB, IN3, IN4, rightSpeed, INVERT_RIGHT_MOTOR);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void spin(Direction dir) {
  if (dir == ANTI_CLOCKWISE) setMotors(-TURN_SPEED, TURN_SPEED);
  else                       setMotors(TURN_SPEED, -TURN_SPEED);
}

// ---------------------------------------------------------------------------
// Turning
// ---------------------------------------------------------------------------

// Rotate in place for at most maxMs, stopping as soon as the center sensor
// lands on a line. The first LEAVE_LINE_MS (and until the center sensor has
// left any line it started on) are ignored so the car does not stop on the
// line it is already sitting on. Returns true if a line was found.
bool rotateUntilLine(Direction dir, unsigned long maxMs) {
  unsigned long start = millis();
  bool leftStartLine = false;

  spin(dir);
  while (millis() - start < maxMs) {
    bool onLine = readSensors().center;
    if (!onLine) leftStartLine = true;
    if (onLine && leftStartLine && millis() - start >= LEAVE_LINE_MS) {
      stopMotors();
      delay(STOP_PAUSE_MS);
      return true;
    }
  }
  stopMotors();
  delay(STOP_PAUSE_MS);
  return false;
}

void handleJunction() {
  Serial.println(F("Junction: stopping"));

  // Sensors are at the front: creep forward so the wheels (turning point)
  // are over the corner before rotating.
  setMotors(CREEP_SPEED, CREEP_SPEED);
  delay(CREEP_MS);
  stopMotors();
  delay(STOP_PAUSE_MS);

  // 1. Try 90 degrees anti-clockwise (left). A bit of margin for slip.
  Serial.println(F("  Checking left (90 deg anti-clockwise)"));
  if (rotateUntilLine(ANTI_CLOCKWISE, TURN_90_MS * 13 / 10)) {
    Serial.println(F("  -> line found on the left"));
  } else {
    // 2. Turn 180 degrees clockwise to see the other side. Extra margin
    //    covers the overshoot from the left search. Straight-ahead is
    //    passed on the way, so left > straight > right priority.
    Serial.println(F("  Nothing left, checking other side (180 deg clockwise)"));
    if (rotateUntilLine(CLOCKWISE, TURN_90_MS * 2 + TURN_90_MS * 4 / 10)) {
      Serial.println(F("  -> line found"));
    } else {
      // 3. Dead end: keep turning clockwise to go back the way we came.
      Serial.println(F("  Dead end, turning back"));
      if (!rotateUntilLine(CLOCKWISE, TURN_90_MS * 15 / 10)) {
        Serial.println(F("  !! Line lost completely, stopping"));
        stopMotors();
        while (true) {}  // halt; reset the board to restart
      }
    }
  }

  lastError = 0;
  junctionCount = 0;
  lastTurnTime = millis();
}

// ---------------------------------------------------------------------------
// Line following
// ---------------------------------------------------------------------------

void followLine(const Sensors &s) {
  // Weighted position of the line under the three middle sensors.
  int count = s.left + s.center + s.right;
  float error;
  if (count == 0) {
    error = lastError;  // keep correcting the way we were going
  } else {
    error = (-1.0 * s.left + 1.0 * s.right) / count;
  }

  float correction = KP * error + KD * (error - lastError);
  lastError = error;

  int leftSpeed  = constrain(BASE_SPEED + (int)correction, -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - (int)correction, -MAX_SPEED, MAX_SPEED);
  setMotors(leftSpeed, rightSpeed);
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(SENSOR_FAR_LEFT,  INPUT);
  pinMode(SENSOR_LEFT,      INPUT);
  pinMode(SENSOR_CENTER,    INPUT);
  pinMode(SENSOR_RIGHT,     INPUT);
  pinMode(SENSOR_FAR_RIGHT, INPUT);

  stopMotors();
  Serial.println(F("Maze car ready, starting in 2 s"));
  delay(2000);
  lastTurnTime = millis();
}

void loop() {
  if (CALIBRATE_TURN) {
    spin(ANTI_CLOCKWISE);
    delay(TURN_90_MS);
    stopMotors();
    delay(3000);
    return;
  }

  Sensors s = readSensors();

  // A 90-degree turn / junction shows up as a far sensor seeing black,
  // or the line disappearing completely (corner already passed the sensors).
  bool junction = s.farLeft || s.farRight || !anyLine(s);
  bool cooledDown = millis() - lastTurnTime > TURN_COOLDOWN_MS;

  if (junction && cooledDown) {
    junctionCount++;
    if (junctionCount >= CONFIRM_READS) {
      handleJunction();
      return;
    }
  } else {
    junctionCount = 0;
  }

  followLine(s);
  delay(5);
}
