#include <Servo.h>

// =========================
// CONFIGURATION
// =========================

#define NUM_JOINTS 5

// Stepper pins
const uint8_t DIR_PINS[NUM_JOINTS]  = {30, 32, 34, 36, 38};
const uint8_t STEP_PINS[NUM_JOINTS] = {31, 33, 35, 37, 39};
const uint8_t ENABLE_PIN = 40;

// Limit switch pins (LOW when triggered)
const uint8_t LIMIT_PINS[NUM_JOINTS] = {A8, A9, A10, A11, A12};

// Steps per radian for each joint (TODO: set to your actual gearing)
const float STEPS_PER_RAD[NUM_JOINTS] = {
  1000.0f,  // J1
  1000.0f,  // J2
  1000.0f,  // J3
  1000.0f,  // J4
  1000.0f   // J5
};

// Normal motion step interval
const unsigned long STEP_INTERVAL_US = 800;  // microseconds between steps

long current_steps[NUM_JOINTS] = {0};
long target_steps[NUM_JOINTS]  = {0};
unsigned long last_step_time_us[NUM_JOINTS] = {0};

// Homing state
bool homed[NUM_JOINTS] = {false, false, false, false, false};
bool homing_active = true;
bool homing_done = false;
int homing_joint_idx = 0;
unsigned long homing_last_step_time_us[NUM_JOINTS] = {0};
const unsigned long HOMING_STEP_INTERVAL_US = 1500; // slower, safer, micros
const int HOMING_DIR = -1;  // move negative until switch hit

// Gripper servo
const uint8_t GRIPPER_SERVO_PIN = 9;
Servo gripper_servo;
float gripper_cmd = 0.0f;  // 0..1
bool gripper_closed = false;

// Serial feedback timing
unsigned long last_feedback_ms = 0;
const unsigned long FEEDBACK_PERIOD_MS = 50; // 20 Hz

// =========================
// SETUP
// =========================

void setupSteppers() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // LOW=enabled for many drivers (check yours)

  for (int i = 0; i < NUM_JOINTS; i++) {
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(STEP_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);

    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
    last_step_time_us[i] = micros();
    homing_last_step_time_us[i] = micros();
  }
}

void setupGripper() {
  gripper_servo.attach(GRIPPER_SERVO_PIN);
  gripper_servo.write(90); // neutral
}

void startHoming() {
  homing_active = true;
  homing_done = false;
  homing_joint_idx = 0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    homed[i] = false;
    current_steps[i] = 0;
    target_steps[i]  = 0;
    homing_last_step_time_us[i] = micros();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupSteppers();
  setupGripper();

  // Start non-blocking homing sequence
  startHoming();
}

// =========================
// HOMING (NON-BLOCKING)
// =========================

void updateHoming() {
  if (!homing_active) return;

  if (homing_joint_idx >= NUM_JOINTS) {
    // All joints homed
    homing_active = false;
    homing_done = true;
    return;
  }

  int j = homing_joint_idx;

  // If limit already hit, mark homed and move to next joint
  if (digitalRead(LIMIT_PINS[j]) == LOW) {
    current_steps[j] = 0;
    target_steps[j]  = 0;
    homed[j] = true;
    homing_joint_idx++;
    return;
  }

  // Otherwise step slowly in negative direction
  unsigned long now = micros();
  if (now - homing_last_step_time_us[j] < HOMING_STEP_INTERVAL_US) {
    return; // not time for next homing step
  }
  homing_last_step_time_us[j] = now;

  // Set direction
  digitalWrite(DIR_PINS[j], (HOMING_DIR > 0) ? HIGH : LOW);

  // One step
  digitalWrite(STEP_PINS[j], HIGH);
  delayMicroseconds(2);  // driver min pulse width
  digitalWrite(STEP_PINS[j], LOW);

  current_steps[j] += HOMING_DIR;
}

// =========================
// STEPPER UPDATES (NORMAL)
// =========================

void updateJointStepper(int idx) {
  long diff = target_steps[idx] - current_steps[idx];
  if (diff == 0) return;

  unsigned long now = micros();
  if (now - last_step_time_us[idx] < STEP_INTERVAL_US) return;
  last_step_time_us[idx] = now;

  int dir = (diff > 0) ? 1 : -1;
  digitalWrite(DIR_PINS[idx], (dir > 0) ? HIGH : LOW);

  digitalWrite(STEP_PINS[idx], HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PINS[idx], LOW);

  current_steps[idx] += dir;
}

void updateAllSteppers() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    updateJointStepper(i);
  }
}

// =========================
// GRIPPER
// =========================

void updateGripper() {
  // Map gripper_cmd (0..1) to servo angle (e.g., 30..150 degrees)
  float min_deg = 30.0f;
  float max_deg = 150.0f;
  float cmd = gripper_cmd;
  if (cmd < 0.0f) cmd = 0.0f;
  if (cmd > 1.0f) cmd = 1.0f;
  float angle = min_deg + cmd * (max_deg - min_deg);
  gripper_servo.write((int)angle);
  gripper_closed = (cmd > 0.5f);
}

// =========================
// SERIAL PROTOCOL
// =========================
//
// INCOMING (from Pi / arm_bridge.py):
//   'A' + 6 floats = 5 joint positions [rad], 1 gripper_cmd
//
// OUTGOING:
//   'a' + 5 floats (joint positions [rad]) + 6 uint8 flags
//      -> 5 homed flags + 1 gripper_closed
//

void readCommandsFromPi() {
  // 'A' + 6 floats
  const uint8_t FLOATS_N = 6;
  const uint16_t BYTES_N = FLOATS_N * 4;

  while (Serial.available() > 0) {
    int b = Serial.read();
    if (b == 'A') {
      if (Serial.available() < BYTES_N) return;
      uint8_t raw[BYTES_N];
      for (int i = 0; i < BYTES_N; i++) {
        raw[i] = (uint8_t)Serial.read();
      }
      float buf[FLOATS_N];
      memcpy(buf, raw, BYTES_N);

      // 6th float is always gripper command
      gripper_cmd = buf[5];

      // If still homing, ignore joint position commands (only gripper works)
      if (homing_active) {
        return;
      }

      // First 5 floats = joint positions (rad)
      for (int i = 0; i < NUM_JOINTS; i++) {
        float rad = buf[i];
        long steps = (long)(rad * STEPS_PER_RAD[i]);
        target_steps[i] = steps;
      }

    } else {
      // ignore others
    }
  }
}

void sendFeedbackToPi() {
  // Current joint positions in rad
  float joint_pos[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; i++) {
    joint_pos[i] = (float)current_steps[i] / STEPS_PER_RAD[i];
  }

  // header
  Serial.write('a');

  // 5 floats
  uint8_t buf[5 * 4];
  memcpy(buf, joint_pos, sizeof(buf));
  for (int i = 0; i < (int)sizeof(buf); i++) {
    Serial.write(buf[i]);
  }

  // 5 homed flags + 1 gripper_closed
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.write(homed[i] ? (uint8_t)1 : (uint8_t)0);
  }
  Serial.write(gripper_closed ? (uint8_t)1 : (uint8_t)0);
}

// =========================
// LOOP
// =========================

void loop() {
  // 1. Read commands
  readCommandsFromPi();

  // 2. Homing or normal motion
  if (homing_active) {
    updateHoming();
  } else {
    updateAllSteppers();
  }

  // 3. Gripper always updated
  updateGripper();

  // 4. Periodic feedback
  unsigned long now_ms = millis();
  if (now_ms - last_feedback_ms >= FEEDBACK_PERIOD_MS) {
    sendFeedbackToPi();
    last_feedback_ms = now_ms;
  }
}
