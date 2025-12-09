#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>

// =========================
// CONFIGURATION CONSTANTS
// =========================

#define NUM_WHEELS 6
#define NUM_STEER  4

// PCA9685 for motor PWM + steering servos
// Address 0x40 by default
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Motor DIR and EN pins (from previous mapping)
const uint8_t MOTOR_DIR_PINS[NUM_WHEELS] = {22, 24, 26, 28, 30, 32};
const uint8_t MOTOR_EN_PINS[NUM_WHEELS]  = {23, 25, 27, 29, 31, 33};

// PCA9685 channels for motor PWM
const uint8_t MOTOR_PWM_CH[NUM_WHEELS] = {0, 1, 2, 3, 4, 5};

// PCA9685 channels for steering servos
const uint8_t STEER_PWM_CH[NUM_STEER] = {6, 7, 8, 9};

// Encoder pins (A=interrupt, B=digital)
const uint8_t ENC_A_PINS[NUM_WHEELS] = {18, 19, 20, 21, 2, 3};
const uint8_t ENC_B_PINS[NUM_WHEELS] = {34, 35, 36, 37, 38, 39};

// Interrupt indices for ENC_A_PINS
// Map: 18→5, 19→4, 20→3, 21→2, 2→0, 3→1
const uint8_t ENC_INT_IDX[NUM_WHEELS] = {5, 4, 3, 2, 0, 1};

// TCA9548A I2C multiplexer address
#define TCA_ADDR 0x70

// TCA channels
#define TCA_CH_IMU    0
#define TCA_CH_INA_L  1
#define TCA_CH_INA_R  2

// INA219 sensors (you can have more if needed)
Adafruit_INA219 ina_left(0x40);
Adafruit_INA219 ina_right(0x41);

// =========================
// STATE VARIABLES
// =========================

// Commanded wheel speed [m/s or rad/s] from Pi
float wheel_speed_cmd[NUM_WHEELS] = {0};

// Commanded steering angle [rad] from Pi
float steer_cmd[NUM_STEER] = {0};

// Encoder counts and derived states
volatile long encoder_counts[NUM_WHEELS] = {0};
long last_encoder_counts[NUM_WHEELS] = {0};
float wheel_pos[NUM_WHEELS] = {0};    // integrated angle (rad), TODO: scale
float wheel_vel[NUM_WHEELS] = {0};    // rad/s, TODO: scale

unsigned long last_vel_time_ms = 0;

// IMU / power placeholders
float quat[4]  = {1, 0, 0, 0};  // w, x, y, z
float gyro[3]  = {0, 0, 0};
float accel[3] = {0, 0, 0};
float power[3] = {12.0, 1.0, 12.0};   // V, I, P

// Timing for feedback publishing
unsigned long last_feedback_ms = 0;
const unsigned long FEEDBACK_PERIOD_MS = 20;  // 50 Hz

// =========================
// TCA9548A helper
// =========================

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// =========================
// ENCODER ISRs
// =========================

void encoderISR0() {
  bool b = digitalRead(ENC_B_PINS[0]);
  encoder_counts[0] += b ? 1 : -1;
}
void encoderISR1() {
  bool b = digitalRead(ENC_B_PINS[1]);
  encoder_counts[1] += b ? 1 : -1;
}
void encoderISR2() {
  bool b = digitalRead(ENC_B_PINS[2]);
  encoder_counts[2] += b ? 1 : -1;
}
void encoderISR3() {
  bool b = digitalRead(ENC_B_PINS[3]);
  encoder_counts[3] += b ? 1 : -1;
}
void encoderISR4() {
  bool b = digitalRead(ENC_B_PINS[4]);
  encoder_counts[4] += b ? 1 : -1;
}
void encoderISR5() {
  bool b = digitalRead(ENC_B_PINS[5]);
  encoder_counts[5] += b ? 1 : -1;
}

void attachEncoders() {
  // A pins must be INPUT_PULLUP; B pins as INPUT_PULLUP
  for (int i = 0; i < NUM_WHEELS; i++) {
    pinMode(ENC_A_PINS[i], INPUT_PULLUP);
    pinMode(ENC_B_PINS[i], INPUT_PULLUP);
  }
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[0]), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[1]), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[2]), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[3]), encoderISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[4]), encoderISR4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[5]), encoderISR5, CHANGE);
}

// =========================
// MOTOR / SERVO OUTPUT
// =========================

void setupMotors() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    pinMode(MOTOR_DIR_PINS[i], OUTPUT);
    pinMode(MOTOR_EN_PINS[i], OUTPUT);
    digitalWrite(MOTOR_EN_PINS[i], HIGH);  // enable by default (depends on driver)
  }

  pwm.begin();
  pwm.setPWMFreq(1000);  // 1 kHz for DC motors; servos will still work OK, or you can use another board
}

void setMotorSpeed(int idx, float cmd) {
  // cmd > 0: forward, cmd < 0: reverse
  bool dir = (cmd >= 0);
  digitalWrite(MOTOR_DIR_PINS[idx], dir ? HIGH : LOW);

  // Map |cmd| to 0..4095
  float mag = fabs(cmd);  // you'll probably normalize cmd externally
  if (mag > 1.0f) mag = 1.0f;  // clip
  uint16_t pwm_val = (uint16_t)(mag * 4095.0f);
  pwm.setPWM(MOTOR_PWM_CH[idx], 0, pwm_val);
}

void setSteerAngle(int idx, float angle_rad) {
  // Simple mapping: center = 1500 µs, ±30° span
  // TODO: calibrate real servo min/max and angle range
  const float max_deg = 30.0;
  float deg = angle_rad * 180.0 / PI;
  if (deg > max_deg) deg = max_deg;
  if (deg < -max_deg) deg = -max_deg;

  float center = 1500;
  float us_per_deg = 10;   // rough guess
  float pulse_us = center + deg * us_per_deg;

  // Convert microseconds to PCA ticks (assuming 50 Hz ~ 20ms => 4096 ticks)
  // 1 tick ~ 4.88 µs
  float ticks = pulse_us / 4.88;
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;

  pwm.setPWM(STEER_PWM_CH[idx], 0, (uint16_t)ticks);
}

void updateMotorsAndSteering() {
  // Apply motor commands
  for (int i = 0; i < NUM_WHEELS; i++) {
    setMotorSpeed(i, wheel_speed_cmd[i]);
  }
  // Apply steering commands
  for (int i = 0; i < NUM_STEER; i++) {
    setSteerAngle(i, steer_cmd[i]);
  }
}

// =========================
// IMU / POWER (STUBS)
// =========================

void setupSensors() {
  Wire.begin();

  // Example INA219 init
  tcaSelect(TCA_CH_INA_L);
  ina_left.begin();

  tcaSelect(TCA_CH_INA_R);
  ina_right.begin();

  // TODO: initialize MPU6050 on TCA_CH_IMU using your MPU library
}

void readIMU() {
  // TODO: Replace with real MPU6050 code
  // For now, send identity quaternion and zeros for gyro/accel
  quat[0] = 1.0f; quat[1] = quat[2] = quat[3] = 0.0f;
  gyro[0] = gyro[1] = gyro[2] = 0.0f;
  accel[0] = accel[1] = accel[2] = 0.0f;
}

void readPower() {
  // Example using INA219
  tcaSelect(TCA_CH_INA_L);
  float vL = ina_left.getBusVoltage_V();
  float iL = ina_left.getCurrent_mA() / 1000.0f;  // A
  tcaSelect(TCA_CH_INA_R);
  float vR = ina_right.getBusVoltage_V();
  float iR = ina_right.getCurrent_mA() / 1000.0f;

  float v = (vL + vR) * 0.5f;
  float i = (iL + iR);
  float p = v * i;

  power[0] = v;
  power[1] = i;
  power[2] = p;
}

// =========================
// ODOM FROM ENCODERS
// =========================

void updateWheelOdom() {
  unsigned long now_ms = millis();
  float dt = (now_ms - last_vel_time_ms) / 1000.0f;
  if (dt <= 0.0f || dt > 1.0f) {
    last_vel_time_ms = now_ms;
    return;
  }
  last_vel_time_ms = now_ms;

  // TODO: Set these based on your hardware
  const float COUNTS_PER_REV = 1024.0f;
  const float tWO_PI = 6.28318530718f;

  for (int i = 0; i < NUM_WHEELS; i++) {
    long c = encoder_counts[i];
    long dc = c - last_encoder_counts[i];
    last_encoder_counts[i] = c;

    // Convert counts to angle
    float dtheta = (float)dc / COUNTS_PER_REV * tWO_PI;
    wheel_pos[i] += dtheta;
    wheel_vel[i] = dtheta / dt;
  }
}

// =========================
// SERIAL PROTOCOL
// =========================

void readCommandsFromPi() {
  // Expect 'C' + 10 floats (6 wheel speeds + 4 steering angles)
  const uint8_t FLOATS_N = 10;
  const uint16_t BYTES_N = FLOATS_N * 4;

  while (Serial.available() > 0) {
    int b = Serial.read();
    if (b == 'C') {
      // Wait until full packet
      if (Serial.available() < BYTES_N) {
        // Not enough yet – wait next loop
        return;
      }

      uint8_t raw[BYTES_N];
      for (int i = 0; i < BYTES_N; i++) {
        raw[i] = (uint8_t)Serial.read();
      }

      float buf[FLOATS_N];
      memcpy(buf, raw, BYTES_N); // little-endian on AVR is fine

      for (int i = 0; i < NUM_WHEELS; i++) {
        wheel_speed_cmd[i] = buf[i];
      }
      for (int i = 0; i < NUM_STEER; i++) {
        steer_cmd[i] = buf[6 + i];
      }
    } else {
      // Unknown header; ignore
    }
  }
}

void sendFeedbackToPi() {
  // Header
  Serial.write('F');

  // Layout: 6 pos, 6 vel, 4 steering, 4 quat, 3 gyro, 3 accel, 3 power
  const int N_POS = 6;
  const int N_VEL = 6;
  const int N_STEER = 4;
  const int N_QUAT = 4;
  const int N_GYRO = 3;
  const int N_ACCEL = 3;
  const int N_PWR = 3;

  // We can serialize directly as bytes
  auto writeFloatArray = [](float* arr, int n) {
    uint8_t* p = (uint8_t*)arr;
    for (int i = 0; i < n * 4; i++) {
      Serial.write(p[i]);
    }
  };

  writeFloatArray(wheel_pos, N_POS);
  writeFloatArray(wheel_vel, N_VEL);
  writeFloatArray(steer_cmd, N_STEER);  // steering feedback == command for now
  writeFloatArray(quat, N_QUAT);
  writeFloatArray(gyro, N_GYRO);
  writeFloatArray(accel, N_ACCEL);
  writeFloatArray(power, N_PWR);
}

// =========================
// SETUP & LOOP
// =========================

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupMotors();
  attachEncoders();
  setupSensors();

  last_vel_time_ms = millis();
  last_feedback_ms = millis();
}

void loop() {
  // 1. Read latest commands from Pi
  readCommandsFromPi();

  // 2. Update actuators
  updateMotorsAndSteering();

  // 3. Update odom / sensors
  updateWheelOdom();
  readIMU();
  readPower();

  // 4. Publish feedback at 50 Hz
  unsigned long now_ms = millis();
  if (now_ms - last_feedback_ms >= FEEDBACK_PERIOD_MS) {
    sendFeedbackToPi();
    last_feedback_ms = now_ms;
  }
}
