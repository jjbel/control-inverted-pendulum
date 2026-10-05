// =========================================================
// EE324 Controls Lab - Experiment 3 - Inverted Pendulum LQR
//
// Arduino Mega + 2x AMT22 (SPI) + L298N motor driver
// Pin mapping follows the Experiment 2 gimbal code exactly:
//   SPI: MOSI 51, MISO 50, SCLK 52  (500 kHz, DIV32)
//   Encoder CS pins: 45 and 48
//   Motor channel: EN 2, IN1 3, IN2 4  (the "yaw"/base motor
//   channel from Exp 2 - the pendulum rig's only motor drives
//   the base arm, same as yaw did)
//
// In Exp 2 the encoder on CS 48 was the BASE (yaw) encoder
// and CS 45 was the other axis. Same assumption here:
//   ARM  (theta) encoder -> CS 48  (on the motor shaft)
//   PEND (alpha) encoder -> CS 45  (at the pendulum pivot)
// If theta/alpha come out swapped in the serial prints,
// exchange the two CS numbers below.
//
// HOW TO RUN:
//  1) Run lqr_pendulum.m in MATLAB, tune Q, copy K1..K4 below.
//  2) Power up / reset with the pendulum HANGING DOWN at
//     rest (don't touch it) and the arm centred -
//     both encoders are zeroed in software during setup()
//     (no hardcoded zero counts needed, unlike Exp 2).
//  3) After zeroing, lift the pendulum up slowly. Within
//     +/-10 deg of vertical the controller engages.
//  4) Motor cuts off if |alpha| > 30 deg or |theta| > 120 deg
//     (safety). Bring the pendulum upright to re-engage.
//
// SIGN CHECK (do this once, gently):
//  - Watch serial: tilt pendulum in the direction that makes
//    alpha POSITIVE. On engagement the arm must move so the
//    pivot chases under the fall. If instead it runs away and
//    the pendulum drops immediately, flip MOTOR_SIGN.
//  - If it "fights itself" (oscillates and throws the
//    pendulum off), flip ALPHA_SIGN.
// =========================================================

#include <SPI.h>

#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCLK 52

// =========================================================
// ENCODER CHIP SELECTS (same two pins as Exp 2)
// =========================================================

#define ARM_CS 45  // theta - base/arm encoder (motor shaft)
#define PEND_CS 48 // alpha - pendulum encoder

// =========================================================
// ARM MOTOR (L298N - the Exp 2 "yaw" channel)
// =========================================================

#define MOTOR_EN 2
#define MOTOR_IN1 3
#define MOTOR_IN2 4

// Flip these (+1 / -1) per the SIGN CHECK above instead of
// rewiring anything.
#define MOTOR_SIGN 1
#define ALPHA_SIGN 1
#define THETA_SIGN 1

// =========================================================
// LQR GAINS  (units: V/rad and V/(rad/s))
//
// >>> REPLACE with the K printed by lqr_pendulum.m <<<
// u = -(K1*theta + K2*alpha + K3*theta_dot + K4*alpha_dot)
// Values below are a typical starting point for
// Q = diag([5 50 0 1]), R = 1. Use your own MATLAB output
// for the report.
// =========================================================

// Q = diag([5 150 0 4]), R = 1 - disturbance-rejection tuned
#define K1 -2.2361 // theta
#define K2 38.5960 // alpha
#define K3 -1.1650 // theta_dot
#define K4 5.0420  // alpha_dot

// Even stiffer, if taps still knock it over
// (Q = diag([10 250 0 6]), R = 1):
// #define K1  -3.1623
// #define K2  48.5160
// #define K3  -1.5630
// #define K4   6.3730

// =========================================================
// CONTROL SETTINGS
// =========================================================

#define LOOP_DT_US 5000UL     // 5 ms -> 200 Hz
#define VMAX 12.0             // motor voltage limit (V)
#define CATCH_ANGLE_DEG 3.0   // engage when |alpha| below
#define ALPHA_LIMIT_DEG 30.0  // disengage above this
#define THETA_LIMIT_DEG 120.0 // arm safety limit
#define VEL_FILTER_ALPHA 0.35 // velocity LPF, 0..1

// L298N + motor deadband: smallest voltage that actually
// makes the motor move. Commands are boosted past this so
// small LQR corrections aren't swallowed by friction and
// the driver's internal drop. Find yours: raise PWM slowly
// with a test sketch until the motor just turns, convert to
// volts (pwm/255*12). Typical: 1.5 - 3.0 V.
#define MOTOR_DEADBAND_V 2.0

// AMT22 14-bit: 16384 counts/rev, direct drive on this rig
#define RAD_PER_COUNT (2.0 * PI / 16384.0)

// =========================================================
// AMT22
// =========================================================

#define AMT22_NOP 0x00
#define RES14 14
#define RES12 12

// =========================================================
// STATE
// =========================================================

uint16_t armZeroCount = 0;
uint16_t pendZeroCount = 0;

float theta = 0.0, alpha = 0.0;       // rad
float thetaDot = 0.0, alphaDot = 0.0; // rad/s (filtered)
float thetaPrev = 0.0, alphaPrev = 0.0;

bool controlActive = false;

unsigned long lastLoopMicros = 0;
int print_variable = 0;

// =========================================================
// SETUP
// =========================================================

void setup() {
  Serial.begin(115200);

  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);

  pinMode(ARM_CS, OUTPUT);
  pinMode(PEND_CS, OUTPUT);
  digitalWrite(ARM_CS, HIGH);
  digitalWrite(PEND_CS, HIGH);

  SPI.begin();

  // 16 MHz / 32 = 500 kHz
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  stopMotor();

  Serial.println();
  Serial.println("====================================");
  Serial.println("*** VEDIKA'S CODE ***");
  Serial.println("INVERTED PENDULUM - LQR BALANCE");
  Serial.println();
  Serial.print("K = [");
  Serial.print(K1);
  Serial.print(", ");
  Serial.print(K2);
  Serial.print(", ");
  Serial.print(K3);
  Serial.print(", ");
  Serial.print(K4);
  Serial.println("]");
  Serial.println();
  Serial.println("Let pendulum HANG DOWN at rest, arm");
  Serial.println("centred. Zeroing encoders in 3 s...");
  Serial.println("====================================");

  delay(3000);

  armZeroCount = averagePosition(ARM_CS);
  pendZeroCount = averagePosition(PEND_CS);

  Serial.print("Arm zero count: ");
  Serial.println(armZeroCount);
  Serial.print("Pendulum zero count (hanging): ");
  Serial.println(pendZeroCount);
  Serial.println();
  Serial.println("Now lift pendulum slowly to the top.");
  Serial.println("ALPHA should read 0 at the top -");
  Serial.println("controller engages within +/-10 deg.");
  Serial.println("====================================");

  lastLoopMicros = micros();
}

// =========================================================
// MAIN LOOP - fixed 200 Hz
// =========================================================

void loop() {
  // Fixed-rate tick: keeps dt constant so the velocity
  // estimates and LQR behave consistently
  while ((micros() - lastLoopMicros) < LOOP_DT_US) {
  }
  lastLoopMicros += LOOP_DT_US;

  const float dt = LOOP_DT_US / 1e6;

  // =======================================================
  // READ BOTH ENCODERS
  // =======================================================

  uint16_t armEncoder = getPositionWithRetry(ARM_CS, RES14);
  uint16_t pendEncoder = getPositionWithRetry(PEND_CS, RES14);

  if (armEncoder == 0xFFFF || pendEncoder == 0xFFFF) {
    // Bad SPI read: skip this tick (state held; safety
    // limits still protect us on the next good read)
    return;
  }

  // =======================================================
  // COUNTS -> ANGLES (rad)
  // =======================================================

  // theta: arm angle from its zero, wrapped to +/-pi
  theta = THETA_SIGN * countsToRad(armEncoder, armZeroCount);

  // alpha: pendulum angle measured from UPRIGHT (top = 0).
  // Encoder is zeroed while hanging down at rest, so shift
  // by 180 deg: top reads 0, bottom reads +/-180.
  alpha = ALPHA_SIGN * wrapPi(countsToRad(pendEncoder, pendZeroCount) - PI);

  // =======================================================
  // VELOCITIES (finite difference + low-pass filter)
  // =======================================================

  float thetaDotRaw = wrapPi(theta - thetaPrev) / dt;
  float alphaDotRaw = wrapPi(alpha - alphaPrev) / dt;
  thetaPrev = theta;
  alphaPrev = alpha;

  thetaDot += VEL_FILTER_ALPHA * (thetaDotRaw - thetaDot);
  alphaDot += VEL_FILTER_ALPHA * (alphaDotRaw - alphaDot);

  // =======================================================
  // ENGAGE / DISENGAGE
  // =======================================================

  float alphaDeg = alpha * 180.0 / PI;
  float thetaDeg = theta * 180.0 / PI;

  if (!controlActive) {
    if (abs(alphaDeg) < CATCH_ANGLE_DEG) {
      controlActive = true;

      // Problem statement: theta is re-defined to 0 deg the
      // moment the balance controller is activated
      armZeroCount = armEncoder;
      theta = 0.0;
      thetaPrev = 0.0;
      thetaDot = 0.0;

      // Also clear the pendulum velocity filter: it holds
      // hand-motion from the lift, which would otherwise
      // produce a large spurious kick at engagement
      alphaPrev = alpha;
      alphaDot = 0.0;

      Serial.println(">>> BALANCE CONTROL ENGAGED <<<");
    }
  } else {
    if (abs(alphaDeg) > ALPHA_LIMIT_DEG || abs(thetaDeg) > THETA_LIMIT_DEG) {
      controlActive = false;
      stopMotor();
      Serial.println(">>> DISENGAGED (limit hit) <<<");
    }
  }

  // =======================================================
  // LQR CONTROL LAW:  u = -K x   (u in Volts)
  // =======================================================

  if (controlActive) {
    float u = -(K1 * theta + K2 * alpha + K3 * thetaDot + K4 * alphaDot);

    if (u > VMAX)
      u = VMAX;
    if (u < -VMAX)
      u = -VMAX;

    setMotorVoltage(MOTOR_SIGN * u);
  } else {
    stopMotor();
  }

  // =======================================================
  // TELEMETRY every ~100 ms
  // =======================================================

  print_variable += 1;
  if (print_variable >= 20) {
    print_variable = 0;
    Serial.print("THETA ");
    Serial.print(thetaDeg, 2);
    Serial.print("  ALPHA ");
    Serial.print(alphaDeg, 2);
    Serial.print("  ");
    Serial.println(controlActive ? "ACTIVE" : "idle");
  }
}

// =========================================================
// ANGLE HELPERS
// =========================================================

// Wrap any angle (rad) to (-pi, pi]
float wrapPi(float a) {
  while (a > PI)
    a -= 2.0 * PI;
  while (a < -PI)
    a += 2.0 * PI;
  return a;
}

// (count - zero) with proper 14-bit wraparound -> radians,
// signed, in (-pi, pi]
float countsToRad(uint16_t count, uint16_t zero) {
  int16_t rel = (int16_t)((count - zero) & 0x3FFF);
  if (rel > 8191)
    rel -= 16384;
  return rel * RAD_PER_COUNT;
}

// =========================================================
// MOTOR CONTROL (L298N, Exp 2 style)
// =========================================================

// Motor voltage (-VMAX..VMAX) -> direction pins + PWM,
// with deadband compensation: |v| is remapped from
// 0..VMAX onto DEADBAND..VMAX so even tiny corrections
// actually move the motor.
void setMotorVoltage(float v) {
  float mag = abs(v);

  if (mag < 0.05) // genuinely zero command -> coast
  {
    stopMotor();
    return;
  }

  float vEff = MOTOR_DEADBAND_V + mag * (VMAX - MOTOR_DEADBAND_V) / VMAX;

  int pwm = constrain((int)(vEff / VMAX * 255.0), 0, 255);

  if (v > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  }

  analogWrite(MOTOR_EN, pwm);
}

void stopMotor() {
  analogWrite(MOTOR_EN, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

// =========================================================
// ENCODER ZEROING (average several reads)
// =========================================================

uint16_t averagePosition(uint8_t cs) {
  uint32_t sum = 0;
  uint8_t n = 0;

  for (uint8_t i = 0; i < 16; i++) {
    uint16_t p = getPositionWithRetry(cs, RES14);
    if (p != 0xFFFF) {
      sum += p;
      n++;
    }
    delay(5);
  }

  return (n > 0) ? (uint16_t)(sum / n) : 0;
}

// =========================================================
// ENCODER READ WITH RETRY (same as Exp 2)
// =========================================================

uint16_t getPositionWithRetry(uint8_t encoder, uint8_t resolution) {
  uint16_t position = getPositionSPI(encoder, resolution);

  if (position != 0xFFFF)
    return position;

  delayMicroseconds(10);

  return getPositionSPI(encoder, resolution);
}

// =========================================================
// AMT22 POSITION READ (same as Exp 2)
// =========================================================

uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution) {
  uint16_t currentPosition;
  bool binaryArray[16];

  digitalWrite(encoder, LOW);
  delayMicroseconds(3);

  currentPosition = ((uint16_t)SPI.transfer(AMT22_NOP)) << 8;
  delayMicroseconds(3);
  currentPosition |= SPI.transfer(AMT22_NOP);

  delayMicroseconds(3);
  digitalWrite(encoder, HIGH);

  for (int i = 0; i < 16; i++) {
    binaryArray[i] = (currentPosition >> i) & 0x01;
  }

  // AMT22 checksum
  if ((binaryArray[15] ==
       !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^
         binaryArray[5] ^ binaryArray[3] ^ binaryArray[1])) &&
      (binaryArray[14] ==
       !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^
         binaryArray[4] ^ binaryArray[2] ^ binaryArray[0]))) {
    currentPosition &= 0x3FFF;
  } else {
    return 0xFFFF;
  }

  if (resolution == RES12)
    currentPosition = currentPosition >> 2;

  return currentPosition;
}