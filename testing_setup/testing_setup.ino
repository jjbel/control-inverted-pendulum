// EE324 Controls Lab - Experiment 3 - Inverted Pendulum LQR
//
// Pin mapping follows the Experiment 2 gimbal code exactly:
//   SPI: MOSI 51, MISO 50, SCLK 52  (500 kHz, DIV32)
//   Encoder CS pins: 45 and 48
//   Motor channel B: EN 7, IN3 5, IN4 6
//
//   ARM  (theta) encoder -> CS 48  (on the motor shaft)
//   PEND (alpha) encoder -> CS 45  (at the pendulum pivot)
// If theta/alpha come out swapped in the serial prints,
// exchange the two CS numbers below.
//
//  1) Pick Q and R, then:
//       python monitor.py --q q1 q2 q3 q4 --upload
//     computes K, writes it into K1..K4 below and flashes.
//     Without --upload the same K is sent over serial
//     instead (lost on reset). lqr_pendulum.py has the model.
//  2) Power up / reset with the pendulum HANGING DOWN at
//     rest (don't touch it) and the arm centred -
//     both encoders are zeroed in software during setup()
//     (no hardcoded zero counts needed, unlike Exp 2).
//  3) After zeroing, lift the pendulum up slowly. Within
//     CATCH_ANGLE_DEG of vertical the controller engages.
//  4) Motor cuts off if |alpha| > 30 deg or |theta| > 120 deg
//     (safety). Bring the pendulum upright to re-engage.
//
// SERIAL COMMANDS (115200, newline-terminated):
//   k <K1> <K2> <K3> <K4>   replace the LQR gains until reset
//
// SIGN CHECK (do this once, gently):
//  - Watch serial: tilt pendulum in the direction that makes
//    alpha POSITIVE. On engagement the arm must move so the
//    pivot chases under the fall. If instead it runs away and
//    the pendulum drops immediately, flip MOTOR_SIGN.
//  - If it "fights itself" (oscillates and throws the
//    pendulum off), flip ALPHA_SIGN.

#include <LibPrintf.h>
#include <SPI.h>

#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCLK 52

// ENCODER CHIP SELECTS
#define ARM_CS 45  // theta - base/arm encoder (motor shaft)
#define PEND_CS 48 // alpha - pendulum encoder

// ARM MOTOR - L298N channel B (channel A is EN 2, IN1 3, IN2 4)
#define MOTOR_EN 7  // ENB
#define MOTOR_IN1 5 // IN3
#define MOTOR_IN2 6 // IN4

// Flip these (+1 / -1) per the SIGN CHECK above instead of
// rewiring anything.
#define MOTOR_SIGN -1
#define ALPHA_SIGN 1
#define THETA_SIGN -1 // flipped after the rig swap: arm ran away at the
                      // rate the model predicts for a backwards theta

// LQR GAINS  (units: V/rad and V/(rad/s))
//
// u = -(K1*theta + K2*alpha + K3*theta_dot + K4*alpha_dot)
// The defaults below; `k` on serial overrides them at runtime.
// monitor.py --upload rewrites the Q line and the four
// defines together, so keep them adjacent.

// Q = diag([8 200 0.5 0]), R = 1
#define K1 -2.8284 // theta
#define K2 47.7090 // alpha
#define K3 -1.6305 // theta_dot
#define K4 5.8459 // alpha_dot

// CONTROL SETTINGS

#define LOOP_DT_US 5000UL     // 5 ms -> 200 Hz
#define VMAX 12.0             // motor voltage limit (V)
#define CATCH_ANGLE_DEG 3.0   // engage when |alpha| below
#define ALPHA_LIMIT_DEG 30.0  // disengage above this
#define THETA_LIMIT_DEG 120.0 // arm safety limit
#define VEL_FILTER_ALPHA 0.35 // velocity LPF, 0..1
#define BAD_READ_LIMIT 10     // failed reads in a row (50 ms) before the motor is cut

// L298N + motor deadband: smallest voltage that actually
// makes the motor move. Commands are boosted past this so
// small LQR corrections aren't swallowed by friction and
// the driver's internal drop. Find yours: raise PWM slowly
// with a test sketch until the motor just turns, convert to
// volts (pwm/255*12). Typical: 1.5 - 3.0 V.
#define MOTOR_DEADBAND_V 2.0

// AMT22 14-bit: 16384 counts/rev, direct drive on this rig
#define RAD_PER_COUNT (2.0 * PI / 16384.0)

// AMT22

#define AMT22_NOP 0x00
#define RES14 14
#define RES12 12

// STATE

uint16_t armZeroCount = 0;
uint16_t pendZeroCount = 0;

float theta = 0.0, alpha = 0.0;       // rad
float thetaDot = 0.0, alphaDot = 0.0; // rad/s (filtered)
float thetaPrev = 0.0, alphaPrev = 0.0;

bool controlActive = false;

float K[4] = {K1, K2, K3, K4};

unsigned long lastLoopMicros = 0;
int print_variable = 0;
uint16_t badReads = 0; // consecutive failed encoder reads

// SETUP
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

  printf("\n====================================\n"
         "INVERTED PENDULUM - LQR BALANCE\n\n");
  printGains();
  printf("\nLet pendulum HANG DOWN at rest, arm\n"
         "centred. Zeroing encoders in 3 s...\n"
         "====================================\n");

  delay(3000);

  armZeroCount = averagePosition(ARM_CS);
  pendZeroCount = averagePosition(PEND_CS);

  printf("Arm zero count: %u\n"
         "Pendulum zero count (hanging): %u\n\n"
         "Now lift pendulum slowly to the top.\n"
         "ALPHA should read 0 at the top -\n"
         "controller engages within +/-%.0f deg.\n"
         "====================================\n",
         armZeroCount, pendZeroCount, CATCH_ANGLE_DEG);

  lastLoopMicros = micros();
}

// MAIN LOOP - fixed 200 Hz
void loop() {
  // Fixed-rate tick: keeps dt constant so the velocity
  // estimates and LQR behave consistently
  while ((micros() - lastLoopMicros) < LOOP_DT_US) {
  }
  lastLoopMicros += LOOP_DT_US;

  const float dt = LOOP_DT_US / 1e6;

  pollSerial();

  // READ BOTH ENCODERS
  uint16_t armEncoder = getPositionWithRetry(ARM_CS, RES14);
  uint16_t pendEncoder = getPositionWithRetry(PEND_CS, RES14);

  if (armEncoder == 0xFFFF || pendEncoder == 0xFFFF) {
    // Bad SPI read: skip this tick, holding the last command.
    // The safety limits can't act without readings, so after
    // BAD_READ_LIMIT in a row cut the motor and disengage, and
    // say which encoder is silent at the telemetry rate.
    badReads++;
    if (badReads >= BAD_READ_LIMIT) {
      if (controlActive) {
        controlActive = false;
        printf(">>> DISENGAGED (encoder not responding) <<<\n");
      }
      stopMotor();
    }
    if (badReads % 20 == 0)
      printf("encoder read failed: arm (CS %d) %s, pend (CS %d) %s\n",
             ARM_CS, armEncoder == 0xFFFF ? "FAIL" : "ok", PEND_CS,
             pendEncoder == 0xFFFF ? "FAIL" : "ok");
    return;
  }
  badReads = 0;

  // COUNTS -> ANGLES (rad)
  // theta: arm angle from its zero, wrapped to +/-pi
  theta = THETA_SIGN * countsToRad(armEncoder, armZeroCount);

  // alpha: pendulum angle measured from UPRIGHT (top = 0).
  // Encoder is zeroed while hanging down at rest, so shift
  // by 180 deg: top reads 0, bottom reads +/-180.
  alpha = ALPHA_SIGN * wrapPi(countsToRad(pendEncoder, pendZeroCount) - PI);

  // VELOCITIES (finite difference + low-pass filter)
  float thetaDotRaw = wrapPi(theta - thetaPrev) / dt;
  float alphaDotRaw = wrapPi(alpha - alphaPrev) / dt;
  thetaPrev = theta;
  alphaPrev = alpha;

  thetaDot += VEL_FILTER_ALPHA * (thetaDotRaw - thetaDot);
  alphaDot += VEL_FILTER_ALPHA * (alphaDotRaw - alphaDot);

  // ENGAGE / DISENGAGE
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

      printf(">>> BALANCE CONTROL ENGAGED <<<\n");
    }
  } else {
    if (abs(alphaDeg) > ALPHA_LIMIT_DEG || abs(thetaDeg) > THETA_LIMIT_DEG) {
      controlActive = false;
      stopMotor();
      printf(">>> DISENGAGED (limit hit) <<<\n");
    }
  }

  // LQR CONTROL LAW:  u = -K x   (u in Volts)
  if (controlActive) {
    float u = -(K[0] * theta + K[1] * alpha + K[2] * thetaDot + K[3] * alphaDot);

    if (u > VMAX)
      u = VMAX;
    if (u < -VMAX)
      u = -VMAX;

    setMotorVoltage(MOTOR_SIGN * u);
  } else {
    stopMotor();
  }

  // TELEMETRY every ~100 ms
  print_variable += 1;
  if (print_variable >= 20) {
    print_variable = 0;
    // kept under the 64-byte TX buffer so it never stalls the loop
    printf("THETA %7.2f  ALPHA %7.2f  %s\n", thetaDeg, alphaDeg,
           controlActive ? "ACTIVE" : "idle");
  }
}

// GAINS OVER SERIAL
void printGains() {
  printf("K = [%.4f, %.4f, %.4f, %.4f]\n", K[0], K[1], K[2], K[3]);
}

// k <K1> <K2> <K3> <K4>
void handleCommand(char *cmd) {
  char *verb = strtok(cmd, " ");
  if (verb == NULL || verb[0] != 'k') {
    printf("unknown command; send: k K1 K2 K3 K4\n");
    return;
  }

  char *args[4];
  for (uint8_t i = 0; i < 4; i++) {
    args[i] = strtok(NULL, " ");
    if (args[i] == NULL) {
      printf("need four gains: k K1 K2 K3 K4\n");
      return;
    }
  }

  // atof, not sscanf %f: avr-libc leaves float parsing out of scanf
  for (uint8_t i = 0; i < 4; i++)
    K[i] = atof(args[i]);
  printGains();
}

// Gather serial bytes into a line, running each command as it completes
void pollSerial() {
  static char buf[64];
  static uint8_t len;

  while (Serial.available()) {
    char c = Serial.read();

    // ignore CR so both line endings behave the same
    if (c == '\r')
      continue;

    if (c != '\n') {
      if (len < sizeof(buf) - 1)
        buf[len++] = c;
      continue;
    }

    if (len > 0) {
      buf[len] = '\0';
      handleCommand(buf);
      len = 0;
    }
  }
}

// ANGLE HELPERS
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

// MOTOR CONTROL (L298N, Exp 2 style)
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

// ENCODER ZEROING (average several reads)
// Averages offsets from the first good read rather than raw
// counts: raw counts straddling the 16383 -> 0 wrap (e.g. an
// encoder whose zero was set at this position) would average
// to somewhere far round the circle instead.
uint16_t averagePosition(uint8_t cs) {
  uint16_t first = 0xFFFF;
  int32_t sum = 0;
  uint8_t n = 0;

  for (uint8_t i = 0; i < 16; i++) {
    uint16_t p = getPositionWithRetry(cs, RES14);
    if (p != 0xFFFF) {
      if (first == 0xFFFF)
        first = p;
      int16_t rel = (int16_t)((p - first) & 0x3FFF);
      if (rel > 8191)
        rel -= 16384;
      sum += rel;
      n++;
    }
    delay(5);
  }

  if (n == 0)
    return 0;
  return (uint16_t)(first + sum / n) & 0x3FFF;
}

// ENCODER READ WITH RETRY (same as Exp 2)
uint16_t getPositionWithRetry(uint8_t encoder, uint8_t resolution) {
  uint16_t position = getPositionSPI(encoder, resolution);

  if (position != 0xFFFF)
    return position;

  delayMicroseconds(10);

  return getPositionSPI(encoder, resolution);
}

// AMT22 POSITION READ (same as Exp 2)
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