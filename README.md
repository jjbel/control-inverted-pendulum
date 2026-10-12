# Exp 3 – Inverted pendulum (LQR balance)

An Arduino Mega balances the rotary pendulum upright using LQR state feedback.
Two encoders read the arm angle θ and the pendulum angle α, and one DC motor
drives the arm.

| File | What it is |
|---|---|
| `testing_setup/testing_setup.ino` | The sketch: encoders, LQR control law, motor driver, serial `k` command |
| `monitor.py` | Flashes the sketch and/or opens the serial port; computes K from Q and either writes it into the sketch or sends it live |
| `lqr_pendulum.py` | The manual's linear model (A, B) and the LQR solve; also finds the Q behind a given K |
| `PendulumSetup_manual.pdf` | Parameters (Table 2) and state-space matrices (Table 3) the model comes from |

Python needs `pip install pyserial numpy scipy`. The sketch needs the
**LibPrintf** library, and uploads use the Arduino 1.8 IDE at
`C:\Program Files (x86)\Arduino` (or `arduino-cli` if it's on PATH).

## Hardware

| Signal | Mega pin |
|---|---|
| SPI MOSI / MISO / SCLK | 51 / 50 / 52 |
| Arm encoder (θ) CS | 45 |
| Pendulum encoder (α) CS | 48 |
| Motor: L298N channel B, ENB / IN3 / IN4 | 7 / 5 / 6 |

The motor goes on the L298N's channel B output (OUT3/OUT4).

## Running it

```
python monitor.py --upload                 # flash testing_setup.ino, then watch serial
python monitor.py                          # just watch serial
python monitor.py --q 8 0 0.5 0 --upload   # compute K from Q, write it into the sketch, flash
python monitor.py --q 8 200 0.5 0          # compute K, send it over serial (no reflash)
python monitor.py --log run1.txt           # log to run1.txt instead of serial_log.txt
python monitor.py --no-log                 # terminal only
```

Everything the sketch prints is also appended to `serial_log.txt`. Each run
starts with a header giving the date and time, the port and the command line,
e.g. `===== 2026-10-12 14:51:36  COM5  monitor.py --q 8 200 0.5 0 =====`.

Each run goes like this:

1. **Opening the port resets the Mega, and the sketch zeroes both encoders 3 s
   after reset.** Have the pendulum hanging still and the arm centred whenever
   you start `monitor.py`.
2. Lift the pendulum slowly. When |α| < 3° (`CATCH_ANGLE_DEG`), the
   controller engages and prints `>>> BALANCE CONTROL ENGAGED <<<`. θ is
   re-zeroed at that moment.
3. The motor cuts out if |α| > 30° or |θ| > 120°. Bring the pendulum back up
   to re-engage.

The telemetry prints every 100 ms: `THETA <deg>  ALPHA <deg>  ACTIVE|idle`.

Gains sent with `--q` (no `--upload`), or typed as `k K1 K2 K3 K4`, last only
until the next reset. Use `--upload` for gains you want to keep. The sketch
prints `K = [...]` whenever its gains change.

### First time on a rig: sign check

Do this before tuning anything. If a sign is wrong, no Q will work.

- **Encoders swapped:** if moving the arm changes ALPHA, swap `ARM_CS` and
  `PEND_CS`.
- **α should read ≈ 0 at the top and ±180 hanging down.** If it doesn't, the
  pendulum wasn't still at reset.
- **Motor direction:** hold the pendulum near upright and let it engage. The
  arm must move so the pivot goes *under* the side the pendulum is falling
  toward. If it runs the other way and the pendulum drops at once, flip
  `MOTOR_SIGN`. Recheck this whenever the motor wiring changes.
- **Pendulum direction:** if it engages, then oscillates harder and harder and
  throws the pendulum off, flip `ALPHA_SIGN`.

## How the controller works

The state is `x = [θ, α, θ̇, α̇]`, where θ is the arm angle and α is the
pendulum angle measured from upright (rad, rad/s). Linearised about upright,
the rig is `ẋ = Ax + Bu`, where u is the motor voltage. A and B come from the
manual's Table 3, built in `lqr_pendulum.py`. The pendulum mode is unstable
(an open-loop pole at +10.2 rad/s), so it falls without feedback.

Every 5 ms (200 Hz) the sketch:

1. reads both encoders and converts them to θ and α;
2. estimates θ̇ and α̇ by finite difference, smoothed with a low-pass filter;
3. applies **`u = −(K1·θ + K2·α + K3·θ̇ + K4·α̇)`**, clipped to ±12 V;
4. drives the L298N, with deadband compensation (see below).

LQR picks K by minimising

    J = ∫ (q₁θ² + q₂α² + q₃θ̇² + q₄α̇² + R·u²) dt

which is a trade-off between keeping the state small (the Q terms) and using
little voltage (the R term). The resulting K always stabilises the linear
model. Tuning only changes how it behaves. Only the ratio Q/R matters.

The signs of K are expected:

- **K2 and K4 (pendulum) are large and positive.** These do the balancing:
  they move the pivot under the falling pendulum, like balancing a broom on
  your hand.
- **K1 and K3 (arm) are negative.** To bring the arm back to centre, the
  controller first has to tip the pendulum toward the centre, so the arm moves
  briefly the "wrong" way.
- **K1 = −√(q₁/R) exactly**, for this plant.

## Where the current gains came from

The sketch had K = [−2.83, 44.16, −1.58, 5.67], labelled
"Q = diag([10 250 0 6])". That label was wrong: that Q gives K2 ≈ 48.5.
Fitting backwards (`python lqr_pendulum.py --from-k -2.83 44.16 -1.58 5.67`)
gives

**Q = diag([8, 0, 0.5, 0]), R = 1**, which reproduces K within 0.2%.

So the current gains put **no weight on the pendulum at all**. The large K2
comes only from LQR having to stabilise the unstable pendulum mode. This Q is
the baseline for everything below.

## How to tune it

### What each weight does

Each row changes one thing from the baseline (computed with `lqr_pendulum.py`).
"Slowest pole" sets how quickly the arm settles. "u at 3°" is the voltage
K2 alone asks for when the pendulum is 3° off.

| Change | K = [K1, K2, K3, K4] | Slowest pole | u at 3° | What you'll see |
|---|---|---|---|---|
| baseline Q = [8 0 0.5 0], R = 1 | −2.83, 44.16, −1.58, 5.67 | −4.1 | 2.3 V | — |
| q₂ = 200 (pendulum angle) | −2.83, 47.71, −1.63, 5.85 | −3.7 ± 1.4j | 2.5 V | barely different |
| q₂ = 3000 | −2.83, 78.54, −2.04, 7.19 | −1.9 ± 1.4j | 4.1 V | much stiffer balance, arm wanders more slowly back |
| q₁ = 20 (arm angle) | −4.47, 52.50, −2.06, 6.80 | −6.3 ± 1.1j | 2.8 V | arm pulled back to centre harder |
| q₃ = 2 (arm velocity) | −2.83, 65.58, −2.25, 8.57 | −2.0 | 3.4 V | arm swings damped; overall more aggressive |
| q₄ = 5 (pendulum velocity) | −2.83, 50.00, −1.66, 6.84 | −3.7 | 2.6 V | pendulum wobble damped |
| R = 2 | −2.00, 34.03, −1.15, 4.30 | −4.4 | 1.8 V | gentler everything, less motor effort |
| R = 0.5 | −4.00, 58.56, −2.19, 7.62 | −4.1 | 3.1 V | more aggressive everything |

What the table shows:

- **q₂ has to be large to matter.** The pendulum is so unstable that K2 is
  already big with q₂ = 0. You need hundreds to thousands, not tens.
  Step it ×3 (200 → 600 → 2000…).
- **There is a trade-off between the two angles.** A heavier pendulum weight
  makes balancing stiffer, but slows the arm's return to centre (the slowest
  pole moves toward 0). Raise q₁ to compensate.
- **R scales everything.** Use it once the *shape* of the response is right
  and you only want it faster or gentler.

A principled starting point is Bryson's rule: q_i = 1 / (largest allowed
value)². For the spec (|α| < 3°, |θ| < 30°) that gives q₂ ≈ 365 and q₁ ≈ 3.6.
But with R = 1/12² it asks for K2 ≈ 312, far beyond this motor, so in practice
start from the baseline and raise weights from there.

### Procedure

1. Do the **sign check** above.
2. Upload the baseline: `python monitor.py --q 8 0 0.5 0 --upload`.
3. Lift the pendulum, let it engage, and watch ALPHA and THETA. Then give it a
   light tap and watch again. Each run lands in `serial_log.txt` under its own timestamp, ready for the report.
4. Change **one** weight at a time using the symptom table below. Check the
   new K and poles first with `python lqr_pendulum.py --q ...`, then try it
   live with `python monitor.py --q ...` (no reflash; the gains are sent once
   the sketch boots).
5. When it meets the spec (|α| < 3°, |θ| < 30°, u < 12 V), make it permanent
   with `--upload` and record Q, R, K and the poles.

### Symptoms → what to change

| What you see | Try |
|---|---|
| Falls immediately on engaging, arm runs away | Sign problem: `MOTOR_SIGN` |
| Engages, oscillates harder each swing, falls | Sign problem: `ALPHA_SIGN` |
| Balances but a light tap knocks it over | Raise q₂ (×3 steps), or lower R |
| Pendulum wobbles quickly around upright | Raise q₄ |
| Arm drifts toward the ±120° cutout | Raise q₁ |
| Arm swings slowly back and forth, never settles | Raise q₃ |
| Motor buzzes or chatters, rig shakes | Raise R; then see deadband and velocity filter below |
| Feels sluggish everywhere | Lower R |

## Settings outside the LQR model

These are in `#define`s near the top of the sketch. LQR assumes none of them,
so if Q and R stop helping, look here.

- **`MOTOR_DEADBAND_V` (2.0 V).** Any command above 0.05 V is boosted to at
  least 2 V, so friction doesn't swallow small corrections. But it means that
  near upright the controller acts like an on/off switch. If it buzzes or holds
  a small steady oscillation once balanced, lower this. Better, measure the
  real deadband: raise PWM slowly until the arm just moves, then convert with
  `pwm / 255 × 12`.
- **`VMAX` (12 V) clipping.** LQR is linear. If the command hits ±12 V
  regularly, the gains are too high for this motor, so raise R.
- **`VEL_FILTER_ALPHA` (0.35, about a 17 Hz cutoff).** Smaller gives smoother
  but later θ̇ and α̇; too much lag looks like K4 is too low. Larger gives
  faster but noisier rates, and the noise turns into motor chatter through K4.
- **`CATCH_ANGLE_DEG` (3°), `ALPHA_LIMIT_DEG` (30°), `THETA_LIMIT_DEG` (120°).**
  The engage window and the safety cutouts.
- **Model accuracy.** K is only as good as the manual's parameters. If the
  rig's pendulum differs (mass, length, Jp), update `lqr_pendulum.py`; the
  manual's section 4.3 shows how to measure Jp from the swing frequency.
