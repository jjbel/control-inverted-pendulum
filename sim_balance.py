"""Simulate the balance loop the way testing_setup.ino runs it, to compare Q choices.

The plant is the manual's linear model (lqr_pendulum.py). Around it, the
things the sketch does that LQR doesn't know about: a 200 Hz loop, 14-bit
encoder quantisation, finite-difference velocities through the low-pass filter,
the deadband boost, and the +/-VMAX clip. An alpha zero offset can be added,
since the hanging-zero is never exact.

Usage:
    python sim_balance.py --q 8 200 0.5 0
    python sim_balance.py --q 8 200 0.5 0 --deadband 0     # without the boost
    python sim_balance.py --q 8 200 0.5 0 --alpha-offset 1.5
"""

import argparse

import numpy as np
from scipy.linalg import expm

from lqr_pendulum import lqr_gain, plant

# testing_setup.ino settings
DT = 0.005
VMAX = 12.0
VEL_FILTER_ALPHA = 0.35
RAD_PER_COUNT = 2 * np.pi / 16384
CATCH_DEG = 3.0


def motor_voltage(u, deadband):
    """setMotorVoltage(): coast below 0.05 V, else boost |u| onto deadband..VMAX."""
    u = np.clip(u, -VMAX, VMAX)
    if abs(u) < 0.05:
        return 0.0
    return np.sign(u) * (deadband + abs(u) * (VMAX - deadband) / VMAX)


def simulate(k, seconds, deadband, alpha_offset_deg, alpha0_deg, motor_gain):
    """
    Run the loop from rest with the pendulum alpha0 off upright. The motor's
    actual drive is motor_gain times what the model assumes (the voltage that
    really reaches the motor after the L298N drop, or a different motor).
    Returns times, true theta and alpha (deg) and commanded u.
    """
    a, b = plant()
    # zero-order hold over one loop period
    m = expm(np.block([[a, b], [np.zeros((1, 5))]]) * DT)
    ad, bd = m[:4, :4], m[:4, 4:]

    x = np.array([0.0, np.radians(alpha0_deg), 0.0, 0.0])
    offset = np.radians(alpha_offset_deg)
    quant = lambda angle: np.round(angle / RAD_PER_COUNT) * RAD_PER_COUNT

    prev = None
    rates = np.zeros(2)
    out = []
    for step in range(int(seconds / DT)):
        measured = np.array([quant(x[0]), quant(x[1]) + offset])
        if prev is not None:
            raw = (measured - prev) / DT
            rates += VEL_FILTER_ALPHA * (raw - rates)
        prev = measured

        state = np.array([measured[0], measured[1], rates[0], rates[1]])
        u = float(-k @ state)
        v = motor_voltage(u, deadband) * motor_gain
        x = ad @ x + (bd * v).ravel()
        out.append((step * DT, np.degrees(x[0]), np.degrees(x[1]), u))

        if abs(np.degrees(x[1])) > 30 or abs(np.degrees(x[0])) > 120:
            break

    return np.array(out)


def summarise(trace, settle_from=2.0):
    t, theta, alpha, u = trace.T
    fell = abs(alpha[-1]) > 30 or abs(theta[-1]) > 120
    late = t >= settle_from
    return {
        "fell_at": t[-1] if fell else None,
        "theta_pk": np.max(np.abs(theta)),
        "alpha_pk": np.max(np.abs(alpha)),
        "theta_late": np.max(np.abs(theta[late])) if late.any() else np.nan,
        "alpha_late": np.max(np.abs(alpha[late])) if late.any() else np.nan,
        "u_pk": np.max(np.abs(u)),
        "theta_mean_late": np.mean(theta[late]) if late.any() else np.nan,
    }


def report(label, s):
    if s["fell_at"] is not None:
        status = f"FELL at {s['fell_at']:.2f} s"
    else:
        status = (
            f"after 2 s: |theta| <= {s['theta_late']:5.1f} deg "
            f"(mean {s['theta_mean_late']:+5.1f}), |alpha| <= {s['alpha_late']:4.2f} deg"
        )
    print(
        f"{label:34s} peak theta {s['theta_pk']:6.1f}  peak alpha {s['alpha_pk']:5.2f}  "
        f"peak u {s['u_pk']:5.1f} V  | {status}"
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--q", type=float, nargs=4, required=True)
    parser.add_argument("--r", type=float, default=1.0)
    parser.add_argument("--deadband", type=float, default=2.0, help="MOTOR_DEADBAND_V")
    parser.add_argument("--alpha-offset", type=float, default=0.0, help="deg")
    parser.add_argument("--alpha0", type=float, default=CATCH_DEG * 0.9, help="deg")
    parser.add_argument("--motor-gain", type=float, default=1.0)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    k = lqr_gain(args.q, args.r)
    print("K =", np.round(k, 3))
    trace = simulate(
        k, args.seconds, args.deadband, args.alpha_offset, args.alpha0, args.motor_gain
    )
    report("", summarise(trace))


if __name__ == "__main__":
    main()
