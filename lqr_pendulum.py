"""LQR gains for the rotary inverted pendulum, from the manual's linear model.

Usage:
    python lqr_pendulum.py --q 8 0 0.5 0             # K for Q = diag(q), R = 1
    python lqr_pendulum.py --q 8 0 0.5 0 --r 2       # ... with R = 2
    python lqr_pendulum.py --from-k -2.83 44.16 -1.58 5.67   # which Q gives this K?

State x = [theta, alpha, theta_dot, alpha_dot] (arm, pendulum from upright,
rad and rad/s), input u = motor volts, control law u = -K x. That is the order
and sign convention inverted_pendulum_control.ino uses for K1..K4.

Needs: pip install numpy scipy
"""

import argparse

import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.optimize import least_squares

# Table 2 of PendulumSetup_manual.pdf (SI units)
Mp = 0.027  # pendulum mass
lp = 0.153  # pivot to pendulum centre of mass
r = 0.0826  # arm pivot to pendulum pivot
Jeq = 1.23e-4  # equivalent inertia about the motor shaft
Jp = 1.10e-4  # pendulum inertia about its pivot
g = 9.81
Rm = 3.30  # armature resistance
Kt = 0.02797  # torque constant
Km = 0.02797  # back-emf constant


def plant():
    """A, B of Table 3, linearised about the pendulum upright."""
    d = Jp * Jeq + Mp * lp**2 * Jeq + Jp * Mp * r**2
    a = np.array(
        [
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, r * Mp**2 * lp**2 * g / d, -Kt * Km * (Jp + Mp * lp**2) / (d * Rm), 0],
            [0, Mp * lp * g * (Jeq + Mp * r**2) / d, -Mp * lp * Kt * r * Km / (d * Rm), 0],
        ]
    )
    b = np.array(
        [[0], [0], [Kt * (Jp + Mp * lp**2) / (d * Rm)], [Mp * lp * Kt * r / (d * Rm)]]
    )
    return a, b


def lqr_gain(q, r_weight=1.0):
    """K minimising the integral of x'Qx + R u^2, with Q = diag(q)."""
    a, b = plant()
    s = solve_continuous_are(a, b, np.diag(q), np.array([[r_weight]]))
    return (b.T @ s / r_weight).ravel()


def closed_loop_poles(k):
    a, b = plant()
    return np.linalg.eigvals(a - b @ np.atleast_2d(k))


def fit_q(k_target, r_weight=1.0, starts=10, seed=0):
    """
    Inverse LQR: the diagonal Q that reproduces k_target for this R.

    theta is a pure integral of theta_dot, which for this plant makes
    K1 = -sqrt(q1 / R) exactly, so q1 comes straight from K1 and only q2..q4
    are searched. They are fitted as p**2 so a weight can land on exactly zero,
    from a few random starts since the Riccati solve can fail far from a fit.
    Returns (q, k_of_q, worst relative error).
    """
    k_target = np.asarray(k_target, dtype=float)
    scale = np.maximum(np.abs(k_target), 1e-6)
    q1 = r_weight * k_target[0] ** 2

    def gains(p):
        return lqr_gain(np.concatenate(([q1], p**2)), r_weight)

    def residual(p):
        try:
            return (gains(p) - k_target)[1:] / scale[1:]
        except (np.linalg.LinAlgError, ValueError):
            return np.full(3, 1e3)

    rng = np.random.default_rng(seed)
    best = None
    for _ in range(starts):
        start = np.sqrt(np.exp(rng.uniform(np.log(0.01), np.log(100), 3)))
        fit = least_squares(residual, start)
        if best is None or fit.cost < best.cost:
            best = fit
        if best.cost < 1e-12:
            break

    q = np.concatenate(([q1], best.x**2))
    k = lqr_gain(q, r_weight)
    return q, k, float(np.max(np.abs((k - k_target) / scale)))


def describe(q, r_weight, k):
    poles = ", ".join(f"{p:.2f}" for p in closed_loop_poles(k))
    return (
        f"Q = diag([{' '.join(f'{v:g}' for v in q)}]), R = {r_weight:g}\n"
        f"K = [{', '.join(f'{v:.4f}' for v in k)}]\n"
        f"closed-loop poles: {poles}"
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--q", type=float, nargs=4, metavar="Q", help="diag(Q)")
    group.add_argument(
        "--from-k", type=float, nargs=4, metavar="K", help="find the Q behind this K"
    )
    parser.add_argument("--r", type=float, default=1.0, help="R weight (default 1)")
    args = parser.parse_args()

    if args.q:
        print(describe(args.q, args.r, lqr_gain(args.q, args.r)))
        return

    q, k, error = fit_q(args.from_k, args.r)
    print(describe(np.round(q, 4), args.r, k))
    print(f"worst gain mismatch: {error * 100:.2f}%")


if __name__ == "__main__":
    main()
