"""Fixed-step integration."""


def rk4_step(f, x, u, dt):
    """One classical Runge-Kutta step of dx/dt = f(x, u) with u held constant."""
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
