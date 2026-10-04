"""NMPC for a lap of the circuit, on the plant's dynamic bicycle model in track coordinates.

The dynamics of [Ux, Uy, r, delta] and the inputs [delta_rate, Fx] are the
plant's (`model.bicycle.dynamics`). The pose is carried relative to the track
instead of in the world frame,

    state  [s, e_y, e_psi, Ux, Uy, r, delta]
    s'     = (Ux cos e_psi - Uy sin e_psi) / (1 - kappa e_y)
    e_y'   = Ux sin e_psi + Uy cos e_psi
    e_psi' = r - kappa s'

so corners enter through the curvature kappa and the track edges are bounds on
e_y. The curvature of each step of the horizon is a parameter, read from the
track at the positions of the previous plan.

Speed is not prescribed per corner: the cost asks for the speed limit, and the
tire model with its friction limit, together with the track-edge and sideslip
constraints, makes the plan brake for a corner it could not otherwise hold.
Traffic cars are kept out by the same soft ellipse as on the straight road,
for the `n_cars` nearest ones, predicted along their lane at constant speed.
"""

import time

import casadi as ca
import numpy as np

from ..model.backend import CASADI
from ..model.bicycle import IDELTA, IFX, IPSI, IR, IRATE, IUX, IUY, IX, IY, NU, NX, dynamics, fx_limits
from ..sim.integrate import rk4_step

NS = 4  # slacks per step: traffic ellipses, track edges, sideslip, speed limit
FAR = 1e4  # s offset that puts an unused traffic slot out of reach [m]


class TrackNMPC:
    """Controller `(t, x) -> [steering rate, Fx]` for `sim.lap.run_lap`; `x` is the plant's world state.

    `lane` is the lane the car keeps when nothing is in the way. The ellipse
    keeps the bodies `margin_lat` apart side by side and `margin_long` apart
    nose to tail; `edge_margin` is kept between the body and the asphalt edge.
    """

    def __init__(
        self,
        circuit,
        params,
        horizon=4.0,
        h=0.1,
        lane="right",
        n_cars=2,
        margin_lat=0.6,
        margin_long=1.0,
        edge_margin=0.2,
        sideslip_max=np.radians(7.0),
        w_speed=1.0,
        w_lane=0.2,
        w_lat=1.0,
        w_accel=1.0,
        w_rate=100.0,
        w_slack=1e3,
        max_iter=200,
    ):
        self.c = c = circuit
        self.p = p = params
        self.h = h
        self.N = N = int(round(horizon / h))
        self.n_cars = n_cars
        self.solve_times = []
        self.failures = 0  # solves that did not converge; their result is still applied

        # ellipse around a traffic car, in ego centre-of-gravity coordinates
        ly = c.car_width + margin_lat
        lx = c.car_length + margin_long
        self.ell_b = B = c.lane_width - 0.4
        self.ell_a = A = lx / np.sqrt(1.0 - (ly / B) ** 2)
        e_max = c.road_half_width - 0.5 * c.car_width - edge_margin
        to_front = 0.5 * c.car_length - c.cg_to_center
        to_rear = 0.5 * c.car_length + c.cg_to_center
        slip = np.tan(sideslip_max)
        e_ref = c.lane_offset(lane)

        # parameters: measured state, curvature of each step, traffic positions at each predicted state
        x_init = ca.SX.sym("x_init", NX)
        kappa = ca.SX.sym("kappa", N)
        car_s = ca.SX.sym("car_s", n_cars, N)
        car_e = ca.SX.sym("car_e", n_cars)
        par = ca.vertcat(x_init, kappa, ca.vec(car_s), car_e)

        x = ca.SX.sym("x", NX, N + 1)
        u = ca.SX.sym("u", NU, N)  # the longitudinal input is the acceleration Fx / m, for scaling
        slack = ca.SX.sym("s", NS, N)

        def f(xk, uk, kap):
            d = dynamics(xk, ca.vertcat(uk[IRATE], p.m * uk[IFX]), p, CASADI)  # with psi = e_psi
            s_dot = d[IX] / (1.0 - kap * xk[IY])
            return ca.vertcat(s_dot, d[IY], d[IPSI] - kap * s_dot, d[IUX:])

        J = 0.0
        g_eq = [x[:, 0] - x_init]
        g_in = []  # each entry <= 0
        for k in range(N):
            xk, uk, nxt, kap = x[:, k], u[:, k], x[:, k + 1], kappa[k]
            g_eq.append(nxt - rk4_step(lambda z, v: f(z, v, kap), xk, uk, h))

            v_lat = nxt[IUX] * ca.sin(nxt[IPSI]) + nxt[IUY] * ca.cos(nxt[IPSI])
            J += (
                w_speed * (nxt[IUX] - c.v_max) ** 2
                + w_lane * (nxt[IY] - e_ref) ** 2
                + w_lat * (v_lat**2 + (nxt[IR] - kap * nxt[IUX]) ** 2)
                + w_accel * uk[IFX] ** 2
                + w_rate * uk[IRATE] ** 2
                + w_slack * (ca.sum1(slack[:, k]) + ca.sumsqr(slack[:, k]))
            )

            for j in range(n_cars):
                ds = nxt[IX] - c.cg_to_center - car_s[j, k]
                g_in.append(1.0 - (ds / A) ** 2 - ((nxt[IY] - car_e[j]) / B) ** 2 - slack[0, k])
            for arm in (to_front, -to_rear):  # asphalt edges, at the front and rear ends of the body
                e_end = nxt[IY] + arm * ca.sin(nxt[IPSI])
                g_in += [e_end - e_max - slack[1, k], -e_max - e_end - slack[1, k]]
            g_in += [nxt[IUY] - slip * nxt[IUX] - slack[2, k], -nxt[IUY] - slip * nxt[IUX] - slack[2, k]]
            g_in.append(nxt[IUX] - c.v_max - slack[3, k])

        g_eq, g_in = ca.vertcat(*g_eq), ca.vertcat(*g_in)
        self.lbg = np.concatenate([np.zeros(g_eq.shape[0]), np.full(g_in.shape[0], -np.inf)])
        self.ubg = np.zeros(g_eq.shape[0] + g_in.shape[0])

        lbx, ubx = np.full((N + 1, NX), -np.inf), np.full((N + 1, NX), np.inf)
        lbx[1:, IDELTA], ubx[1:, IDELTA] = -p.delta_max, p.delta_max
        lbx[1:, IUX] = 3.0  # the tire model is not meant for lower speeds
        lbu = np.tile([-p.delta_rate_max, 0.0], N)  # force limits are set per solve
        ubu = np.tile([p.delta_rate_max, 0.0], N)
        self.lbw = np.concatenate([lbu, lbx.ravel(), np.zeros(NS * N)])
        self.ubw = np.concatenate([ubu, ubx.ravel(), np.full(NS * N, np.inf)])

        nlp = {"x": ca.vertcat(ca.vec(u), ca.vec(x), ca.vec(slack)), "p": par, "f": J, "g": ca.vertcat(g_eq, g_in)}
        options = {
            "print_time": False,
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "ipopt.max_iter": max_iter,
            "ipopt.tol": 1e-6,
        }
        self.solver = ca.nlpsol("nmpc_track", "ipopt", nlp, options)
        self._warm = None  # previous primal and dual solution
        self.plan = None  # last predicted states in track coordinates, shape (N + 1, NX)

    def model_state(self, x):
        """World state as the model's [s, e_y, e_psi, Ux, Uy, r, delta]."""
        z = np.array(x, dtype=float)
        z[IX], z[IY], z[IPSI] = self.c.track_state(x)
        return z

    def _step_curvature(self, s_plan):
        """Mean track curvature over each step of the predicted positions `s_plan` (N + 1,)."""
        w = np.linspace(0.0, 1.0, 5)[:, None]
        return self.c.track.curvature(s_plan[:-1] * (1.0 - w) + s_plan[1:] * w).mean(axis=0)

    def _traffic(self, t, s0):
        """Predicted s and lane offset of the nearest cars, unwrapped to lie around `s0`."""
        lap = self.c.track.length
        times = t + self.h * np.arange(1, self.N + 1)
        slots = []
        for car in self.c.traffic:
            s = self.c.traffic_s(car, times)
            ahead = (s[0] - s0 + 0.5 * lap) % lap - 0.5 * lap  # signed distance to the car, along the track
            slots.append((abs(ahead), s0 + ahead + np.unwrap(s - s[0], period=lap), self.c.lane_offset(car.lane)))
        slots.sort(key=lambda slot: slot[0])
        slots = slots[: self.n_cars]
        while len(slots) < self.n_cars:
            slots.append((0.0, np.full(self.N, s0 + FAR), 0.0))
        return np.array([slot[1] for slot in slots]), np.array([slot[2] for slot in slots])

    def __call__(self, t, x):
        p, N = self.p, self.N
        z0 = self.model_state(x)

        if self.plan is None:
            s_plan = z0[IX] + z0[IUX] * self.h * np.arange(N + 1)
        else:  # the previous plan's progress, from where the car is now
            s_plan = z0[IX] + self.plan[:, IX] - self.plan[0, IX]
        car_s, car_e = self._traffic(t, z0[IX])
        par = np.concatenate([z0, self._step_curvature(s_plan), car_s.ravel(order="F"), car_e])

        fx_min, fx_max = fx_limits(x[IUX], p)
        self.lbw[IFX : NU * N : NU] = fx_min / p.m
        self.ubw[IFX : NU * N : NU] = fx_max / p.m

        args = dict(p=par, lbx=self.lbw, ubx=self.ubw, lbg=self.lbg, ubg=self.ubg)
        if self._warm is None:
            guess = np.tile(z0, (N + 1, 1))
            guess[:, IX] = s_plan
            guess[1:, [IPSI, IUY, IR, IDELTA]] = 0.0
            args["x0"] = np.concatenate([np.zeros(NU * N), guess.ravel(), np.zeros(NS * N)])
        else:
            args.update(self._warm)
            # the warm start's s values belong to the previous lap position; move them to this one
            xs = args["x0"][NU * N : NU * N + NX * (N + 1)].reshape(N + 1, NX)
            xs[:, IX] += z0[IX] - xs[0, IX]

        start = time.perf_counter()
        sol = self.solver(**args)
        self.solve_times.append(time.perf_counter() - start)
        if not self.solver.stats()["success"]:
            self.failures += 1

        w = np.asarray(sol["x"]).ravel()
        self._warm = dict(x0=w, lam_x0=sol["lam_x"], lam_g0=sol["lam_g"])
        self.plan = w[NU * N : NU * N + NX * (N + 1)].reshape(N + 1, NX).copy()
        return np.array([w[IRATE], p.m * w[IFX]])
