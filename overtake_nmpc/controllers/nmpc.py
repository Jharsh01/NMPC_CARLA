"""NMPC for the overtake on the same dynamic bicycle model the plant integrates.

State and input are the plant's own,

    state  [X, Y, psi, Ux, Uy, r, delta]
    input  [delta_rate, Fx]

and the prediction model is `model.bicycle.dynamics` (Fiala tires with
friction-circle derating, load transfer), built on CasADi symbols and
discretised with one RK4 step per interval. The plant adds steering lag and
tire relaxation on top of this model; the controller does not model those.

The problem keeps the structure of the course formulation
(`legacy/nmpc_takeover_student.py`): the lead car is kept out by an ellipse,
the cost holds the entry speed and the ego-lane centre and penalises lateral
velocity, yaw rate and the inputs. The ellipse, the road-edge limits and a
sideslip limit are soft, so the problem stays feasible when the car is pushed
past a limit it planned to ride along. It is solved with IPOPT in a receding horizon, warm
started from the previous solution.
"""

import time

import casadi as ca
import numpy as np

from ..model.backend import CASADI
from ..model.bicycle import IDELTA, IFX, IPSI, IR, IRATE, IUX, IUY, IX, IY, NU, NX, dynamics, fx_limits
from ..sim.integrate import rk4_step

NS = 3  # slacks per step: lead-car ellipse, road edges, sideslip


class NMPC:
    """Controller `(t, x) -> [steering rate, Fx]` for `sim.closed_loop.run_overtake`.

    Plans `horizon` seconds ahead in steps of `h`. The ellipse is centred on
    the lead car and is the smallest one of lateral semi-axis `ellipse_width`
    that keeps the two bodies `margin_lat` apart side by side and
    `margin_long` apart nose to tail. `edge_margin` is the distance kept
    between the body and the road edges, `sideslip_use` the share of the
    scenario's sideslip limit the plan may use. The weights apply to
    speed error [m/s], lane offset [m], lateral velocity [m/s] and yaw rate
    [rad/s], longitudinal acceleration Fx/m [m/s^2], steering rate [rad/s] and
    constraint violation.
    """

    def __init__(
        self,
        scenario,
        params,
        horizon=5.0,
        h=0.1,
        margin_lat=0.6,
        margin_long=1.0,
        ellipse_width=None,
        edge_margin=0.3,
        sideslip_use=0.7,
        w_speed=1.0,
        w_lane=0.2,
        w_lat=1.0,
        w_accel=1.0,
        w_rate=100.0,
        w_slack=1e3,
        max_iter=200,
    ):
        self.sc = sc = scenario
        self.p = p = params
        self.h = h
        self.N = N = int(round(horizon / h))
        self.v_des = None
        self.solve_times = []
        self.failures = 0  # solves that did not converge; their result is still applied

        # ellipse around the lead car, in ego centre-of-gravity coordinates
        ly = sc.car_width + margin_lat
        lx = sc.car_length + margin_long
        self.ell_b = B = ellipse_width if ellipse_width is not None else sc.lane_width - 0.4
        if B <= ly:
            raise ValueError("ellipse_width must exceed car_width + margin_lat")
        self.ell_a = A = lx / np.sqrt(1.0 - (ly / B) ** 2)
        self.ell_x = sc.cg_to_center  # the ego body centre is this far behind its centre of gravity

        right, left = sc.road_edges
        y_min = right + 0.5 * sc.car_width + edge_margin
        y_max = left - 0.5 * sc.car_width - edge_margin
        to_front = 0.5 * sc.car_length - sc.cg_to_center
        to_rear = 0.5 * sc.car_length + sc.cg_to_center
        slip = np.tan(sideslip_use * sc.sideslip_max)

        # parameters: measured state, lead car position now, desired speed
        x_init = ca.SX.sym("x_init", NX)
        lead_now = ca.SX.sym("lead_now")
        v_des = ca.SX.sym("v_des")
        par = ca.vertcat(x_init, lead_now, v_des)

        # decision variables; the longitudinal input is the acceleration Fx / m, for scaling
        x = ca.SX.sym("x", NX, N + 1)
        u = ca.SX.sym("u", NU, N)
        slack = ca.SX.sym("s", NS, N)

        def f(xk, uk):
            return dynamics(xk, ca.vertcat(uk[IRATE], p.m * uk[IFX]), p, CASADI)

        J = 0.0
        g_eq = [x[:, 0] - x_init]
        g_in = []  # each entry <= 0
        for k in range(N):
            xk, uk, nxt = x[:, k], u[:, k], x[:, k + 1]
            g_eq.append(nxt - rk4_step(f, xk, uk, h))

            v_lat = nxt[IUX] * ca.sin(nxt[IPSI]) + nxt[IUY] * ca.cos(nxt[IPSI])
            J += (
                w_speed * (nxt[IUX] - v_des) ** 2
                + w_lane * nxt[IY] ** 2
                + w_lat * (v_lat**2 + nxt[IR] ** 2)
                + w_accel * uk[IFX] ** 2
                + w_rate * uk[IRATE] ** 2
                + w_slack * (ca.sum1(slack[:, k]) + ca.sumsqr(slack[:, k]))
            )

            # the lead car, at the predicted state
            dx = nxt[IX] - (lead_now + sc.v_lead * h * (k + 1)) - self.ell_x
            g_in.append(1.0 - (dx / A) ** 2 - (nxt[IY] / B) ** 2 - slack[0, k])
            # road edges, at the front and rear ends of the body
            for arm in (to_front, -to_rear):
                y_end = nxt[IY] + arm * ca.sin(nxt[IPSI])
                g_in += [y_end - y_max - slack[1, k], y_min - y_end - slack[1, k]]
            # sideslip
            g_in += [nxt[IUY] - slip * nxt[IUX] - slack[2, k], -nxt[IUY] - slip * nxt[IUX] - slack[2, k]]

        g_eq, g_in = ca.vertcat(*g_eq), ca.vertcat(*g_in)
        self.lbg = np.concatenate([np.zeros(g_eq.shape[0]), np.full(g_in.shape[0], -np.inf)])
        self.ubg = np.zeros(g_eq.shape[0] + g_in.shape[0])

        # simple bounds: steering rate (force limits are set per solve), steering angle, slacks
        lbx, ubx = np.full((N + 1, NX), -np.inf), np.full((N + 1, NX), np.inf)
        lbx[1:, IDELTA], ubx[1:, IDELTA] = -p.delta_max, p.delta_max
        lbx[1:, IUX] = p.Ux_min
        self.lbu = np.tile([-p.delta_rate_max, 0.0], (N, 1))
        self.ubu = np.tile([p.delta_rate_max, 0.0], (N, 1))
        self.lbw = np.concatenate([self.lbu.ravel(), lbx.ravel(), np.zeros(NS * N)])
        self.ubw = np.concatenate([self.ubu.ravel(), ubx.ravel(), np.full(NS * N, np.inf)])

        nlp = {"x": ca.vertcat(ca.vec(u), ca.vec(x), ca.vec(slack)), "p": par, "f": J, "g": ca.vertcat(g_eq, g_in)}
        options = {
            "print_time": False,
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "ipopt.max_iter": max_iter,
            "ipopt.tol": 1e-6,
        }
        self.solver = ca.nlpsol("nmpc", "ipopt", nlp, options)
        self._warm = None  # previous primal and dual solution
        self.plan = None  # last predicted states, shape (N + 1, NX)

    def initial_guess(self, t, x0):
        """Constant speed, riding along the top of the ellipse past the lead car."""
        k = np.arange(self.N + 1)
        X = x0[IX] + x0[IUX] * self.h * k
        dx = X - self.sc.lead_x(t + self.h * k) - self.ell_x
        y = np.maximum(x0[IY], 1.05 * self.ell_b * np.sqrt(np.clip(1.0 - (dx / self.ell_a) ** 2, 0.0, None)))
        y[0] = x0[IY]
        xs = np.tile(x0, (self.N + 1, 1))
        xs[:, IX], xs[:, IY] = X, y
        xs[1:, IPSI] = np.arctan2(np.gradient(y, self.h), x0[IUX])[1:]
        xs[1:, [IUY, IR, IDELTA]] = 0.0
        return np.concatenate([np.zeros(NU * self.N), xs.ravel(), np.zeros(NS * self.N)])

    def __call__(self, t, x):
        p, N = self.p, self.N
        if self.v_des is None:  # hold the entry speed, like the baseline
            self.v_des = x[IUX]
        par = np.concatenate([x, [self.sc.lead_x(t), self.v_des]])

        # force limits at the current speed, as accelerations
        fx_min, fx_max = fx_limits(x[IUX], p)
        self.lbw[IFX : NU * N : NU] = fx_min / p.m
        self.ubw[IFX : NU * N : NU] = fx_max / p.m

        args = dict(p=par, lbx=self.lbw, ubx=self.ubw, lbg=self.lbg, ubg=self.ubg)
        if self._warm is None:
            args["x0"] = self.initial_guess(t, x)
        else:
            args.update(self._warm)

        start = time.perf_counter()
        sol = self.solver(**args)
        self.solve_times.append(time.perf_counter() - start)
        if not self.solver.stats()["success"]:
            self.failures += 1

        w = np.asarray(sol["x"]).ravel()
        self._warm = dict(x0=w, lam_x0=sol["lam_x"], lam_g0=sol["lam_g"])
        self.plan = w[NU * N : NU * N + NX * (N + 1)].reshape(N + 1, NX)
        return np.array([w[IRATE], p.m * w[IFX]])
