"""Vehicle parameters."""

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParams:
    """Single-track model parameters.

    Defaults model CARLA 0.9.15's `vehicle.tesla.model3`. Each value is tagged
    with where it comes from; see docs/vehicle_parameters.md for the derivations.
      [carla]   read from the blueprint's physics control
      [derived] computed from [carla] values
      [assumed] not exposed by CARLA; estimate to be confirmed by a test
    Cornering stiffnesses are per axle (both tires).
    """

    m: float = 1845.0  # mass [kg]  [carla]
    Izz: float = 3958.0  # yaw inertia [kg m^2]  [assumed] m*a*b
    a: float = 1.168  # CoG to front axle [m]  [derived]
    b: float = 1.836  # CoG to rear axle [m]  [derived]
    h: float = 0.45  # CoG height [m]  [assumed]
    C_alpha_f: float = 106.5e3  # front axle cornering stiffness [N/rad]  [derived]
    C_alpha_r: float = 67.8e3  # rear axle cornering stiffness [N/rad]  [derived]
    mu: float = 0.9  # tire-road friction coefficient [-]  [assumed], scenario variable
    drive_front: float = 0.5  # share of drive force on the front axle [-]  [assumed] 4WD
    brake_front: float = 0.5  # share of brake force on the front axle [-]  [derived]
    F_drive_max: float = 18073.0  # peak drive force at the wheels [N]  [derived]
    F_brake_max: float = 7568.0  # peak brake force at the wheels [N]  [derived]
    P_max: float = 700e3  # engine power [W]  [derived]
    CdA: float = 0.483  # drag coefficient times frontal area [m^2]  [derived]
    Crr: float = 0.012  # rolling resistance coefficient [-]  [assumed]
    rho: float = 1.225  # air density [kg/m^3]
    delta_max: float = 0.855  # road-wheel steering angle limit above 120 km/h [rad]  [derived]
    delta_rate_max: float = 0.6  # road-wheel steering rate limit [rad/s], design choice
    Ux_min: float = 1.0  # slip-angle denominator guard [m/s]; model is meant for Ux > 3
    g: float = 9.81  # gravity [m/s^2]

    @property
    def L(self) -> float:
        """Wheelbase [m]."""
        return self.a + self.b

    @property
    def understeer_gradient(self) -> float:
        """Linear-range understeer gradient [rad per m/s^2]."""
        Wf = self.m * self.g * self.b / self.L
        Wr = self.m * self.g * self.a / self.L
        return (Wf / self.C_alpha_f - Wr / self.C_alpha_r) / self.g
