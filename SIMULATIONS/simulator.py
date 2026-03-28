from enum import Enum
import numpy as np
from sympy import symbols, lambdify
from ROBOTS.amr.kinematic_model import KinematicModel
from ROBOTS.amr.mobile import Mobile

class StepType(Enum):
    EULER = 1
    RK4 = 2

class Simulator():
    def __init__(self):
        pass

class KinematicSimulator(Simulator):
    def __init__(self, KM: KinematicModel, method: StepType = StepType.EULER):
        self.Kinematic_Model = KM
        self.method = method
        self.q_syms = [symbols(c) for c in KM.coords]
        self.G_func = lambdify(self.q_syms, KM.G, modules='numpy')

    def _check_compatibility(self, robot: Mobile):
        assert robot.KM.coords == self.KM.coords, \
            f"Robot KM coords {robot.KM.coords} non compatibili con Simulator KM coords {self.KM.coords}"

    def step(self, robot: Mobile, u: np.array, dt: float):
        self._check_compatibility(robot)
        if self.method == StepType.EULER:
            self._euler(robot, u, dt)
        elif self.method == StepType.RK4:
            self._rk4(robot, u, dt)

    def _euler(self, robot: Mobile, u: np.array, dt: float):
        G_num = self.G_func(*robot.q)
        q_dot = G_num @ u
        robot.q = robot.q + q_dot * dt

    def _rk4(self, robot: Mobile, u: np.array, dt: float):
        q = robot.q
        k1 = self.G_func(*q)             @ u
        k2 = self.G_func(*(q + dt/2*k1)) @ u
        k3 = self.G_func(*(q + dt/2*k2)) @ u
        k4 = self.G_func(*(q + dt*k3))   @ u
        robot.q = q + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

class Dynamic_Simulator(Simulator):
    def __init__(self):
        pass