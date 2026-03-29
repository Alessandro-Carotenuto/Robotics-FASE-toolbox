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
    def __init__(self, robot: Mobile, method: StepType = StepType.EULER):
        self.KM = robot.Kinematic_Model
        self.method = method
        self.q_syms = [symbols(c) for c in self.KM.coords]

        # Sostituisce i parametri fisici (es. l=1.5) nel G simbolico
        G_subst = self.KM.G.subs(robot.physical_parameters)

        free_syms = G_subst.free_symbols - set(self.q_syms)
        assert len(free_syms) == 0, \
            f"G contiene simboli non sostituiti: {free_syms}. " \
            f"Aggiungili a physical_parameters del robot."

        self.G_func = lambdify(self.q_syms, G_subst, modules='numpy')

    def _check_compatibility(self, robot: Mobile):
        assert robot.Kinematic_Model.coords == self.KM.coords, \
            f"Robot KM coords {robot.Kinematic_Model.coords} non compatibili con Simulator KM coords {self.KM.coords}"

    def step(self, robot: Mobile, u: np.array, dt: float):
        assert robot.q is not None, "robot.q not initialized"
        self._check_compatibility(robot)
        if self.method == StepType.EULER:
            self._euler(robot, u, dt)
        elif self.method == StepType.RK4:
            self._rk4(robot, u, dt)

    def _euler(self, robot: Mobile, u: np.array, dt: float):
        G_num = np.array(self.G_func(*robot.q), dtype=float)
        q_dot = G_num @ u
        robot.q = robot.q + q_dot * dt

    def _rk4(self, robot: Mobile, u: np.array, dt: float):
        q = robot.q
        k1 = np.array(self.G_func(*q),           dtype=float) @ u
        k2 = np.array(self.G_func(*(q + dt/2*k1)), dtype=float) @ u
        k3 = np.array(self.G_func(*(q + dt/2*k2)), dtype=float) @ u
        k4 = np.array(self.G_func(*(q + dt*k3)),   dtype=float) @ u
        robot.q = q + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

class Dynamic_Simulator(Simulator):
    def __init__(self):
        pass