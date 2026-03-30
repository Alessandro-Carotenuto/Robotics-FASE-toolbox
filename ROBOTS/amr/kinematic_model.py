import sympy as sp
from sympy import Matrix, Symbol, symbols, sin, cos, tan, pprint
from typing import List, Optional
from enum import Enum
from utils import Pfaffian_from_constraints, LieBracket2
import numpy as np


class KinematicPreset(Enum):
    UNICYCLE     = 1
    BICYCLE_RWD  = 2
    BICYCLE_FWD  = 3
    CAR_WITH_TRAILER = 4

class KinematicModel():
    """
    Derives and represents the kinematic model of a mobile robot: q_dot = G(q) * u

    Can be instantiated from explicit Pfaffian constraints or from a known preset.

    Attributes:
        G                   : Matrix       — input/distribution matrix G(q),  shape (n x m)
        velocity_expression : Matrix       — symbolic q_dot = G(q)*u,         shape (n x 1)
        velocity_symbols    : List[Symbol] — velocity symbols [x_dot, y_dot, ...], length n
        coords              : List[str]    — coordinate names ['x', 'y', ...],     length n
        constraint_matrix   : Matrix       — Pfaffian matrix A^T(q),          shape (k x n)  [None for presets]
        velocity_map        : dict         — {coord: velocity_expression[i]}

    Usage (from constraints):
        model = KinematicModel(constraints=[C1, C2], coords=['x_f', 'y_f', 'theta', 'phi'])

    Usage (from preset):
        model = KinematicModel(preset=KinematicPreset.UNICYCLE)
        model = KinematicModel(preset=KinematicPreset.BICYCLE_RWD)
    """

    def __init__(
        self,
        constraints: Optional[List]           = None,
        coords:      Optional[List[str]]       = None,
        preset:      Optional[KinematicPreset] = None,
        display:     bool = True,
    ):
        self.constraint_matrix = None
        self.extra_points = None
        self.null_vecs = None

        if preset is not None:
            self._load_preset(preset)
        elif constraints is not None and coords is not None:
            self.coords = coords
            self.constraint_matrix, self.velocity_symbols = Pfaffian_from_constraints(constraints, coords)
            self.constraint_matrix = sp.trigsimp(self.constraint_matrix)

            null_vecs = self.constraint_matrix.nullspace()
            self.null_vecs = [sp.trigsimp(v) for v in null_vecs]

            self.n_inputs = len(null_vecs)
            u        = [symbols(f'u_{i+1}') for i in range(n_inputs)]
            self.G   = Matrix.hstack(*null_vecs)
            self.velocity_expression = sp.trigsimp(self.G * Matrix(u))
        else:
            raise ValueError("Provide either 'preset' or both 'constraints' and 'coords'.")

        self.velocity_map = {
            coord: self.velocity_expression[i]
            for i, coord in enumerate(self.coords)
        }

        if display:
            print("Kinematic Model:  q_dot = G(q) * u\n")
            pprint(sp.Eq(Matrix(self.velocity_symbols), self.velocity_expression))

    def _load_preset(self, preset: KinematicPreset):
        """Assigns directly the known kinematic model from literature."""

        match preset:
            case KinematicPreset.UNICYCLE:
                # q = [x, y, theta],  u = [v, omega]
                # x_dot     = v * cos(theta)
                # y_dot     = v * sin(theta)
                # theta_dot = omega
                theta = symbols('theta')
                u1, u2 = symbols('u_1 u_2')
                self.coords           = ['x', 'y', 'theta']
                self.velocity_symbols = list(symbols('x_dot y_dot theta_dot'))
                self.G = Matrix([
                    [cos(theta), 0],
                    [sin(theta), 0],
                    [0,          1],
                ])
                self.velocity_expression = self.G * Matrix([u1, u2])

            case KinematicPreset.BICYCLE_RWD:
                # q = [x, y, theta, phi],  u = [v, phi_dot]
                # Reference point: rear axle
                # x_dot     = v * cos(theta)
                # y_dot     = v * sin(theta)
                # theta_dot = v/l * tan(phi)
                # phi_dot   = phi_dot
                theta, phi, l = symbols('theta phi l')
                u1, u2 = symbols('u_1 u_2')
                self.coords           = ['x', 'y', 'theta', 'phi']
                self.velocity_symbols = list(symbols('x_dot y_dot theta_dot phi_dot'))
                self.G = Matrix([
                    [cos(theta),    0],
                    [sin(theta),    0],
                    [tan(phi) / l,  0],
                    [0,             1],
                ])
                self.velocity_expression = self.G * Matrix([u1, u2])

            case KinematicPreset.BICYCLE_FWD:
                # q = [x_f, y_f, theta, phi],  u = [v_f, phi_dot]
                # Reference point: front axle
                # x_f_dot   = v_f * cos(theta + phi)  -- NOT cos(phi)*cos(theta)... check literature
                # y_f_dot   = v_f * sin(theta + phi)
                # theta_dot = v_f/l * sin(phi)
                # phi_dot   = phi_dot
                theta, phi, l = symbols('theta phi l')
                u1, u2 = symbols('u_1 u_2')
                self.coords           = ['x_f', 'y_f', 'theta', 'phi']
                self.velocity_symbols = list(symbols('x_f_dot y_f_dot theta_dot phi_dot'))
                self.G = Matrix([
                    [cos(theta + phi), 0],
                    [sin(theta + phi), 0],
                    [sin(phi) / l,     0],
                    [0,                1],
                ])
                self.velocity_expression = self.G * Matrix([u1, u2])
            
            case KinematicPreset.CAR_WITH_TRAILER:
                # q = [x, y, theta, phi, beta],  u = [v, phi_dot]
                # beta = angolo di articolazione car-trailer (0 = allineati)
                # l = passo del car,  L = lunghezza del trailer (hitch -> asse)
                # x_dot     = v * cos(theta)
                # y_dot     = v * sin(theta)
                # theta_dot = v/l * tan(phi)
                # phi_dot   = phi_dot
                # beta_dot  = v/l * tan(phi) - v/L * sin(beta)

                self.extra_points = lambda q, params: [
                    (q[0] - params[symbols('L')] * np.cos(q[2] - q[4]),
                    q[1] - params[symbols('L')] * np.sin(q[2] - q[4]))
                ]



                theta, phi, beta, l, L = symbols('theta phi beta l L')
                u1, u2 = symbols('u_1 u_2')
                self.coords           = ['x', 'y', 'theta', 'phi', 'beta']
                self.velocity_symbols = list(symbols('x_dot y_dot theta_dot phi_dot beta_dot'))
                self.G = Matrix([
                    [cos(theta),                    0],
                    [sin(theta),                    0],
                    [tan(phi) / l,                  0],
                    [0,                             1],
                    [tan(phi)/l - sin(beta)/L,      0],
                ])
                self.velocity_expression = self.G * Matrix([u1, u2])

    def ControllabilityAnalysis(self):
        """self.null_vecs, self.n_inputs, self.G"""

        if self.null_vecs is None:
            self.null_vecs = [self.G[:, i] for i in range(self.G.shape[1])]
            self.n_inputs  = self.G.shape[1]
        q_syms = [symbols(c) for c in self.coords]

        DoF=len(self.coords)
        rank=self.G.rank()
        ControlMatrix=self.G.copy()
        g=self.null_vecs
        
        oneshottest=True
        while (oneshottest): #rank<DoF
            if self.n_inputs<3:
                print("calc.")
                ControlMatrix=Matrix.hstack(ControlMatrix,LieBracket2(g[0],g[1],q_syms))
                rank=ControlMatrix.rank()
                oneshottest=False
            else:
                raise NotImplementedError("Controllability analysis for more than 2 inputs not implemented yet.")
        
        pprint(ControlMatrix)
        print(f"Control matrix rank: {rank}")
        return True
        

        
