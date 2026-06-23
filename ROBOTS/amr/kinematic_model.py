import sympy as sp
from sympy import Matrix, Symbol, symbols, sin, cos, tan, pprint
from typing import List, Optional
from enum import Enum
from utils import Pfaffian_from_constraints, LieBracket2
import numpy as np
from FCL import FCL


class KinematicPreset(Enum):
    UNICYCLE         = 1
    BICYCLE_RWD      = 2
    BICYCLE_FWD      = 3
    CAR_WITH_TRAILER = 4
    UNICYCLE_POLAR   = 5
    UNICYCLE_CHAINED_2_3 = 6
    GENERAL_CHAINED_2_3  = 7

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
            u        = [symbols(f'u_{i+1}') for i in range(self.n_inputs)]
            self.G   = Matrix.hstack(*null_vecs)
            self.velocity_expression = sp.trigsimp(self.G * Matrix(u))
            self.velocity_expression = Matrix([sp.factor(e) for e in self.velocity_expression])
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
            
            case KinematicPreset.UNICYCLE_POLAR:
                # q = [rho, gamma, delta],  u = [v, omega]
                # rho_dot   = -v * cos(gamma)
                # gamma_dot = (sin(gamma) / rho) * v - omega
                # delta_dot = (sin(gamma) / rho) * v
                rho, gamma = symbols('rho gamma')
                v, omega = symbols('v omega')
                self.coords           = ['rho', 'gamma', 'delta']
                self.velocity_symbols = list(symbols('rho_dot gamma_dot delta_dot'))
                self.G = Matrix([
                    [-cos(gamma),      0],
                    [sin(gamma)/rho,  -1],
                    [sin(gamma)/rho,   0],
                ])
                self.velocity_expression = self.G * Matrix([v, omega])

            case KinematicPreset.UNICYCLE_CHAINED_2_3:
                # (2,3) chained form of the unicycle — state/input transformation:
                #   z1=theta, z2=x*cos(theta)+y*sin(theta), z3=x*sin(theta)-y*cos(theta)
                #   v1=omega,  v2=v - z3*omega
                # z1_dot = v1 | z2_dot = v2 | z3_dot = z2*v1
                z2 = symbols('z_2')
                v1, v2 = symbols('v_1 v_2')
                self.coords           = ['z_1', 'z_2', 'z_3']
                self.velocity_symbols = list(symbols('z_1_dot z_2_dot z_3_dot'))
                self.G = Matrix([
                    [1,  0],
                    [0,  1],
                    [z2, 0],
                ])
                self.velocity_expression = self.G * Matrix([v1, v2])

            case KinematicPreset.GENERAL_CHAINED_2_3:
                # Abstract (2,3) chained form: z_dot = G(z)*[v1,v2]
                # z1_dot = v1 | z2_dot = v2 | z3_dot = z2*v1
                z2 = symbols('z_2')
                v1, v2 = symbols('v_1 v_2')
                self.coords           = ['z_1', 'z_2', 'z_3']
                self.velocity_symbols = list(symbols('z_1_dot z_2_dot z_3_dot'))
                self.G = Matrix([
                    [1,  0],
                    [0,  1],
                    [z2, 0],
                ])
                self.velocity_expression = self.G * Matrix([v1, v2])

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

    def addVelCoord(self, vel_name: str, input_name: str):
        """Add a new coordinate whose velocity equals a new input symbol.

        e.g. model.addVelCoord('v_dot', 'a_v') adds coord 'v' with equation v_dot = a_v
        """
        if not vel_name.endswith('_dot'):
            raise ValueError(f"vel_name must end with '_dot', got '{vel_name}'")

        coord_name = vel_name[:-4]
        new_input  = symbols(input_name)
        new_vel    = symbols(vel_name)

        n, m = self.G.shape
        self.G = Matrix.vstack(
            Matrix.hstack(self.G, sp.zeros(n, 1)),
            Matrix([[sp.Integer(0)] * m + [sp.Integer(1)]])
        )

        self.coords.append(coord_name)
        self.velocity_symbols.append(new_vel)
        self.velocity_expression = Matrix.vstack(self.velocity_expression, Matrix([new_input]))
        self.velocity_map[coord_name] = new_input

    def MotionRegulation(
            self,
            coords: List[str],
            desired_endpoints: List[float],
            control_inputs: List[str],
            k: float = 1.0
        ) -> dict:
        """Given coordinates and desired endpoints, solves for control inputs via P-control on velocity."""

        input_syms = [symbols(s) for s in control_inputs]

        equations = []
        for coord, desired in zip(coords, desired_endpoints):
            if coord not in self.coords:
                raise ValueError(f"Coordinate '{coord}' not found in model coordinates {self.coords}.")
            model_vel  = self.velocity_map[coord]
            target_vel = FCL.Pcontrol(coord, str(desired), Kp=k)
            equations.append(sp.Eq(model_vel, target_vel))

        sol = sp.solve(equations, input_syms, dict=True)
        if not sol:
            raise ValueError("System of equations has no solution for the given control inputs.")

        return {sym: sp.simplify(val) for sym, val in sol[0].items()}


    def ControllabilityAnalysis(self):
        """self.null_vecs, self.n_inputs, self.G"""

        if self.null_vecs is None:
            self.null_vecs = [self.G[:, i] for i in range(self.G.shape[1])]
            self.n_inputs  = self.G.shape[1]
        q_syms = [symbols(c) for c in self.coords]

        DoF=len(self.coords)
        rank=self.G.rank()
        ControlMatrix=self.G.copy()
        g=self.null_vecs.copy()
        
        checked={}  #Dictionary for computing the complete Lie Bracket
        lblevel=0
        while(lblevel == 0 or rank>lastlbrank):
            lastlbrank=rank
            newg=g.copy()
            for i in range(len(g)-1):
                for j in range(i+1,len(g)):
                    
                    if i not in checked:
                        checked[i]=[]
                    if j not in checked:
                        checked[j]=[]
                
                    if j not in checked[i]:
                        LB=LieBracket2(g[i],g[j],q_syms)
                        checked[i].append(j)
                        checked[j].append(i)
                        newg.append(LB)
                        ControlMatrix=Matrix.hstack(ControlMatrix,LB)
                        rank=ControlMatrix.rank()
                        if (rank>=DoF):
                            print("System is Controllable")
                            rows, cols = ControlMatrix.shape
                            ControlMatrix = Matrix(rows, cols, [sp.factor(e) for e in ControlMatrix])
                            pprint(ControlMatrix)
                            print(f"Control matrix rank: {rank}")
                            return True
                    else:
                        pass
            g=newg.copy()
            lblevel+=1
            
        print("Reached the involutive closure")
        rows, cols = ControlMatrix.shape
        ControlMatrix = Matrix(rows, cols, [sp.factor(e) for e in ControlMatrix])
        pprint(ControlMatrix)
        print(f"Control matrix rank: {rank}")
        return False
        

        
