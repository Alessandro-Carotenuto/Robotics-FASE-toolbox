import sympy as sp
from sympy import symbols, Expr
from utils import expression_to_sympy
from typing import Optional


class FCL:
    @staticmethod
    def Pcontrol(
        coord:             str,
        coord_desired:     str,
        coord_desired_dot: Optional[str]   = None,
        Kp:                Optional[float] = None,
    ) -> Expr:
        """P control: u = Kp*(y_d - y) [+ ẏ_d feedforward]"""
        if Kp is None:
            Kp = symbols('Kp', positive=True)
        y   = symbols(coord)
        y_d = expression_to_sympy(coord_desired)
        u   = Kp * (y_d - y)
        if coord_desired_dot is not None:
            u += expression_to_sympy(coord_desired_dot)
        return u

    @staticmethod
    def PDcontrol(
        coord:              str,
        coord_desired:      str,
        coord_dot:          str,
        coord_desired_dot:  Optional[str]   = None,
        coord_desired_ddot: Optional[str]   = None,
        Kp:                 Optional[float] = None,
        Kd:                 Optional[float] = None,
    ) -> Expr:
        """PD control with optional feedforward acceleration:
        u = ÿ_d + Kp*(y_d - y) + Kd*(ẏ_d - ẏ)
        """
        if Kp is None:
            Kp = symbols('Kp', positive=True)
        if Kd is None:
            Kd = symbols('Kd', positive=True)

        y      = symbols(coord)
        y_dot  = symbols(coord_dot)
        y_d    = expression_to_sympy(coord_desired)
        y_d_dot  = expression_to_sympy(coord_desired_dot)  if coord_desired_dot  is not None else sp.Integer(0)
        y_d_ddot = expression_to_sympy(coord_desired_ddot) if coord_desired_ddot is not None else sp.Integer(0)

        return y_d_ddot + Kp * (y_d - y) + Kd * (y_d_dot - y_dot)
