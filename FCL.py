import sympy as sp
from sympy import symbols, Expr
from utils import expression_to_sympy
from typing import Optional, Union


def _to_expr(x: Union[str, Expr]) -> Expr:
    return x if isinstance(x, sp.Expr) else expression_to_sympy(x)


class FCL:
    @staticmethod
    def Pcontrol(
        coord:             str,
        coord_desired:     Union[str, Expr],
        coord_desired_dot: Optional[Union[str, Expr]] = None,
        Kp:                Optional[float]            = None,
    ) -> Expr:
        """P control: u = Kp*(y_d - y) [+ ẏ_d feedforward]"""
        if Kp is None:
            Kp = symbols('Kp', positive=True)
        y   = symbols(coord)
        y_d = _to_expr(coord_desired)
        u   = Kp * (y_d - y)
        if coord_desired_dot is not None:
            u += _to_expr(coord_desired_dot)
        return u

    @staticmethod
    def PDcontrol(
        coord:              str,
        coord_desired:      Union[str, Expr],
        coord_dot:          Union[str, Expr],
        coord_desired_dot:  Optional[Union[str, Expr]] = None,
        coord_desired_ddot: Optional[Union[str, Expr]] = None,
        Kp:                 Optional[float]            = None,
        Kd:                 Optional[float]            = None,
    ) -> Expr:
        """PD control with optional feedforward acceleration:
        u = ÿ_d + Kp*(y_d - y) + Kd*(ẏ_d - ẏ)
        """
        if Kp is None:
            Kp = symbols('Kp', positive=True)
        if Kd is None:
            Kd = symbols('Kd', positive=True)

        y        = symbols(coord)
        y_dot    = coord_dot if isinstance(coord_dot, sp.Expr) else symbols(coord_dot)
        y_d      = _to_expr(coord_desired)
        y_d_dot  = _to_expr(coord_desired_dot)  if coord_desired_dot  is not None else sp.Integer(0)
        y_d_ddot = _to_expr(coord_desired_ddot) if coord_desired_ddot is not None else sp.Integer(0)

        return y_d_ddot + Kp * (y_d - y) + Kd * (y_d_dot - y_dot)
