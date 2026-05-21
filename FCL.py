from sympy import Matrix, symbols
from utils import expression_to_sympy
from typing import Optional


class FCL:
    @staticmethod
    def Pcontrol(coord: str, coord_desired: str, coord_desired_dot: Optional[str] = None, Kp: Optional[float] = None) -> Matrix:
        if Kp is None:
            Kp = symbols('Kp', positive=True)
        coord_sym   = symbols(coord)
        desired_sym = expression_to_sympy(coord_desired)
        if coord_desired_dot is not None:
            desired_dot_sym = expression_to_sympy(coord_desired_dot)
            return Kp * (desired_sym - coord_sym) + desired_dot_sym
        return Kp * (desired_sym - coord_sym)
    
    
