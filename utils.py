from sympy import cos, sin, Matrix, symbols, Matrix, Symbol, pprint, shape, diff
from sympy.parsing.sympy_parser import parse_expr
import sympy as sp
from typing import List


def validate_joint_string(s: str) -> bool:
    assert isinstance(s, str), "Joint string must be a string"
    assert len(s) > 0, "Joint string cannot be empty"

    
    i = 0
    while i < len(s):
        if s[i].isdigit():
            while i < len(s) and s[i].isdigit():
                i += 1
            assert i < len(s) and s[i] in ('R', 'P'), \
                f"Number must be followed by R or P at position {i}"
        elif s[i] in ('R', 'P'):
            i += 1
        else:
            assert False, f"Invalid character '{s[i]}' at position {i}"
    
    return True

def process_joint_string(s: str) -> str:
    result = ""
    i = 0
    while i < len(s):
        if s[i].isdigit():
            num = ""
            while i < len(s) and s[i].isdigit():
                num += s[i]
                i += 1
            result += s[i] * int(num)
            i += 1
        else:
            result += s[i]
            i += 1
    return result

def rot_x(theta):
    """Rotation matrix around the X axis."""
    return Matrix([
        [1, 0,          0         ],
        [0, cos(theta), -sin(theta)],
        [0, sin(theta), cos(theta) ]
    ])

def rot_y(theta):
    """Rotation matrix around the Y axis."""
    return Matrix([
        [cos(theta),  0, sin(theta)],
        [0,           1, 0         ],
        [-sin(theta), 0, cos(theta)]
    ])

def rot_z(theta):
    """Rotation matrix around the Z axis."""
    return Matrix([
        [cos(theta), -sin(theta), 0],
        [sin(theta), cos(theta),  0],
        [0,          0,           1]
    ])

def skew(v):
    """
    Returns the skew-symmetric matrix S such that skew(v) * w = v x w.
    Input v can be a list, tuple, or sympy Matrix with 3 elements.
    """
    return Matrix([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ]
    ])

def expression_to_sympy(s: str):
    """
    Parses a mathematical expression string into a SymPy expression.
    Symbols are created automatically from the string.
    Velocities should be written in _dot notation (e.g. x_dot, theta_dot).

    Usage:
        expr = expression_to_sympy("x_dot*sin(theta) - y_dot*cos(theta)")
    """
    return parse_expr(s, transformations='all')

def Pfaffian_from_constraints(constraints: List, coords: List[str]):
    """
    Builds the Pfaffian constraint matrix A^T(q) from a list of
    holonomic/non-holonomic constraints linear in q_dot.
    
    Each constraint must be a SymPy expression equal to zero,
    with velocities in _dot notation (e.g. x_dot, theta_dot).

    Args:
        constraints : list of SymPy expressions (each = 0)
        coords      : list of coordinate names as strings
                      e.g. ['x_f', 'y_f', 'theta', 'phi']

    Returns:
        A^T(q) : SymPy Matrix of shape (n_constraints x n_coords)
                 such that A^T(q) * q_dot = 0

    Usage:
        C1 = expression_to_sympy("x_dot*sin(theta) - y_dot*cos(theta)")
        C2 = expression_to_sympy("x_f_dot*sin(phi+theta) - y_f_dot*cos(theta+phi)")
        A  = Pfaffian_from_constraints([C1, C2], ['x_f', 'y_f', 'theta', 'phi'])
    """

    q_dots = [symbols(c + '_dot') for c in coords]
    rows = []
    for constraint in constraints:
        constraint = constraint.expand()  # <-- add this
        row = [constraint.coeff(qdot) for qdot in q_dots]
        rows.append(row)
    return Matrix(rows), q_dots

def LieBracket2(v1: Matrix, v2: Matrix, coord: List):
    assert shape(v1)[1]==1, f"Vector {v1} should be a column-vector"
    assert shape(v2)[1]==1, f"Vector {v2} should be a column-vector"

    #[g1,g2] = dg2q * g1 - dg1q * g2

    g2_wrt_q=[]
    g1_wrt_q=[]
    for q in coord:
        g1_wrt_q.append(diff(v1,q))
        g2_wrt_q.append(diff(v2,q))
    
    g1_wrt_q=Matrix.hstack(*g1_wrt_q)
    g2_wrt_q=Matrix.hstack(*g2_wrt_q)

    return g2_wrt_q*v1 - g1_wrt_q*v2

