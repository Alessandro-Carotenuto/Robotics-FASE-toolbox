from sympy import cos, sin, Matrix, symbols, Matrix, pprint
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

def KinematicModelFromConstraints(constraints: List, coords: List[str], display=True):
    """
    Derives the kinematic model q_dot = G(q) * u from a set of Pfaffian constraints.

    The function builds the Pfaffian constraint matrix A^T(q), computes its null space
    to obtain the input matrix G(q), and returns the kinematic model.

    Args:
        constraints : list of SymPy expressions (each = 0)
                      Constraints linear in q_dot, written using _dot notation.
                      Can be pre-processed (e.g. substituting x_dot, y_dot in terms
                      of front-frame coordinates) before being passed in.
        coords      : list of coordinate names as strings
                      e.g. ['x_f', 'y_f', 'theta', 'phi']
        display     : if True, prints the kinematic model q_dot = G(q) * u

    Returns:
        G     : SymPy Matrix (n x m) — the input/distribution matrix
        q_dot : SymPy Matrix (n x 1) — symbolic expression of q_dot = G(q)*u
        q_dots: list of SymPy symbols — the coordinate velocity symbols [x_f_dot, ...]

    Usage:
        l, theta, phi = symbols('l theta phi')
        x_f_dot, y_f_dot, theta_dot = symbols('x_f_dot y_f_dot theta_dot')

        # Rear wheel pure rolling, substituting x_dot, y_dot via rigid body relation
        C1 = expression_to_sympy("x_dot*sin(theta)-y_dot*cos(theta)")
        C1 = C1.subs(symbols('x_dot'), x_f_dot + l*sin(theta)*theta_dot)
        C1 = C1.subs(symbols('y_dot'), y_f_dot - l*cos(theta)*theta_dot)

        # Front wheel pure rolling
        C2 = expression_to_sympy("x_f_dot*sin(phi+theta)-y_f_dot*cos(theta+phi)")

        G, q_dot, q_dots = KinematicModelFromConstraints([C1, C2], ['x_f', 'y_f', 'theta', 'phi'])
    """
        
    A, q_dots= Pfaffian_from_constraints(constraints, coords)
    A = sp.trigsimp(A)

    null_vecs = A.nullspace()
    null_vecs = [sp.trigsimp(v) for v in null_vecs]
    n_inputs=len(null_vecs)
    u = [symbols(f'u_{i+1}') for i in range(n_inputs)]
    G = sp.Matrix.hstack(*null_vecs)
    u_vec = sp.Matrix(u)
    q_dot = sp.trigsimp(G * u_vec)


    if display:
        print("Kinematic Model:  q_dot = G(q) * u\n")
        pprint(sp.Eq(sp.Matrix(q_dots), q_dot))
    return G, q_dot, q_dots