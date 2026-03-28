import sympy as sp
from sympy import pi, symbols, pprint, sin, cos
from utils import expression_to_sympy, Pfaffian_from_constraints, KinematicModelFromConstraints

l, theta, phi = symbols('l theta phi')
x_f_dot, y_f_dot, theta_dot = symbols('x_f_dot y_f_dot theta_dot')

Constraint_1 = expression_to_sympy("x_dot*sin(theta)-y_dot*cos(theta)")
Constraint_1 = Constraint_1.subs(symbols('x_dot'), x_f_dot + l*sin(theta)*theta_dot)
Constraint_1 = Constraint_1.subs(symbols('y_dot'), y_f_dot - l*cos(theta)*theta_dot)

Constraint_2 = expression_to_sympy("x_f_dot*sin(phi+theta)-y_f_dot*cos(theta+phi)")

Constraint = [Constraint_1, Constraint_2]
Coordinates = ['x_f', 'y_f', 'theta', 'phi']

A = KinematicModelFromConstraints(Constraint, Coordinates)
