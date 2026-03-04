from robot import Robot


# Test su robot 2R planare (alpha=0, d=0 per tutti)
from sympy import symbols, pi, simplify

r = Robot("RPP")

l1, l2 = symbols('l1 l2', positive=True)

r.jointlist[0].a = l1
r.jointlist[0].alpha = 0
r.jointlist[0].d = 0

r.jointlist[1].a = l2
r.jointlist[1].alpha = 0
r.jointlist[1].d = 0

T = r.ForwardKinematics()

print(simplify(T))