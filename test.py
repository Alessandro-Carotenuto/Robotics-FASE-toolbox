from robot import Robot


r = Robot("2R")
from sympy import symbols

l1, l2 = symbols('l1 l2', positive=True)
r.jointlist[0].a = l1
r.jointlist[0].alpha = 0
r.jointlist[0].d = 0
r.jointlist[1].a = l2
r.jointlist[1].alpha = 0
r.jointlist[1].d = 0

# Ricalcola FK con i nuovi parametri
r.FKlist = r.ForwardKinematics()

J = r.getGeometricJacobian()
print(J)