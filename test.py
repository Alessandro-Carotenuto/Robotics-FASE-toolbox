import sys
from joints import LinkBodyAssumptions
from manipulator import Manipulator
from sympy import *
import matplotlib.pyplot as plt

def show_matrix_latex(M, title=""):
    n = M.shape[0]
    fig, axes = plt.subplots(n, n, figsize=(28, 8))
    for i in range(n):
        for j in range(n):
            axes[i,j].axis('off')
            expr_str = f"${latex(M[i,j])}$"
            fontsize = max(5, 10 - len(expr_str) // 50)
            axes[i,j].text(0.5, 0.5, expr_str,
                           fontsize=fontsize, ha='center', va='center',
                           transform=axes[i,j].transAxes)
    plt.suptitle(title)
    plt.tight_layout()
    plt.show()

a2, a3 = symbols('a_2 a_3', positive=True)
rob = Manipulator("3R", LinkBodyAssumptions.CYLIDNRIC)
rob.setDHParameters(
    param_list  = ['alpha', 'a', 'd',
                   'alpha', 'a', 'd',
                   'alpha', 'a', 'd'],
    index_list  = [0,  0,  0,
                   1,  1,  1,
                   2,  2,  2],
    value_list  = [pi/2, 0,  0,
                     0, a2,  0,
                     0, a3,  0]
)

n = rob.n_joints
M_simplified = Matrix.zeros(n)
for i in range(n):
    for j in range(n):
        M_simplified[i, j] = trigsimp(simplify(rob.M[i, j]))

show_matrix_latex(M_simplified, "M(q)")

print("\n\n========== M(q)*q̈ ==========")
q1d, q2d, q3d = symbols('q_ddot_1 q_ddot_2 q_ddot_3')
qdd = Matrix([q1d, q2d, q3d])
Mqdd = trigsimp(simplify(rob.M * qdd))
for i, row in enumerate(Mqdd):
    print(f"\n  [M*q̈]_{i+1} =")
    pprint(trigsimp(expand(row)))