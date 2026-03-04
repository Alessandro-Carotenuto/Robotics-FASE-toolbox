from enum import Enum
from sympy import symbols, cos, sin, Matrix

class JointTypes(Enum):
    PRISMATIC=1
    REVOLUTE=2

class Joint():

    def __init__(self, type: JointTypes, index: int):
        self.type=type
        self.idx=index

        self.alpha=symbols(f'alpha_{self.idx}')
        self.a=symbols(f'a_{self.idx}')
        if self.type==JointTypes.PRISMATIC:
            self.d=symbols(f'q_{self.idx}')
            self.theta=symbols(f'theta_{self.idx}')
            self.q=self.d
        else:
            self.d=symbols(f'd_{self.idx}')
            self.theta=symbols(f'q_{self.idx}')
            self.q=self.theta

        self.distance_CoM=symbols(f'dc_{self.idx}')

    def getDHTransform(self):
        a=self.a
        alpha=self.alpha
        d=self.d
        theta=self.theta
        return Matrix([
            [cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta)],
            [sin(theta),  cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
            [0,           sin(alpha),              cos(alpha),            d           ],
            [0,           0,                       0,                     1           ]
        ])