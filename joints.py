from enum import Enum

class JointTypes(Enum):
    PRISMATIC=1
    REVOLUTE=2

class Joint():

    def __init__(self, type: JointTypes):
        self.type=type
        pass