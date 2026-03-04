from joints import JointTypes, Joint
from utils import process_joint_string, validate_joint_string
from sympy import Matrix

class Robot():

    def __init__(self, joint_sequence: str):
        if not validate_joint_string(joint_sequence):
            raise ValueError(f"Invalid joint string: {joint_sequence}")
        
        self.jointsequence=process_joint_string(joint_sequence)
        self.n_joints=len(self.jointsequence)
        self.jointlist=[]

        for i in range(0,self.n_joints):
            if self.jointsequence[i]=='R':
                self.jointlist.append(Joint(JointTypes.REVOLUTE,i+1))
            elif self.jointsequence[i]=='P':
                self.jointlist.append(Joint(JointTypes.PRISMATIC,i+1))
            else:
                print("Error occurred while constructing joints for the robot")

        pass

    def ForwardKinematics(self, upto=None):
        if upto==None:
            upto=self.n_joints

        FK=Matrix.eye(4)
        for i in range(0, upto):
            FK=FK*self.jointlist[i].getDHTransform()
        return FK


