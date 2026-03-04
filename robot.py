from joints import JointTypes, Joint
from utils import process_joint_string, validate_joint_string

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


