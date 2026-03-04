from joints import JointTypes, Joint
from utils import process_joint_string, validate_joint_string

class Robot():

    def __init__(self, joint_sequence: str):
        if not validate_joint_string(joint_sequence):
            raise ValueError(f"Invalid joint string: {joint_sequence}")
        
        self.jointsequence=process_joint_string(joint_sequence)
        self.n_joints=len(self.joint_sequence)
        self.jointlist=[]
        
        for char in joint_sequence:
            if char=='R':
                self.jointlist.append(Joint(JointTypes.REVOLUTE))
            elif char=='P':
                self.jointlist.append(Joint(JointTypes.PRISMATIC))
            else:
                print("Error occurred while constructing joints for the robot")


        pass


