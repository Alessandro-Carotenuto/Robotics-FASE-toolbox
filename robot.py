from joints import JointTypes, Joint
from utils import process_joint_string, validate_joint_string
from sympy import Matrix, simplify

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

        self.FKlist=self.ForwardKinematics()

    def ForwardKinematics(self, upto=None):
        if upto==None:
            upto=self.n_joints

        FKlist=[]
        FK=Matrix.eye(4)
        for i in range(0, upto):
            FK=FK*self.jointlist[i].getDHTransform()
            FKlist.append(FK)
        return FKlist
    
    def getGeometricJacobian(self):

        pos_ee = self.FKlist[-1][:3,3]
        Traslational_Jacobian=[]
        Rotational_Jacobian=[]

        for i in range(0,len(self.FKlist)):
            if i==0:
                z_vector_current=Matrix([0, 0, 1])          #CASO BASE
            else:
                z_vector_current=self.FKlist[i-1][:3,2]
            

            if self.jointlist[i].type==JointTypes.REVOLUTE:

                if i==0:
                    pos_vector_current=Matrix([0, 0, 0])
                else:
                    pos_vector_current=self.FKlist[i-1][:3,3]
                offset=pos_ee-pos_vector_current
                Traslational_Jacobian.append(z_vector_current.cross(offset))
                Rotational_Jacobian.append(z_vector_current)
            elif self.jointlist[i].type==JointTypes.PRISMATIC: 
                Traslational_Jacobian.append(z_vector_current)
                Rotational_Jacobian.append(Matrix([0, 0, 0]))
            else:
                print("Error occurred while calculating the Jacobian Matrix (Geometric)")

        Jv = Matrix.hstack(*Traslational_Jacobian)
        Jw = Matrix.hstack(*Rotational_Jacobian)
        Jacobian = Jv.col_join(Jw) 
        return simplify(Jacobian)



