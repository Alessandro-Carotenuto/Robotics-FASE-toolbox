from joints import JointTypes, Joint
from utils import process_joint_string, validate_joint_string
from sympy import Matrix, simplify, Rational, diff

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
        self.T=self.getKineticEnergy()
        self.M=self.getInertiaMatrix()
        self.c=0

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
    
    def getKineticEnergy(self):
        T, _, _, _ = self.MovingFrames()
        return sum(T)
    
    def MovingFrames(self):
        Linear_Velocity=[]
        Angular_Velocity=[]
        Traslational_Velocity=[]
        Link_KE=[]

        for i in range(0,self.n_joints):
            if self.jointlist[i].type==JointTypes.REVOLUTE:
                sigma=0
            elif self.jointlist[i].type==JointTypes.PRISMATIC:
                sigma=1
            else:
                print("Error occurred while calculating the Kinetic Energy trough Moving Frames Algorithm)")
            
            Rot=self.jointlist[i].getDHTransform()[:3,:3] 
            qdot=self.jointlist[i].qdot
            
            if i==0:
                omega=Matrix([0, 0, 0])
                linvel=Matrix([0, 0, 0])
            else:
                omega=Angular_Velocity[i-1]
                linvel=Linear_Velocity[i-1]
            
            
            z=Matrix([0, 0, 1])   #cause moving frames is z vector of prev frame wrt Reference Frame of previous frame
            r=Rot.T*self.jointlist[i].getDHTransform()[:3,3]
            r_com = Matrix([0, 0, self.jointlist[i].distance_CoM])

            Angular_Velocity.append(Rot.T*(omega+(1-sigma)*qdot*z))
            Linear_Velocity.append(Rot.T*(linvel+sigma*qdot*z)+Angular_Velocity[i].cross(r))
            Traslational_Velocity.append(Linear_Velocity[i]+Angular_Velocity[i].cross(r_com))

            I = self.jointlist[i].I
            m=self.jointlist[i].m
            Link_KE.append(Rational(1,2) * m * Traslational_Velocity[i].dot(Traslational_Velocity[i]) + Rational(1,2) * (Angular_Velocity[i].T * I * Angular_Velocity[i])[0])
        return Link_KE,Traslational_Velocity,Linear_Velocity,Angular_Velocity
    
    def getInertiaMatrix(self):
        qdots = [self.jointlist[i].q_dot for i in range(self.n_joints)]
        M = []
        for i in range(self.n_joints):
            row = []
            for j in range(self.n_joints):
                row.append(diff(diff(self.T, qdots[i]), qdots[j]))
            M.append(row)
        return simplify(2 * Matrix(M))

    def getCoriolisMatrix(self):
        q = [self.jointlist[i].q for i in range(self.n_joints)]
        qdots = [self.jointlist[i].q_dot for i in range(self.n_joints)]
        Christoffel=[]
        Coriolis=[]
        for i in range(0, self.n_joints):
            M_column=self.M[:,i]
            Term_1=[]
            for joint in q:
                grad=diff(M_column,joint)
                Term_1.append(grad)
            Term_1=Matrix.hstack(*Term_1)
            Term_2=Term_1.T
            Christoffel.append(Rational(1,2)*(Term_1+Term_2-diff(self.M,self.jointlist[i].q)))
            qdots_m=Matrix(qdots)
            Coriolis.append((qdots_m.T*Christoffel[i]*qdots_m)[0])
        
        return Matrix.vstack(*Coriolis)

