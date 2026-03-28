from ROBOTS.manipulators.joints import JointTypes, Joint, LinkBodyAssumptions
from utils import process_joint_string, validate_joint_string
from sympy import Matrix, simplify, Rational, diff, symbols, trigsimp
from typing import List, Union
from ROBOTS.robot import Robot

class Manipulator(Robot):
    #TO DO : check len assumption if matches joint sequence after processing
    #TO DO : Expand the inheritance of Robot and constructor
    #TO DO : Implement Verbose debugs for calculations, maybe as a list, or maybe into setDHParameters
    #TO DO : Implement Dynamic Parameters for the whole system obtaining
    #   using expand() then make_args (add or not), using as_indipendent(params) (params to define or detect)
    #   then a loop where FOR THE SAME TERM we add term that multiply same kinematic portion
    #TO DO : Implement obtain inertia matrix without derivation?
    #TO DO : Singularities + COnstraints both for Reachable Workspace and for Dextrous Workspace
    #TO DO : Implement Compatibility with URDF file types
    #TO DO : Implement Inverse Kinematics both Numerical (Easy) but  mostly analytical (find a way)

    #TO DO : Trajectory Planning for manipulator

    #TO DO : Kinematic Control
    #TO DO : Dynamic Control
    #TO DO : Adaptive Control

    #TO DO : Formally check Coriolis and Matrices with textbooks

    #TO DO : Readibility of outoputs
    #TO DO : Efficiency: Propagated Jacobian Method, Screw Theory of Lie algebra
    #TO DO : Symbolic Recursive Newthon-Euler


    def __init__(self, joint_sequence: str, assumptions: Union[LinkBodyAssumptions, List[LinkBodyAssumptions]] = LinkBodyAssumptions.GENERAL, verbose: bool = False):
        super().__init__(verbose_calc=verbose)  
        if not validate_joint_string(joint_sequence):
            raise ValueError(f"Invalid joint string: {joint_sequence}")
        

        
        self.jointsequence=process_joint_string(joint_sequence)
        self.n_joints=len(self.jointsequence)
        
        if isinstance(assumptions, LinkBodyAssumptions):
            assumptions = [assumptions] * self.n_joints

        self.jointlist=[]

        for i in range(0,self.n_joints):
            if self.jointsequence[i]=='R':
                self.jointlist.append(Joint(JointTypes.REVOLUTE,i+1,assumptions[i]))
            elif self.jointsequence[i]=='P':
                self.jointlist.append(Joint(JointTypes.PRISMATIC,i+1,assumptions[i]))
            else:
                print("Error occurred while constructing joints for the robot")

        self.FKlist=self.ForwardKinematics()
        self.T=self.getKineticEnergy()
        self.M=self.getInertiaMatrix() #NOT SIMPLIFIED
        self.c=self.getCoriolisMatrix()
        self.G=self.getGravityVector() #NOT SIMPLIFIED
        self.J = self.getGeometricJacobian()

    def ForwardKinematics(self, upto=None):
        if upto==None:
            upto=self.n_joints
        print("Calculating Forward Kinematics: ",end="")
        FKlist=[]
        FK=Matrix.eye(4)
        for i in range(0, upto):
            FK=FK*self.jointlist[i].getDHTransform()
            FKlist.append(FK)
        print("Complete")
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

        Jv = simplify(Matrix.hstack(*Traslational_Jacobian))
        Jw = simplify(Matrix.hstack(*Rotational_Jacobian))
        Jacobian = Jv.col_join(Jw) 
        return Jacobian, Jv, Jw #RETURNS FULL JACOBIAN, LINEAR, ANGULAR
    
    def getKineticEnergy(self):
        print("Calculating Kinetic Energy with Moving Frames: ",end="")
        T, _, _, _ = self.MovingFrames()
        print("Complete")
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
            qdot=self.jointlist[i].q_dot
            
            if i==0:
                omega=Matrix([0, 0, 0])
                linvel=Matrix([0, 0, 0])
            else:
                omega=Angular_Velocity[i-1]
                linvel=Linear_Velocity[i-1]
            
            
            z=Matrix([0, 0, 1])   #cause moving frames is z vector of prev frame wrt Reference Frame of previous frame
            r=Rot.T*self.jointlist[i].getDHTransform()[:3,3]
            r_com = Matrix([-(self.jointlist[i].a - self.jointlist[i].distance_CoM), 0, 0])  #CYLINDRICAL BODY ASSUMPTION?

            Angular_Velocity.append(Rot.T*(omega+(1-sigma)*qdot*z))
            Linear_Velocity.append(Rot.T*(linvel+sigma*qdot*z)+Angular_Velocity[i].cross(r))
            Traslational_Velocity.append(Linear_Velocity[i]+Angular_Velocity[i].cross(r_com))

            I = self.jointlist[i].I
            m=self.jointlist[i].m
            Link_KE.append(Rational(1,2) * m * Traslational_Velocity[i].dot(Traslational_Velocity[i]) + Rational(1,2) * (Angular_Velocity[i].T * I * Angular_Velocity[i])[0])
        return Link_KE,Traslational_Velocity,Linear_Velocity,Angular_Velocity
    
    def getInertiaMatrix(self):
        print("Calculating Inertia Matrix: ",end="")
        qdots = [self.jointlist[i].q_dot for i in range(self.n_joints)]
        M = []
        for i in range(self.n_joints):
            row = []
            for j in range(self.n_joints):
                row.append(diff(diff(self.T, qdots[i]), qdots[j]))
            M.append(row)
        print("Complete")
        return (Matrix(M))    #Previously was (2 * Matrix(M))

    def getCoriolisMatrix(self):
        print("Calculating Coriolis Matrix: ",end="")
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
        print("Complete")
        return Matrix(Coriolis)

    def getGravityVector(self):
        print("Calculating Gravity Vector: ",end="")
        q = [self.jointlist[i].q for i in range(self.n_joints)]
        U=[]
        G=[]
        g=symbols("g")
        base_gravity_vector=Matrix([0,0,-g])
        for i in range(0, self.n_joints):
            pos_joint_i=self.FKlist[i][:3,3]
            rot_joint_i=self.FKlist[i][:3,:3]
            dc_vector = Matrix([self.jointlist[i].distance_CoM, 0, 0])
            pos_com_i=pos_joint_i+rot_joint_i*dc_vector
            U.append((-self.jointlist[i].m*base_gravity_vector.T*pos_com_i)[0])
        U=sum(U)
        
        for joint in q:
            G.append(diff(U,joint))
        print("Complete")
        return Matrix(G)
    
    def setDHParameters(self,param_list: List,index_list: List,value_list :List):
        #VALIDATE
        allowed = {"a", "alpha", "d", "theta"}
        for e in param_list:
            if e not in allowed:
                raise ValueError(f"{e} non è un parametro DH valido")

        for i in range(0, len(param_list)):
            joint=self.jointlist[index_list[i]]
            setattr(joint,param_list[i],value_list[i])
        
        #Recalculate the robot component
        self.FKlist = self.ForwardKinematics()
        self.T      = self.getKineticEnergy()
        self.M      = self.getInertiaMatrix()
        self.c      = self.getCoriolisMatrix()
        self.G      = self.getGravityVector()
        self.J      = self.getGeometricJacobian()
    
    def getDynamicCoefficients(self): #WIP
        L=[]
        return L

    def getReachableWorkspace(self):
        Jv=self.J[:3,:]
        pass
    
    def getDextrousWorkspace(self):
        pass
