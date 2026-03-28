from ROBOTS.robot import Robot
from ROBOTS.amr.kinematic_model import KinematicModel
#TO DO : Feedback Control

#TO DO : Path Planning -> Artificial potential fields
#TO DO : RRT & RRT*

#TO DO : Trajectory Tracking

#TO DO : Kalman Filter, SLAM

class Mobile(Robot):
    def __init__(self, kinematic_model: KinematicModel):
        super().__init__() 
        self.Kinematic_Model=kinematic_model