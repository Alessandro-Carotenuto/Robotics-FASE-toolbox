from ROBOTS.robot import Robot
from ROBOTS.amr.kinematic_model import KinematicModel
import numpy as np

#TO DO : Feedback Control

#TO DO : Path Planning -> Artificial potential fields
#TO DO : RRT & RRT*

#TO DO : Trajectory Tracking

#TO DO : Kalman Filter, SLAM

class Mobile(Robot):
    def __init__(self, kinematic_model: KinematicModel):
        super().__init__() 
        self.Kinematic_Model=kinematic_model
        self.q = np.zeros(len(kinematic_model.coords))  # [0.0, ..., 0.0]
        self.physical_parameters = {}