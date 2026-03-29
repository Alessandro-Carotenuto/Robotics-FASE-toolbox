from ROBOTS.robot import Robot
from ROBOTS.amr.kinematic_model import KinematicModel
import numpy as np


class Mobile(Robot):
    def __init__(self, kinematic_model: KinematicModel, physical_parameters: dict = None):
        super().__init__()
        self.Kinematic_Model = kinematic_model
        self.q = np.zeros(len(kinematic_model.coords))
        self.physical_parameters = physical_parameters or {}
    
    def get_extra_points(self):
        if self.Kinematic_Model.extra_points is None:
            return None
        return self.Kinematic_Model.extra_points(self.q, self.physical_parameters)