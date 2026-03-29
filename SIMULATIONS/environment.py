from ROBOTS.amr.kinematic_model import KinematicModel, KinematicPreset, Pfaffian_from_constraints
from sympy import symbols, Matrix, cos, pprint
from SIMULATIONS.simulator import KinematicSimulator, StepType
from ROBOTS.amr.mobile import Mobile
import numpy as np
from typing import List, Union


class Environment:
    def __init__(self, robotlist: Union[Mobile, List[Mobile]] = None, defaultmethod: StepType = StepType.RK4):
        self.simulators = {}    # id(robot) -> KinematicSimulator
        self.robots = []        # Robot List
        self.commands = {}      # robot -> u
        self.trajectories = {}  # robot -> list of q for logging

        if robotlist is not None:
            for robot in robotlist:
                self.add_robot(robot, defaultmethod)
        
    def add_robot(self, robot: Mobile, method: StepType = StepType.RK4):
        self.simulators[id(robot)] = KinematicSimulator(robot, method)  
        self.robots.append(robot)
        self.commands[id(robot)]     = None
        self.trajectories[id(robot)] = [robot.q.copy()]

    def setCommand(self, robot: Mobile, u: np.array):
        """Assign a control input to a robot for the next simulation step."""
        assert robot in self.robots, "Robot non registrato nell'environment"
        self.commands[id(robot)] = u

    def step(self, dt: float):
        for robot in self.robots:
            u   = self.commands[id(robot)]
            sim = self.simulators[id(robot)]   
            sim.step(robot, u, dt)


    def run(self, steps: int, dt: float):
        """Run the simulation for a given number of steps."""
        for step in range(steps):
            self.step(dt)
            for robot in self.robots:
                self.trajectories[id(robot)].append(robot.q.copy())

    def get_trajectory(self, robot: Mobile):
        """Get the trajectory of a robot as a numpy array."""
        assert robot in self.robots, "Robot non registrato nell'environment"
        return np.array(self.trajectories[id(robot)])
    
    def get_all_trajectories(self, robot: Mobile):
        """Get the trajectories of all robots as a dictionary."""
        return {robot: np.array(traj) for robot, traj in self.trajectories.items()}