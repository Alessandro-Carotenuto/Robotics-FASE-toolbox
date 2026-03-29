from SIMULATIONS.environment import Environment
import numpy as np
import matplotlib.pyplot as plt

#TO DO: OPTIMIZE ANIMATION WITH SET DATA 


class Displayer():
    def __init__(self,ENV: Environment):
        self.env=ENV
        self.trajectories=ENV.trajectories
        self.colorlist=['r', 'g', 'b', 'c', 'm', 'y', 'k', 'w']
    
    def display(self, Title="Simulation Plot",xlabel="X",ylabel="Y"):

        fig, ax = plt.subplots(figsize=(6, 6))
        i=0
        for robot in self.env.robots:
            currentcolor=self.colorlist[i]
            trajectory=self.env.trajectories[id(robot)]
            trajectory = np.array(trajectory)
            
            ax.plot(trajectory[:, 0], trajectory[:, 1],color=currentcolor)

            if (i==len(self.colorlist)-1):
                i=0
            else:
                i+=1


        ax.set_title(Title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.axis("equal")
        ax.grid(True)

        plt.tight_layout()
        plt.show()
    
    def animate_display(self, Title="Animated Simulation", xlabel="X", ylabel="Y"):
        fig, ax = plt.subplots(figsize=(6, 6))
        plt.ion()

        extra_trajs = {}
        for robot in self.env.robots:
            if robot.Kinematic_Model.extra_points is not None:
                trajectory = np.array(self.env.trajectories[id(robot)])
                extra_trajs[id(robot)] = np.array([
                    robot.Kinematic_Model.extra_points(trajectory[k], robot.physical_parameters)
                    for k in range(len(trajectory))
                ])
            else:
                extra_trajs[id(robot)] = None

        for step in range(len(self.env.trajectories[id(self.env.robots[0])])):
            ax.clear()
            i = 0
            for robot in self.env.robots:
                currentcolor = self.colorlist[i]
                trajectory = np.array(self.env.trajectories[id(robot)])

                ax.plot(trajectory[:step, 0], trajectory[:step, 1], color=currentcolor)
                if step > 0:
                    ax.plot(trajectory[step-1, 0], trajectory[step-1, 1], 'o', color=currentcolor, markersize=8)

                extra_traj = extra_trajs[id(robot)]
                if extra_traj is not None and step > 0:
                    for j in range(len(extra_traj[0])):
                        ax.plot(extra_traj[:step, j, 0], extra_traj[:step, j, 1], '--', color=currentcolor)
                        ax.plot(extra_traj[step-1, j, 0], extra_traj[step-1, j, 1], 's', color=currentcolor, markersize=8)

                if i == len(self.colorlist) - 1:
                    i = 0
                else:
                    i += 1

            ax.set_title(Title)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.axis("equal")
            ax.grid(True)
            plt.tight_layout()
            plt.pause(0.01)

        plt.ioff()
        plt.show()

    