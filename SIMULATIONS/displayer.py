from SIMULATIONS.environment import Environment
import numpy as np
import matplotlib.pyplot as plt


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
    
    def animate_display(self, Title="Animated Simulation",xlabel="X",ylabel="Y", fixed_view=False):

        fig, ax = plt.subplots(figsize=(6, 6))
        plt.ion()
                               
        for step in range(len(self.env.trajectories[id(self.env.robots[0])])):
            ax.clear()
            i=0
            for robot in self.env.robots:
                currentcolor=self.colorlist[i]
                trajectory=self.env.trajectories[id(robot)]
                trajectory = np.array(trajectory)
                
                ax.plot(trajectory[:step, 0], trajectory[:step, 1], color=currentcolor)

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
            plt.pause(0.01)

        plt.ioff()
        plt.show()


    