#!/usr/bin/env python3
import numpy as np
import rclpy
import matplotlib.pyplot as plt
from collections import deque

class Evaluations:
    def __init__(self):
        self.distances_100 = deque([0], maxlen=100)
        self.time_steps = deque([0], maxlen=100)
        self.distances = []
        self.desired_distance = None  # Initialize variable to store target distance
        self.target_line = None  # Placeholder for dynamic target line
        self.current_score = 0
        self.score_text = None

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-', label="Real-Time Car Position From Wall")

        self.ax.set_title("Real-Time Distance Oscillations")
        self.ax.set_xlabel("Time Step")
        self.ax.set_ylabel("Distance (m)")
        self.ax.legend()

        # Add initial score box
        self.score_text = self.ax.text(0.02, 0.95, f"Score: {self.current_score:.2f}", 
                                       transform=self.ax.transAxes, fontsize=12, 
                                       bbox=dict(facecolor='white', alpha=0.8, edgecolor='black'))


    def update(self, distance, desired_distance):
        self.distances_100.append(distance)
        self.distances.append(self.distances_100[-1])
        self.time_steps.append(self.time_steps[-1] + 1)

        self.desired_distance = desired_distance  # Update the target distance dynamically
        self.calculate_score()
        return self.current_score
        #self.update_plot()

    def update_plot(self):
        self.line.set_xdata(self.time_steps)
        self.line.set_ydata(self.distances_100)

        # Remove old target line if it exists
        if self.target_line:
            self.target_line.remove()

        # Draw new target line at updated desired distance
        self.target_line = self.ax.axhline(y=self.desired_distance, color='b', linestyle='--', label=f'Target Distance ({self.desired_distance:.2f}m)')

        # Update the score display
        self.score_text.set_text(f"Score: {self.current_score:.2f}")

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.legend()

        plt.draw()
        plt.pause(0.01)

    def calculate_score(self):
        np_distances = np.array(self.distances)
        loss = np.sum(np.abs(np_distances - self.desired_distance))
        self.current_score = loss/len(np_distances)
        #alpha = .01
        #self.current_score = 1/(1+(alpha*loss)**(2))
        
        
        
