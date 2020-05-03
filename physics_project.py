import random as rand
import math
import numpy as np
from matplotlib import pyplot as plt
from time import sleep
from sys import stdin


### all the following equations are integrals/derivatives of each other
def eq_pos(x, a=-10, b=3, c=-0.2):
    return a*x+b*x**2+c*x**3
def eq_vel(x):
    return -10+6*x+-0.6*x**2
def eq_acc(x):
    return 6-1.2*x


# takes the derivative by calculating the slope between all adjacent points
# by default, calculates the first derivative (deg). Higher degrees calculated recursively
# note: |d_points_y| = |points_y| - 1
def points_derivative(points_x, points_y, deg=1):
    if deg <= 0:
        return (points_x, points_y)

    d_points_y = [] # y of derivative
    d_points_x = [] # ditto for x
    for i in range(len(points_y)-1): # compares adjacent pairs so one fewer than # points
        ## x and y values for adjacent pairs
        ax = points_x[i]
        bx = points_x[i+1]

        ay = points_y[i]
        by = points_y[i+1]

        # the slope of the adjacent point is the y value of the derivative at the x point between the pair
        slope = (by-ay)/(bx-ax)
        d_points_y.append(slope)

        d_points_x.append((ax+bx)/2)

    return points_derivative(d_points_x, d_points_y, deg-1)


# takes the integral by treating the y value between a pair of points as the slope of the integral between those two points
# the point between the x values of the same points
# note: |i_points_y| = |points_y|
def points_integral(points_x, points_y, c=0, deg=1):
    if deg <= 0:
        return (points_x, points_y)

    i_points_y = [c]
    for i in range(len(points_y)-1):
        ax = points_x[i]
        bx = points_x[i+1]
        dx = bx-ax

        slope = (points_y[i]+points_y[i+1])/2

        dy = slope*dx

        y = i_points_y[-1]+dy

        i_points_y.append(y)

    return points_integral(points_x, i_points_y, c, deg-1)
        

## draws a line from the list of floats x and y
def add_line(p, x, y, color, name, pause=0.1):
    p.plot(x, y, color+"-", label=name)
    p.legend()
    #plt.show(block=False)
    #plt.pause(pause)

## simulates running a rangefinder by using eq_pos() to get the position values and derives the velocity and acceleration
def simulate_rangefinder(p, x):
    ## setup plot
    p.grid(color="black", linestyle="-", linewidth=1)
    p.set_title("rangefinder simulation")

    
    y = eq_pos(x) # measurements taken
    add_line(p, x, y, "r", "position")
    
    vel_x, vel_y = points_derivative(x, y)
    add_line(p, vel_x, vel_y, "g", "velocity")

    acc_x, acc_y = points_derivative(vel_x, vel_y)
    add_line(p, acc_x, acc_y, "b", "acceleration")

## simulates an accelometer using eq_acc to get the acceleration values that would be recorded and integrating to get the
#### velocity and position values
## uses known values of c from eq_pos
def simulate_accelometer(p, x):
    p.grid(color="black", linestyle="-", linewidth=1)
    p.set_title("accelometer simulation")
    
    acc_y = eq_acc(x) # measurements taken
    add_line(p, x, acc_y, "b", "acceleration")

    vel_x, vel_y = points_integral(x, acc_y, c=-10)
    add_line(p, vel_x, vel_y, "g", "velocity")

    pos_x, pos_y = points_integral(x, vel_y)
    add_line(p, pos_x, pos_y, "r", "position")


# this uses the equations for pos, vel, and acc rather than deriving/integrating one to get the other values
def calc_plot(p, x):
    p.grid(color="black", linestyle="-", linewidth=1)
    p.set_title("calculated plot")

    acc_y = eq_acc(x) # measurements taken
    add_line(p, x, acc_y, "b", "acceleration")

    vel_y = eq_vel(x)
    add_line(p, x, vel_y, "g", "velocity")

    pos_y = eq_pos(x)
    add_line(p, x, pos_y, "r", "position")
    

if __name__ == "__main__":
    x = np.arange(0, 10, 0.1)

    
    plt.figure(figsize=(10,7))
    rangefinder_plot = plt.subplot2grid((2,2),(0,0))
    accelometer_plot = plt.subplot2grid((2,2), (1,0))
    calculated_plot = plt.subplot2grid((2,2), (0,1))
                  
    simulate_rangefinder(rangefinder_plot, x)
    simulate_accelometer(accelometer_plot, x)
    calc_plot(calculated_plot, x)


    plt.show()
