import numpy as np
import math

def short(theta0, theta1):
    """
        Finds shortest angle between two angles
    """
    diff = (theta1 - theta0 + math.pi) % (2*math.pi) - math.pi
    if diff < -math.pi:
        return diff + 2*math.pi
    else:
        return diff

def solve_corners(pose, drive):
    """
        Finds the corners of the robot's bumpers
    """
    x, y, theta = pose
    width = drive.width
    length = drive.length
    sin = np.sin(theta)
    cos = np.cos(theta)
    p0 = (x+(width/2)*cos-(length/2)*sin, y+(length/2)*cos+(width/2)*sin)
    p1 = (x-(width/2)*cos-(length/2)*sin, y+(length/2)*cos-(width/2)*sin)
    p2 = (x+(width/2)*cos+(length/2)*sin, y-(length/2)*cos+(width/2)*sin)
    p3 = (x-(width/2)*cos+(length/2)*sin, y-(length/2)*cos-(width/2)*sin)
    return [[p0, p1], [p0, p2], [p1, p3], [p2, p3]]