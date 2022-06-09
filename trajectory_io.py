import json
from math import *
import json
import numpy as np

def export_trajectory(x, y, theta, vx, vy, omega, dts, N_per_segment):
    ts = [0]
    for dt in dts:
        for k in range(N_per_segment):
            ts.append(ts[-1] + dt)
    xs, ys, thetas = [], [], []
    new_dt = 0.02
    time = 0
    index = 0
    for k in range(int(ts[-1] / 0.02)):
        while ts[index + 1] < k * 0.02:
            index += 1
        percent = (k * 0.02 - ts[index]) / (ts[index + 1] - ts[index])
        xs.append(round((x[index + 1] - x[index]) * percent + x[index], 4))
        ys.append(round((y[index + 1] - y[index]) * percent + y[index], 4))
        thetas.append(round((theta[index + 1] - theta[index]) * percent + theta[index], 4))
    xs.append(x[-1])
    ys.append(y[-1])
    thetas.append(theta[-1])

    # f = open("src/main/java/frc/paths/" + name + ".java", "w")
    # f.write("package frc.paths;\n")
    # f.write("\n")
    # f.write("import frc.lib.control.SwerveTrajectory;\n")
    # f.write("\n")
    # f.write("public class " + name + " extends Path {\n")
    # f.write("   private final static double[][] points = {\n")
    # for j in range(len(x)):
    #     f.write("       {"+str(round(ts[j],4))+","+str(round(x[j],4))+","+str(round(y[j],4))+","+str(round(theta[j],4))+","+str(round(vx[j],4))+","+str(round(vy[j],4))+","+str(round(omega[j],4))+"},\n")
    # f.write("   };\n")
    # f.write("   public SwerveTrajectory getPath() {\n")
    # f.write("       return new SwerveTrajectory(points);\n")
    # f.write("   }\n")
    # f.write("}\n")
    # f.close()

    f = open("out.json", "w")
    trajectory = []
    for j in range(len(x)):
        # trajectory.append([str(round(ts[j],4)), str(round(x[j],4)), str(round(y[j],4)), str(round(theta[j],4)), str(round(vx[j],4)), str(round(vy[j],4)), str(round(omega[j],4))])
        trajectory.append({"ts": str(round(ts[j],4)), "x": str(round(x[j],4)), "y": str(round(y[j],4)), "theta": str(round(theta[j],4)), "vx": str(round(vx[j],4)), "vy": str(round(vy[j],4)), "omega": str(round(omega[j],4))})
    with f as write:
        json.dump(trajectory, write, indent=4)
    return xs, ys, theta

def import_path(file):
    return json.load(open("src/main/python/paths/" + file + ".json"))