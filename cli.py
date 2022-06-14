import argparse
import json
import os
from pathlib import Path
import drive.swerve_drive as swerve_drive
from trajectory_generator import trajectory_generator

def main():
    parser = argparse.ArgumentParser(description="""A tool for generating time-optimal trajectories
        for FRC robots using CasADi. Accepts waypoints and robot characteristics and outputs a list
        of sample points that a trajectory follower can use.""")
    parser.add_argument('--input', '-i', help='A path to a file containing json path data')
    parser.add_argument('--pathname', '-p', default = None, help='Which path to parse in the document')
    parser.add_argument('--out', '-o', default=None, help='Output file location')
    args = parser.parse_args()

    file_path = args.input
    selected_path = args.pathname
    out_file_path = args.out

    try: # to open the file
        file = open(file_path)
        try: # to load json data
            data = json.load(file)
            swerve_model = swerve_drive.swerve_drive(0.622, 0.572, 0.954, 0.903, 46.7, 5.6, 70, 1.9, 0.051)
            if isinstance(data, dict): # must be json object
                if 'robot_configuration' in data: # use custom configuration if provided
                    robot_config = data['robot_configuration']
                    for key in robot_config:
                        if key == 'bumper_length':
                            swerve_model.length = robot_config['bumper_length']
                        elif key == 'bumper_width':
                            swerve_model.width = robot_config['bumper_width']
                        elif key == 'wheel_horizontal_distance':
                            swerve_model.wheelbase_x = robot_config['wheel_horizontal_distance']
                        elif key == 'wheel_vertical_distance':
                            swerve_model.wheelbase_y = robot_config['wheel_vertical_distance']
                        elif key == 'mass':
                            swerve_model.mass = robot_config['mass']
                        elif key == 'moment_of_inertia':
                            swerve_model.moi = robot_config['moment_of_inertia']
                        elif key == 'motor_max_angular_speed':
                            swerve_model.omega_max = robot_config['motor_max_angular_speed']
                        elif key == 'motor_max_torque':
                            swerve_model.tau_max = robot_config['motor_max_torque']
                if 'paths' in data and not(selected_path is None) and isinstance(data['paths'], list):
                    found_path = None
                    for path in data['paths']:
                        if isinstance(path, dict) and 'name' in path and 'waypoints' in path and path['name'] == selected_path:
                            found_path = path
                            break
                    if not(found_path is None):
                        waypoints = found_path['waypoints']
                        try:
                            generator = trajectory_generator(swerve_model)
                            trajectory = generator.generate(waypoints)
                            out_file = out_file_path
                            if os.path.exists(out_file):
                                out_writer = open(out_file, 'w')
                            else:
                                out_writer = open(out_file, 'x')
                            out_writer.write(json.dumps(trajectory, indent=4))
                        except Exception as e:
                            print('Error generating trajectory, check waypoints.')
                            print(str(e))
                    else:
                        print('Could not find path "' + selected_path + '" in input json.')
                else:
                    print('Unable to find a path to build a trajectory on. Check the structure of the input json.') 
            else:
                print('Unable to deduce path information from json data.')
        except json.JSONDecodeError as e:
            print('Error parsing json data: ' + str(e))
    except (OSError, IOError) as e:
        print('Error loading input file "' + file_path + '": ' + str(e))

if __name__ == '__main__':
    main()