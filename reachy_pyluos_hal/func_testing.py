import numpy as np
from functions import turn_on, turn_off_smoothly, get_joint_positions, goto
import time


goal_positions_1 = {'l_shoulder_pitch': 0.0, 'l_shoulder_roll': 0.0, 'l_arm_yaw': 0.0, 'l_elbow_pitch': 0.0, 
                            'l_forearm_yaw': 0.0, 'l_wrist_pitch': 0.0, 'l_wrist_roll': 0.0, 'l_gripper': 0.0, 
                            'r_shoulder_pitch': 0.0, 'r_shoulder_roll': 0.0, 'r_arm_yaw': 0.0, 'r_elbow_pitch': -1.5, 
                            'r_forearm_yaw': 0.0, 'r_wrist_pitch': 0.0, 'r_wrist_roll': 0.0, 'r_gripper': 0.0}

goal_positions_2 = {'l_shoulder_pitch': 0.0, 'l_shoulder_roll': 0.0, 'l_arm_yaw': 0.0, 'l_elbow_pitch': 0.0, 
                            'l_forearm_yaw': 0.0, 'l_wrist_pitch': 0.0, 'l_wrist_roll': 0.0, 'l_gripper': 0.0, 
                            'r_shoulder_pitch': 0.5, 'r_shoulder_roll': 0.0, 'r_arm_yaw': 0.0, 'r_elbow_pitch': -1.5, 
                            'r_forearm_yaw': 0.0, 'r_wrist_pitch': 0.0, 'r_wrist_roll': 0.0, 'r_gripper': 0.0}

print("turning on")

turn_on()
print("goto")
goto(goal_positions = goal_positions_1, duration = 5, sampling_freq = 100, interpolation_mode = 'minimum_jerk')
print("\n\nprinting joint positions\n\n")
print(get_joint_positions())
#print("sleeping for 5 seconds")
time.sleep(5.0)
print("second goto")
goto(goal_positions = goal_positions_2, duration = 2.0, sampling_freq = 100, interpolation_mode = 'minimum_jerk')
print("\n\nprinting joint positions\n\n")
print(get_joint_positions())
#print("sleeping for 5 seconds")
time.sleep(5.0)
print("\n\nturning off\n\n")
turn_off_smoothly()