import logging
from reachy_pyluos_hal.joint_hal_modified import JointLuos
import time


logging.basicConfig(level=logging.INFO)

joint_hal = JointLuos('full_kit_full_advanced', logging.getLogger())

print("**************** INITIALIZATION DONE ****************")

def turn_on():
    with joint_hal as hal:
        hal.set_compliance(torque_enabled_)


def turn_off():
    with joint_hal as hal:
        hal.set_compliance(compliant_)
        hal.stop()


def goto(goal_position, goal_velocities):
    
    with joint_hal as hal:
        hal.set_goal_velocities(goal_velocities)
        hal.set_goal_positions(goal_position)



compliant_ = {'l_shoulder_pitch': True, 'l_shoulder_roll': True, 'l_arm_yaw': True, 'l_elbow_pitch': True, 
                    'l_forearm_yaw': True, 'l_wrist_pitch': True, 'l_wrist_roll': True, 'l_gripper': True, 
                    'r_shoulder_pitch': True, 'r_shoulder_roll': True, 'r_arm_yaw': True, 'r_elbow_pitch': True, 
                    'r_forearm_yaw': True, 'r_wrist_pitch': True, 'r_wrist_roll': True, 'r_gripper': True}

torque_enabled_ = {'l_shoulder_pitch': False, 'l_shoulder_roll': False, 'l_arm_yaw': False, 'l_elbow_pitch': False, 
                    'l_forearm_yaw': False, 'l_wrist_pitch': False, 'l_wrist_roll': False, 'l_gripper': False, 
                    'r_shoulder_pitch': False, 'r_shoulder_roll': False, 'r_arm_yaw': False, 'r_elbow_pitch': False, 
                    'r_forearm_yaw': False, 'r_wrist_pitch': False, 'r_wrist_roll': False, 'r_gripper': False}

goal_velocities_ = {'l_shoulder_pitch': 0.5, 'l_shoulder_roll': 0.5, 'l_arm_yaw': 0.5, 'l_elbow_pitch': 0.5, 
                    'l_forearm_yaw': 0.5, 'l_wrist_pitch': 0.5, 'l_wrist_roll': 0.5, 'l_gripper': 0.5, 
                    'r_shoulder_pitch': 0.5, 'r_shoulder_roll': 0.5, 'r_arm_yaw': 0.5, 'r_elbow_pitch': 1.5, 
                    'r_forearm_yaw': 0.5, 'r_wrist_pitch': 0.5, 'r_wrist_roll': 0.5, 'r_gripper': 0.5}

goal_positions_ = {'l_shoulder_pitch': 0.0, 'l_shoulder_roll': 0.0, 'l_arm_yaw': 0.0, 'l_elbow_pitch': 0.0, 
                            'l_forearm_yaw': 0.0, 'l_wrist_pitch': 0.0, 'l_wrist_roll': 0.0, 'l_gripper': 0.0, 
                            'r_shoulder_pitch': 0.0, 'r_shoulder_roll': 0.0, 'r_arm_yaw': 0.0, 'r_elbow_pitch': -1.5, 
                            'r_forearm_yaw': 0.0, 'r_wrist_pitch': 0.0, 'r_wrist_roll': 0.0, 'r_gripper': 0.0}

goal_positions_2_ = {'l_shoulder_pitch': 0.0, 'l_shoulder_roll': 0.0, 'l_arm_yaw': 0.0, 'l_elbow_pitch': 0.0, 
                            'l_forearm_yaw': 0.0, 'l_wrist_pitch': 0.0, 'l_wrist_roll': 0.0, 'l_gripper': 0.0, 
                            'r_shoulder_pitch': 0.5, 'r_shoulder_roll': 0.0, 'r_arm_yaw': 0.0, 'r_elbow_pitch': -1.5, 
                            'r_forearm_yaw': 0.0, 'r_wrist_pitch': 0.0, 'r_wrist_roll': 0.0, 'r_gripper': 0.0}


# compliant_ = {'l_shoulder_pitch': True, 'l_shoulder_roll': True, 'l_arm_yaw': True, 'l_elbow_pitch': True, 
#                     'l_forearm_yaw': True, 'l_wrist_pitch': True, 'l_wrist_roll': True, 'l_gripper': True}

# torque_enabled_ = {'l_shoulder_pitch': False, 'l_shoulder_roll': False, 'l_arm_yaw': False, 'l_elbow_pitch': False, 
#                     'l_forearm_yaw': False, 'l_wrist_pitch': False, 'l_wrist_roll': False, 'l_gripper': False}

# goal_velocities_ = {'l_shoulder_pitch': 0.5, 'l_shoulder_roll': 0.5, 'l_arm_yaw': 0.5, 'l_elbow_pitch': 0.5, 
#                     'l_forearm_yaw': 0.5, 'l_wrist_pitch': 0.5, 'l_wrist_roll': 0.5, 'l_gripper': 0.5}

# goal_positions_ = {'l_shoulder_pitch': 0, 'l_shoulder_roll': 0.0, 'l_arm_yaw': 0.0, 'l_elbow_pitch': -2, 
#                             'l_forearm_yaw': 0.0, 'l_wrist_pitch': 0.0, 'l_wrist_roll': 0.0, 'l_gripper': 0.0}

def get_joint_positions():
    with joint_hal as hal:
        joint_names = hal.get_all_joint_names()
        joint_positions = hal.get_joint_positions(joint_names)
        print(joint_positions)


# turn_off()

print("turning on")

turn_on()
print("goto")
goto(goal_positions_, goal_velocities_)
time.sleep(5.0)
get_joint_positions()
print("sleeping for 10 seconds")
time.sleep(10.0)
print("second goto")
goto(goal_positions_2_, goal_velocities_)
time.sleep(5.0)
get_joint_positions()
print("sleeping for 5 seconds")
time.sleep(5.0)
print("turning off")
turn_off()

# with joint_hal as hal:
#     joint_names = hal.get_all_joint_names()
#     joint_positions = hal.get_joint_positions(joint_names)
#     print(joint_positions)


# with joint_hal as hal:
#     joint_names = hal.get_all_joint_names()
#     joint_velocities = hal.get_joint_velocities(joint_names)
    # print(joint_velocities)


# with joint_hal as hal:
#     joint_names = hal.get_all_joint_names()
#     joint_efforts = hal.get_joint_efforts(joint_names)
    # print(joint_efforts)




#     goal_pos = hal.get_goal_positions(['l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw', 'l_elbow_pitch', 'l_forearm_yaw', 'l_wrist_pitch', 'l_wrist_roll', 'l_gripper', 
#  'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw', 'r_elbow_pitch', 'r_forearm_yaw', 'r_wrist_pitch', 'r_wrist_roll', 'r_gripper'])
    
#     goal_vel = hal.get_goal_velocities(['l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw', 'l_elbow_pitch', 'l_forearm_yaw', 'l_wrist_pitch', 'l_wrist_roll', 'l_gripper', 
#  'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw', 'r_elbow_pitch', 'r_forearm_yaw', 'r_wrist_pitch', 'r_wrist_roll', 'r_gripper'])
    
#     compliancy_ = hal.get_compliant(['l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw', 'l_elbow_pitch', 'l_forearm_yaw', 'l_wrist_pitch', 'l_wrist_roll', 'l_gripper', 
#  'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw', 'r_elbow_pitch', 'r_forearm_yaw', 'r_wrist_pitch', 'r_wrist_roll', 'r_gripper'])
   
#     pids_ = hal.get_joint_pids(['l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw', 'l_elbow_pitch', 'l_forearm_yaw', 'l_wrist_pitch', 'l_wrist_roll', 'l_gripper', 
#  'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw', 'r_elbow_pitch', 'r_forearm_yaw', 'r_wrist_pitch', 'r_wrist_roll', 'r_gripper']) 

#     pres_pos = hal.get_joint_positions(['l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw', 'l_elbow_pitch', 'l_forearm_yaw', 'l_wrist_pitch', 'l_wrist_roll', 'l_gripper', 
# 'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw', 'r_elbow_pitch', 'r_forearm_yaw', 'r_wrist_pitch', 'r_wrist_roll', 'r_gripper'])
    
#     pre_vel = hal.get_joint_velocities(['l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw', 'l_elbow_pitch', 'l_forearm_yaw', 'l_wrist_pitch', 'l_wrist_roll', 'l_gripper', 
# 'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw', 'r_elbow_pitch', 'r_forearm_yaw', 'r_wrist_pitch', 'r_wrist_roll', 'r_gripper'])
    
#     pres_eff = hal.get_joint_efforts(['l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw', 'l_elbow_pitch', 'l_forearm_yaw', 'l_wrist_pitch', 'l_wrist_roll', 'l_gripper', 
# 'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw', 'r_elbow_pitch', 'r_forearm_yaw', 'r_wrist_pitch', 'r_wrist_roll', 'r_gripper'])
    
    





