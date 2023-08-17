import asyncio
import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor
from queue import Queue
from typing import Dict, Optional

import logging
from reachy_pyluos_hal.joint_hal_modified import JointLuos


logging.basicConfig(level=logging.INFO)
joint_hal = JointLuos('full_kit_full_advanced', logging.getLogger())

compliant_ = {'l_shoulder_pitch': True, 'l_shoulder_roll': True, 'l_arm_yaw': True, 'l_elbow_pitch': True, 
                    'l_forearm_yaw': True, 'l_wrist_pitch': True, 'l_wrist_roll': True, 'l_gripper': True, 
                    'r_shoulder_pitch': True, 'r_shoulder_roll': True, 'r_arm_yaw': True, 'r_elbow_pitch': True, 
                    'r_forearm_yaw': True, 'r_wrist_pitch': True, 'r_wrist_roll': True, 'r_gripper': True}

torque_enabled_ = {'l_shoulder_pitch': False, 'l_shoulder_roll': False, 'l_arm_yaw': False, 'l_elbow_pitch': False, 
                    'l_forearm_yaw': False, 'l_wrist_pitch': False, 'l_wrist_roll': False, 'l_gripper': False, 
                    'r_shoulder_pitch': False, 'r_shoulder_roll': False, 'r_arm_yaw': False, 'r_elbow_pitch': False, 
                    'r_forearm_yaw': False, 'r_wrist_pitch': False, 'r_wrist_roll': False, 'r_gripper': False}

smooth_turn_off_positions = {'l_shoulder_pitch': 0.0, 'l_shoulder_roll': 0.0, 'l_arm_yaw': 0.0, 'l_elbow_pitch': 0.0, 
                            'l_forearm_yaw': 0.0, 'l_wrist_pitch': 0.0, 'l_wrist_roll': 0.0, 'l_gripper': 0.0, 
                            'r_shoulder_pitch': 0.0, 'r_shoulder_roll': 0.0, 'r_arm_yaw': 0.0, 'r_elbow_pitch': 0, 
                            'r_forearm_yaw': 0.0, 'r_wrist_pitch': 0.0, 'r_wrist_roll': 0.0, 'r_gripper': 0.0}

def turn_on():
    with joint_hal as hal:
        hal.set_compliance(torque_enabled_)


def turn_off_smoothly():
    goto(goal_positions = smooth_turn_off_positions, duration = 5.0, sampling_freq = 100, interpolation_mode = 'minimum_jerk')
    with joint_hal as hal:
        hal.set_compliance(compliant_)
        hal.stop()

def turn_off():
    with joint_hal as hal:
        hal.set_compliance(compliant_)
        hal.stop()

def get_joint_positions():
    with joint_hal as hal:
        joint_names = hal.get_all_joint_names()
        joint_positions = hal.get_joint_positions(joint_names)
        joint_pos_dict = {joint_names[i]: joint_positions[i] for i in range(len(joint_names))}
        return joint_pos_dict

def goto(
    goal_positions,
    duration,
    starting_positions = None,
    sampling_freq = 100,
    interpolation_mode = 'minimum_jerk',
):
    """Send joints command to move the robot to a goal_positions within the specified duration.

    This function will block until the movement is over. See goto_async for an asynchronous version.

    The goal positions is expressed in joints coordinates. You can use as many joints target as you want.
    The duration is expressed in seconds.
    You can specify the starting_position, otherwise its current position is used,
    for instance to start from its goal position and avoid bumpy start of move.
    The sampling freq sets the frequency of intermediate goal positions commands.
    You can also select an interpolation method use (linear or minimum jerk) which will influence directly the trajectory.

    """
    exc_queue: Queue[Exception] = Queue()

    def _wrapped_goto():
        try:
            asyncio.run(
                goto_async(
                    goal_positions=goal_positions,
                    duration=duration,
                    starting_positions=starting_positions,
                    sampling_freq=sampling_freq,
                    interpolation_mode=interpolation_mode,
                ),
            )
        except Exception as e:
            exc_queue.put(e)

    with ThreadPoolExecutor() as exec:
        exec.submit(_wrapped_goto)
    if not exc_queue.empty():
        raise exc_queue.get()


def goto_hal(goal_position, goal_velocities):
    
    with joint_hal as hal:
        hal.set_goal_velocities(goal_velocities)
        hal.set_goal_positions(goal_position)


async def goto_async(
    goal_positions,
    duration,
    starting_positions = None,
    sampling_freq = 100,
    interpolation_mode = 'minimum_jerk',
    #interpolation_mode: InterpolationMode = InterpolationMode.LINEAR,
):
    
    #for key in goal_positions.keys():
    #    if not isinstance(key, Joint):
    #        raise ValueError('goal_positions keys should be Joint!')

    goal_velocities = {'l_shoulder_pitch': 0.5, 'l_shoulder_roll': 0.5, 'l_arm_yaw': 0.5, 'l_elbow_pitch': 0.5, 
                    'l_forearm_yaw': 0.5, 'l_wrist_pitch': 0.5, 'l_wrist_roll': 0.5, 'l_gripper': 0.5, 
                    'r_shoulder_pitch': 0.5, 'r_shoulder_roll': 0.5, 'r_arm_yaw': 0.5, 'r_elbow_pitch': 1.5, 
                    'r_forearm_yaw': 0.5, 'r_wrist_pitch': 0.5, 'r_wrist_roll': 0.5, 'r_gripper': 0.5}
    
    
    # Insert current position assignment here
    if starting_positions is None:
        #print("getting starting positions")
        starting_positions = get_joint_positions()
        #print("got starting positions")

    # Make sure both starting and goal positions are in the same order
    #print("starting_positions: ", starting_positions)
    starting_positions = {j: starting_positions[j] for j in goal_positions.keys()}

    length = round(duration * sampling_freq)
    if length < 1:
        raise ValueError('Goto length too short! (incoherent duration {duration} or sampling_freq {sampling_freq})!')

    joints = list(goal_positions.keys())
    #joints = starting_positions.keys()
    dt = 1 / sampling_freq

    if interpolation_mode == 'linear':
        traj_func = linear(
            np.array(list(starting_positions.values())),
            np.array(list(goal_positions.values())),
            duration,
        )
    else:
        traj_func = minimum_jerk(
            np.array(list(starting_positions.values())),
            np.array(list(goal_positions.values())),
            duration,
        )

    t0 = time.time()
    while True:
        elapsed_time = time.time() - t0
        if elapsed_time > duration:
            break

        point = traj_func(elapsed_time)
        #for j, pos in zip(joints, point):
        #    j.goal_position = pos
        
        curr_pos = get_joint_positions()
        differential_position = {joints[i] : point[i] for i in range(len(joints ))}
        dx = {j : np.abs(differential_position[j] - curr_pos[j]) for j in differential_position.keys()}
        dv = {j : dx[j]/dt for j in differential_position.keys()}

        #print("dv: ", dv)
        
        #goto_hal(differential_position, goal_velocities)
        goto_hal(differential_position, dv)
        #print("*******")
        #print("elapsed_time: ", elapsed_time)
        #print("differential_position: ", differential_position, "\n")

        await asyncio.sleep(dt)


def linear(
    starting_position: np.ndarray,
    goal_position: np.ndarray,
    duration: float,
):
    """Compute the linear interpolation function from starting position to goal position."""
    def f(t: float) -> np.ndarray:
        return starting_position + (goal_position - starting_position) * t / duration
    return f


def minimum_jerk(
    starting_position: np.ndarray,
    goal_position: np.ndarray,
    duration: float,
    starting_velocity: Optional[np.ndarray] = None,
    starting_acceleration: Optional[np.ndarray] = None,
    final_velocity: Optional[np.ndarray] = None,
    final_acceleration: Optional[np.ndarray] = None,
):
    """Compute the mimimum jerk interpolation function from starting position to goal position."""
    if starting_velocity is None:
        starting_velocity = np.zeros(starting_position.shape)
    if starting_acceleration is None:
        starting_acceleration = np.zeros(starting_position.shape)
    if final_velocity is None:
        final_velocity = np.zeros(goal_position.shape)
    if final_acceleration is None:
        final_acceleration = np.zeros(goal_position.shape)

    a0 = starting_position
    a1 = starting_velocity
    a2 = starting_acceleration / 2

    d1, d2, d3, d4, d5 = [duration ** i for i in range(1, 6)]

    A = np.array((
        (d3, d4, d5),
        (3 * d2, 4 * d3, 5 * d4),
        (6 * d1, 12 * d2, 20 * d3)
    ))
    B = np.array((
        goal_position - a0 - (a1 * d1) - (a2 * d2),
        final_velocity - a1 - (2 * a2 * d1),
        final_acceleration - (2 * a2)
    ))
    X = np.linalg.solve(A, B)

    coeffs = [a0, a1, a2, X[0], X[1], X[2]]

    def f(t: float) -> np.ndarray:
        return np.sum([
            c * t ** i
            for i, c in enumerate(coeffs)
        ], axis=0)

    return f