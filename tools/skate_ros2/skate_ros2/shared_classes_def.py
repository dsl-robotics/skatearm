# Vendored verbatim from Rbotic/skate_teleop -> teleop/utils/shared_classes_def.py
# Copyright 2026 R.Botic
# SPDX-License-Identifier: Apache-2.0
# (Licensed under the Apache License, Version 2.0; see the upstream repository
#  https://github.com/Rbotic/skate_teleop for the full license text.)
#
# WHY VENDORED: the Skate firmware streams telemetry as pickled instances of
# these classes, and pickle resolves them by module name 'shared_classes_def'.
# skate_ros2.protocol registers this module under that name in sys.modules so
# the robot's packets unpickle without the official teleop repo installed.
# Do not rename or reorder fields — this is the wire format.

from typing import List, Tuple
import numpy as np
import dataclasses
import math

@dataclasses.dataclass
class FeedbackResp:
    servo_id: int
    errors: List[int]
    mode: int
    angle: float
    velocity: float
    torque: float
    temp: float

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class base__repr__:
    def print_format_dict(self, d):
        return "\n" + "\n".join(
            [f"  {k}: {str(v):<30}" for k, v in sorted(d.items())]
        )

    def __repr__(self):
        repr_str = f"{self.__class__.__name__}(\n"

        # Include instance attributes
        for attr_name, attr_value in vars(self).items():
            if isinstance(attr_value, dict):  # Format dictionaries
                repr_str += f"  {attr_name}={self.print_format_dict(attr_value)}\n"
            else:  # Directly format non-dictionaries
                repr_str += f"  {attr_name}={attr_value}\n"

        # Include class attributes
        for attr_name, attr_value in self.__class__.__dict__.items():
            # Skip special/magic methods and attributes
            if not attr_name.startswith("__") and not callable(attr_value):
                if isinstance(attr_value, dict):  # Format dictionaries
                    repr_str += f"  {attr_name}={self.print_format_dict(attr_value)}\n"
                else:  # Directly format non-dictionaries
                    repr_str += f"  {attr_name}={attr_value}\n"

        repr_str += ")"
        return repr_str

class motor_command(base__repr__):
    def __init__(self):
        rs_kp = 10*0 #RS
        rs_kd = 1*0
        od_kp = 2*0 #2 is a good testing value
        od_kd = 0.5/10 #0.5 seems like the max stable

        self.motor_pos_cmd = {
            0: [0, 0, 0, 0], # CAN0: motors 1, 2, 3, 4     (left leg)
            1: [0, 0, 0, 0], # CAN1: motors 1, 2, 3, 4     (right leg)
            2: [0, 0, 0, 0, 0, 0, 0, 0], # CAN2: motors 1,2,3,4,5,6,7,8     (left arm)
            3: [0, 0, 0, 0, 0, 0, 0, 0], # CAN3: motors 1,2,3,4,5,6,7,8     (right arm)
            4: [0, 0] # CAN4: motors 1, 2     (head)
        }

        self.motor_vel_cmd = {
            0: [0, 0, 0, 0],
            1: [0, 0, 0, 0],
            2: [0, 0, 0, 0, 0, 0, 0, 0],
            3: [0, 0, 0, 0, 0, 0, 0, 0],
            4: [0, 0]
        }

        self.motor_torque_cmd = {
            0: [0, 0, 0, 0],
            1: [0, 0, 0, 0],
            2: [0, 0, 0, 0, 0, 0, 0, 0],
            3: [0, 0, 0, 0, 0, 0, 0, 0],
            4: [0, 0]
        }

        self.motor_kp_cmd = {
            0: [rs_kp, rs_kp, rs_kp, od_kp],
            1: [rs_kp, rs_kp, rs_kp, od_kp],
            2: [od_kp, od_kp, od_kp, od_kp, 0, 0, 0, 0],
            3: [od_kp, od_kp, od_kp, od_kp, 0, 0, 0, 0],
            4: [od_kp, od_kp]
        }

        self.motor_kd_cmd = {
            0: [rs_kd, rs_kd, rs_kd, od_kd],
            1: [rs_kd, rs_kd, rs_kd, od_kd],
            2: [od_kd, od_kd, od_kd, od_kd, 0, 0, 0, 0],
            3: [od_kd, od_kd, od_kd, od_kd, 0, 0, 0, 0],
            4: [od_kd, od_kd]
        }

class motor_state(base__repr__):
    def __init__(self):
        self.motor_pos = {
            0: [0, 0, 0, 0], # CAN0: motors 1, 2, 3, 4     (left leg)
            1: [0, 0, 0, 0], # CAN1: motors 1, 2, 3, 4     (right leg)
            2: [0, 0, 0, 0, 0, 0, 0, 0], # CAN2: motors 1,2,3,4,5,6,7,8     (left arm)
            3: [0, 0, 0, 0, 0, 0, 0, 0], # CAN3: motors 1,2,3,4,5,6,7,8     (right arm)
            4: [0, 0] # CAN4: motors 1, 2     (head)
        }

        self.motor_vel = {
            0: [0, 0, 0, 0],
            1: [0, 0, 0, 0],
            2: [0, 0, 0, 0, 0, 0, 0, 0],
            3: [0, 0, 0, 0, 0, 0, 0, 0],
            4: [0, 0]
        }

        self.motor_torque = {
            0: [0, 0, 0, 0],
            1: [0, 0, 0, 0],
            2: [0, 0, 0, 0, 0, 0, 0, 0],
            3: [0, 0, 0, 0, 0, 0, 0, 0],
            4: [0, 0]
        }

        self.motor_error = {
            0: [None, None, None, None],
            1: [None, None, None, None],
            2: [None, None, None, None, None, None, None, None],
            3: [None, None, None, None, None, None, None, None],
            4: [None, None]
        }

        self.motor_temp = {
            0: [0, 0, 0, 0],
            1: [0, 0, 0, 0],
            2: [0, 0, 0, 0, 0, 0, 0, 0],
            3: [0, 0, 0, 0, 0, 0, 0, 0],
            4: [0, 0]
        }

    def reformat_motor_state(self, feedback_data: List[Tuple[int, int, FeedbackResp]]):
        # Sort feedback data first by CAN bus index and then motor index
        sorted_data = sorted(feedback_data, key=lambda x: (x[0], x[1]))  # Sort by CAN bus index, then by motor index

        # Iterate over the sorted data and populate the motor state dictionaries
        for can_bus_index, motor_index, feedback in sorted_data:
            self.motor_pos[can_bus_index][motor_index] = feedback.angle
            self.motor_vel[can_bus_index][motor_index] = feedback.velocity
            self.motor_torque[can_bus_index][motor_index] = feedback.torque
            self.motor_error[can_bus_index][motor_index] = feedback.errors
            self.motor_temp[can_bus_index][motor_index] = feedback.temp
        return self

    def update_motor_states_from_servo_state(self, servo_state: "motor_state"):
        for arr in ("motor_pos", "motor_vel", "motor_torque", "motor_temp"):
            self_arr = getattr(self, arr)
            servo_arr = getattr(servo_state, arr)
            self_arr[2][-4:] = servo_arr[2][-4:]
            self_arr[3][-4:] = servo_arr[3][-4:]
        return self

    def update_servo_pos_from_mot_cmds(self, mot_cmds: motor_command):
        left_servos = np.array(mot_cmds.motor_pos_cmd[2][-4:]) #last 4
        right_servos = np.array(mot_cmds.motor_pos_cmd[3][-4:])
        self.motor_pos[2][-4:] = map_value(left_servos, 0, 180, -np.pi/2, np.pi/2) #convert motor cmds (0 - 180deg) to assumed servo positions (-90 deg to 90 deg in rad)
        self.motor_pos[3][-4:] = map_value(right_servos, 0, 180, -np.pi/2, np.pi/2)
        self.motor_pos[2][-1:] = map_value(left_servos[-1:], 0, 180, 0, np.pi) #convert motor cmds (0 - 180deg) to assumed servo positions (-90 deg to 90 deg in rad)
        self.motor_pos[3][-1:] = map_value(right_servos[-1:], 0, 180, 0, np.pi)
        return self

class state_est(base__repr__):
    def __init__(self):
        self.dof_pos = {
            0: [0, 0, 0, 0], # CAN0: motors 1, 2, 3, 4     (left leg)
            1: [0, 0, 0, 0], # CAN1: motors 1, 2, 3, 4     (right leg)
            2: [0, 0, 0, 0, 0, 0, 0, 0], # CAN2: motors 1,2,3,4,5,6,7,8     (left arm)
            3: [0, 0, 0, 0, 0, 0, 0, 0], # CAN3: motors 1,2,3,4,5,6,7,8     (right arm)
            4: [0, 0] # CAN4: motors 1, 2     (head)
        }

        self.dof_vel = {
            0: [0, 0, 0, 0],
            1: [0, 0, 0, 0],
            2: [0, 0, 0, 0, 0, 0, 0, 0],
            3: [0, 0, 0, 0, 0, 0, 0, 0],
            4: [0, 0]
        }

        self.dof_torque = {
            0: [0, 0, 0, 0],
            1: [0, 0, 0, 0],
            2: [0, 0, 0, 0, 0, 0, 0, 0],
            3: [0, 0, 0, 0, 0, 0, 0, 0],
            4: [0, 0]
        }

        self.base_vel_IK_est = np.zeros(3)
        self.base_pos_IK_est = np.zeros(3)

        self.is_ground_contact = np.array([True, True])

        self.leg_ee_pos = np.zeros((2, 3))
        self.leg_ee_vel = np.zeros((2, 3))

        self.foot_contact_vel = np.zeros((2, 3))

        self.dof_offest_cal = { #default offset to compensate assuming encoders at 0,0,0,0,0,0,0 at crouching position. Signage to convert motor pos to dof pos (what dof_pos whats it zeroed at?)
            0: [0, math.radians(180), math.radians(-158.8), 0], #joints do not always initialise at zero properly, sometimes starts at 6.28 (2pi) as zero
            1: [0, math.radians(180), math.radians(-158.8), 0],
            2: [0, math.radians(-0), math.radians(0), math.radians(-0), 0, 0, 0, math.radians(0)], #TODO mod values #elbow +- 2.65
            3: [0, math.radians(-0), math.radians(0), math.radians(-0), 0, 0, 0, math.radians(0)],
            4: [0, math.radians(90+22.5)]
        }

        self.mot_dir_cal = {
            0: [1, 1, -1, -1],
            1: [1, -1, 1, 1],
            2: [1, -1, -1, 1, -1, -1, -1, -1],
            3: [1, -1, 1, -1, 1, 1, -1, -1],
            4: [-1, 1]
        }

        self.mot_offset_cal = { #to compensate for motor pos, if initialises to 6.28
            0: [0, 0, 0, 0],
            1: [0, 0, 0, 0],
            2: [0, 0, 0, 0, 0, 0, 0, 0],
            3: [0, 0, 0, 0, 0, 0, 0, 0],
            4: [0, 0]
        }

    def extract_and_concat_arm_pos(self):
        left_arm = self.dof_pos[2][:7]  # Extract first 7 elements of left arm
        right_arm = self.dof_pos[3][:7]  # Extract first 7 elements of right arm
        return np.array(left_arm + right_arm)  # Concatenate both lists

    def extract_and_concat_dof_pos(self):
        left_leg = self.dof_pos[0][:]
        right_leg = self.dof_pos[1][:]
        left_arm = self.dof_pos[2][:]
        right_arm = self.dof_pos[3][:]
        head = self.dof_pos[4][:]
        return np.array(left_leg + right_leg + left_arm + right_arm + head)  # Concatenate

class INS_fusion_state(object):
    def __init__(self):
        self.out_gyr = np.zeros(3) #raw gyro
        self.out_acc = np.zeros(3) #raw acc
        self.out_rot_matrix = None #output rotation matrix
        self.out_quat = np.zeros(4) #output global acceleration
        self.out_grav_vec = np.zeros(3) #output gravity vector
        self.out_global_acc = np.zeros(3) #output global acceleration
        self.out_global_vel = np.zeros(3) #output global velocity
        self.out_global_pos = np.zeros(3) #output global position

    def __repr__(self):
        return (
            f"INS_fusion(\n"
            f"  out_quat={self.out_quat},\n"
            f"  out_grav_vec={self.out_grav_vec},\n"
            f"  out_global_acc={self.out_global_acc},\n"
            f"  out_global_vel={self.out_global_vel},\n"
            f"  out_global_pos={self.out_global_pos}\n"
            f")"
        )
