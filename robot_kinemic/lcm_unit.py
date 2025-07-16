# !/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

# # 获取当前工作目录
# pwd_path = os.getcwd()
# # 往前后退 2 级
# moca_path = os.path.dirname(os.path.dirname(pwd_path))
# sys.path.append(moca_path)

from lcm import LCM

import numpy as np
import threading
from robot_kinemic.upper_body_cmd_package import upper_body_cmd_package
from robot_kinemic.upper_body_data_package import upper_body_data_package
from robot_kinemic.t12_command_response import t12_command_response

# from biped_lcm_types.python.floating_base_cmd_lcmt import floating_base_cmd_lcmt


class ArmState(object):
    def __init__(self):
        self.dimension = 30
        # self.default_arm_control_mode  = [4, 4, 4, 4, 4, 4, 4] + [4, 4, 4, 4, 4, 4, 4]
        self.default_arm_control_mode = [5 for i in range(14)]
        self.default_hand_control_mode = [4 for dim0 in range(12)]
        self.default_waist_control_mode = [5 for dim0 in range(2)]
        self.default_head_control_mode = [4 for dim0 in range(2)]
        self.default_control_mode = self.default_arm_control_mode + self.default_hand_control_mode + self.default_waist_control_mode + self.default_head_control_mode

        self.is_used = np.zeros(self.dimension, dtype=int)
        self.error_code = np.zeros(self.dimension, dtype=int)
        self.status = np.zeros(self.dimension, dtype=int)
        self.q = np.zeros(self.dimension, dtype=np.float64)
        self.dot_q = np.zeros(self.dimension, dtype=np.float64)
        self.current_or_torque = np.zeros(self.dimension, dtype=np.float64)


class T12CMDRes(object):
    def __init__(self):
        self.a_flag = False
        self.b_flag = False
        self.c_flag = False
        self.d_flag = False

class lcmUnit(LCM):
    def __init__(self) -> None:
        # super().__init__('udpm://239.255.76.67:7667?ttl=1')
        super().__init__('udpm://239.255.76.67:7667?ttl=1')
        self.upper_body_cmd_topic = 'upper_body_cmd'
        self.upper_body_data_topic = 'upper_body_data'
        self.t12_command_response_topic = "t12_cmd_response"
        # self.wbc_float_base_ctrl_topic = "floating_base_cmd"
        self.current_robot_state = ArmState()
        self.current_t12_res = T12CMDRes()
        self.update_once_arm = False
        self.update_t12_once_arm = False
        self.running = True

        self.pre_pose = np.zeros(30)
        self.pre_sent_pose = np.zeros(30)

        self.subscribe(self.upper_body_data_topic, self.upper_body_data_listener_cb)
        self.subscribe(self.t12_command_response_topic, self.t12_cmd_response_cb)
        self.lcm_thread_handle = threading.Thread(target=self.lcm_handle, daemon=True)
        self.lcm_thread_handle.start()

    def t12_cmd_response_cb(self, channel, data):
        try:
            msg = t12_command_response.decode(data)
            self.current_t12_res.a_flag = msg.a_flag
            self.current_t12_res.b_flag = msg.b_flag
            self.current_t12_res.c_flag = msg.c_flag
            self.current_t12_res.d_flag = msg.d_flag
            self.update_t12_once_arm = True
        except Exception as e:
            print(f"upper_body_data_listener_cb err: {e}")

    def upper_body_data_listener_cb(self, channel, data):
        try:
            msg = upper_body_data_package.decode(data)
            self.current_robot_state.q = np.array(msg.curJointPosVec)
            self.current_robot_state.status = np.array(msg.curStatusVec)
            self.current_robot_state.dot_q = np.array(msg.curSpeedVec)
            self.current_robot_state.current_or_torque = np.array(msg.curCurrentVec)
            if self.update_once_arm == False:
                self.pre_sent_pose = self.current_robot_state.q.copy()
            self.update_once_arm = True
        except Exception as e:
            print(f"upper_body_data_listener_cb err: {e}")


    def lcm_handle(self):
        while self.running:
            self.handle()
    
    def stop_lcm(self):
        self.running = False

    def constraint_update(self, robot_30dof_solution):

        max_speed = [3.141592654, 3.141592654, 4.08407045, 4.08407045, 4.08407045, 100, 100]  # arm_joint 1-7
        max_speed = max_speed + max_speed[:7]

        delta_t = 0.01

        # 初始化约束后的角度列表
        constrained_angle = robot_30dof_solution.copy()

        # 对关节 1-14 的速度和角度进行处理
        for i in range(14):
            velocity = (robot_30dof_solution[i] - self.pre_sent_pose[i]) / delta_t
            if abs(velocity) > max_speed[i]:
                velocity = max_speed[i] if velocity > 0 else -max_speed[i]
            constrained_angle[i] = self.pre_sent_pose[i] + velocity * delta_t

        # 更新 pre_sent_pose 为当前帧计算后的约束角度
        self.pre_sent_pose = constrained_angle.copy()

        return constrained_angle

    def send_to_robot(self, data):
        upper_body_cmd_msg = self.load_upper_body_cmd_package(data)
        self.publish(self.upper_body_cmd_topic, upper_body_cmd_msg.encode())
        print(f"send data: {self.upper_body_cmd_topic, upper_body_cmd_msg}\n")

    def load_upper_body_cmd_package(self, robot_30dof_solution):
        upper_body_cmd_msg = upper_body_cmd_package()
        upper_body_cmd_msg.isUsed = 0
        upper_body_cmd_msg.control_mode = (5 * np.ones(7).astype(int)).tolist() + \
                                          (5 * np.ones(7).astype(int)).tolist() + \
                                          (4 * np.ones(12).astype(int)).tolist() + \
                                          (5 * np.ones(2).astype(int)).tolist() + \
                                          (4 * np.ones(2).astype(int)).tolist()

        robot_30dof_solution_update = self.constraint_update(robot_30dof_solution)

        upper_body_cmd_msg.jointPosVec = robot_30dof_solution_update

        upper_body_cmd_msg.jointSpeedVec = np.zeros_like(robot_30dof_solution).tolist()
        upper_body_cmd_msg.jointKp = np.zeros_like(robot_30dof_solution).tolist()
        upper_body_cmd_msg.jointKd = np.zeros_like(robot_30dof_solution).tolist()

        upper_body_cmd_msg.jointSpeedVec = np.zeros_like(robot_30dof_solution).tolist()
        upper_body_cmd_msg.jointCurrentVec = np.zeros_like(robot_30dof_solution).tolist()

        upper_body_cmd_msg.jointCurrentVec = np.zeros_like(robot_30dof_solution).tolist()

        # upper_body_cmd_msg.control_mode[14:20] = (20 * np.ones(6).astype(int)).tolist()
        # upper_body_cmd_msg.control_mode[20:26] = (20 * np.ones(6).astype(int)).tolist()
        return upper_body_cmd_msg


    def wbc_floatbase_ctrl(self, data):
        wbc_float_base_msg = floating_base_cmd_lcmt()
        wbc_float_base_msg.abs_ori_cmd = np.array(data)
        self.publish(self.wbc_float_base_ctrl_topic, wbc_float_base_msg.encode())

if __name__ == "__main__":
    s = lcmUnit()