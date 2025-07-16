
import json
import copy
import math
import time
import sys
import numpy as np

# from keyboard_util import * 
from robot_kinemic.kinemic_utils import *
from robot_kinemic.ros_node import *
from robot_kinemic.kinemic import loadUrdf
from robot_kinemic.mocap_data_manage import mocapDataManage
from robot_kinemic.ros_node import *
from robot_kinemic.teleop_pico import TeleopData
from robot_kinemic.conf import *
from robot_kinemic.mocap_unit import mocapUnit, btnCtrlUnit
# from robot_kinemic.log.log_manage import log_m
from robot_kinemic.config_info import Config
from robot_kinemic.p2p_motion import P2PMotion
from robot_kinemic.lcm_unit_gripper import lcmUnit
from robot_kinemic.to_ready import to_ready, test_tt
from robot_kinemic.to_recoverstand import to_recoverstand



# TeleopData_IP = "192.168.22.117"
# SensorPosition = "Thumb_side"
# SAVE_PATH =  "action_make_waving_waist_v3.csv"
# IS_REAL= False
# RECORD_CSV = False
# IS_GRIPPER = False

class mocapManage():
    def __init__(self, config=None):
        self.config = config
        if config is None:
            self.config = Config()
        self.wrist_l_m =    mocapUnit(self.config.wrist_l, 2, 0.8)
        self.wrist_r_m =    mocapUnit(self.config.wrist_r, 2, 0.8)
        self.perl_m =       mocapUnit(self.config.perl, 3, 0.01)
        self.head_m =       mocapUnit(self.config.head, 2, 1)
        self.gripper_l_m = btnCtrlUnit(0.5, [0, 80], 0)
        self.gripper_r_m = btnCtrlUnit(0.5, [0, 80], 0)
        self.finger_l_m = [
            btnCtrlUnit(0.02, self.config.hand_l[0], 3),
            btnCtrlUnit(0.02, self.config.hand_l[1], 3),
            btnCtrlUnit(0.02, self.config.hand_l[2], 3),
            btnCtrlUnit(0.02, self.config.hand_l[3], 3),
            btnCtrlUnit(0.02, self.config.hand_l[4], 0.92),
            btnCtrlUnit(0.02, self.config.hand_l[5], 2.87),
            ]
        self.finger_r_m = [
            btnCtrlUnit(0.02, self.config.hand_r[0], 3),
            btnCtrlUnit(0.02, self.config.hand_r[1], 3),
            btnCtrlUnit(0.02, self.config.hand_r[2], 3),
            btnCtrlUnit(0.02, self.config.hand_r[3], 3),
            btnCtrlUnit(0.02, self.config.hand_r[4], 0.92),
            btnCtrlUnit(0.02, self.config.hand_r[5], 2.87)
            ]
        
        # self.wrist_l_m =  mocapUnit([[-0.9948, 0.9948], [-0.4538, 0.4538]], 2, 0.8)
        # self.wrist_r_m =  mocapUnit([[-0.9948, 0.9948], [-0.4538, 0.4538]], 2, 0.8)
        # self.perl_m = mocapUnit([[-0.2618, 0.2618], [0,0.5], [-1.5708, 1.5708]], 3, 0.01)
        # self.head_m = mocapUnit([[-0.5236, 0.5236], [-0.3491, 0.1745]], 2, 1)
        # self.gripper_l_m = btnCtrlUnit(0.5, [0, 80], 0)
        # self.gripper_r_m = btnCtrlUnit(0.5, [0, 80], 0)
        # self.finger_l_m = [btnCtrlUnit(0.02, [0.5, 3], 3),btnCtrlUnit(0.02, [0.5, 3], 3),btnCtrlUnit(0.02, [0.5, 3], 3),btnCtrlUnit(0.02, [0.5, 3], 3),btnCtrlUnit(0.02, [0.5, 0.92], 0.92),btnCtrlUnit(0.02, [0.77, 2.87], 2.87)]
        # self.finger_r_m = [btnCtrlUnit(0.02, [1.5, 3], 3),btnCtrlUnit(0.02, [1.5, 3], 3),btnCtrlUnit(0.02, [1.5, 3], 3),btnCtrlUnit(0.02, [1.5, 3], 3),btnCtrlUnit(0.02, [0.5, 0.92], 0.92),btnCtrlUnit(0.02, [0.77, 2.87], 2.87)]
        
    def update_wrist_state(self,  wrist_l_p_e, wrist_r_p_e):
        wrist_l_quat = eul_to_quat(wrist_l_p_e[3:])
        wrist_r_quat = eul_to_quat(wrist_r_p_e[3:])
        if self.config.sensor_position == "Back_of_the_hand":
            self.wrist_l_m.move([-1*wrist_l_quat[0], wrist_l_quat[2]])
            self.wrist_r_m.move([wrist_r_quat[0], 1 * wrist_r_quat[2]])
        elif self.config.sensor_position == "Palm_of_the_hand":
            self.wrist_l_m.move([wrist_l_quat[0], wrist_l_quat[2]])
            self.wrist_r_m.move([-1*wrist_r_quat[0], 1 * wrist_r_quat[2]])
            # print(f"wrist_r_quat: {wrist_r_quat}")
        elif self.config.sensor_position == "Thumb_side":
            self.wrist_l_m.move([wrist_l_quat[1], wrist_l_quat[2]])
            self.wrist_r_m.move([1*wrist_r_quat[1], 1 * wrist_r_quat[2]])
            
    def update_head_state(self, head_p_e):
        head_quat = eul_to_quat(head_p_e[3:])
        self.head_m.move([-1*head_quat[2], -1*head_quat[0]])

    def update_perl_state(self, perl_p_e):
        perl_quat = eul_to_quat(perl_p_e[3:])
        # self.perl_m.move([-2*perl_quat[2],  -1 * perl_quat[0], 2*perl_quat[1]])
        self.perl_m.move([0,  -1 * perl_quat[0], 2*perl_quat[1]])

    def update_gripper_state(self,btn_l, btn_r):
        btn_idx = 2
        move_speed = 5
        if(btn_l[btn_idx]==True):
            self.gripper_l_m.move(move_speed)    

        elif(btn_l[btn_idx]==False):
            self.gripper_l_m.move(-move_speed)

        if(btn_r[btn_idx]==True):
            self.gripper_r_m.move(move_speed)

        elif(btn_r[btn_idx]==False):
            self.gripper_r_m.move(-move_speed)

    def update_finger_state(self,btn_l, btn_r):
        btn_idx_f = 2
        btn_idx_t = 3
        move_speed = 1
        if(btn_l[btn_idx_f]==True):
            for i in range(5):
                self.finger_l_m[i].move(-move_speed)

        elif(btn_l[btn_idx_f]==False):
            for i in range(5):
                self.finger_l_m[i].move(move_speed)

        if(btn_r[btn_idx_f]==True):
            for i in range(5):
                self.finger_r_m[i].move(-move_speed)

        elif(btn_r[btn_idx_f]==False):
            for i in range(5):
                self.finger_r_m[i].move(move_speed)


        if(btn_l[btn_idx_t]==True):
            self.finger_l_m[5].move(-1)

        elif(btn_l[btn_idx_t]==False):
            self.finger_l_m[5].move(1)

        if(btn_r[btn_idx_t]==True):
            self.finger_r_m[5].move(-1)

        elif(btn_r[btn_idx_t]==False):
            self.finger_r_m[5].move(1)
            

    def update_state(self, mocap_data):

        self.update_wrist_state(mocap_data.left_wrist_ctrl.deta_pos_eul, mocap_data.right_wrist_ctrl.deta_pos_eul)

        self.update_head_state(mocap_data.hmd_ctrl.deta_pos_eul)

        self.update_perl_state(mocap_data.chest_ctrl.deta_pos_eul)

        if self.config.is_gripper == True:
            self.update_gripper_state(mocap_data.left_wrist_ctrl.btn, mocap_data.right_wrist_ctrl.btn)
        else:
            self.update_finger_state(mocap_data.left_wrist_ctrl.btn, mocap_data.right_wrist_ctrl.btn)


    def get_wrist_state(self):
        # return self.wrist_l_m.val[1], self.wrist_l_m.val[0], -1*self.wrist_r_m.val[1],self.wrist_r_m.val[0]
        return self.wrist_l_m.val[1], self.wrist_l_m.val[0], self.wrist_r_m.val[1],self.wrist_r_m.val[0]
        # if SensorPosition == "Back_of_the_hand":
        #     return self.wrist_l_m.val[1], self.wrist_l_m.val[0], -1*self.wrist_r_m.val[1],self.wrist_r_m.val[0]
        # elif SensorPosition == "Palm_of_the_hand":
        #     return self.wrist_l_m.val[1], self.wrist_l_m.val[0], -1*self.wrist_r_m.val[1],self.wrist_r_m.val[0]
        # elif SensorPosition == "Thumb_side": # 需要真机测试
        #     return self.wrist_l_m.val[1], -1*self.wrist_l_m.val[0], -1*self.wrist_r_m.val[1], 1*self.wrist_r_m.val[0]
    
    def get_perl_state(self):
        return [self.perl_m.val[0], self.perl_m.val[1], self.perl_m.val[2]]  #ROLL PITCH YAW
    
    def get_head_state(self):
        return self.head_m.val
    
    def get_gripper_state(self):
        return self.gripper_l_m.val, self.gripper_r_m.val

    def get_finger_state(self):
        output_list = []
        for i in range(6):
            output_list.append(self.finger_l_m[i].val)
        for i in range(6):
            output_list.append(self.finger_r_m[i].val)

        return output_list

class mocapActionInfo(robotControlRos):
    def __init__(self, config_path=None):
        robotControlRos.__init__(self)
        self.config = Config()
        if config_path is not None:
            self.config.read_config_file(config_path)
        self.pre_head = [0, 0]
        self.pre_perl = [0, 0, 0]
        self.pre_arm = [0 for _ in range(14)]
        self.pre_arm_vel = [0 for _ in range(14)]

        self.pre_wrist = [0,0,0,0]
        self.mocap_data_manage = mocapDataManage(self.config)
        self.kinemic_manage_l = loadUrdf(True, self.config)
        self.kinemic_manage_r = loadUrdf(False, self.config)
        self.teleop = None
        # self.teleop = TeleopData(TeleopData_IP)
        # if self.teleop_switch == False:
        #     self.teleop = None
        # else:
        #     self.teleop = TeleopData(self.config.teleop_data_ip)
        self.current_time = time.time()
        self.current_time_v2 = time.time()


        self.mocap_m = mocapManage(self.config)

        self.p2p_m = None
        self.lcm_unit = None
        # if(self.config.is_real):
        #     self.lcm_unit = lcmUnit()
        #     self.p2p_m = P2PMotion(self.lcm_unit)

        self.is_start = False
        self.is_send_data = False
        self.start_record = False
        self.start_save = False
        self.wrist_pos = np.zeros(2*6)
        self.once = False

    def period_get_data(self):
        if self.period_get_data_switch == False:
            return 
        
        ratio = 0.6

        if not self.is_start:
            return
        
        next(self.kinemic_manage_l.iteration)
        next(self.kinemic_manage_r.iteration)

        self.pub_robot_info(self.kinemic_manage_l.pos.tolist() + self.kinemic_manage_l.eul.tolist())        

        
        arm_list = [0 for _ in range(14)]
        qpos_mark_l = [1, 1, 1, 1, 1, 1, 1]
        qpos_mark_r = [1, 1, 1, 1, 1, 1, 1]
        for i in range(7):
            arm_list[i] = self.kinemic_manage_l.qpos[i] * qpos_mark_l[i]
            arm_list[i + 7] = self.kinemic_manage_r.qpos[i] * qpos_mark_r[i]

        # if(False):
        #     arm_list[4:7] = self.kinemic_manage_l.wrist_control()
        #     arm_list[11:14] = self.kinemic_manage_r.wrist_control()

        #     # self.kinemic_manage_l.wrist_control()
        #     # self.kinemic_manage_r.wrist_control()
        # # print(arm_list)

        arm_list, pre_arm_vel = self.interpolate_v3(self.pre_arm, arm_list, self.pre_arm_vel, ratio)
        self.pre_arm = copy.deepcopy(arm_list)
        self.pre_arm_vel = copy.deepcopy(pre_arm_vel)
        self.mocap_m.update_state(self.mocap_data_manage)
        wrist_list = self.mocap_m.get_wrist_state()
        wrist_list = self.interpolate_v2(self.pre_wrist, wrist_list, 0.3)
        self.pre_wrist = copy.deepcopy(wrist_list)
        arm_list[5:7],arm_list[12:14]  = wrist_list[:2], wrist_list[2:]
        # perl_val = self.mocap_m.get_perl_state()
        mocap_data = [0 for _ in range(30)]
        arm_list_send = arm_list
        mocap_data[:14] = arm_list_send

        # mocap_data[26:28] = perl_val

        head_val = self.mocap_m.get_head_state()
        mocap_data[28:30] = head_val
        perl_val = self.mocap_m.get_perl_state()
        mocap_data[26:28] = [perl_val[2], perl_val[0]]
        #mocap_data[28:30] = [0, 0.34]

        # print(f"perl_val : {perl_val}\n")
        if self.config.is_gripper == True:
            grriper_val = self.mocap_m.get_gripper_state()
            mocap_data[14] = grriper_val[0]
            mocap_data[20] = grriper_val[1]
        else:
            finger_val = self.mocap_m.get_finger_state()
            mocap_data[14:26] = finger_val

        # print(mocap_data[5:7])
        self.pub_act_qpos(mocap_data)
        # log_m.qpos_info.update_log_info("INFO", str(mocap_data))
                    
        if self.config.record_csv == True:
            if self.start_record == True and self.start_save == True:
                with open(self.config.save_path, 'a', encoding='utf-8') as file:
                    value = f"{mocap_data}\r\n"
                    val = value.replace('[', '').replace(']', '')
                    file.write(val)
                    
        # 真机调试记得打开这里

        if(self.config.is_real):
            if self.is_send_data:
            # if False:
                pass
                mocap_data = self.p2p_m.update_qpos(mocap_data)
                self.lcm_unit.send_to_robot(np.array(mocap_data))
                # self.lcm_unit.wbc_floatbase_ctrl(perl_val)


    def update_mocap_inf(self):
        if self.update_mocap_inf_switch == False:
            return 

        self.update_keyboard_state()
        mocap_info = self.teleop.update_mocap_info()
        self.mocap_data_manage.update_button(mocap_info)
        if(not self.is_start):
            return

        self.mocap_data_manage.parse_data(mocap_info)
        self.kinemic_manage_l.update_target_pos_diff(self.mocap_data_manage.left_arm_ctrl.deta_pos_eul)
        self.kinemic_manage_r.update_target_pos_diff(self.mocap_data_manage.right_arm_ctrl.deta_pos_eul)
        # log_m.target_pos_eul.update_log_info("INFO", "left arm : " + str(self.mocap_data_manage.left_arm_ctrl.deta_pos_eul))
        # log_m.target_pos_eul.update_log_info("INFO", "right arm : " + str(self.mocap_data_manage.right_arm_ctrl.deta_pos_eul))
        self.pub_mocap_info(self.mocap_data_manage.left_arm_ctrl.deta_pos_eul.tolist())


    def update_keyboard_state(self):
        self.current_time = time.time()
        if self.mocap_running == False:
            return 

        if(self.current_time - self.current_time_v2<2):
            return
        if self.once == False:
            self.once = True
        if(self.mocap_data_manage.left_wrist_ctrl.btn[0] == True):
            if(self.config.is_real):
                self.p2p_m.start_p2p_move()
            self.is_start = not self.is_start
            if(not self.is_start):
                self.is_send_data = False
                if self.teleop:
                    self.teleop.stream_contrller.update_X(0, "exit")
                print("exit()")
                self.close_mocap()
                # sys.exit()
            else:
                if self.teleop:
                    self.teleop.stream_contrller.update_X(1, "is_start")
            self.current_time_v2 =  self.current_time
            print(f"act qpos_ start : {self.is_start}!\n")
        if(self.is_start):
            if(self.mocap_data_manage.left_wrist_ctrl.btn[1]):
                self.is_send_data = not self.is_send_data
                self.current_time_v2 =  self.current_time
                if self.teleop:
                    self.teleop.stream_contrller.update_X(2, "is_send_data")

        if(self.mocap_data_manage.right_wrist_ctrl.btn[0] == True):
            self.start_save_data(0)
            self.current_time_v2 =  self.current_time
            self.start_record = True
            self.start_save = True
        if(self.mocap_data_manage.right_wrist_ctrl.btn[1] == True):
            self.start_save_data(1)
            self.current_time_v2 =  self.current_time
            self.start_record = False
            self.start_save = False
            
    def close_mocap(self):
        # mocap state
        self.is_start = False
        self.is_send_data = False
        # mocap period state
        self.period_get_data_switch = False
        self.update_mocap_inf_switch = False
        if  self.teleop:
            self.teleop.stream_contrller.running = False


    def publish_feedback(self, handle, goal_key, str_value):
        try:
            feedback_msg = MocapCmd.Feedback()
            # goal_key = "start_mocap"
            feedback_msg.cmd = goal_key
                
            # 判断是否可启动
            feedback_msg.feedback = str_value
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.feedback))
            handle.publish_feedback(feedback_msg)
        except Exception as e:
            self.get_logger().info('publish_feedback err: {0}'.format(e))
            
    def start_mocap(self, handle):
        goal_key = handle.request.cmd
        goal_value = handle.request.cmd_param
            
        # 判断是否可启动
        
        if self.mocap_running is True:
            self.publish_feedback(handle, goal_key, "running")
            handle.abort()
            return 
        self.mocap_running = True
        self.publish_feedback(handle, goal_key, "idle")
        # 检查是否在46的状态下
        if self.config.is_real:
            if self.lcm_unit is None and self.p2p_m is None:
                self.lcm_unit = lcmUnit()
                self.p2p_m = P2PMotion(self.lcm_unit)
            if self.p2p_m.get_current_pos() is False:
                self.publish_feedback(handle, goal_key, "running")
                handle.abort()
                return 
        
        # 连接PICO
        if self.teleop is None:
            self.publish_feedback(handle, goal_key, "connect pico")
            try:
                self.teleop = TeleopData(goal_value)
            except Exception as e:
                self.get_logger().info('start_mocap err: {0}'.format(e))
        if self.teleop is None:
            self.publish_feedback(handle, goal_key, "Check PICO network or tracker")
            self.teleop = None
            handle.abort()
            return 
        elif self.teleop.stream_contrller.running == False:
            self.publish_feedback(handle, goal_key, "Check PICO running failed, check tracker.")
            self.teleop = None
            handle.abort()
            return
        else:
            self.publish_feedback(handle, goal_key, "connected pico")
            self.teleop.stream_contrller.update_X(0, "p2p start!!!!!")
        
        time.sleep(1)
        # 运行到准备抓
        self.publish_feedback(handle, goal_key, "to_ready start")
        if self.config.is_real == True:
            to_ready()
        time.sleep(1)
        self.publish_feedback(handle, goal_key, "to_ready ok")
        # 启动摇操周期性任务
        self.teleop_switch = True          # PICO 连接开关
        self.teleop_connect_state = True   # PICO 连接状态
        # self.is_start = False
        # self.is_send_data = False
        self.period_get_data_switch = True
        self.update_mocap_inf_switch = True
        
        self.publish_feedback(handle, goal_key, "teleop ok")
        self.publish_feedback(handle, goal_key, "running") 
        self.teleop.stream_contrller.update_X(0, "ready!!!!!")
        handle.succeed()   
    
    def stop_mocap(self, handle):
        
        goal_key = handle.request.cmd
        #if self.mocap_running is False:
        #    self.publish_feedback(handle, goal_key, "running")
        #    return
        self.mocap_running = True
        # 判断摇操是否关闭----不判断，再关一次
        self.publish_feedback(handle, goal_key, "idle")
        self.close_mocap()
        if self.config.is_real:
            if self.p2p_m is None and self.lcm_unit is None:
                self.lcm_unit = lcmUnit()
                self.p2p_m = P2PMotion(self.lcm_unit)
            if self.p2p_m.get_current_pos() is False:
                self.publish_feedback(handle, goal_key, "running")
                handle.abort()
                return 
        # 断开PICO
        if self.teleop:
            self.teleop.stream_contrller.stop_streaming()
        self.teleop = None
        self.teleop_switch = False          # PICO 连接开关
        self.teleop_connect_state = False   # PICO 连接状态
        self.publish_feedback(handle, goal_key, "stop_mocap")
        time.sleep(1)
        
        self.publish_feedback(handle, goal_key, "stop_mocap")
        # 运动到recover stand
        goal_key = handle.request.cmd
        self.publish_feedback(handle, goal_key, "0")
        if self.config.is_real == True:
            to_recoverstand()
        time.sleep(1)
        self.publish_feedback(handle, goal_key, "100")
        self.mocap_running = False
        handle.succeed()
        self.lcm_unit.stop_lcm()
        time.sleep(1)
        self.p2p_m = None
        self.lcm_unit = None


    @staticmethod
    def interpolate(pre_qpos, target_qpos, step):
        step_qpos = []
        for i in range(step):
            tmp_list = []
            for j in range(len(pre_qpos)):
                tmp_list.append((target_qpos[j] - pre_qpos[j]) / step * (i + 1) + pre_qpos[j])
            step_qpos.append(copy.deepcopy(tmp_list))
        return step_qpos

    @staticmethod
    def interpolate_v2(pre_qpos, target_qpos, ratio):
        step_qpos = [0 for _ in range(len(pre_qpos))]
        for i in range(len(pre_qpos)):
            step_qpos[i] = target_qpos[i] * ratio + pre_qpos[i] * (1 - ratio)
        return step_qpos

    @staticmethod
    def interpolate_v3(pre_qpos, target_qpos, pre_vel, ratio):
        vel = [0 for _ in range(len(target_qpos))]
        for i in range(len(target_qpos)):
            vel[i] = target_qpos[i] - pre_qpos[i]

        vel[i] = vel[i] * ratio + pre_vel[i] * (1 - ratio)

        step_qpos = [0 for _ in range(len(pre_qpos))]
        for i in range(len(target_qpos)):
            step_qpos[i] = pre_qpos[i] + vel[i]
        return step_qpos, vel
        vel = [0 for _ in range(len(target_qpos))]
        for i in range(len(target_qpos)):
            vel[i] = target_qpos[i] - pre_qpos[i]

        vel[i] = vel[i] * ratio + pre_vel[i] * (1 - ratio)

        step_qpos = [0 for _ in range(len(pre_qpos))]
        for i in range(len(target_qpos)):
            step_qpos[i] = pre_qpos[i] + vel[i]
        return step_qpos, vel
