import numpy as np
import copy

from robot_kinemic.conf import *
from robot_kinemic.kinemic_utils import *
from robot_kinemic.config_info import Config

PI = 3.1415926

class baseMocapHandle():
    def __init__(self) -> None:
        self.init_pos_eul = np.zeros(6)
        self.init_pos_eul_base = np.zeros(6)
        self.pos_eul = np.zeros(6)
        self.pos_eul_base = np.zeros(6)
        self.deta_pos_eul = np.zeros(6)
        self.ratio = [0.8, 0.8, 0.8, 1, 1, 1]


class controlHandle(baseMocapHandle):
    def __init__(self, is_left= True, config=None) -> None:
        super().__init__()
        self.config = config
        if config is None:
            self.config = Config()
        self.is_left = is_left
        self.btn = [4, 4, 4, 4, 4]
        self.r_axis = [0, 0]
        self.deta_pos_eul = np.zeros(6)
        # self.ratio = [0.5, 0.5, 0.5, 1, 1, 1]

        self.ratio = [1.2, 1.2, 1.2, 1, 1, 1]

    def prepare(self, pos_eul, base_pos_eul):
        self.init_pos_eul_base = copy.deepcopy(pos_eul)
        data_odd = get_odd_data(pos_eul)
        base_odd = get_odd_data(np.array(base_pos_eul))
        # data_odd_trans = np.linalg.inv(base_odd) @ data_odd
        data_odd_trans = self.inv_transform(base_odd) @ data_odd
        odd = self.odd_trans(data_odd_trans, self.is_left)
        self.init_odd = odd

    def update(self, pos_eul, base_pos_eul):
        self.pos_eul_base = copy.deepcopy(pos_eul)
        data_odd = get_odd_data(pos_eul)
        base_odd = get_odd_data(np.array(base_pos_eul))
        # data_odd_trans = np.linalg.pinv(self.init_odd) @  self.odd_trans(np.linalg.inv(base_odd) @ data_odd, self.is_left) 
        data_odd_trans = self.inv_transform(self.init_odd) @  self.odd_trans(self.inv_transform(base_odd) @ data_odd, self.is_left) 
        deta_eul = rot_to_eul(data_odd_trans[:3,:3])
        for i in range(3):
            self.deta_pos_eul[i] = data_odd_trans[i,3] * self.ratio[i]
            self.deta_pos_eul[i+3] = deta_eul[i]

    def update_btn(self, data_map):
        if (data_map.get("btn")):
            self.btn = data_map.get("btn")
        if (data_map.get("r_axis")):
            self.r_axis = data_map.get("r_axis")

    def prepare_base(self, pos_eul):
        self.init_pos_eul = copy.deepcopy(pos_eul)

    def update_base(self, pos_eul):
        self.pos_eul = copy.deepcopy(pos_eul)
        # deta_eul = rot_to_eul(np.linalg.pinv(eul_to_rot(self.init_pos_eul[3:])) @ eul_to_rot(self.pos_eul[3:]))
        deta_eul = rot_to_eul(eul_to_rot(self.init_pos_eul[3:]).T @ eul_to_rot(self.pos_eul[3:]))

        for i in range(3):
            self.deta_pos_eul[i] = (self.pos_eul[i] - self.init_pos_eul[i]) * self.ratio[i]
            self.deta_pos_eul[i+3] = deta_eul[i]


    def eul_trans(self, eul):
        if self.is_left:
            rot_eul = eul_to_rot(np.array((1.5708,3.1415926,0)))
            arm_rot_new = eul_to_rot(eul) @ rot_eul
            eul = rot_to_eul(arm_rot_new)
        else:
            rot_eul = eul_to_rot(np.array((1.5708,0,0)))
            arm_rot_new = eul_to_rot(eul) @ rot_eul
            eul = rot_to_eul(arm_rot_new)
        return eul

    def odd_trans(self, odd,is_left):
        odd_trans = np.zeros((4,4))
        
        if self.config.sensor_position == "Back_of_the_hand":
            # 手背位置
            if(is_left):
                odd_trans[:3,:3] = eul_to_rot(np.array((1.5708,3.1415926, 0)))
            else:
                odd_trans[:3,:3] = eul_to_rot(np.array((1.5708,0, 0)))
        elif self.config.sensor_position == "Palm_of_the_hand":
            # 手心位置
            if(is_left):
                odd_trans[:3,:3] = eul_to_rot(np.array((-1.5708,3.1415926, 3.1415926)))
            else:
                odd_trans[:3,:3] = eul_to_rot(np.array((-1.5708,0, 3.1415926)))
        elif self.config.sensor_position == "Thumb_side":
            # 大拇指侧
            if(is_left):
                odd_trans[:3,:3] = eul_to_rot(np.array((-1.5708, 1.5708, 3.1415926)))  # 未知 x ；未知
            else:
                odd_trans[:3,:3] = eul_to_rot(np.array((-1.5708, 1.5708, 3.1415926)))
        odd_trans[3,3] = 1
        odd_new = odd @ odd_trans
        return odd_new
    
    @staticmethod
    def inv_transform(T: np.ndarray) -> np.ndarray:
        assert T.shape == (4, 4), "输入必须是4x4矩阵"
        R = T[:3, :3]
        t = T[:3, 3]
        R_inv = R.T
        t_inv = -R_inv @ t
        T_inv = np.eye(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv
        return T_inv



class mocapDataManage():
    def __init__(self, config):
        self.config = config
        if config is None:
            self.config = Config()
        self.left_wrist_ctrl = controlHandle(True, self.config)
        self.right_wrist_ctrl = controlHandle(False, self.config)
        self.left_arm_ctrl = controlHandle(True, self.config)
        self.right_arm_ctrl = controlHandle(False, self.config)
        self.chest_ctrl = controlHandle(False, self.config)
        self.hmd_ctrl = controlHandle(False, self.config)
        self.pre_count = 5

    def parse_data(self, data_dict):
        if (self.pre_count > 0):
            print("data prepare ! \n")
            self.pre_count -= 1
            return
        elif (self.pre_count == 0):
            self.chest_ctrl.prepare_base(self.pos_eul_trans(data_dict['chest']["p_e"]))
            self.base_pos_eul = self.chest_ctrl.init_pos_eul
            self.hmd_ctrl.prepare(self.pos_eul_trans(data_dict["head"]["p_e"]), self.base_pos_eul)
            self.left_arm_ctrl.prepare(self.pos_eul_trans(data_dict["l_w"]["p_e"]), self.base_pos_eul)
            self.right_arm_ctrl.prepare(self.pos_eul_trans(data_dict["r_w"]["p_e"]), self.base_pos_eul)
            self.left_wrist_ctrl.prepare(self.pos_eul_trans(data_dict["l_h"]["p_e"]), self.left_arm_ctrl.init_pos_eul_base)
            self.right_wrist_ctrl.prepare(self.pos_eul_trans(data_dict["r_h"]["p_e"]), self.right_arm_ctrl.init_pos_eul_base)
            self.pre_count -= 1
        else:
            self.chest_ctrl.update_base(self.pos_eul_trans(data_dict['chest']["p_e"]))
            self.hmd_ctrl.update(self.pos_eul_trans(data_dict["head"]["p_e"]), self.chest_ctrl.pos_eul)
            self.left_arm_ctrl.update(np.array(data_dict["l_w"]["p_e"]), self.chest_ctrl.pos_eul)
            self.right_arm_ctrl.update(self.pos_eul_trans(data_dict["r_w"]["p_e"]), self.chest_ctrl.pos_eul)
            self.left_wrist_ctrl.update(np.array(data_dict["l_h"]["p_e"]), self.left_arm_ctrl.pos_eul_base)
            self.right_wrist_ctrl.update(self.pos_eul_trans(data_dict["r_h"]["p_e"]), self.right_arm_ctrl.pos_eul_base)
            self.pre_count -= 1

    def pos_eul_trans(self, pos_eul):
        pos_eul_array = np.zeros(6)
        for i in range(6):
            pos_eul_array[i] = pos_eul[i]
        return pos_eul_array
    
    def update_button(self, data_dict):
        self.left_wrist_ctrl.update_btn(data_dict["l_h"])
        self.right_wrist_ctrl.update_btn(data_dict["r_h"])
