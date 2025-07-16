# !/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
from robot.seven_planner_humanoid import get_pos_list_seven_segment
from lcm_unit import lcmUnit
import numpy as np


class P2PMotion():
    def __init__(self):
        self.lcm_unit = lcmUnit()
        self.speed = 45/180*math.pi
        self.data_list_t = None
        self.d_len = 0
        # self.target = [
        #     1.24410675e-02, 0.2618,  1.50013662e+00, -1.83325429e+00,
        #     -1.70745857e+00, -1.22576949e-06,  2.06133346e-06, -3.81118213e-01,
        #     -0.2618, -1.37148176e+00,  1.38292912e+00,  2.46545877e+00,
        #     -1.47669107e-05, -7.96941091e-06,  6.83408215e-06,  1.14110296e-06,
        #     3.05860849e-06,  6.66661872e-06,  2.43357843e-06,  1.06333822e-05,
        #     1.01175246e+01, -5.28018910e-06,  2.01167534e-06,  1.99306752e-06,
        #     7.44955598e-06, -3.34471457e-06,  1.57864069e-02, -1.04619560e-05,
        #     7.94330128e-06,  6.99263050e-06
        # ]

        self.target = [
            0.0, 0.0,  1.5707963, -1.5707963, -1.5707963, 0.0,  0.0,
            0.0, 0.0,  -1.5707963, 1.5707963, 1.5707963, 0.0,  0.0,
            6.83408215e-06,  1.14110296e-06,
            3.05860849e-06,  6.66661872e-06,  2.43357843e-06,  1.06333822e-05,
            1.01175246e+01, -5.28018910e-06,  2.01167534e-06,  1.99306752e-06,
            7.44955598e-06, -3.34471457e-06,  1.57864069e-02, -1.04619560e-05,
            7.94330128e-06,  6.99263050e-06
        ]
        self.q = [0 for i in range(30)]
        
    def get_current_pos(self):
        if self.lcm_unit.update_once_arm is True:
            self.q = self.lcm_unit.current_robot_state.q
            return True
        else:
            return False
        
    def get_t12_cmd_res(self):
        if self.lcm_unit.update_t12_once_arm is True:
            print(f"test--------------------- {self.lcm_unit.current_t12_res.a_flag} ")
            print(f"test--------------------- {self.lcm_unit.current_t12_res.b_flag} ")
            print(f"test--------------------- {self.lcm_unit.current_t12_res.c_flag} ")
            print(f"test--------------------- {self.lcm_unit.current_t12_res.d_flag} ")
            return True
        else:
            return False
    
    def get_pos_list(self, start, end):
        self.data_list_t, self.d_len = get_pos_list_seven_segment(start, end, self.speed)
        return self.data_list_t, self.d_len
        
    def reset_pose(self):
        # self.pub_act_qpos(self.qpos_numpy)

        for i in range(self.d_len):
            self.lcm_unit.send_to_robot(np.array(self.data_list_t[i]))
            time.sleep(0.02)

        num_count = 5
        for k in range(num_count):
            print(num_count-k)
            time.sleep(1)

def main():
    p2p_motion = P2PMotion()
    while(True):
        res = p2p_motion.get_t12_cmd_res()
        if res:
            print("--------------------------------ok ")
        time.sleep(1)

if __name__ == "__main__":
    main()