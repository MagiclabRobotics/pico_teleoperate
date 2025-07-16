# !/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import numpy as np
import copy

from robot_kinemic.lcm_unit_gripper import P2PMotion


def act_p2p(handler, start, end):
    data_list_t, d_len = handler.get_pos_list(start, end)
    handler.reset_pose()


RECOVERPOS =  [
    -0.04, 0.12, -0.0, -0.20, 0.03, 0.0, 0.0, 
    -0.04, -0.12, -0.0, 0.20, -0.03, 0.0, 0.0, 
    3.08, 3.08, 3.08, 3.08, 0.93, 2.86, 
    3.08, 3.07, 3.08, 3.07, 0.93, 2.87, 
    0.01, 0.0, -0.0, 0.0]

def test_tt():
    print("hello test")

def to_ready():
    p2p_motion = P2PMotion()
    while(True):
        res = p2p_motion.get_current_pos()
        if res:
            '''
            回位
            '''
            # act_p2p(p2p_motion, p2p_motion.q, RECOVERPOS)
            print(f"go ready")
            
            res = p2p_motion.get_current_pos()
            tt = copy.copy(p2p_motion.q)
            tt[1] = 0.3
            tt[8] = -0.3
            tt[5] = 0.52
            tt[12] = -0.52
            
            formatted_numbers = [round(num, 2) for num in tt]
            print(f"p2p_motion.q: {formatted_numbers}")
            act_p2p(p2p_motion, p2p_motion.q, tt)
            
            
            '''
            转 3 5关节, 保证4关节超前
            '''
            tt_pre = copy.copy(tt)
            res = p2p_motion.get_current_pos()
            tt = copy.copy(tt_pre)
            tt[0] = 0.2
            tt[7] = 0.2
            tt[2] = 1.5707963
            tt[4] = -1.5707963
            tt[9] = -1.5707963
            tt[11] = 1.5707963
            tt[5] = 0
            tt[6] = 0
            tt[12] = 0
            
            formatted_numbers = [round(num, 2) for num in tt]
            print(f"p2p_motion.q: {formatted_numbers}")
            act_p2p(p2p_motion, tt_pre, tt)
            
            '''
            后拉 抬手
            0关节往后
            4关节往前 实际显示向地面
            '''
            tt_pre = copy.copy(tt)
            res = p2p_motion.get_current_pos()
            tt = copy.copy(tt_pre)
            tt[0] = 1
            tt[7] = 1
            tt[1] = 0.5
            tt[8] = -0.5
            tt[3] = -1.7
            tt[10] = 1.7
            tt[12] = 0
            
            formatted_numbers = [round(num, 2) for num in tt]
            print(f"p2p_motion.q: {formatted_numbers}")
            act_p2p(p2p_motion, tt_pre, tt)
            
            '''
            手往前推 
            
            
            0关节往后
            4关节往前 实际显示向地面
            '''
            tt_pre = copy.copy(tt)
            res = p2p_motion.get_current_pos()
            tt = copy.copy(tt_pre)
            tt[0] = 0
            tt[7] = 0
            tt[3] = -1.5
            tt[10] = 1.5
            tt[12] = 0
            # tt[29] = 0.34
            
            tt[5] = 0.52
            tt[12] = -0.52
            formatted_numbers = [round(num, 2) for num in tt]
            print(f"p2p_motion.q: {formatted_numbers}")
            act_p2p(p2p_motion, tt_pre, tt)
            
            '''
            抱箱子
            '''
            # init_pos = [
            #     0.0, 0.5,  1.7,-1.2, -1.57, 0.0, -0.0, 
            #     0.0, -0.5, -1.7, 1.2, 1.57, 0.0, -0.0, 
            #     3.07, 3.08, 3.08, 3.08, 0.92, 2.81, 
            #     3.07, 3.08, 3.08, 3.08, 0.92, 2.83, 
            #     0.01, -0.01, 0.0, 0.0]
            # act_p2p(p2p_motion, tt, init_pos)
            
            break
        time.sleep(1)
    p2p_motion.close()
    time.sleep(1)

if __name__ == "__main__":
    to_ready()