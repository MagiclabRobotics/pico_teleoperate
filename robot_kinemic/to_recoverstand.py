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

GO_HOME_READY = [
    0.03, 0.52, 1.55, -1.5, -1.55, 0.0, -0.0, 
    0.03, -0.53, -1.55, 1.5, 1.55, 0.0, -0.0, 
    3.07, 3.08, 3.08, 3.08, 0.92, 2.72, 
    3.08, 3.07, 3.08, 3.07, 0.93, 2.74, 
    0.01, 0.0, 
    -0.0, -0.0]

def to_recoverstand():
    p2p_motion = P2PMotion()
    while(True):
        res = p2p_motion.get_current_pos()
        print(f"res: {res}")
        if res:
            '''
            回位
            '''
            # act_p2p(p2p_motion, p2p_motion.q, GO_HOME_READY)
            # print(f"go home")
            
            
            '''
            把手收回
            '''
            tt_pre = copy.copy(p2p_motion.q)
            res = p2p_motion.get_current_pos()
            tt = copy.copy(p2p_motion.q)
            tt[0] = 1
            tt[7] = 1
            tt[1] = 1
            tt[8] = -1
            # tt[3] = 0
            # tt[10] = 0
            
            
            tt_pre = copy.copy(p2p_motion.q)
            res = p2p_motion.get_current_pos()
            tt = copy.copy(p2p_motion.q)
            tt[0] = 1
            tt[7] = 1
            # tt[1] = 0.33
            # tt[8] = -0.33
            tt[1] = 1
            tt[8] = -1
            
            tt[3] = -1.7
            tt[10] = 1.7
            
            
            # tt[0] = 0
            # tt[7] = 0
            # tt[3] = 0
            # tt[10] = 0
            
            formatted_numbers = [round(num, 2) for num in tt]
            print(f"p2p_motion.q: {formatted_numbers}")
            act_p2p(p2p_motion, tt_pre, tt)
            
            '''
            把肘子放下去
            '''
            tt_pre = copy.copy(tt)
            res = p2p_motion.get_current_pos()
            tt = copy.copy(p2p_motion.q)
            tt[0] = 0
            tt[7] = 0
            tt[3] = 0
            tt[10] = 0
            formatted_numbers = [round(num, 2) for num in tt]
            print(f"p2p_motion.q: {formatted_numbers}")
            act_p2p(p2p_motion, tt_pre, tt)
            '''
            把关节收回去
            '''
            tt_pre = copy.copy(tt)
            res = p2p_motion.get_current_pos()
            tt = copy.copy(p2p_motion.q)
            tt[0] = 0.03
            tt[7] = 0.03
            tt[2] = 0
            tt[4] = 0
            tt[9] = 0
            tt[11] = 0
            
            formatted_numbers = [round(num, 2) for num in tt]
            print(f"p2p_motion.q: {formatted_numbers}")
            act_p2p(p2p_motion, tt_pre, tt)
            '''
            到R
            '''
            tt_pre = copy.copy(tt)
            res = p2p_motion.get_current_pos()
            
            formatted_numbers = [round(num, 2) for num in RECOVERPOS]
            print(f"p2p_motion.q: {formatted_numbers}")
            act_p2p(p2p_motion, tt_pre, RECOVERPOS)
            break
        time.sleep(1)
    p2p_motion.close()
    time.sleep(1)

if __name__ == "__main__":
    to_recoverstand()