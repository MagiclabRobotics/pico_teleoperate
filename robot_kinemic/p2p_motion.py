
import math
import time
import numpy as np
import copy
import sys
from robot_kinemic.lcm_unit import lcmUnit

class P2PMotion():
    def __init__(self, lcm_unit):
        self.lcm_unit = lcm_unit
        self.speed = 45/180*math.pi
        self.data_list_t = None
        self.d_len = 0

        self.dim = 30
        self.q = [0 for i in range(self.dim)]
        self.is_get_q = False

        self.is_ready = [False for _ in range(self.dim)]
        self.init_ready()
    
    def init_ready(self):
        for i in range(14,self.dim):
            self.is_ready[i] = True
        
    def get_current_pos(self):
        while not  self.lcm_unit.update_once_arm:
            time.sleep(0.1)
            print(f"get lcm position")
        self.q = self.lcm_unit.current_robot_state.q
        self.is_get_q = True

        
    
    def start_p2p_move(self):
        self.get_current_pos()
        self.ratio = 0.1

    def update_qpos(self, target_qpos):
        if(not self.is_get_q):
            print("Could not get qpos ,exit!\n")
            sys.exit()
            
            
        gap = 0.001
        for i in range(self.dim):
            if(not self.is_ready[i]):
                if(abs(self.q[i]-target_qpos[i])<gap):
                    print(f"---------joint {i} READY--------\n")
                    self.is_ready[i] = True
                elif(self.q[i]-target_qpos[i]>0):
                    self.q[i] -= self.ratio*gap 
                else:
                    self.q[i] += self.ratio*gap
                target_qpos[i] = self.q[i]

        self.ratio += 0.01
        self.ratio = min(self.ratio, 1)

        return target_qpos





        
        

  
if __name__ == "__main__":
    main()
