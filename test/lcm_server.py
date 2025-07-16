from lcm import LCM

import numpy as np
import threading
from robot_kinemic.upper_body_cmd_package import upper_body_cmd_package
from robot_kinemic.upper_body_data_package import upper_body_data_package

class lcmUnit(LCM):
    def __init__(self) -> None:
        super().__init__('udpm://239.255.76.67:7669?ttl=1')
        self.upper_body_cmd_topic   = 'upper_body_cmd'
        self.upper_body_data_topic  = 'upper_body_data'

        self.subscribe(self.upper_body_cmd_topic, self.upper_body_data_listener_cb)



    def upper_body_data_listener_cb(self, channel, data):
        '''sensor readings are stored in self.current_robot_state'''

        msg = upper_body_cmd_package.decode(data)
        print(f"get channel : {channel} ,  data:{msg.jointPosVec}")
        return 


    def lcm_handle(self):
        while True:
            self.handle()


    def send_to_robot(self,data):
        upper_body_cmd_msg = self.load_upper_body_cmd_package(data)
        self.publish(self.upper_body_cmd_topic, upper_body_cmd_msg.encode())
        print("send data\n")


    def load_upper_body_cmd_package(self, robot_30dof_solution):
        upper_body_cmd_msg  = upper_body_cmd_package()
        upper_body_cmd_msg.isUsed          = 0
        upper_body_cmd_msg.control_mode    = (5*np.ones(7).astype(int)).tolist() + \
                                             (5*np.ones(7).astype(int)).tolist() + \
                                             (4*np.ones(12).astype(int)).tolist() + \
                                             (200*np.ones(2).astype(int)).tolist() + \
                                             (200*np.ones(2).astype(int)).tolist()

        upper_body_cmd_msg.jointPosVec     = robot_30dof_solution
        upper_body_cmd_msg.jointSpeedVec   = np.zeros_like(robot_30dof_solution).tolist()
        upper_body_cmd_msg.jointKp = np.zeros_like(robot_30dof_solution).tolist()
        upper_body_cmd_msg.jointKd = np.zeros_like(robot_30dof_solution).tolist()

        # if self.pre_pose is None:
        #     upper_body_cmd_msg.jointSpeedVec    = np.zeros_like(robot_30dof_solution).tolist()
        #     upper_body_cmd_msg.jointCurrentVec  = np.zeros_like(robot_30dof_solution).tolist()    
        # else:
        #     speed = robot_30dof_solution-self.pre_pose
        #     speed*=50
        #     upper_body_cmd_msg.jointSpeedVec    = speed.tolist()
        #     upper_body_cmd_msg.jointCurrentVec  = np.zeros_like(robot_30dof_solution).tolist()
        

        upper_body_cmd_msg.jointSpeedVec    = np.zeros_like(robot_30dof_solution).tolist()
        upper_body_cmd_msg.jointCurrentVec  = np.zeros_like(robot_30dof_solution).tolist()  

        upper_body_cmd_msg.jointCurrentVec = np.zeros_like(robot_30dof_solution).tolist()

        upper_body_cmd_msg.control_mode[14:20] = (20*np.ones(6).astype(int)).tolist()
        upper_body_cmd_msg.control_mode[20:26] = (20*np.ones(6).astype(int)).tolist()
        return upper_body_cmd_msg



if __name__ =="__main__":
    s = lcmUnit()
    while True:
        s.handle()
