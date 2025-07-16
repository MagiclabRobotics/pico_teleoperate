import lcm
import time
import os, sys

# # 获取当前工作目录
# pwd_path = os.getcwd()
# # 往前后退 2 级
# # moca_path = os.path.dirname(os.path.dirname(pwd_path))
# moca_path = os.path.dirname(pwd_path)
# sys.path.append(moca_path)
# print(f"moca_path: {moca_path}")

from robot_kinemic.lcm_response_lcmt import lcm_response_lcmt
from robot_kinemic.lcm_command_struct import lcm_command_struct


class RobotFSMStateSet():
    def __init__(self) -> None:
        self.lcm = lcm.LCM('udpm://239.255.76.67:7667?ttl=1')
    
    def convert_to_lcm_com_cmd_package_msg(self, control_mode):
        lcm_command_msg = lcm_command_struct()
        lcm_command_msg.robot_fsm = control_mode
        lcm_command_msg.x_vel_des = 0
        lcm_command_msg.y_vel_des = 0
        lcm_command_msg.yaw_vel_des = 0
        lcm_command_msg.stop = 0
        return lcm_command_msg
    
    def set_state(self, state):
        cmd_msg = self.convert_to_lcm_com_cmd_package_msg(state)
        self.lcm.publish('lcm_com_cmd', cmd_msg.encode())

def run(state_name):
    r_lcm_handler = RobotFSMStateSet()
    while(True):
        r_lcm_handler.set_state(state_name)
        time.sleep(1)
        print("dida")

if __name__ == "__main__":
    state_name = 45                                     # set the state here
    run(state_name)
