import copy
import sys
import numpy as np
import time

from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, Int32
from humanoid_msgs.action import MocapCmd


class robotControlRos(Node):
    def __init__(self):
        super().__init__('ROBOT_CONTROL')

        self.publisher_qpos = self.create_publisher(Float32MultiArray, '/act_qpos', 10)
        self.timer_period_get_data = self.create_timer(0.01, self.period_get_data)
        self.timer_update_mocap_inf = self.create_timer(0.02, self.update_mocap_inf)
        self.publisher_start_save_data = self.create_publisher(Int32, '/start_save_data', 10)
        self.publisher_change_task_mode = self.create_publisher(Int32, '/task_mode', 10)
        self.publisher_mocap_info  = self.create_publisher(Float32MultiArray, '/mocap_info', 10)
        self.publisher_robot_info  = self.create_publisher(Float32MultiArray, '/robot_info', 10)
        
        
        self.mocap_running = False          # 摇操启动开关
        
        self.teleop_switch = False          # PICO 连接开关
        self.teleop_connect_state = False   # PICO 连接状态
        
        self.period_get_data_switch = False
        self.update_mocap_inf_switch = False
        
        self.mocap_runing_err = False
        self.mocap_running_string = ""
        
        self.to_p2p = False
        self.to_ready_pos = False
        self.to_recoverstand_pos = False
        
        
        self._mocap_action_server = ActionServer(
            self,
            MocapCmd,
            'mocap_action',
            self.execute_callback)

    def pub_act_qpos(self, qpos):
        qpos_pub = Float32MultiArray(data=qpos)
        self.publisher_qpos.publish(qpos_pub)
        # self.get_logger().info('Publishing: "%s"' % qpos_pub)


    def pub_mocap_info(self, mocap_info):
        mocap_info_pub = Float32MultiArray(data=mocap_info)
        self.publisher_mocap_info.publish(mocap_info_pub)



    def pub_robot_info(self, robot_info):
        robot_info_pub = Float32MultiArray(data=robot_info)
        self.publisher_robot_info.publish(robot_info_pub)

        
    def period_get_data(self):
        pass

    def update_mocap_inf(self):
        pass

    def start_save_data(self, data):
        self.get_logger().info('Publishing : start save data\n' )
        data = Int32(data=data)
        self.publisher_start_save_data.publish(data)
        
    def start_mocap(self, handle):
        pass
    
    def stop_mocap(self, handle):
        pass
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # 获取goal中的key
        goal_key = goal_handle.request.cmd
        if goal_key == 1:
            self.start_mocap(goal_handle)
        elif goal_key == 2:
            self.stop_mocap(goal_handle)
        
        # 发送反馈
        # feedback_msg = MocapCmd.Feedback()
        # feedback_msg.key = goal_key
        # for i in range(30):
        #     feedback_msg.feedback = f"Processing {i+1}/30 for key {goal_key}"
        #     self.get_logger().info('Feedback: {0}'.format(feedback_msg.feedback))
        #     goal_handle.publish_feedback(feedback_msg)
        #     time.sleep(1)  # 模拟处理时间

        # 创建结果
        result = MocapCmd.Result()
        result.cmd = goal_key
        result.description = f"Processed value for {goal_key}"

        self.get_logger().info('Returning result...')
        return result
    

# if __name__ == '__main__':
#     main()
