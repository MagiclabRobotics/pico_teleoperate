# from __init__ import *
import rclpy
from robot_kinemic.act_mocap import *
import time

time.sleep(3)


def main():
    rclpy.init()
    robot_ctrl_ros = mocapActionInfo()

        # 运行节点
    rclpy.spin(robot_ctrl_ros)
    #
    # # 销毁节点，退出ROS2
    robot_ctrl_ros.destroy_node()
    rclpy.shutdown()


if __name__ =="__main__":

    main()