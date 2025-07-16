# from __init__ import *
import rclpy
from robot_kinemic.act_mocap import *
import time

time.sleep(3)


def main():
    rclpy.init()
    with pkg_resources.path("robot_kinemic.config", "config.ini") as config_path:
        launch_config_path = config_path
    robot_ctrl_ros = mocapActionInfo(launch_config_path)

        # 运行节点
    rclpy.spin(robot_ctrl_ros)
    #
    # # 销毁节点，退出ROS2
    robot_ctrl_ros.destroy_node()
    rclpy.shutdown()


if __name__ =="__main__":

    main()