import numpy as np
from kinemic import loadUrdf
from kinemic_utils import *
import matplotlib.pyplot as plt
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer, LookupException



    
    
class P5RVIZ_View(Node):
    def __init__(self):
        super().__init__("rviz_publisher")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        
        self.publisher_arm_trajectory_left = self.create_publisher(Path, 'arm_trajectory_left', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path = Path()
        self.path.header.frame_id = 'link_wy'
        # self.timer = self.create_timer(0.1, self.timer_callback_traj_left)
        self.timer_pos = self.create_timer(0.05, self.update_pose)

        # self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_state)

        # 关节名称
        self.joint_names = [
            "JOINT_HIP_ROLL_L", "JOINT_HIP_YAW_L", "JOINT_HIP_PITCH_L", "JOINT_KNEE_PITCH_L",
            "JOINT_ANKLE_PITCH_L", "JOINT_ANKLE_ROLL_L", "JOINT_HIP_ROLL_R", "JOINT_HIP_YAW_R",
            "JOINT_HIP_PITCH_R", "JOINT_KNEE_PITCH_R", "JOINT_ANKLE_PITCH_R", "JOINT_ANKLE_ROLL_R",
            "joint_wr", "joint_wy", "joint_hy", "joint_hp",  # 前 16 个关节 LCM实际顺序是  ["joint_wy","joint_wr",  "joint_hy", "joint_hp"]

            "joint_la1", "joint_la2", "joint_la3", "joint_la4", "joint_la5", "joint_la6", "joint_la7", # 0-14
            "joint_ra1", "joint_ra2", "joint_ra3", "joint_ra4", "joint_ra5", "joint_ra6", "joint_ra7",

            "left_f1", "left_f2", "left_f3", "left_f4", "left_f5", "left_f6",               # 15-26
            "right_f1", "right_f2", "right_f3", "right_f4", "right_f5", "right_f6",         # 
            "left_f1_2", "left_f2_2", "left_f3_2", "left_f4_2", "left_f5_2",                # 这些值均给0
            "right_f1_2", "right_f2_2", "right_f3_2", "right_f4_2", "right_f5_2"            # 
        ]

        self.velocity = [0.0] * len(self.joint_names)
        self.effort = [0.0] * len(self.joint_names)
    
    def timer_callback_traj_left(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('link_wy', 'link_la5', now)
            pose = PoseStamped()
            pose.header.frame_id = 'link_wy'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            self.path.poses.append(pose)
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.publisher_arm_trajectory_left.publish(self.path)
        except LookupException as e:
            self.get_logger().warn('Could not transform end_effector to world: {}'.format(str(e)))
            
    def update_pose(self):
        pass
    
    def publish_joint_state(self, left_arm):
        """
        发布 JointState 消息。
        """
        # 创建 JointState 消息
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = [
            0.0, 0.0, -0.3, 0.9, -0.6, 0.0,
            0.0, 0.0, -0.3, 0.9, -0.6, 0.0,
            # 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            # 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,            
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0
        ]

        for i in range(len(left_arm)):
            joint_state_msg.position[16 + i] = float(left_arm[i])  # 填充左手

        # 获取当前行的关节数据
        # current_positions = self.joint_positions[self.current_index]

        # 填充最后 4 个关节数据到指定位置
        # last_positions = current_positions[26:30]  # 获取当前行的最后 4 个关节值

        # position_mapping = [13, 12, 14, 15]  # 映射顺序: 27 -> 14, 28 -> 13, 29 -> 15, 30 -> 16

        # for i in range(len(last_positions)):
        #     joint_state_msg.position[position_mapping[i]] = float(last_positions[i])  # 按映射填充到指定位置


        # 填充前 14 个关节数据到位置 17–30
        # first_positions = current_positions[:14]  # 获取当前行的前 14 个关节值
        # for i in range(len(first_positions)):
        #     joint_state_msg.position[16 + i] = float(first_positions[i])  # 填充到后 17–30 个位置


        # left_finger_positions = current_positions[14:19] 
        # for i in range(len(left_finger_positions)):
        #     joint_state_msg.position[30 + i] = float(left_finger_positions[i] - np.pi)/2 
        
        # joint_state_msg.position[34] = -current_positions[18] + 0.930850804
        # joint_state_msg.position[35] = current_positions[19] + np.pi


        # right_finger_positions = current_positions[20:24] 
        # for i in range(len(right_finger_positions)):
        #     joint_state_msg.position[36 + i] = float(np.pi - right_finger_positions[i])/2 

        # joint_state_msg.position[40] = -current_positions[24] + 0.930850804
        # joint_state_msg.position[41] = - current_positions[25] + np.pi

        # joint_state_msg.position[42:47] = joint_state_msg.position[30:35]
        # joint_state_msg.position[47:52] = joint_state_msg.position[36:41]


        self.velocity = []
        self.effort = []


        # 发布消息
        self.publisher.publish(joint_state_msg)
        # self.get_logger().info(f"Published JointState message with index {self.current_index}")
        # self.current_index += 1
    

class KinemicTest(P5RVIZ_View):
    def __init__(self):
        P5RVIZ_View.__init__(self)
        self.kinemic_manage_l = loadUrdf(True)
        self.trajectory = None
        self.t = 0
        self.t_max = 0
        self.init_trajectory()
        self.mocap_info = [0,0,0]
        self.pos_eul = [0,0,0,0,0,0]
        self.act_qpos = [0,0,0,0,0,0,0]
        self.result_trajectory = []
        self.init_pos = copy.copy(self.kinemic_manage_l.init_tp)
        
        
    def period_get_data(self):
        ratio = 0.6
        next(self.kinemic_manage_l.iteration)

        self.pub_robot_info(self.kinemic_manage_l.pos.tolist() + self.kinemic_manage_l.eul.tolist())
        arm_list = [0 for _ in range(14)]
        qpos_mark_l = [1, 1, 1, 1, 1, 1, 1]
        for i in range(7):
            arm_list[i] = self.kinemic_manage_l.qpos[i] * qpos_mark_l[i]


    def update_mocap_inf(self):
        
        if self.t < self.t_max:
            x, y, z = self.trajectory[self.t]
            self.mocap_info = [x,y,z]
            self.t +=1
        else:
            self.t = 0
        
    def get_act_qpos(self):
        # inv_kinemic 进行更新
        # print(f"qpos: {self.kinemic_manage_l.qpos}")
        pass
    
    def get_pos_eul(self):
        # update_ik 进行更新
        # print(f"pos: {self.kinemic_manage_l.pos}")
        # self.result_trajectory.append(self.kinemic_manage_l.pos)
        # print(f"eul: {self.kinemic_manage_l.eul}")
        pass


    def load_rect_trajectory(self, a, n, normal_vector=[0, 0, 1], center=[0, 0, 0]):
            """
            获取三维空间中方形边界上的点坐标，每条边均匀取 n 个点。

            参数:
                a (float): 方形的边长。
                n (int): 每条边取的点数。
                normal_vector (list): 方形的法向量，默认为 [0, 0, 1]（XY 平面）。
                center (list): 方形的中心点坐标，默认为 [0, 0, 0]。

            返回:
                list: 方形边界上的点坐标列表，格式为 [(x1, y1, z1), (x2, y2, z2), ...]。
            """
            if a <= 0:
                raise ValueError("边长必须大于 0！")
            if n <= 1:
                raise ValueError("每条边的点数必须大于 1！")

            # 将法向量归一化
            normal_vector = np.array(normal_vector, dtype=float)
            normal_vector /= np.linalg.norm(normal_vector)

            # 定义方形的局部坐标系
            if np.allclose(normal_vector, [0, 0, 1]):
                # 如果法向量是 [0, 0, 1]，方形在 XY 平面
                u = np.array([1, 0, 0])
                v = np.array([0, 1, 0])
            elif np.allclose(normal_vector, [0, 1, 0]):
                # 如果法向量是 [0, 1, 0]，方形在 XZ 平面
                u = np.array([1, 0, 0])
                v = np.array([0, 0, 1])
            elif np.allclose(normal_vector, [1, 0, 0]):
                # 如果法向量是 [1, 0, 0]，方形在 YZ 平面
                u = np.array([0, 1, 0])
                v = np.array([0, 0, 1])
            else:
                # 对于任意法向量，计算局部坐标系
                u = np.array([1, 0, 0]) - normal_vector[0] * normal_vector
                u /= np.linalg.norm(u)
                v = np.cross(normal_vector, u)

            # 计算方形边界上的点
            trajectory = []

            # 上边界 (从左到右)
            for i in range(n):
                t = i * (a / (n - 1)) - a / 2
                point = center + t * u - (a / 2) * v
                trajectory.append(tuple(point))

            # 右边界 (从上到下)
            for i in range(1, n):
                t = i * (a / (n - 1)) - a / 2
                point = center + (a / 2) * u + t * v
                trajectory.append(tuple(point))

            # 下边界 (从右到左)
            for i in range(1, n):
                t = i * (a / (n - 1)) - a / 2
                point = center - t * u + (a / 2) * v
                trajectory.append(tuple(point))

            # 左边界 (从下到上)
            for i in range(1, n - 1):
                t = i * (a / (n - 1)) - a / 2
                point = center - (a / 2) * u - t * v
                trajectory.append(tuple(point))

            return trajectory
        
    

    def test_trajectory_demo(self):
        a = 0.2
        n = 10
        normal_vector = [1, 1, 1]
        center = [0, 0, 0]
        trajectory = self.load_rect_trajectory(a, n, normal_vector, center)
        x = [point[0] for point in trajectory]
        y = [point[1] for point in trajectory]
        z = [point[2] for point in trajectory]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x, y, z, marker='o', linestyle='-', color='b')
        ax.set_title("三维方形轨迹")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.show()
        
        
    def show_trajectory_map(self, trajectory, trajectory_ref=None):
        x = [point[0] for point in trajectory]
        y = [point[1] for point in trajectory]
        z = [point[2] for point in trajectory]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x, y, z, marker='o', linestyle='-', color='b')
        ax.set_title("trajectory")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        
        if trajectory_ref:
            x = [point[0] for point in trajectory_ref]
            y = [point[1] for point in trajectory_ref]
            z = [point[2] for point in trajectory_ref]
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(x, y, z, marker='o', linestyle='-', color='b')
            ax.set_title("trajectory_ref")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
        
        plt.show()
        
    def init_trajectory(self):
        a = 0.15
        n = 100
        normal_vector = [1, 1, 1]
        center = [0, 0, 0]
        trajectory = self.load_rect_trajectory(a, n, normal_vector, center)
        # x = [point[0] for point in trajectory]
        # y = [point[1] for point in trajectory]
        # z = [point[2] for point in trajectory]
        self.trajectory = trajectory
        self.t_max = len(trajectory)
        
    def demo(self):
        trajectory = []
        for i in range(1000):
            self.update_mocap_inf()
            # update_target_pos_diff 更新pos_eul
            # 输入是位置的delta
            target_pos_eul_dff = [self.mocap_info[0], self.mocap_info[1], self.mocap_info[2], 0, 0,0 ]
            self.kinemic_manage_l.update_target_pos_diff(target_pos_eul_dff)
            next(self.kinemic_manage_l.iteration)
            # self.kinemic_manage_l.inv_kinemic()
            # print(f"self.kinemic_manage_l.pos: {self.kinemic_manage_l.pos}")
            self.result_trajectory.append([self.kinemic_manage_l.pos[0], self.kinemic_manage_l.pos[1], self.kinemic_manage_l.pos[2]])
            # self.result_trajectory.append(self.mocap_info)
            
            trajectory.append([self.mocap_info[2] + self.init_pos[0], self.mocap_info[0] + self.init_pos[1], -1*self.mocap_info[1] + self.init_pos[2]])
            self.get_act_qpos()
            self.get_pos_eul()
            print(f"qpos: {self.kinemic_manage_l.qpos}")
            time.sleep(0.1)
            self.publish_joint_state(self.kinemic_manage_l.qpos)
            
    def update_pose(self):
        self.update_mocap_inf()
        # update_target_pos_diff 更新pos_eul
        # 输入是位置的delta
        target_pos_eul_dff = [self.mocap_info[0], self.mocap_info[1], self.mocap_info[2], 0, 0,0 ]
        self.kinemic_manage_l.update_target_pos_diff(target_pos_eul_dff)
        next(self.kinemic_manage_l.iteration)
        # self.kinemic_manage_l.inv_kinemic()
        # print(f"self.kinemic_manage_l.pos: {self.kinemic_manage_l.pos}")
        # self.result_trajectory.append([self.kinemic_manage_l.pos[0], self.kinemic_manage_l.pos[1], self.kinemic_manage_l.pos[2]])
        # self.result_trajectory.append(self.mocap_info)
        
        # trajectory.append([self.mocap_info[2] + self.init_pos[0], self.mocap_info[0] + self.init_pos[1], -1*self.mocap_info[1] + self.init_pos[2]])
        self.get_act_qpos()
        self.get_pos_eul()
        print(f"qpos: {self.kinemic_manage_l.qpos}")
        self.publish_joint_state(self.kinemic_manage_l.qpos)
        # self.show_trajectory_map(self.result_trajectory, trajectory)
        # self.show_trajectory_map(self.trajectory)


    

    
    
    
if __name__ == "__main__":
    # handler = KinemicTest()
    # handler.test_trajectory_demo()
    # handler.demo()
    
    rclpy.init()
    handler = KinemicTest()
    # handler.demo()
        # 运行节点
    rclpy.spin(handler)
    #
    # # 销毁节点，退出ROS2
    handler.destroy_node()
    rclpy.shutdown()