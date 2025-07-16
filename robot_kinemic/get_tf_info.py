from kinemic_utils import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer, LookupException



    
    
class TFPathView(Node):
    def __init__(self):
        super().__init__("rviz_publisher")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        
        self.publisher_arm_trajectory_left = self.create_publisher(Path, 'arm_trajectory_left', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path = Path()
        self.path.header.frame_id = 'link_wy'
        self.timer = self.create_timer(0.1, self.timer_callback_traj_left)
    
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
            
   
    

    
    
    
if __name__ == "__main__":
    # handler = KinemicTest()
    # handler.test_trajectory_demo()
    # handler.demo()
    
    rclpy.init()
    handler = TFPathView()
    # handler.demo()
        # 运行节点
    rclpy.spin(handler)
    #
    # # 销毁节点，退出ROS2
    handler.destroy_node()
    rclpy.shutdown()