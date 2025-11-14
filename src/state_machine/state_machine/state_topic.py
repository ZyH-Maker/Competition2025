import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from example_interfaces.msg import Float32MultiArray
from std_msgs.msg import Float32


class ArmPosePub(Node):
    '''
    向上位机发送三个目标物料位置的节点
    消息为一个数组，含有三个表示位置的子数组
    '''
    def __init__(self, pose_array):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'arm_pose', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        pose1 = Pose()
        pose1.position.x = pose_array[0][0]
        pose1.position.y = pose_array[0][1]
        pose1.position.z = pose_array[0][2]
        pose1.orientation.w = 1.0

        pose2 = Pose()
        pose2.position.x = pose_array[1][0]
        pose2.position.y = pose_array[1][1]
        pose2.position.z = pose_array[1][2]
        pose2.orientation.w = 1.0

        pose3 = Pose()
        pose3.position.x = pose_array[2][0]
        pose3.position.y = pose_array[2][1]
        pose3.position.z = pose_array[2][2]
        pose3.orientation.w = 1.0

        self.pose_array_msg = PoseArray()
        self.pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_array_msg.header.frame_id = 'world'
        self.pose_array_msg.poses = [pose1, pose2, pose3]

    def timer_callback(self):
        self.publisher_.publish(self.pose_array_msg)
        poses_str = ', '.join([f'x: {p.position.x}, y: {p.position.y}, z: {p.position.z}' for p in self.pose_array_msg.poses])
        self.get_logger().info(f'Published PoseArray with 3 poses: {poses_str}.')


class ChassisOffsetPub(Node):
    '''
    向下位机发送底盘纠偏数据的节点
    消息为表示相机识别得到底盘误差的三维数组
    '''
    def __init__(self, offset_error):
        super().__init__('chassis_offset_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'camera_correction_data', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.float_array_msg = Float32MultiArray()
        self.float_array_msg.data = offset_error

    def timer_callback(self):
        self.publisher_.publish(self.float_array_msg)
        array_str = ', '.join([str(f) for f in self.float_array_msg.data])
        self.get_logger().info(f'Published Float32MultiArray: {array_str}.')


class ChassisMovePub(Node):
    '''
    向下位机发送浮点数 发布底盘移动指令 
    '''
    def __init__(self, job):
        super().__init__('chassis_move_publisher')
        self.publisher_ = self.create_publisher(Float32, 'set_path_joint', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.data_to_publish = Float32()
        self.data_to_publish.data = job

    def timer_callback(self):
        self.publisher_.publish(self.data_to_publish)
        self.get_logger().info('Publishing: "%f"' % self.data_to_publish.data)


class FinishSub(Node):
    '''
    接收下位机完成指令的节点
    收到一个Float32数据即代表完成
    '''
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('end_param', 0.0)
        self.sub = self.create_subscription(Float32, "finish", self.listener_callback, 0)

    def listener_callback(self, msg):
        return


def offsetPub(pos_error):
    node_offset_pub = ChassisOffsetPub(pos_error)
    rclpy.spin_once(node_offset_pub)
    node_offset_pub.destroy_node()
    return

def chassisMovePub(job):
    node_offset_pub = ChassisMovePub(job)
    rclpy.spin_once(node_offset_pub)
    node_offset_pub.destroy_node()
    return

def finishSub():
    finish_node = FinishSub("finish_node")
    rclpy.spin_once(finish_node)
    finish_node.destroy_node()
    return







