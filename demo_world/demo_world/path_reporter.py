from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        #self.subscription = self.create_subscription(
        #    Odometry,
        #    '/odom',
        #    self.odom_callback,
        #    10)
        #self.goal_position = self.create_subscription(
        #    PoseStamped,
        #    'target_pose',
        #    self.on_command,
        #)
        self.pub = self.create_publisher(String, '/registration', 10)
        #self.subscription  # prevent unused variable warning
        self.get_logger().info('Pose subscriber node started')
        self.timer = self.create_timer(1.0, self.heartbeat)

    def heartbeat(self):
        ns = self.get_namespace()
        self.pub.publish(String(data=ns))

    def odom_callback(self, msg):
        # Extracting position (x, y, z)
        position = msg.pose.pose.position
        # Extracting orientation (quaternion: x, y, z, w)
        orientation = msg.pose.pose.orientation
        self.get_logger().info(
            f"Robot Pose: Position (x: {position.x:.2f}, y: {position.y:.2f}, z: {position.z:.2f}), "
            f"Orientation (x: {orientation.x:.2f}, y: {orientation.y:.2f}, z: {orientation.z:.2f}, w: {orientation.w:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = RobotController()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

