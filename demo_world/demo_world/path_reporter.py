from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.costmap_2d import PyCostmap2D
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
import time
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
import numpy as np


def costmap_to_occupancy_grid(cost_msg: Costmap) -> OccupancyGrid:
    """
    Converts a nav2_msgs/Costmap message to a nav_msgs/OccupancyGrid message.
    
    :param cost_msg: The incoming nav2_msgs/msg/Costmap message
    :return: A standard nav_msgs/msg/OccupancyGrid message
    """
    grid = OccupancyGrid()

    # 1. Transfer Header
    grid.header = cost_msg.header

    # 2. Transfer Metadata
    # Nav2 uses 'size_x/y', OccupancyGrid uses 'width/height'
    grid.info.resolution = cost_msg.metadata.resolution
    grid.info.width = cost_msg.metadata.size_x
    grid.info.height = cost_msg.metadata.size_y
    grid.info.origin = cost_msg.metadata.origin
    
    # Nav2 CostmapMetaData has 'map_load_time' and 'update_time'
    # We typically use map_load_time for the static map info
    grid.info.map_load_time = cost_msg.metadata.map_load_time

    # 3. Convert Data
    # Nav2 Costmap (uint8): 0 (Free) ... 254 (Lethal) ... 255 (Unknown)
    # OccupancyGrid (int8): 0 (Free) ... 100 (Lethal) ... -1 (Unknown)
    
    # We use numpy for efficiency as maps can be large
    raw_data = np.array(cost_msg.data, dtype=np.uint8)
    
    # Initialize output array with 0 (Free)
    occ_data = np.zeros_like(raw_data, dtype=np.int8)

    # Logic:
    # 255 (NO_INFORMATION) -> -1
    # 254 (LETHAL_OBSTACLE) -> 100
    # 0-253 -> Scaled to 0-99
    
    # Create masks
    mask_unknown = (raw_data == 255)
    mask_lethal = (raw_data == 254)
    mask_cost = (raw_data < 254)

    # Apply conversions
    # Scale: value * 100 / 254 roughly maps 0-253 to 0-99
    # We use floating point division then cast to int
    occ_data[mask_cost] = (raw_data[mask_cost].astype(np.float32) * 100.0 / 254.0).astype(np.int8)
    
    occ_data[mask_lethal] = 100
    occ_data[mask_unknown] = -1

    # Flatten and convert to list for the message
    grid.data = occ_data.tolist()

    return grid


"""
This is in charge of telling where the robot should head next.
It works by registering the robot to the /registration topic
and then it 
"""
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.registration_pub = self.create_publisher(String, '/registration', 10)
        #self.subscription  # prevent unused variable warning
        self.get_logger().info('Pose subscriber node started')
        self.timer = self.create_timer(1.0, self.heartbeat)
        self.nav = BasicNavigator(namespace=self.get_namespace())
        #self.nav.waitUntilNav2Active()
        self.get_logger().info("NAV2 now ACTIVE!")
        qos_profile = QoSProfile(
            depth=1, # Keep last 10 messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.reentrant_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(Point, 'global_costmap/next_goal', self.next_goal_recv, qos_profile, callback_group=self.reentrant_group)
        self.prev_point = None
        self.sub

    def heartbeat(self):
        ns = self.get_namespace()
        self.registration_pub.publish(String(data=ns))

    def next_goal_recv(self, point: Point):
        self.get_logger().info(f"Received next goal as {point}")
        gcp = PyCostmap2D(costmap_to_occupancy_grid(self.nav.getGlobalCostmap()))
        #print(f"{gcp}")
        lcp = PyCostmap2D(costmap_to_occupancy_grid(self.nav.getLocalCostmap()))
        #print(lcp.header)
        gx, gy = gcp.worldToMapValidated(point.x, point.y)
        if self.prev_point is None:
            self.get_logger().info("Setting new nav2 goal")
            pose = PoseStamped()
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = "map"
            
            self.nav.goToPose(pose)
            self.prev_point = point
            return
        dist = (self.prev_point.x - point.x)**2 + (self.prev_point.y - point.y)**2

        if dist < 1:
            self.get_logger().info("Prev goal near current goal, doing nothing")
            return

        self.get_logger().info("Setting new nav2 goal")
        pose = PoseStamped()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.w = 1.0
        pose.header.frame_id = "map"
        self.nav.goToPose(pose)
        self.prev_point = point
        return

def main(args=None):
    time.sleep(30)
    rclpy.init(args=args)
    pose_subscriber = RobotController()
    # cant use waitUntilNav2Active cause it sets the wrong pose
    #pose_subscriber.nav.waitUntilNav2Active()
    executor = MultiThreadedExecutor()
    executor.add_node(pose_subscriber)
    executor.spin()
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

