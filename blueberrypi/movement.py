import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from mavros_msgs.msg import ManualControl    # the ManualControl message type definition
import numpy as np

TIMER_PERIOD = 1.0
MOVEMENT_VECTOR = np.array([0.0, 0.0, 0.0, 0.0])

class Move(Node):
    def __init__(self):
        super().__init__("movement")    # names the node when running

        self.final_msg = ManualControl()

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.depth_movement = self.create_subscription(
            ManualControl,
            "depth_movement",
            self.add_depth,
            10
        )

        self.heading_movement = self.create_subscription(
            ManualControl,
            "heading_movement",
            self.add_rotation,
            10
        )



        self.translation_movement = self.create_subscription(
            ManualControl,
            "translational_movement",
            self.add_translational,
            10
        )
            

        self.get_logger().info("initialized publisher node")

    def add_depth(self, msg):
        self.final_msg.z = msg.z
        self.publish_movement()
    
    def add_rotation(self, msg):
        self.final_msg.r = msg.r
        self.publish_movement()
    
    def add_translational(self, msg):
        self.final_msg.x = msg.x
        self.final_msg.y = msg.y
        self.publish_movement()
    
    
    def publish_movement(self):
        self.pub.publish(self.final_msg)
    
    

def main(args=None):
    rclpy.init(args=args)
    node = Move()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()