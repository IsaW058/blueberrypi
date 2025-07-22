import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from mavros_msgs.msg import ManualControl    # the ManualControl message type definition
import numpy as np

TIMER_PERIOD = 1.0
MOVEMENT_VECTOR = np.array([-80.0, 0.0])

class Move(Node):
    def __init__(self):
        super().__init__("translational")    # names the node when running

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "translational_movement",    # the topic name
            10              # QOS (will be covered later)
        )
 
        self.timer = self.create_timer(
            TIMER_PERIOD,    # timer period (sec)
            self.publish_movement    # callback function
        )

        self.get_logger().info("initialized publisher node")

    def publish_movement(self):
        self.msg = ManualControl()
        self.msg.x = MOVEMENT_VECTOR[0]
        self.msg.y = MOVEMENT_VECTOR[1]
        self.pub.publish(self.msg)
    
    

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