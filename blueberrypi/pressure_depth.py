import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
import math
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32

STARTING_PRESSURE = 0.0
G = 9.81
class SendDepth(Node):
    def __init__(self):
        super().__init__("tutorial_subscriber")    # names the node when running

        self.meow = False
        self.depth = 0.0

        

        self.pressure_sub = self.create_subscription(
            FluidPressure,
            "pressure",
            self.pressure_callback,
            10
        )

        self.c_depth_pub = self.create_publisher(
            Float32,        # the message type
            "current_depth",    # the topic name
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized subscriber node")

    def pressure_callback(self, msg):
        if(self.meow == False):
            self.meow = True
            global STARTING_PRESSURE
            STARTING_PRESSURE = msg.fluid_pressure
        self.depth = float(calculate_depth(msg.fluid_pressure))
        msg1 = Float32()
        msg1.data = float(self.depth)
        self.c_depth_pub.publish(msg1)
        





def main(args=None):
    rclpy.init(args=args)
    node = SendDepth()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def calculate_depth(pressure):
    """
    Function that calculates the depth in water given a pressure
   
    @param pressure: the pressure in pascals

    @ret the depth in meters
    """

    return (pressure - STARTING_PRESSURE)/G/1000