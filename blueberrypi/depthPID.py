import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ManualControl
import math
from std_msgs.msg import Float32


STARTING_PRESSURE = 0 #volts
G = 9.81
TARGET_DEPTH = 3.0 #relative to starting depth?
KP = 14
KI = .55
KD = 0
class PressureSensor(Node):
    def __init__(self):
        super().__init__("pressure_subscriber")

        self.pressure = None
        self.counter = 0
        self.prev_time = None
        self.current_time = None
        self.current_e = None
        self.prev_e = None
        self.current_depth = None
        self.U = 0
        self.integral_e = 0
        self.start_time = 0

        # self.pressure_sub = self.create_subscription(
        #     FluidPressure,
        #     "pressure",
        #     self.pressure_callback,
        #     10
        # )

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "/manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.sub = self.create_subscription(
            Float32,
            "target_depth",
            self.get_t_depth,
            10
        )

        self.c_depth_sub = self.create_subscription(
            Float32,
            "current_depth",
            self.get_c_depth,
            10
        )

        self.get_logger().info("publisher node initiated")



    def PID_part(self):
        # self.pressure = msg.fluid_pressure
        if (self.counter == 0):
            # global STARTING_PRESSURE
            # STARTING_PRESSURE = self.pressure
            self.counter += 1
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0] + (10 ** -9 * self.get_clock().now().seconds_nanoseconds()[1])
            self.get_logger().info(f"START TIME: {self.start_time}")
            self.current_time = self.get_clock().now().seconds_nanoseconds()[0] + (10 ** -9 * self.get_clock().now().seconds_nanoseconds()[1]) - self.start_time
            #self.current_depth = calculate_depth(self.pressure)
            self.current_e = TARGET_DEPTH - self.current_depth
            self.get_logger().info(f"Time: {self.current_time}, Depth: {self.current_depth}")
        else:
            #self.current_depth = calculate_depth(self.pressure) 
            self.prev_e = self.current_e
            self.current_e = TARGET_DEPTH - self.current_depth
            self.prev_time = self.current_time
            self.get_logger().info(f"Time: {self.prev_time} e: {self.prev_e}")
            self.current_time = self.get_clock().now().seconds_nanoseconds()[0] + (10 ** -9 * self.get_clock().now().seconds_nanoseconds()[1]) - self.start_time
            self.get_logger().info(f"Time0: {self.current_time}, e1: {self.current_e}")
            KPe = KP * self.current_e
            self.get_logger().info(f"linear one: {KPe}")
            self.integral_e += KI * (self.current_e*(self.current_time - self.prev_time))
            self.get_logger().info(f"integral: {self.integral_e}")
            de_dt = KD * (self.current_e - self.prev_e) / (self.current_time - self.prev_time)
            self.get_logger().info(f"de/dt: {de_dt}")
            self.U = -1 * (KPe + self.integral_e + de_dt)

        self.get_logger().info(str(self.U))

        self.move(0.0, 0.0, float(self.U), 0.0)

    def move(self, x, y, z, r):
            msg = ManualControl()
            msg.x = x
            msg.y = y
            msg.z = z
            msg.r = r
            self.pub.publish(msg)

    def get_c_depth(self, msg):
        self.current_depth = msg.data
        self.PID_part()
    
    def get_t_depth(self, msg):
        global TARGET_DEPTH
        TARGET_DEPTH = msg.data
    
def main(args=None):
    rclpy.init(args=args)
    pressure_node = PressureSensor()
    try:
        rclpy.spin(pressure_node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        pressure_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def calculate_depth(pressure):
    """
    Function that calculates the depth in water given a pressure
   
    @param pressure: the pressure in pascals

    @ret the depth in meters
    """

    return (pressure - STARTING_PRESSURE)/G/1000