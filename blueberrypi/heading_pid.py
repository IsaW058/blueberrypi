import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Int16
from mavros_msgs.msg import ManualControl

# first make a subscription to take initial pressure
# make a counter that says the first time it recieves smth logs as initial, then pass into pid the rest of them tht
#current time from ros clock - last time
target_heading = 250
G = 9.81
Kp = 3
Ki = 0.0
Kd = .5

#Previous
# Kp = .4
# Ki = .02
# Kd = .05
class calculate_heading_pid(Node):
    def __init__(self):
        super().__init__("heading_pid")    # names the node when running
        self.heading_sub = self.create_subscription(Int16, "/heading", self.heading_callback, 10)
        self.pub = self.create_publisher(ManualControl, "heading_movement", 10)
        self.target_heading_sub = self.create_subscription(Int16, "/target_heading", self.set_target_heading, 10)

        self.get_logger().info("initialized subscriber node")
        self.heading_initial = None
        self.heading_current = None
        self.heading_previous = None
        self.time_current = None
        self.time_previous = None
        self.flag = False
        self.u = None
        self.error_accumulator = 0.0
        self.previous_error = 0.0


    def move(self, x, y, z, r):
        msg = ManualControl()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.r = r
        self.pub.publish(msg)

    def set_target_heading(self, msg):
        global target_heading
        target_heading = msg.data

    def heading_callback(self, msg):

        if self.flag == False:
            self.flag = True
            self.heading_initial = msg.data
            self.time_current = self.get_clock().now().seconds_nanoseconds()[0] + (self.get_clock().now().seconds_nanoseconds()[1] * 10**-9)

        self.heading_previous = self.heading_current
        self.heading_current = msg.data

        self.time_previous = self.time_current
        self.time_current = self.get_clock().now().seconds_nanoseconds()[0] + (self.get_clock().now().seconds_nanoseconds()[1] * 10**-9)
        #self.get_logger().info(f"current pressure: {self.pressure_current}")
        #self.get_logger().info(f"previous pressure: {self.pressure_previous}")
        self.calculate_pid()

    def calculate_pid(self):
        if self.heading_previous is None:
            self.heading_previous = self.heading_current  # now safe to proceed next time
            return

        error = abs(target_heading-self.heading_current)

        if (abs(target_heading-self.heading_current) <= 180):
            if target_heading-self.heading_current > 0: #turn counterclockwise by that much
                x = True
            else:
                x = False
        else:
            error = 360 - error
            if target_heading-self.heading_current > 0:
                x = False
            else:
                x = True

        previous_error = abs(target_heading-self.heading_previous)

        #PROPORTIONAL
        self.get_logger().info(f"error: {target_heading-self.heading_current}")
        proportional = Kp * error
        #INTEGRAL
        self.error_accumulator += error * (self.time_current-self.time_previous)# dt is the time since the last update
        integral = Ki * self.error_accumulator
        integral = min(Ki * self.error_accumulator, 1.0)
        #DERIVATIVE
        derivative = Kd * (error - previous_error) / (self.time_current-self.time_previous) # dt is the time since the last update
        previous_error = error

        u = proportional+integral+derivative
        self.get_logger().info(f"velocity: {u}")
        self.get_logger().info(f"heading: {self.heading_current}")

        # u = max(u, 65.0)

        if x==True:
            u = u
        else:
            u = -u

        self.get_logger().info(f"p: {proportional}, i: {integral}, d: {derivative}, u: {u}")

        self.move(0.0, 0.0, 0.0, u)




def main(args=None):
    rclpy.init(args=args)
    node = calculate_heading_pid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()