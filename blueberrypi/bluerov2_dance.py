import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_srvs.srv import SetBool

class Move(Node):

    def __init__(self):
        super().__init__("move")    # names the node when running
        self.pub = self.create_publisher(
        ManualControl,        # the message type
        "/manual_control",    # the topic name
        10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized publisher node")

    def move(self, x, y, z, r):
        msg = ManualControl()
        # msg = ManualControl(x=x,y=y,z=z,r=r)
        msg.x = x
        msg.y = y
        msg.z = z
        msg.r = r
        self.pub.publish(msg)

    def dance(self):
        self.start_time_sec = self.get_clock().now().seconds_nanoseconds()[0] # current seconds (seconds, nanoseconds)
        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 4 and rclpy.ok():
            self.move(20.0, 0.0, 0.0, 0.0) #leave spawn to center for 8 seconds, x, y, z, r

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
        #    if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 0):
               self.move(20.0, 0.0, 5.0, 0.0) #move upwards and slightly forward at the same time “And now your song is on repeat”
               self.get_logger().info("yayayayya ")
        self.get_logger().info("yayayayayayyayyaayayayayay ")
    

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 4):
                self.move(0.0, -20.0, 0.0, 0.0) #move right “And I’m dancin’ on to your heartbeat”
                self.get_logger().info("yayayayya ")
        self.get_logger().info("yayayayya ")

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 8):
                self.move(10.0, 50.0, -20.0, 20.0) #move in a circle going down “And… incomplete”
                break
        self.get_logger().info("yayayayya ")

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 13):
                self.move(0.0, -20.0, 0.0, 0.0) #move to center “So… truth”
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 15):
                self.move(20.0, 0.0, 10.0, 0.0) #move forward toward us and up “I… symphony”
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 20):
                self.move(5.0, 0.0, 20.0, 0.0) #bop up “Will… go”
                
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 24):
                self.move(10.0, 10.0, 0.0, 20.0) #revolve “Symphony”
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 28):
                self.move(0.0, 0.0, 0.0, 0.0) #stay still on surface
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 32):
                self.move(-10.0, 0.0, -20.0, 0.0) #go down “will you hold me tight and not let go”
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 36):
                self.move(20.0, -20.0, 0.0, 0.0) #go forward/right 45 “Ah, ah, ah”
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 39):
                self.move(0.0, 20.0, -10.0, 0.0) #go left straight “interlude”
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 44):
                self.move(-20.0, -20.0, -10.0, 0.0) #go backward/right ˚45
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 47):
                self.move(-20.0, 0.0, 20.0, 0.0) #go upwards backwards
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 51):
                self.move(0.0, 0.0, 0.0, 20.0) #spin
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 54):
                self.move(0.0, 0.0, -20.0, 0.0) #go down “And … heartbeat”
                self.get_logger().info("yayayayya ")
                break

        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 60):
                self.move(0.0, 0.0, -20.0, 40.0) #stay down and rotate “so if you want the truth”
                self.get_logger().info("yayayayya ")
                break

        while True:
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 65):
                self.move(0.0, 0.0, 20.0, 0.0) #launch up
                self.get_logger().info("yayayayya ")
                break

        while True:
            if (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec >= 70): #ending time
                self.move(0.0, 0.0, 0.0, 0.0)
                self.get_logger().info("quitting...")
                self.get_logger().info("yayayayya ")
                break

        return
    
    def test(self):
        self.start_time_sec = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 5 and rclpy.ok():
            self.move(30.0, 0.0, 0.0, 0.0)
        while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 10 and rclpy.ok():
            self.move(0.0, 30.0, 0.0, 0.0)

#class MinimalClientAsync(Node):

#     def __init__(self):
#         super().__init__('minimal_client_async')
#         self.cli = self.create_client(SetBool, 'arming')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = SetBool.Request()

#         self.get_logger().info('Arming')
#         self.send_arm_request(True)

#         while rclpy.ok() and not self.future.done():
#             rclpy.spin_once(self)

#     def send_arm_request(self, arm):
#         self.req = SetBool.Request()
#         self.req.data = arm
#         self.future = self.cli.call_async(self.req)

#     def send_disarm_request(self):
#         self.get_logger().info('Disarming ROV')
#         self.send_arm_request(False)

#         while rclpy.ok() and not self.future.done():
#             rclpy.spin_once(self)

#         try:
#             result = self.future.result()
#             if result.success:
#                 self.get_logger().info('ROV successfully disarmed.')
#             else:
#                 self.get_logger().warn(f'Disarm failed: {result.message}')
#         except Exception as e:
#             self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Move()

    try:
        node.dance()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

