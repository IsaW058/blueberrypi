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
           self.move(50.0, 0.0, 0.0, 0.0) #move forward 
           self.get_logger().info("And now your song is on repeat")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 8 and rclpy.ok():
           self.move(50.0, -20.0, 0.0, 0.0) #move right and forward        
           self.get_logger().info("And I’m dancin’ on to your heartbeat”")
     
       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 11 and rclpy.ok():
        self.move(25.0, 25.0, -50.0, 50.0) #move in a circle going down   
        self.get_logger().info("And… incomplete")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 14 and rclpy.ok():
        self.move(0.0, -25.0, 0.0, 0.0) #move to center “So… truth”
        self.get_logger().info("So.. truth")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 19 and rclpy.ok():
        self.move(50.0, 0.0, 25.0, 0.0) #move forward toward us and up
        self.get_logger().info("I just wanna be part of your symphony ")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 22 and rclpy.ok():
        self.move(5.0, 0.0, 50.0, 0.0) #bop up 
        self.get_logger().info("Will you hold me tight and not let go? ")
       
       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 26 and rclpy.ok():
         self.move(25.0, 25.0, 0.0, 50.0) #revolve “Symphony”
         self.get_logger().info("Symphony ")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 30 and rclpy.ok():
         self.move(0.0, 0.0, 0.0, 0.0) #stay still on surface
         self.get_logger().info("Like a love song on the radio ")
               
       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 34 and rclpy.ok():
          self.move(-25.0, 0.0, -50.0, 0.0) #go down 
          self.get_logger().info("Will you hold me tight and not let go?")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 39 and rclpy.ok():
          self.move(50.0, -50.0, 0.0, 0.0) #go forward/right
          self.get_logger().info("Ah, ah, ah ")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 43 and rclpy.ok():
          self.move(0.0, 50.0, -25.0, 0.0) #go left straight
          self.get_logger().info("interlude")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 47 and rclpy.ok():
          self.move(-50.0, -50.0, -25.0, 0.0) #go backward/right ˚45
          self.get_logger().info("Ah, ah, ah")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 50 and rclpy.ok():
          self.move(-50.0, 0.0, 50.0, 0.0) #go upwards backwards
          self.get_logger().info("interlude ")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 54 and rclpy.ok():
          self.move(0.0, 0.0, 0.0, 50.0) #spin
          self.get_logger().info("And now your song is on repeat")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 58 and rclpy.ok():
           self.move(0.0, 0.0, -50.0, 0.0) 
           self.get_logger().info("And I’m dancing to your heartbeat")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec < 61 and rclpy.ok():
           self.move(0.0, 0.0, -50.0, 50.0) #stay down and rotate 
           self.get_logger().info("So if you want the truth ")

       while self.get_clock().now().seconds_nanoseconds()[0] - self.start_time_sec <= 65 and rclpy.ok():
           self.move(0.0, 0.0, 50.0, 0.0) #launch up
           self.get_logger().info("I JUST WANT TO BE PART OF YOUR SYMPHONY!!!!! ")
        
    
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

