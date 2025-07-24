import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_srvs.srv import SetBool

class BlueROVArm(Node):
    def __init__(self):
        super().__init__("blue_rov_Arm")    # names the node when running
        self.cli = self.create_client(SetBool, 'arming')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, arm = True):
        self.req.data = arm
        return self.cli.call_async(self.req) ##change

def yay(args=None):

    rclpy.init(args=args)
    arm = BlueROVArm()
    future = arm.send_request(True)
    rclpy.spin_until_future_complete(arm, future)
    response = future.result()

    if response.success == True:
        arm.get_logger().info(f"the robot is armed.")
        # try:
        #     #rclpy.spin(arm)
        #     print("hi")
        # except KeyboardInterrupt:
        #     response = arm.send_request(arm=False)
        #     if response is not None:
        #         print("\nKeyboardInterrupt received, shutting down...")
        #     else:
        #         print("it didn't work :(")
    
    arm.get_logger().info(f"Meow")
    start_time_sec = arm.get_clock().now().seconds_nanoseconds()[0]

    while True:
        if (arm.get_clock().now().seconds_nanoseconds()[0] - start_time_sec >= 80):
            future = arm.send_request(False)
            rclpy.spin_until_future_complete(arm, future)
            response = future.result()
            if response.success == True:
                arm.get_logger().info(f"the robot is disarmed.")
                break
                
    arm.destroy_node()
    if rclpy.ok():
        rclpy.shutdown() 