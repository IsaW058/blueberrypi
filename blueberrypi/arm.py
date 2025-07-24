import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class rov2Arm(Node):
    def __init__(self):
        super().__init__('robot_armer')
        self.cli = self.create_client(SetBool, 'arming')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, data):
        self.req.data = data
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    client = rov2Arm()
    future = client.send_request(True)
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    if (response.success):
        client.get_logger().info("Arming successful")
    else:
        client.get_logger().info("Arming failed, " + response.message)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        future1 = client.send_request(False)
        # rclpy.spin_until_future_complete(client, future1)
        # response1 = future1.result()
        # if (response1.success):
        #     client.get_logger().info("Disarming successful")
        # else:
        #     client.get_logger().info("Disarming failed, " + response.message)
        print(" Robot likely disarmed, check to make sure")
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()