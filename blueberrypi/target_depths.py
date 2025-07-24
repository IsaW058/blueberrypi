import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Float32

class SendDepth (Node):
    def __init__(self):
        super().__init__("depth_publisher")    # names the node when running

        self.depth_time_list = [[0.0, 0.0]]

        self.start_time = self.get_time()

        self.pub = self.create_publisher(
            Float32,        # the message type
            "target_depth",    # the topic name
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized publisher node")

    def publish_depths(self):
        for depth_time in self.depth_time_list:
            while self.get_time() - self.start_time < depth_time[1]:
                msg = Float32()
                msg.data = float(depth_time[0])
                self.pub.publish(msg)
    
    def get_time(self):
        return float(self.get_clock().now().seconds_nanoseconds()[0] + (10 ** -9 * self.get_clock().now().seconds_nanoseconds()[1]))
    
    def get_depths(self):
        flag = False
        exit = True
        counter = 1
        while exit:
            if flag == False:
                self.depth_time_list[0][0] = input("Enter depth: ")
                self.depth_time_list[0][1] = input("Enter time for that depth: ")
                flag = True
            else:
                temp1 = input(f"Enter depth #{counter} or \"meow\" to exit" )
                temp2 = input(f"Enter time#{counter}: ")
                if(temp1 == "meow"):
                    break
                else:
                    self.depth_time_list.append([temp1, temp2])

def main(args=None):
    rclpy.init(args=args)
    node = SendDepth()

    try:
#        node.get_depths()
        node.publish_depths()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
