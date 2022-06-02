import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TestDrone(Node):

    def __init__(self):
        super().__init__('test_drone')
        self.publisher_ = self.create_publisher(String, 'drone', 10)
        
        msg = String()
        msg.data = 'command'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        time.sleep(2)        

        msg.data = 'takeoff'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        time.sleep(7)
        
        msg.data = 'cw 360'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        time.sleep(7)


        msg.data = 'land'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        time.sleep(3)

        
def main(args=None):
    rclpy.init(args=args)

    test_drone = TestDrone()
#    test_drone.run_tests()
#    rclpy.spin(test_drone)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
