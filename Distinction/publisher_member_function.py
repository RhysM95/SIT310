import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from datetime import datetime

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.subscription = self.create_subscription(
            String,
            'clock/setalarm',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        self.i = datetime.now()
        time = '%s:%s:%s %s:%s:%s' %(self.i.year, self.i.month, self.i.day, self.i.hour, self.i.minute, self.i.second)
            
        if msg.data > time:
            self.get_logger().info('Alarm time set for: "%s"' % msg.data)
            
        if time == msg.data:
            self.publisher_ = self.create_publisher(String, 'clock/alarm', 10)
            msg = String()
            msg.data = '%s:%s:%s %s:%s:%s' %(self.i.year, self.i.month, self.i.day, self.i.hour, self.i.minute, self.i.second)
            self.publisher_.publish(msg)
            self.get_logger().info('Alarm is off')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
