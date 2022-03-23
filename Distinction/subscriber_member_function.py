import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from datetime import datetime

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(String, 'clock/setalarm', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = datetime.now()
        self.j = datetime.strptime('2022/23/03 13:05:00',"%Y/%m/%d  %H:%M:%S")
        

    def timer_callback(self):
        msg = String()
        time = '%s:%s:%s %s:%s:%s' %(self.i.year, self.i.month, self.i.day, self.i.hour, self.i.minute, self.i.second)
        msg.data = '%s:%s:%s %s:%s:%s' % (self.j.year, self.j.month, self.j.day, self.j.hour, self.j.minute, self.j.second)
        self.publisher_.publish(msg)
        if msg.data > time:
            self.get_logger().info('Alarm set for: "%s"' % msg.data)
        
        self.subscription = self.create_subscription(
            String,
            'clock/alarm',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Alarm system working')
        rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
