from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'point_gps', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = NavSatFix()
        msg.latitude = 46.8015409
        msg.longitude = 15.8382641
        msg.altitude = 338.917
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s, %s, %s"' % (str(msg.latitude), str(msg.longitude), str(msg.altitude)))
        self.i += 1