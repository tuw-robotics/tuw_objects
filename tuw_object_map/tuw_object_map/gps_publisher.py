import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.declare_parameters(
            namespace='', 
            parameters=[
                ('latitude', 46.8015409),
                ('longitude', 15.8382641),
                ('altitude', 338.917),
                ('update', 0.5)
            ]
        )
        
        self.publisher_ = self.create_publisher(NavSatFix, 'point_gps', 10)
        timer_period = self.get_parameter('update').get_parameter_value().double_value  # seconds
        if(timer_period == 0):
            self.timer_callback()
        else:
            self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = NavSatFix()
        msg.latitude = self.get_parameter('latitude').get_parameter_value().double_value
        msg.longitude = self.get_parameter('longitude').get_parameter_value().double_value
        msg.altitude = self.get_parameter('altitude').get_parameter_value().double_value
        self.publisher_.publish(msg)
        self.get_logger().info('%s : latitude:=%s longitude:=%s altitude:=%s' % (str(self.i), str(msg.latitude), str(msg.longitude), str(msg.altitude)))
        self.i += 1