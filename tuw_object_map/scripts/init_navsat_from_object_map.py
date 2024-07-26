#! /usr/bin/env python3
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose


import rclpy
from rclpy.node import Node
import tf_transformations

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('init_navsat')
        self.cli = self.create_client(SetDatum, 'datum')

        self.subscription = self.create_subscription(
            GeoPose,
            'geo_pose_map',
            self.callback_geo_pose,
            10)
        self.subscription  # prevent unused variable warning


    def callback_geo_pose(self, msg):
        self.get_logger().info('callback_geo_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetDatum.Request()

        self.req.geo_pose =   msg
        
        p = self.req.geo_pose.position
        q = self.req.geo_pose.orientation
        self.get_logger().info( 'datum %f, %f, %f, q: %f, %f, %f, %f' %  (p.latitude, p.longitude, p.altitude, q.x, q.y, q.z, q.w))
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()