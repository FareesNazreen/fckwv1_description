import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile

class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')
        qos_profile = QoSProfile(depth=10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        self.get_logger().info('Odom to TF Node has been started')

    def odom_callback(self, msg):
        # Create a TransformStamped message
        transform = TransformStamped()

        # Set the header
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        # Set the transform
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        # Publish the transform to the /tf topic
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info('Published transform from %s to %s' %
                               (msg.header.frame_id, msg.child_frame_id))

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
