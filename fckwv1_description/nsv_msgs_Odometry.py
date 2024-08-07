import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('encoder_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/encoder_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
       self.vx = odom_msg.twist.twist.linear.x
       self.vtheta = odom_msg.twist.twist.angular.z


    def __init__(self):
       super().__init__ ('odometry_publisher')
       self.publisher = self.create_publisher (Odometry, 'odom', 10)
       self.timer = self.create_timer(0.1, self.publish_odometry)
       self.odom_frame_id = 'odom'
       self.child_frame_id = 'base_link'
       self.odom_broadcaster = TransformBroadcaster(self)
       
       self.x = 0.0
       self.y = 0.0
       self.theta = 0.0
    #    self.vx = 0.1 #例子中使用的线速度
    #    self.vtheta = 0.1 #例子中使用的角速度

    def publish_odometry(self):

      #更新里程计信息
      self.x += self.vx * math.cos(self.theta)
      self.y += self.vx * math.sin(self.theta)
      self.theta += self.vtheta
      
      #TransformStampede
      odom_trans = TransformStamped()
      odom_trans.header.stamp = self.get_clock().now().to_msg()
      odom_trans.header.frame_id = self.odom_frame_id
      odom_trans.child_frame_id = self.child_frame_id
      odom_trans.transform.translation.x = self.x
      odom_trans.transform.translation.y = self.y
      odom_trans.transform.rotation.w = math.cos(self.theta / 2)
      odom_trans.transform.rotation.z = math.sin(self.theta / 2)
      self.odom_broadcaster.sendTransform(odom_trans)

      #发布Odometry消息
      odom_msg = Odometry()
      odom_msg.header.stamp = self.get_clock().now().to_msg()
      odom_msg.header.frame_id = self.odom_frame_id
      odom_msg.child_frame_id = self.child_frame_id
      odom_msg.pose.pose.position.x = self.x
      odom_msg.pose.pose.position.y = self.y
      odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
      odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
      odom_msg.twist.twist.linear.x = self.vx
      odom_msg.twist.twist.angular.z = self.vtheta

      self.publisher.publish(odom_msg)

def main(args=None):
       rclpy.init(args=args)
       node = OdometryPublisher()
       rclpy.spin(node)
       rclpy.shutdown()
       
if __name__ == '__main__': 
       main()
