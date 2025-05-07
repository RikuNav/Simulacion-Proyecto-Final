import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np

class DynamicTFPublisher(Node):
    # Constructor
    def __init__(self):
        super().__init__('dynamic_tf_publisher')

        # Create two TransformBroadcaster
        self.br = TransformBroadcaster(self)

        # Create the message for the transforms
        self.t = TransformStamped()

        # Create a timer for publishing transforms
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_transforms)

        # Create publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Variables to use
        self.start_time = self.get_clock().now()
        self.omega = 0.5
        self.omega_wheel = 2.5

        # Create a JointState message
        self.ctrlJoint = JointState()
        self.ctrlJoint.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoint.name = ['wheel_r_joint', 'wheel_l_joint']
        self.ctrlJoint.position = [0.0] * 2
        self.ctrlJoint.velocity = [0.0] * 2
        self.ctrlJoint.effort = [0.0] * 2

    # Timer Callback
    def publish_transforms(self):
        # Get elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9

        # Update the first message
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'base_footprint'
        self.t.transform.translation.x = 1.0 * np.sin(self.omega*elapsed_time)
        self.t.transform.translation.y = 1.0 * np.cos(self.omega*elapsed_time)
        self.t.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, -self.omega * elapsed_time)
        self.t.transform.rotation.x = q[1]
        self.t.transform.rotation.y = q[2]
        self.t.transform.rotation.z = q[3]
        self.t.transform.rotation.w = q[0]

        time = self.get_clock().now().nanoseconds/1e9

        self.ctrlJoint.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoint.position[0] = self.omega_wheel * time
        self.ctrlJoint.position[1] = self.omega_wheel * time
        
        self.joint_pub.publish(self.ctrlJoint)
        self.br.sendTransform(self.t)

def main(args=None):
    rclpy.init(args=args)
    dynamic_tf_publisher = DynamicTFPublisher()
    rclpy.spin(dynamic_tf_publisher)
    dynamic_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()