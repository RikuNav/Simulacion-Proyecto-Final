import mlr_nav2_puzzlebot.utils.puzzlebot_kinematics as puzzlebot_kinematics
import numpy as np
import rclpy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class PuzzlebotJointPublisher(Node):
    def __init__(self):
        super().__init__('puzzlebot_joint_state_publisher_node')

        # Namespace
        self.namespace = self.get_namespace().rstrip('/')

        # Robot parameters declaration
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        # Get the parameters
        r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        l = self.get_parameter('wheel_base').get_parameter_value().double_value
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Publishers
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Node variables
        self.inverse_puzzlebot_kinematic_model = puzzlebot_kinematics.get_inverse_puzzlebot_kinematic_model(r, l)
        self.wheels_angles = np.array([0., 0.])
        self.last_time = self.get_clock().now()

        # Log the node start
        self.get_logger().info('Puzzlebot Joint Publisher Node has been started.')

    def odom_callback(self, msg):
        # Create a TransformStamped message for the odom to base_footprint transform and assign the odometry message pose values
        odom_base_footprint_transform = TransformStamped()
        odom_base_footprint_transform.header.stamp = self.get_clock().now().to_msg()
        odom_base_footprint_transform.header.frame_id = 'odom'
        odom_base_footprint_transform.child_frame_id = f'{self.namespace}/base_footprint'
        odom_base_footprint_transform.transform.translation.x = msg.pose.pose.position.x
        odom_base_footprint_transform.transform.translation.y = msg.pose.pose.position.y
        odom_base_footprint_transform.transform.translation.z = 0.0
        odom_base_footprint_transform.transform.rotation.x = msg.pose.pose.orientation.x
        odom_base_footprint_transform.transform.rotation.y = msg.pose.pose.orientation.y
        odom_base_footprint_transform.transform.rotation.z = msg.pose.pose.orientation.z
        odom_base_footprint_transform.transform.rotation.w = msg.pose.pose.orientation.w

        # Publish the transform
        self.tf_broadcaster.sendTransform(odom_base_footprint_transform)  

        # Get the wheels speeds from the odometry message Twist values and the inverse kinematic model
        wheels_speeds = self.inverse_puzzlebot_kinematic_model @ np.array([msg.twist.twist.linear.x, msg.twist.twist.angular.z])

        # Get delta time
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9

        # Update the wheels angles based on the wheels speeds and dt
        self.wheels_angles[0] += wheels_speeds[0] * dt
        self.wheels_angles[1] += wheels_speeds[1] * dt
        # Normalize the angles in the range [-pi, pi]
        self.wheels_angles[0] = np.arctan2(np.sin(self.wheels_angles[0]), np.cos(self.wheels_angles[0]))
        self.wheels_angles[1] = np.arctan2(np.sin(self.wheels_angles[1]), np.cos(self.wheels_angles[1]))

        # Create a JointState message for the wheels angles
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = 'base_footprint'
        joint_state_msg.name = ['wheel_right_joint', 'wheel_left_joint']
        joint_state_msg.position = [self.wheels_angles[0], self.wheels_angles[1]]
        joint_state_msg.velocity = [0.0, 0.0]
        joint_state_msg.effort = [0.0, 0.0]

        # Publish the joint state
        self.joint_state_publisher.publish(joint_state_msg)   

        # Last time update
        self.last_time = self.get_clock().now()

def main():
    rclpy.init()
    node = PuzzlebotJointPublisher()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info('Node interrupted. Shutting down...')
        node.get_logger().info(f'Error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()