import mlr_nav2_puzzlebot.utils.puzzlebot_kinematics as puzzlebot_kinematics
import numpy as np
import rclpy
import transforms3d as t3d

from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

class Localization(Node):
    def __init__(self):
        super().__init__('puzzlebot_localization_node')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('localization_update_rate', 25.0)
        self.declare_parameter('initial_pose.x', 0.0)
        self.declare_parameter('initial_pose.y', 0.0)
        self.declare_parameter('initial_pose.theta', 0.0)
        # Get the parameters
        r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        l = self.get_parameter('wheel_base').get_parameter_value().double_value
        update_rate = self.get_parameter('localization_update_rate').get_parameter_value().double_value
        initial_pose = [self.get_parameter('initial_pose.x').get_parameter_value().double_value,
                        self.get_parameter('initial_pose.y').get_parameter_value().double_value,
                        self.get_parameter('initial_pose.theta').get_parameter_value().double_value]

        # Subscribers
        self.create_subscription(Float32, 'VelocityEncR', self.wr_callback, 10)
        self.create_subscription(Float32, 'VelocityEncL', self.wl_callback, 10)
        
        # Publishers
        self.odometry_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Timers
        self.create_timer(1.0/update_rate, self.localize_puzzlebot)
        
        # Node variables
        self.puzzlebot_kinematic_model = puzzlebot_kinematics.get_puzzlebot_kinematic_model(r, l)
        self.wheels_speeds = np.array([0., 0.])
        self.puzzlebot_pose = np.array(initial_pose)
        self.covariance_matrix = np.zeros((3, 3))
        self.process_noise_covariance_matrix = np.array([[0.00000567, -0.0000065, -0.0000163],
                                                         [-0.0000065, 0.00002631, 0.00003897],
                                                         [-0.0000163, 0.00003897, 0.00011256]])

        self.last_time = self.get_clock().now()

        # Log the node start
        self.get_logger().info('Puzzlebot localization node started.')
        
    def wr_callback(self, msg):
        self.wheels_speeds[0] = msg.data

    def wl_callback(self, msg):
        self.wheels_speeds[1] = msg.data

    def localize_puzzlebot(self):
        # Get the linear and angular speeds from the wheels speeds and the kinematic model
        speeds = self.puzzlebot_kinematic_model @ self.wheels_speeds

        # Get the delta time
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9

        # Get the current linearized puzzlebot model
        linearized_puzzlebot_model = puzzlebot_kinematics.get_linearized_puzzlebot_model_matrix(speeds[0], self.puzzlebot_pose[2], dt)

        # Decompose the linear and angular speeds into [vx, vy, w]
        decomposed_speeds = puzzlebot_kinematics.speeds_decomposer(speeds[0], speeds[1], self.puzzlebot_pose[2])

        # Update the covariance matrix
        self.covariance_matrix = linearized_puzzlebot_model @ self.covariance_matrix @ linearized_puzzlebot_model.T + self.process_noise_covariance_matrix

        # Update the pose [x, y, theta] based on [vx, vy, w] and dt
        self.puzzlebot_pose[0] += decomposed_speeds[0] * dt
        self.puzzlebot_pose[1] += decomposed_speeds[1] * dt
        self.puzzlebot_pose[2] += decomposed_speeds[2] * dt
        # Normalize the angle in the range [-pi, pi]
        self.puzzlebot_pose[2] = np.arctan2(np.sin(self.puzzlebot_pose[2]), np.cos(self.puzzlebot_pose[2]))

        # Odom message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.puzzlebot_pose[0]
        odom_msg.pose.pose.position.y = self.puzzlebot_pose[1]
        odom_msg.pose.pose.position.z = 0.0
        q = t3d.euler.euler2quat(0.0, 0.0, self.puzzlebot_pose[2])
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]
        odom_msg.pose.pose.orientation.w = q[0]
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = self.covariance_matrix[0, 0]
        odom_msg.pose.covariance[1] = self.covariance_matrix[0, 1]
        odom_msg.pose.covariance[5] = self.covariance_matrix[0, 2]
        odom_msg.pose.covariance[6] = self.covariance_matrix[1, 0]
        odom_msg.pose.covariance[7] = self.covariance_matrix[1, 1]
        odom_msg.pose.covariance[11] = self.covariance_matrix[1, 2]
        odom_msg.pose.covariance[30] = self.covariance_matrix[2, 0]
        odom_msg.pose.covariance[31] = self.covariance_matrix[2, 1]
        odom_msg.pose.covariance[35] = self.covariance_matrix[2, 2]
        odom_msg.twist.twist.linear.x = speeds[0]
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = speeds[1]

        # Publish the odometry message
        self.odometry_publisher.publish(odom_msg)

        # Update the last time
        self.last_time = self.get_clock().now()

def main():
    rclpy.init()
    node = Localization()
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