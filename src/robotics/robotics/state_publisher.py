from math import cos, pi, sin, sqrt, acos, degrees, atan2

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()

        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'base_link'

    def get_position(self, pose):
        """Set arms angles by choosen point."""
        L1 = L2 = 0.5
        point = pose.position
        b = sqrt(point.x**2 + point.y**2)

        if b > 1:
            self.get_logger().info('This position is unreacheable!')
            return

        joint_state = JointState()

        joint1 = atan2(point.y, point.x) + acos((L1**2 - L2**2 + b**2)/(2*b*L1))
        joint2 = -(pi - acos((L1**2 + L2**2 - (b**2))/(2*L1*L2)))

        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['arm1_joint', 'arm2_joint', 'ef_joint']
        joint_state.position = [joint1, joint2, 0]
        joint_state.velocity = []

        self.get_logger().info(f'Angles: {degrees(joint1)},{degrees(joint2)}')

        self.joint_pub.publish(joint_state)




def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler's rotation to Quaternion."""

    return Quaternion(
        x=(
            sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - 
            cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        ), 
        y= (
            cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + 
            sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
        ), 
        z=(
            cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - 
            sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
        ), 
        w=(
            cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + 
            sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        )
    )

def main():
    StatePublisher()


if __name__ == '__main__':
    main()
