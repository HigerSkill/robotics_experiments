import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from .state_publisher import StatePublisher


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('goal_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'{msg.pose}')
        StatePublisher().get_position(msg.pose)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
