import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclPoseRepublisher(Node):
    def __init__(self):
        super().__init__('amcl_pose_republisher')

        self.latest_msg = None  # 최신 메시지를 저장

        # Subscriber to /amcl_pose
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.listener_callback,
            10
        )

        # Publisher to /my_amcl_pose
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'my_amcl_pose',
            10
        )

        # Timer: 0.1초 주기로 콜백 호출
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Republishing /amcl_pose to /my_amcl_pose at 10Hz...')

    def listener_callback(self, msg):
        self.latest_msg = msg  # 최신 메시지를 저장

    def timer_callback(self):
        if self.latest_msg is not None:
            self.publisher.publish(self.latest_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
