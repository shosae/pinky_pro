import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VelocitySmoother(Node):
    def __init__(self):
        super().__init__('velocity_smoother')

        # 파라미터 선언
        self.declare_parameter('max_accel', 0.5)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_angular_accel', 1.0)
        self.declare_parameter('max_angular_decel', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('dt', 0.1)

        # 파라미터 가져오기
        self.max_accel = self.get_parameter('max_accel').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.max_angular_decel = self.get_parameter('max_angular_decel').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.dt = self.get_parameter('dt').value

        self.target_cmd = Twist()
        self.current_linear = 0.0
        self.current_angular = 0.0

        # 토픽 연결
        self.create_subscription(Twist, 'cmd_vel_smoother', self.target_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 타이머 시작
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def target_callback(self, msg):
        self.target_cmd = msg

    def odom_callback(self, msg):
        self.current_linear = msg.twist.twist.linear.x
        self.current_angular = msg.twist.twist.angular.z

    def timer_callback(self):
        output = Twist()

        # 선속도 처리
        linear_target = self.target_cmd.linear.x
        linear_current = self.current_linear
        linear_diff = linear_target - linear_current

        if linear_diff > 0:
            linear_step = min(linear_diff, self.max_accel * self.dt)
            smoothed_linear = linear_current + linear_step
        else:
            smoothed_linear = linear_target

        smoothed_linear = max(min(smoothed_linear, self.max_speed), -self.max_speed)

        angular_target = self.target_cmd.angular.z
        angular_current = self.current_angular
        angular_diff = angular_target - angular_current

        if angular_diff > 0:
            angular_step = min(angular_diff, self.max_angular_accel * self.dt)
        else:
            angular_step = max(angular_diff, -self.max_angular_decel * self.dt)

        smoothed_angular = angular_current + angular_step
        smoothed_angular = max(min(smoothed_angular, self.max_angular_speed), -self.max_angular_speed)

        # 정시 신호가 들어왔을 경우 바로 멈추기
        if linear_target == 0.0:
            smoothed_linear = 0.0

        if angular_target == 0.0:
            smoothed_angular = 0.0

        # 결과 퍼블리시
        output.linear.x = smoothed_linear
        output.angular.z = smoothed_angular

        self.cmd_pub.publish(output)

        # self.get_logger().info(
        #     f'목표속도 → 선속도: {linear_target:.2f} → {smoothed_linear:.2f}, '
        #     f'회전: {angular_target:.2f} → {smoothed_angular:.2f}'
        # )

def main(args=None):
    rclpy.init(args=args)
    smoother = VelocitySmoother()
    rclpy.spin(smoother)
    smoother.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
