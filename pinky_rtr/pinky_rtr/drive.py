#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from megacity_interfaces.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry, Path
from megacity_navigation.control_apps import PID
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import math


def get_yaw_from_orientation(q):
    quat = [q.x, q.y, q.z, q.w]
    _, _, yaw = euler_from_quaternion(quat)
    return yaw

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

# Define possible state results
class StateResult:
    CONTINUE = 0
    COMPLETE = 1
    AGAIN = 2
    WARNING = 3

# Abstract state class that returns a (Twist, status) tuple
class ControllerState:
    def __init__(self, controller):
        self.controller = controller

    def update(self, current_pose):
        raise NotImplementedError("update() must be implemented by subclasses")

# RotateToGoalState: turn to face the goal
class RotateToGoalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        # Calculate desired heading from current position to goal
        desired_heading = math.atan2(self.controller.goal_pose.y - current_pose.y,
                                        self.controller.goal_pose.x - current_pose.x)
        error_angle = normalize_angle(desired_heading - current_pose.theta)
        error_msg = Float64()
        error_msg.data = error_angle
        # Publish angle error
        self.controller.angle_error_publisher.publish(error_msg)
        # Publish current state string
        self.controller.state_publisher.publish(String(data="RotateToGoal"))
        self.controller.get_logger().info(f"[RotateToGoal] state")
        
        if abs(error_angle) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(error_angle)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return twist_msg, StateResult.CONTINUE
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.controller.get_logger().info("[RotateToGoal] complete.")
            return twist_msg, StateResult.COMPLETE

# MoveToGoalState: move forward toward the goal while correcting heading
class MoveToGoalState(ControllerState):
    def update(self, current_pose):
        dx = self.controller.goal_pose.x - current_pose.x
        dy = self.controller.goal_pose.y - current_pose.y

        distance_to_goal = math.sqrt(dx**2 + dy**2)

        # 3. 제자리 회전 버그를 방지하기 위해 모든 계산에 앞서 도착 여부를 먼저 확인합니다.
        if distance_to_goal < self.controller.distance_tolerance:
            self.controller.get_logger().info(
                f"[MoveToGoal] Already at the goal (distance: {distance_to_goal:.3f}m). Skipping."
            )
            return Twist(), StateResult.COMPLETE

        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        
        twist_msg = Twist()
        twist_msg.linear.x = self.controller.linear_pid.update(distance_error)
        
        desired_heading = math.atan2(dy, dx)
        angle_error = normalize_angle(desired_heading - current_pose.theta)
        twist_msg.angular.z = self.controller.angular_pid.update(angle_error)
        
        self.controller.distance_error_publisher.publish(Float64(data=distance_error))
        self.controller.state_publisher.publish(String(data="MoveToGoal"))
        self.controller.get_logger().info(f"[MoveToGoal] state. Current speed: {twist_msg.linear.x:.2f} m/s")

        if self.controller.obstacle_detected:
            return twist_msg, StateResult.WARNING
    
        if abs(distance_error) > self.controller.distance_tolerance + self.controller.linear_pid.max_state:
            return twist_msg, StateResult.CONTINUE
        else:
            if self.controller.index < len(self.controller.goal_list) - 1:
                next_x, next_y, _ = self.controller.goal_list[self.controller.index + self.controller.direction]
                next_desired_heading = math.atan2(next_y - current_pose.y, next_x - current_pose.x)
                next_heading_error = normalize_angle(next_desired_heading - current_pose.theta)

                dx_goal_to_next = next_x - self.controller.goal_pose.x
                dy_goal_to_next = next_y - self.controller.goal_pose.y
                goal_to_next_dist = math.sqrt(dx_goal_to_next**2 + dy_goal_to_next**2)
                
                if abs(next_heading_error) < self.controller.straight_angle:
                    if abs(distance_error) < goal_to_next_dist:
                        self.controller.get_logger().info("!!! [MoveToGoal] Waypoint skip: moving to next goal (AGAIN).")
                        return twist_msg, StateResult.AGAIN
                    else:
                        return twist_msg, StateResult.CONTINUE

            self.controller.get_logger().info("[MoveToGoal] State complete.")
            return twist_msg, StateResult.COMPLETE
        
class SlowToGoalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        dx = self.controller.goal_pose.x - current_pose.x
        dy = self.controller.goal_pose.y - current_pose.y
        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        error_msg = Float64()
        error_msg.data = distance_error
        # Publish distance error and state
        self.controller.distance_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="SlowToGoal"))
        self.controller.get_logger().info(f"[SlowToGoal] state")

        if abs(distance_error) > self.controller.distance_tolerance:
            linear_correction = self.controller.linear_pid.update(distance_error)
            # twist_msg.linear.x = linear_correction

            target_speed = self.controller.decel_gain * linear_correction
            twist_msg.linear.x = max(self.controller.min_speed, min(target_speed, self.controller.max_speed))
            
            desired_heading = math.atan2(
                self.controller.goal_pose.y - current_pose.y,
                self.controller.goal_pose.x - current_pose.x)
            angle_error = normalize_angle(desired_heading - current_pose.theta)
            angular_correction = self.controller.angular_pid.update(angle_error)
            twist_msg.angular.z = angular_correction

            self.controller.get_logger().info(f"[SlowToGoal] currnet speed {twist_msg.linear.x:.2f} m/s")

            if self.controller.obstacle_detected:
                return twist_msg, StateResult.WARNING

            return twist_msg, StateResult.CONTINUE
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.controller.get_logger().info("[SlowToGoal] complete.")
            return twist_msg, StateResult.COMPLETE
        
class ObstacleStopState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        dx = self.controller.goal_pose.x - current_pose.x
        dy = self.controller.goal_pose.y - current_pose.y
        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        error_msg = Float64()
        error_msg.data = distance_error
        # Publish distance error and state
        self.controller.distance_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="ObstacleStop"))
        self.controller.get_logger().info(f"[ObstacleStop] state")

        if self.controller.obstacle_detected:
            if self.controller.min_distance <= self.controller.base_distance + 0.1:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.controller.get_logger().warn(f"[ObstacleStop] Obstacle very close")
                
                return twist_msg, StateResult.CONTINUE

            if abs(distance_error) > self.controller.distance_tolerance:
                linear_correction = self.controller.linear_pid.update(distance_error)
                #target_speed = min(self.controller.decel_gain * linear_correction, self.controller.max_speed)
                obstacle_speed = min(self.controller.decel_gain * self.controller.current_linear, self.controller.max_speed)

                twist_msg.linear.x = 0.0 if obstacle_speed <= 0.1 else obstacle_speed
                #twist_msg.linear.x = max(0.0, min(target_speed, obstacle_speed))
                
                desired_heading = math.atan2(
                    self.controller.goal_pose.y - current_pose.y,
                    self.controller.goal_pose.x - current_pose.x)
                angle_error = normalize_angle(desired_heading - current_pose.theta)
                angular_correction = self.controller.angular_pid.update(angle_error)
                twist_msg.angular.z = angular_correction
                
                self.controller.get_logger().warn(
                    f'장애물 감지됨! 속도: {self.controller.current_linear:.2f} m/s, 감지 거리: {self.controller.min_distance:.2f} m'
                )

                self.controller.get_logger().info(f"[ObstacleStop] currnet speed {twist_msg.linear.x:.2f} m/s")

                return twist_msg, StateResult.CONTINUE
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.controller.get_logger().info("[ObstacleStop] complete.")
                return twist_msg, StateResult.COMPLETE
        
        else:
            twist_msg.linear.x = self.controller.current_linear
            twist_msg.angular.z = self.controller.current_angular
            self.controller.get_logger().info("[ObstacleStop] Obstacle clear.")
            return twist_msg, StateResult.COMPLETE
    

# RotateToFinalState: rotate in place to match final orientation
class RotateToFinalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        final_error = normalize_angle(self.controller.goal_pose.theta - current_pose.theta)
        error_msg = Float64()
        error_msg.data = final_error
        # Publish angle error for final orientation
        self.controller.angle_error_publisher.publish(error_msg)
        # Publish current state string
        self.controller.state_publisher.publish(String(data="RotateToFinal"))
        self.controller.get_logger().info(f"[RotateToFinal] state")

        if abs(final_error) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(final_error)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return twist_msg, StateResult.CONTINUE
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.controller.get_logger().info("[RotateToFinal] complete.")
            return twist_msg, StateResult.COMPLETE

# The state for goal reached simply returns zero control
class GoalReachedState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        # Publish current state string
        self.controller.state_publisher.publish(String(data="GoalReached"))

        self.controller.get_logger().info("[DRIVE] complete.")
        #rclpy.shutdown()
        return twist_msg, StateResult.COMPLETE

# Main controller node that manages state transitions
class GoalController(Node):
    def __init__(self):
        super().__init__('goal_controller')

        self.index = 0
        self.direction = 1

        # Declare ROS2 parameters for tolerances and PID parameters
        self.declare_parameter('angle_tolerance', 0.1)
        self.declare_parameter('distance_tolerance', 0.1)
        
        # Angular PID parameters
        self.declare_parameter('angular_P', 1.0)
        self.declare_parameter('angular_I', 0.0)
        self.declare_parameter('angular_D', 0.0)
        self.declare_parameter('angular_max_state', 2.0)
        self.declare_parameter('angular_min_state', -2.0)
        
        # Linear PID parameters
        self.declare_parameter('linear_P', 1.0)
        self.declare_parameter('linear_I', 0.0)
        self.declare_parameter('linear_D', 0.0)
        self.declare_parameter('linear_max_state', 0.8)  # Maximum linear speed is 2
        self.declare_parameter('linear_min_state', -0.8)
        
        # Get initial parameter values
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # Initialize Angular PID
        angular_P = self.get_parameter('angular_P').value
        angular_I = self.get_parameter('angular_I').value
        angular_D = self.get_parameter('angular_D').value
        angular_max_state = self.get_parameter('angular_max_state').value
        angular_min_state = self.get_parameter('angular_min_state').value
        self.angular_pid = PID()
        self.angular_pid.P = angular_P
        self.angular_pid.I = angular_I
        self.angular_pid.D = angular_D
        self.angular_pid.max_state = angular_max_state
        self.angular_pid.min_state = angular_min_state
            
        # Initialize Linear PID
        linear_P = self.get_parameter('linear_P').value
        linear_I = self.get_parameter('linear_I').value
        linear_D = self.get_parameter('linear_D').value
        linear_max_state = self.get_parameter('linear_max_state').value
        linear_min_state = self.get_parameter('linear_min_state').value
        self.linear_pid = PID()
        self.linear_pid.P = linear_P
        self.linear_pid.I = linear_I
        self.linear_pid.D = linear_D
        self.linear_pid.max_state = linear_max_state
        self.linear_pid.min_state = linear_min_state

        # Initialize current state and goal pose
        self.state_instance = None
        self.goal_pose = None
        self.goal_list = None

        self.current_linear = None
        self.current_angular = None

        # waypoint skip angle
        self.declare_parameter('straight_angle', 0.3)
        self.straight_angle = self.get_parameter('straight_angle').value
        
        # set decel
        self.declare_parameter('decel_gain', 0.9)
        self.decel_gain = self.get_parameter('decel_gain').value
        self.declare_parameter('decel_min_speed', 0.2)
        self.min_speed = self.get_parameter('decel_min_speed').value
        
        self.max_speed = linear_max_state
         
        # Subscribers and publishers
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'my_amcl_pose',
            self.pose_callback,
            10)
        
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Path, 'plan', self.plan_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel_smoother', 10)
        # Existing error publisher used for some errors; now add separate publishers
        self.angle_error_publisher = self.create_publisher(Float64, 'angle_error', 10)
        self.distance_error_publisher = self.create_publisher(Float64, 'distance_error', 10)
        # Publisher for current state information
        self.state_publisher = self.create_publisher(String, 'state', 10)
        
        # Register parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

        # 장애물 인식 파라미터
        self.declare_parameter('front_angle_deg', 30) # 전방 라이다각도 ±
        self.declare_parameter('base_distance', 0.1) # 기본 감지 거리
        self.declare_parameter('scale', 1.0) # 속도 scale
        self.declare_parameter('speed_threshold', 0.1) # 속도 임계값(직진 판단)      
        self.declare_parameter('min_cluster_size', 3) # 장애물 인식 크기
        self.declare_parameter('detection_threshold', 3) # 장애물 감지 threshold
        self.declare_parameter('clear_threshold', 5) # 장애물 없어짐 판단 threshold
        self.declare_parameter('min_lidar', 0.5) # 라이다 최소 인식 거리

        # 장애물 감지 파라미터
        self.declare_parameter('front_angle_deg', 30.0) # 전방 감지 각도 (좌우로)
        self.declare_parameter('base_distance', 0.1) # 기본 감지 거리
        self.declare_parameter('scale', 1.0) # 속도에 비례한 추가 감지 거리 스케일
        self.declare_parameter('speed_threshold', 0.1) # 직진으로 판단할 속도 임계값
        self.declare_parameter('min_cluster_size', 3) # 장애물로 판단할 최소 Lidar 포인트 수
        self.declare_parameter('detection_threshold', 3) # 장애물로 인식하기 위한 카운트
        self.declare_parameter('clear_threshold', 5) # 장애물이 사라졌다고 판단하기 위한 카운트
        self.declare_parameter('min_lidar', 0.5) # 라이다 최소 인식 거리
        
        self.front_angle_deg = self.get_parameter('front_angle_deg').value
        self.base_distance = self.get_parameter('base_distance').value
        self.scale = self.get_parameter('scale').value
        self.speed_threshold = self.get_parameter('speed_threshold').value 
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.clear_threshold = self.get_parameter('clear_threshold').value
        self.min_lidar = self.get_parameter('min_lidar').value

        self.clear_distance = self.base_distance +  0.3
        
        self.obstacle_detected = False
        self.detect_counter = 0
        self.clear_counter = 0
        self.min_distance = float('inf')

        self.current_linear = None
        self.current_angular = None

    def laser_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        n = len(ranges)

        if self.current_linear is not None:
            moving_forward = abs(self.current_linear) >= self.speed_threshold
            dynamic_distance = self.base_distance + self.scale * abs(self.current_linear)

            # 사용할 라이다 인덱스 범위 계산
            if moving_forward or self.obstacle_detected:
                front_rad = math.radians(self.front_angle_deg)
                min_angle = -front_rad
                max_angle = front_rad

                start_idx = round((min_angle - angle_min) / angle_increment)
                end_idx = round((max_angle - angle_min) / angle_increment)
                start_idx = max(0, start_idx)
                end_idx = min(n - 1, end_idx)
            else:
                start_idx = 0
                end_idx = n - 1

            
            if self.obstacle_detected:
                # 장애물이 이미 감지된 상태라면
                dynamic_distance = self.clear_distance            
            
            # 장애물 감지 + 최소 거리 계산
            found, self.min_distance = self.has_dense_cluster_with_min_dist(
                ranges, start_idx, end_idx, dynamic_distance, self.min_cluster_size
            )

            # 히스테리시스 적용
            if found:
                self.detect_counter += 1
                self.clear_counter = 0
            else:
                self.detect_counter = 0
                self.clear_counter += 1

            if not self.obstacle_detected and self.detect_counter >= self.detection_threshold:
                self.obstacle_detected = True
                self.get_logger().warn(
                    f'장애물 감지됨! 속도: {self.current_linear:.2f} m/s, 감지 거리: {dynamic_distance:.2f} m, '
                    f'최소 거리: {self.min_distance:.2f} m'
                )

            elif self.obstacle_detected and self.clear_counter >= self.clear_threshold:
                self.obstacle_detected = False
                self.get_logger().info("장애물 사라짐")

    def has_dense_cluster_with_min_dist(self, ranges, start_idx, end_idx, dynamic_distance, min_cluster_size):
        """연속된 장애물 포인트 감지 + 최소 거리 추출"""
        count = 0
        min_dist = float('inf')

        for i in range(start_idx, end_idx + 1):
            r = ranges[i]
            if self.min_lidar < r < dynamic_distance:
                min_dist = min(min_dist, r)
                count += 1
                if count >= min_cluster_size:
                    return True, min_dist
            else:
                count = 0

        return False, min_dist
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'angle_tolerance':
                self.angle_tolerance = param.value
                # self.get_logger().info(f"Updated angle_tolerance: {param.value}")
            elif param.name == 'distance_tolerance':
                self.distance_tolerance = param.value
                # self.get_logger().info(f"Updated distance_tolerance: {param.value}")
            elif param.name == 'angular_P':
                self.angular_pid.P = param.value
                # self.get_logger().info(f"Updated angular_PID P: {param.value}")
            elif param.name == 'angular_I':
                self.angular_pid.I = param.value
                # self.get_logger().info(f"Updated angular_PID I: {param.value}")
            elif param.name == 'angular_D':
                self.angular_pid.D = param.value
                # self.get_logger().info(f"Updated angular_PID D: {param.value}")
            elif param.name == 'angular_max_state':
                self.angular_pid.max_state = param.value
                # self.get_logger().info(f"Updated angular_PID max_state: {param.value}")
            elif param.name == 'angular_min_state':
                self.angular_pid.min_state = param.value
                # self.get_logger().info(f"Updated angular_PID min_state: {param.value}")
            elif param.name == 'linear_P':
                self.linear_pid.P = param.value
                # self.get_logger().info(f"Updated linear_PID P: {param.value}")
            elif param.name == 'linear_I':
                self.linear_pid.I = param.value
                # self.get_logger().info(f"Updated linear_PID I: {param.value}")
            elif param.name == 'linear_D':
                self.linear_pid.D = param.value
                # self.get_logger().info(f"Updated linear_PID D: {param.value}")
            elif param.name == 'linear_max_state':
                self.linear_pid.max_state = param.value
                # self.get_logger().info(f"Updated linear_PID max_state: {param.value}")
            elif param.name == 'linear_min_state':
                self.linear_pid.min_state = param.value
                # self.get_logger().info(f"Updated linear_PID min_state: {param.value}")
        return SetParametersResult(successful=True)
    
    def odom_callback(self, msg):
        self.current_linear = msg.twist.twist.linear.x
        self.current_angular = msg.twist.twist.angular.z

    def plan_callback(self, msg):
        self.goal_list = []

        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            yaw = get_yaw_from_orientation(pose_stamped.pose.orientation)

            self.goal_list.append([x, y, yaw])

        self.get_logger().info(f"Plan received with {len(self.goal_list)} points")
        
        self.index = 0

        self.state_instance = RotateToGoalState(self)
    
    def update_index(self):
        self.index += self.direction

        if self.index == len(self.goal_list) - 1:
            self.direction = -1
        elif self.index == 0:
            self.direction = 1 
        
    def pose_callback(self, msg):
        if self.goal_list is None or self.state_instance is None:
            return
        
        current_pose = Pose()
        current_pose.x = msg.pose.pose.position.x
        current_pose.y = msg.pose.pose.position.y
        current_pose.theta = get_yaw_from_orientation(msg.pose.pose.orientation)

        self.current_pose = msg.pose.pose

        self.goal_pose = Pose()
        self.goal_pose.x = self.goal_list[self.index][0]
        self.goal_pose.y = self.goal_list[self.index][1]
        self.goal_pose.theta = self.goal_list[self.index][2]
        # self.get_logger().info(f"goal pose: {self.goal_pose}")

        twist_msg, status = self.state_instance.update(current_pose)

        if status == StateResult.COMPLETE:
            # Transition to the next state based on the current state's type
            if isinstance(self.state_instance, RotateToGoalState):
                self.state_instance = MoveToGoalState(self)
            elif isinstance(self.state_instance, ObstacleStopState):
                self.state_instance = MoveToGoalState(self)
            elif isinstance(self.state_instance, MoveToGoalState):
                self.state_instance = SlowToGoalState(self)
            elif isinstance(self.state_instance, SlowToGoalState):
                if self.index == len(self.goal_list) - 1:
                    self.state_instance = RotateToFinalState(self)
                else:
                    self.state_instance = RotateToGoalState(self)
                    self.index += 1
            
            elif isinstance(self.state_instance, RotateToFinalState):
                self.state_instance = GoalReachedState(self)
            elif isinstance(self.state_instance, GoalReachedState):
                self.get_logger().info(f"drive complete!")
               # rclpy.shutdown()
        
        elif status == StateResult.AGAIN:
            if isinstance(self.state_instance, MoveToGoalState):
                self.index += 1
                self.state_instance = MoveToGoalState(self)

        elif status == StateResult.WARNING:
            if isinstance(self.state_instance, MoveToGoalState):
                self.state_instance = ObstacleStopState(self)
            elif isinstance(self.state_instance, SlowToGoalState):
                self.state_instance = ObstacleStopState(self)
        
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
         node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()