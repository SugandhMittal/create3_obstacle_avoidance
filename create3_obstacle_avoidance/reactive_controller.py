# reactive_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector, IrIntensityVector
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock, RotateAngle
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from rclpy.duration import Duration
import math
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ReactiveController(Node):
    def __init__(self):
        super().__init__('reactive_controller')
        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.marker_pub = self.create_publisher(Marker, '/ir_intensity_marker', 10)

        self.is_docked = None
        self.undocking = False
  
        self.safe_distance = 80

        self.stuck_counter = 0
        self.previous_command = None
        self.current_command = None
        self.verification_in_progress = False
        self.verification_step = 0
        self.left_sensor_values = []
        self.last_verification_time = self.get_clock().now()
        self.right_sensor_values = []
        self.ir_subscriber = self.create_subscription(IrIntensityVector, '/ir_intensity', self.sensor_callback, sensor_qos)
        self.dock_subscriber = self.create_subscription(DockStatus, '/dock_status', self.dock_status_callback, sensor_qos)
        self.hazard_subscriber = self.create_subscription(HazardDetectionVector,'/hazard_detection', self.hazard_callback, sensor_qos)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.undock_client = ActionClient(self, Undock, '/undock')
        # self.rotate_client = ActionClient(self, RotateAngle, '/rotate_angle')


    def publish_ir_markers(self, ir_msg):
        marker = Marker()
        marker.header = ir_msg.header
        marker.ns = "ir_intensity"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0

        marker.points = []
        marker.colors = []

        max_intensity = 2000.0  # Adjust based on your sensor max value

        # The exact sensor angles in degrees

        sensor_angles_deg = [-65.3, -38, -20, -3, 14.25, 34, 65.3]

        radius = 0.3  # 30 cm radius from robot center

        for i, reading in enumerate(ir_msg.readings):
            angle_rad = math.radians(sensor_angles_deg[i])

            point = Point()
            point.x = radius * math.cos(angle_rad)
            point.y = radius * math.sin(angle_rad)
            point.z = 0.1  # slight height above ground

            marker.points.append(point)

            intensity = min(reading.value / max_intensity, 1.0)
            color = ColorRGBA()
            color.r = intensity
            color.g = 0.0
            color.b = 1.0 - intensity
            color.a = 1.0
            marker.colors.append(color)

        self.marker_pub.publish(marker)

    # def send_rotation_goal(self, angle_rad):
    #     goal_msg = RotateAngle.Goal()
    #     goal_msg.angle = angle_rad
    #     goal_msg.max_rotation_speed = 0.5

    #     self.rotate_client.wait_for_server()
    #     self.rotate_client.send_goal_async(goal_msg)


    # def respond_to_bump(self, bump_events):
    #     if 'front_left' in bump_events:
    #         angle_rad = -0.5236  # 30 clockwise
    #     elif 'front_right' in bump_events:
    #         angle_rad = 0.5236   # 30 counter-clockwise
    #     elif 'left' in bump_events:
    #         angle_rad = -1.0472  # 60 clockwise
    #     elif 'right' in bump_events:
    #         angle_rad = 1.0472   # 60 counter-clockwise
    #     else:
    #         angle_rad = -1.5708  # 90 clockwise
    #     self.get_logger().info(f"Bump detected: {bump_events} → Rotating {angle_rad:.4f} rad")
    #     self.move_backward(Twist(), 0.3)
    #     self.send_rotation_goal(angle_rad)

    
    def stuck_detection(self, current_command):
            if self.previous_command == current_command:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
                self.previous_command = current_command

            if self.stuck_counter == 200:
                self.get_logger().info("Stuck condition detected")
                self.verification_in_progress = True
                self.verification_step = 0
                self.left_sensor_values = []
                self.right_sensor_values = []
                return True
            return False
   
    def verify_actual_or_forward(self, twist, msg):
        now = self.get_clock().now()

        if self.verification_step == 0:
            self.get_logger().info("Step 1: Turning left to scan")
            self.turn_left(twist, 0.3)
            self.last_verification_time = now
            self.verification_step = 1
            return True

        elif self.verification_step == 1:
            if now - self.last_verification_time < Duration(seconds=1.0):
                return True
            self.left_sensor_values = [msg.readings[i].value for i in [0,1,2,3]]
            self.get_logger().info("Step 2: Turning right to scan")
            self.turn_right(twist, 0.6)
            self.last_verification_time = now
            self.verification_step = 2
            return True

        elif self.verification_step == 2:
            if now - self.last_verification_time < Duration(seconds=1.0):
                return True
            self.right_sensor_values = [msg.readings[i].value for i in [4, 5, 6]]
            self.get_logger().info("Step 3: Returning to center")
            self.turn_left(twist, 0.3)
            self.last_verification_time = now
            self.verification_step = 3
            return True

        elif self.verification_step == 3:
            if now - self.last_verification_time < Duration(seconds=1.0):
                return True
            self.get_logger().info("Step 4: Final decision")

            left_obstacle = sum(value > self.safe_distance for value in self.left_sensor_values) / 4
            right_obstacle = sum(value > self.safe_distance for value in self.right_sensor_values) / 3

            if left_obstacle > right_obstacle:
                self.get_logger().info("Obstacle on left after checking")
                self.turn_right(twist, 0.3)
            elif right_obstacle > left_obstacle:
                self.get_logger().info("Obstacle on right after checking")
                self.turn_left(twist, 0.3)
            else:
                self.get_logger().info("No obstacles detected, no correction required")


            # Reset state
            self.verification_in_progress = False
            self.verification_step = 0
            self.stuck_counter = 0
            return False

   
   
    def dock_status_callback(self, msg: DockStatus):
        # self.get_logger().info(f"/dock_status message received: is_docked = {msg.is_docked}")
        self.is_docked = msg.is_docked

    def send_undock_goal(self):
        if not self.undock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Undock action server not available!')
            return

        goal_msg = Undock.Goal()
        self.get_logger().info('Sending undock goal...')
        self._undock_future = self.undock_client.send_goal_async(goal_msg)
        self._undock_future.add_done_callback(self.undock_response_callback)
        self.undocking = True
   
    def undock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Undock goal rejected')
            self.undocking = False
            return
        self.get_logger().info('Undock goal accepted')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.undock_result_callback)
   
    def undock_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Undock complete')
        self.undocking = False

    def move_forward(self, twist, speed=0.2):
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def move_backward(self, twist, speed=0.2):
        twist.linear.x = -speed
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
    
    def turn_right(self, twist, angular_speed=0.5):
        twist.linear.x = 0.0
        twist.angular.z = -angular_speed  # Negative for right turn
        self.publisher_.publish(twist)

    def turn_left(self, twist, angular_speed=0.5):
        twist.linear.x = 0.0
        twist.angular.z = angular_speed  # Positive for left turn
        self.publisher_.publish(twist)

    def stop(self, twist):
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def hazard_callback(self, msg):
        bump_events = []
        for hazard in msg.detections:
            if hazard.header.frame_id == 'bump_right':
                bump_events.append('right')
            elif hazard.header.frame_id == 'bump_front_right':
                bump_events.append('front_right')
            elif hazard.header.frame_id == 'bump_left':
                bump_events.append('left')
            elif hazard.header.frame_id == 'bump_front_left':
                bump_events.append('front_left')
            else:
                self.get_logger().debug(f"Ignored unknown hazard source: {hazard.header.frame_id}")
        if bump_events:
            self.get_logger().info(f"Bump detected at: {bump_events}")
            self.respond_to_bump(bump_events)

    # def respond_to_bump(self, bump_events):
    #     twist = Twist()
    #     if 'front_right' in bump_events[-1] or 'right' in bump_events[-1]:
    #         self.get_logger().info("Backing up and turning left")
    #         self.move_backward(twist, 0.3)
    #         self.turn_left(twist, 0.3)
    #     elif 'front_left' in bump_events[-1] or 'left' in bump_events[-1]:
    #         self.get_logger().info("Backing up and turning right")
    #         self.move_backward(twist, 0.3)
    #         self.turn_right(twist, 0.3)
    #     else:
    #         self.get_logger().info("Generic bump - backing up")
    #         self.move_backward(twist, 0.6)

    def respond_to_bump(self, bump_events):
        self.handling_bump = True  #Set flag
        twist = Twist()
        self.get_logger().info("Backing up due to bump")
        self.move_backward(twist, 0.2)
        time.sleep(1.0)
        if 'front_right' in bump_events[-1] or 'right' in bump_events[-1]:
            self.get_logger().info("Turning left after bump")
            self.turn_left(twist, 0.3)
            time.sleep(1.0)
        elif 'front_left' in bump_events[-1] or 'left' in bump_events[-1]:
            self.get_logger().info("Turning right after bump")
            self.turn_right(twist, 0.3)
            time.sleep(1.0)

        self.stop(twist)
        self.handling_bump = False  #Clear flag

    
    def sensor_callback(self, msg: IrIntensityVector):
        if getattr(self, 'handling_bump', False):
            self.get_logger().debug("Ignoring IR sensor data during bump handling")
            return
        
        twist = Twist()
        self.publish_ir_markers(msg)

        if self.is_docked is None:
            self.get_logger().info("Waiting for initial dock_status message... Ignoring sensor data.")
            return

        if self.is_docked and not self.undocking:
            self.send_undock_goal()
            return

        if self.undocking:
            return

        if self.verification_in_progress:
            self.verify_actual_or_forward(twist, msg)
            return

        readings = msg.readings
        distance_values = [readings[i].value for i in range(7)]
        threshold = self.safe_distance - 0.05


# Sensor indices:
        # 0: front_center_left
        # 1: front_center_right
        # 2: front_left
        # 3: front_right
        # 4: left
        # 5: right
        # 6: side_left

        # Necessary checks for obstacles
        front_check = any(distance_values[i] > threshold for i in [3])
        obstacle_left_check = any(distance_values[i] > threshold for i in [0, 1, 2])
        obstacle_right_check = any(distance_values[i] > threshold for i in [4, 5, 6])
        left_avg = sum([distance_values[i] for i in [0, 1, 2]]) / 3
        right_avg = sum([distance_values[i] for i in [4, 5, 6]]) / 3
        if front_check:
            # Obstacle directly in front - choose turning direction
            if obstacle_left_check and not obstacle_right_check:
                self.get_logger().info("Obstacle front and left - Turning Right")
                current_command = 1
                self.move_backward(twist, 0.3)
                self.turn_right(twist, angular_speed=0.5)
            elif obstacle_right_check and not obstacle_left_check:
                self.get_logger().info("Obstacle front and right - Turning Left")
                current_command = 2
                self.move_backward(twist, 0.3)
                self.turn_left(twist, angular_speed=0.5)
            elif obstacle_left_check and obstacle_right_check:
                self.get_logger().info("Obstacles on both side - Moving Back") 
                current_command = 3
                self.move_backward(twist, 0.3)
            else:
                current_command = 3
                self.move_backward(twist, 0.3)
        elif obstacle_left_check and left_avg> right_avg:
            self.get_logger().info("Left obstacle - Turning Slight Right")
            current_command = 6
            self.turn_right(twist, angular_speed=0.4)
        elif obstacle_right_check and right_avg > left_avg:
            self.get_logger().info("Right obstacle - Turning Slight Left")
            current_command = 7
            self.turn_left(twist, angular_speed=0.3)
        else:
            self.get_logger().info("Moving forward")
            current_command = 0
            self.move_forward(twist, speed=0.3)
        
        if self.stuck_detection(current_command):
            return



def main(args=None):
    rclpy.init(args=args)
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
