import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool  # Standard service for a boolean response
import math

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Set up the publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Set up the subscriber to odometry (or encoder data)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        
        # Set up the subscriber to the waypoints topic
        self.waypoints_sub = self.create_subscription(String, '/waypoints', self.waypoint_callback, 10)
        
        # Service to stop the robot
        self.stop_service = self.create_service(SetBool, '/stop_robot', self.stop_callback)
        
        # List of waypoints (to be updated dynamically)
        self.checkpoints = []
        self.is_absolute = []
        
        # Current index in the checkpoint list
        self.current_checkpoint_idx = 0
        
        # Current position of the robot (initialized to 0,0)
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0  # Orientation
        
        # State to track if robot is stopped
        self.is_stopped = False
        
        self.create_timer(0.1, self.control_loop)  # Control loop every 0.1 seconds

    def odom_callback(self, msg):
        # Extract robot's current position and orientation from the odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Get the robot's orientation (quaternion to euler conversion)
        orientation = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.current_theta = (yaw) % (2 * math.pi)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z  # In radians

    def control_loop(self):
        if self.is_stopped:
            # If stopped, publish zero velocity to keep the robot stationary
            stop_twist = Twist()  # Zero velocities
            self.cmd_vel_pub.publish(stop_twist)
            return
        
        # Ensure there are waypoints to follow
        if not self.checkpoints:
            self.get_logger().info('Waiting for waypoints...')
            return
        
        if self.current_checkpoint_idx >= len(self.checkpoints):
            self.get_logger().info('Trajectory complete')
            return
        
        # Get the target checkpoint (either absolute or relative)
        target_x, target_y = self.get_target_checkpoint(self.checkpoints[self.current_checkpoint_idx], self.is_absolute[self.current_checkpoint_idx])
        
        # Calculate the distance and angle to the next checkpoint
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        
        # Normalize the angle to turn
        angular_velocity = self.normalize_angle(angle_to_target - self.current_theta)
        
        # If the robot is not facing the waypoint, turn to face it
        if abs(angular_velocity) > 0.3:  # If the angle difference is large enough to require turning
            self.turn_towards_waypoint(angular_velocity)
        else:
            # Once the robot is facing the target, move towards it
            if distance < 0.02:
                # Stop at the waypoint
                self.get_logger().info(f"Arrived at waypoint: ({target_x}, {target_y})")
                self.stop_at_waypoint()
            else:
                self.move_towards_waypoint(distance,angular_velocity)

        # Log the robot's current coordinates after each step
        self.get_logger().info(f"Current Position: x={self.current_x:.2f}, y={self.current_y:.2f}, theta={self.current_theta:.2f} (radians)")

    def turn_towards_waypoint(self, angular_velocity):
        """Turn the robot towards the target waypoint"""
        twist = Twist()
        min_angvel=0.1
        max_angvel=1
        angular_velocity=max(min(angular_velocity,max_angvel),-max_angvel)
        if abs(angular_velocity)<min_angvel:
            angular_velocity=min_angvel if angular_velocity>0 else -min_angvel
        twist.angular.z =float(angular_velocity)  # Only turn
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Turning towards waypoint: angular_velocity={angular_velocity:.2f}")

    def move_towards_waypoint(self, distance,angular_velocity):
        """Move the robot towards the target waypoint"""
        twist = Twist()
        speed=0.5*distance
        min_speed=0.1
        max_speed=0.5
        twist.linear.x = float(max(min(speed,max_speed),min_speed))  # Move forward
        twist.angular.z = float(angular_velocity)
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Moving towards waypoint: distance={distance:.2f}")

    def stop_at_waypoint(self):
        """Stop the robot at the current waypoint"""
        stop_twist = Twist()  # Zero velocities
        self.cmd_vel_pub.publish(stop_twist)
        self.current_checkpoint_idx += 1  # Move to the next checkpoint after stopping
        self.get_logger().info(f"Stopping at waypoint {self.current_checkpoint_idx}")

    def get_target_checkpoint(self, checkpoint, is_absolute):
        """Get the target (x, y) for the checkpoint, depending on whether it's absolute or relative"""
        if is_absolute:
            return checkpoint  # Absolute coordinate (x, y)
        else:
            # Add relative coordinates to the current position (dx, dy)
            return checkpoint
        
    def normalize_angle(self, angle):
        """Normalize the angle to be between -pi and pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def waypoint_callback(self, msg):
        """Callback function to handle multiple waypoint input via topic"""
        input_str = msg.data.split(';')  # Split by semicolon to handle multiple waypoints
        
        for wp_str in input_str:
            wp_str = wp_str.strip()  # Remove any leading or trailing spaces
            if not wp_str:
                continue
            wp_parts = wp_str.split()
            if len(wp_parts) != 3:
                self.get_logger().info(f"Invalid waypoint input: {wp_str}. Format should be: <absolute/relative> <x> <y>")
                continue

            # Extract the type and the coordinates
            wp_type = wp_parts[0]
            x = float(wp_parts[1])
            y = float(wp_parts[2])
            
            if wp_type == "absolute":
                self.checkpoints.append((x, y))
                self.is_absolute.append(True)
                self.get_logger().info(f"Added absolute waypoint: ({x}, {y})")
            elif wp_type == "relative":
                self.checkpoints.append((self.checkpoints[-1][0]+x, self.checkpoints[-1][1]+y))
                self.is_absolute.append(False)
                self.get_logger().info(f"Added relative waypoint: ({x}, {y})")
            else:
                self.get_logger().info(f"Invalid waypoint type: {wp_type}. Use 'absolute' or 'relative'.")

        # If waypoints were added, resume movement
        if self.checkpoints and self.is_stopped:
            self.is_stopped = False
            self.get_logger().info("Resuming movement.")

    def stop_callback(self, request, response):
        """Stop the robot by sending zero velocity and set the stop state"""
        self.get_logger().info("Stopping the robot.")
        self.is_stopped = True
        stop_twist = Twist()  # Zero velocities
        self.cmd_vel_pub.publish(stop_twist)
        
        # Send a success response
        response.success = True
        response.message = "Robot stopped successfully."
        return response

def main(args=None):
    rclpy.init(args=args)
    trajectory_generator = TrajectoryGenerator()
    rclpy.spin(trajectory_generator)
    trajectory_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

