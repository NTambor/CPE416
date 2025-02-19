import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

class roomba_program(Node):
    def __init__(self):
        super().__init__('roomba')

        # Create Publisher for Twist
        self.publisher = self.create_publisher(Twist, "diff_drive/cmd_vel", 10)
        
        # Create a subscriber for the Odometry
        self.odom_subscriber = self.create_subscription(Odometry, "diff_drive/odometry", self.odom_callback, 10)
        
        # Create a subscriber for the Lidar
        self.lidar_subscriber = self.create_subscription(LaserScan, "diff_drive/scan", self.scan_callback, 10)
        
        # Create a timer to trigger the timer_callback function every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize Twist message for robot velocity and other variables
        self.drive = Twist()
        self.odometry = Odometry()
        self.scan = LaserScan()

        # State variables
        self.obstacle_detected = False
        self.state = 'MOVE_FORWARD'  # Initial state

        # Timing variables for state transitions
        self.backup_start_time = None
        self.spin_start_time = None
        self.recovery_start_time = None

    # Callback function triggered when odometry data is received
    def odom_callback(self, msg):
        self.odometry = msg  # Update the odometry data
        # Extract the current orientation in quaternion format and convert to Euler angles (yaw, pitch, roll)
        current_orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

    # Callback function triggered when Lidar scan data is received
    def scan_callback(self, msg):
        self.scan = msg  # Update the lidar data
        
        # Check if any obstacle is closer than 0.3 meters in front (1 ft)
        front_distances = msg.ranges[0:10] + msg.ranges[-10:]  # Front lidar readings
        self.obstacle_detected = any(d < 0.3 for d in front_distances if d > 0.0)  # Detect obstacle

    # Function to drive robot forward
    def move_forward(self):
        self.drive.linear.x = 1.0  # Move forward at 1 m/s
        self.drive.angular.z = 0.0  # No turning
        self.publisher.publish(self.drive)

    # Function to detect obstacle (already handled by scan_callback)
    def detect_obstacle(self):
        return self.obstacle_detected

    # Function to back up the robot for 1 second
    def back_up(self):
        self.drive.linear.x = -0.5  # Move backward at 0.5 m/s
        self.drive.angular.z = 0.0  # No turning
        self.publisher.publish(self.drive)

    # Function to spin robot in place
    def spin_in_place(self):
        self.drive.linear.x = 0.0  # No forward movement
        self.drive.angular.z = 1.0  # Spin with angular velocity of 1 rad/s
        self.publisher.publish(self.drive)

    # Function for recovery actions (back up and turn)
    def recovery(self):
        self.drive.linear.x = -0.3  # Back up slightly
        self.drive.angular.z = 0.5  # Turn in place
        self.publisher.publish(self.drive)

    # Callback function triggered by the timer every 0.1 seconds
    def timer_callback(self):
        current_time = self.get_clock().now()

        if self.state == 'MOVE_FORWARD':
            self.move_forward()
            if self.detect_obstacle():
                self.state = 'BACK_UP'
                self.backup_start_time = current_time  # Start backup timer
            else:
                self.move_forward()

        elif self.state == 'BACK_UP':
            self.back_up()
            if (current_time - self.backup_start_time).nanoseconds > 1e9:  # Backup for 1 second
                self.state = 'SPIN'
                self.spin_start_time = current_time  # Start spin timer
            else:
                self.back_up()

        elif self.state == 'SPIN':
            self.spin_in_place()
            if not self.detect_obstacle():
                self.state = 'MOVE_FORWARD'  # Go back to moving forward
            else:
                if (current_time - self.spin_start_time).nanoseconds > 5e9:  # If spinning for more than 5 seconds
                    self.state = 'RECOVERY'
                    self.recovery_start_time = current_time  # Start recovery timer
                else:
                    self.spin_in_place()

        elif self.state == 'RECOVERY':
            self.recovery()
            if (current_time - self.recovery_start_time).nanoseconds > 3e9:  # Recover for 3 seconds
                self.state = 'MOVE_FORWARD'  # Try moving forward again
            else:
                self.recovery()

# Main function to run the node
def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)  
    # Create an instance of the roomba_program node
    roomba = roomba_program()  
    # Keep the node running and responsive
    rclpy.spin(roomba)  
    # Destroy the node after execution ends
    roomba.destroy_node()  
    # Shutdown ROS 2
    rclpy.shutdown()  

# Run the main function if this script is executed directly
if __name__ == '__main__':
    main()
