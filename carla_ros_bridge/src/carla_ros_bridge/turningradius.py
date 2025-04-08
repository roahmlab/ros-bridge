import rospy
import carla
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from carla_msgs.msg import CarlaEgoVehicleControl

class VehicleLogger:
    def __init__(self, world):
        rospy.init_node('vehicle_turning_radius_logger')
        self.current_pose = None
        self.pose_received = False
        self.world = world
        self.positions = []  # Store positions to calculate the turning radius

        # Publisher for vehicle control
        self.pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_callback)
        self.rate = rospy.Rate(10)  # 10 Hz for publishing control commands and receiving data

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.pose_received = True
        self.log_position_and_heading()

    def log_position_and_heading(self):
        if self.current_pose:
            position = self.current_pose.position
            orientation = self.current_pose.orientation
            _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

            rospy.loginfo(f"Position: x={position.x}, y={position.y}, z={position.z}, Heading (yaw): {np.degrees(yaw)} degrees")
            
            # Store the x, y positions
            self.positions.append((position.x, position.y))
            rospy.loginfo(f"Collected {len(self.positions)} data points.")

    def calculate_turning_radius(self):
        if len(self.positions) < 3:
            rospy.loginfo("Not enough data points to calculate turning radius.")
            return None

        # Fit a circle to the x, y positions to find the turning radius
        x = np.array([pos[0] for pos in self.positions])
        y = np.array([pos[1] for pos in self.positions])

        # Calculate circle fitting using least squares method
        A = np.column_stack([x, y, np.ones_like(x)])
        b = x**2 + y**2
        c, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        center_x, center_y = c[0] / 2, c[1] / 2
        radius = np.sqrt(center_x**2 + center_y**2 + c[2])

        rospy.loginfo(f"Calculated Turning Radius: {radius}")
        return radius

    def start_logging(self):
        # Setting the vehicle to a constant speed and full steer
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = 0.9
        control_msg.steer = 1.0  # Full steering angle for testing

        try:
            start_time = rospy.get_time()
            while not rospy.is_shutdown():
                self.pub.publish(control_msg)
                rospy.loginfo("Published control command: throttle=0.2, steer=1.0")
                self.rate.sleep()

                # Calculate the turning radius after collecting a significant number of points
                if len(self.positions) >= 100:  # Trigger calculation after 100 points
                    radius = self.calculate_turning_radius()
                    if radius:
                        rospy.loginfo(f"Calculated Turning Radius: {radius}")
                    break
        except rospy.ROSInterruptException:
            pass

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    vehicle_logger = VehicleLogger(world)
    vehicle_logger.start_logging()

if __name__ == '__main__':
    main()

