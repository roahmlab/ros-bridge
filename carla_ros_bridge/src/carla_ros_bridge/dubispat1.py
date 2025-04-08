import rospy
import carla
import dubins
import numpy as np
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class VehicleController:
    def __init__(self, world):
        rospy.init_node('vehicle_movement_controller')
        self.current_pose = None
        self.pose_received = False
        self.world = world  # Set the CARLA world object
        self.turning_radius = 1.0

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.pose_received = True

    def start_moving(self):
        pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_callback)
        rate = rospy.Rate(20)  # 20 Hz for responsive control

        while not self.pose_received and not rospy.is_shutdown():
            rospy.loginfo("Waiting for vehicle's current pose...")
            rate.sleep()

        if self.pose_received:
            rospy.loginfo("Vehicle pose received. Starting movement.")

            # Define the hardcoded waypoints (skip the first as instructed)
            waypoints = [
                (self.current_pose.position.x, self.current_pose.position.y),
                (-28.96, -105.75),
                (-14.68, -174.61),
                (-86.83, -183.85),
                (-88.23, -15.68),
                (-51.66, -60.93),
                (-10.15, -11.42),
                (-7.34, -120.40),
                (self.current_pose.position.x, self.current_pose.position.y),
            ]

            turning_radius = 1.0  # Adjust as needed
            waypoint_threshold = 2.0  # Distance threshold to consider waypoint reached
            
            # Visualize the path waypoints as green dots
            for wp in waypoints:
                self.publish_view_approximation(wp[0], wp[1], 0, carla.Color(0, 255, 0),600)  # Green color for path waypoints

            for i in range(len(waypoints) - 1):
                start = waypoints[i] + (self.get_yaw_from_path(waypoints[i], waypoints[i+1]),)
                goal = waypoints[i + 1] + (self.get_yaw_from_path(waypoints[i], waypoints[i+1]),)
                rospy.loginfo(f"Moving towards waypoint {i+1}: x={goal[0]}, y={goal[1]}")
                
                dubins_path = self.compute_dubins_path(start, goal, turning_radius)
                goal_reached = False

                while not goal_reached and not rospy.is_shutdown():
                    for configuration in dubins_path:
                        x, y, yaw = configuration
                        control_msg = self.compute_control(x, y, yaw)

                        pub.publish(control_msg)
                        rospy.loginfo(f"Published control command: throttle={control_msg.throttle}, steer={control_msg.steer}")

                        self.publish_view_approximation(x, y, 0, carla.Color(255, 0, 0), 3.0)  # Red color for current target waypoint

                        current_position = [self.current_pose.position.x, self.current_pose.position.y]
                        distance_to_goal = np.linalg.norm([goal[0] - current_position[0], goal[1] - current_position[1]])

                        if distance_to_goal < waypoint_threshold:
                            goal_reached = True
                            rospy.loginfo(f"Reached waypoint {i+1}: x={goal[0]}, y={goal[1]}")
                            break

                        rate.sleep()

                    if goal_reached and i == len(waypoints) - 2:
                        rospy.loginfo("Reached the final waypoint. Stopping the vehicle.")
                        control_msg = CarlaEgoVehicleControl()
                        control_msg.throttle = 0.0
                        control_msg.brake = 1.0
                        pub.publish(control_msg)
                        return

    def compute_dubins_path(self, start, goal, turning_radius):
        path = dubins.shortest_path(start, goal, turning_radius)
        configurations, _ = path.sample_many(1.0)
        return configurations

    def get_yaw_from_path(self, start, end):
        return np.arctan2(end[1] - start[1], end[0] - start[0])

    def compute_control(self, target_x, target_y, target_yaw):
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_pose(self.current_pose)

        # Calculate the yaw error based on the path's desired yaw
        yaw_error = target_yaw - current_yaw
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        elif yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        # Calculate the distance to the target
        distance_to_goal = np.linalg.norm([target_x - current_x, target_y - current_y])

        # Determine if the vehicle is close enough to the target to start turning
        if distance_to_goal < self.turning_radius:
            # Begin turn based on yaw error
            steer = np.clip(yaw_error / np.pi, -1.0, 1.0)
            throttle = 0.2  # Slow down for precise turning
        else:
            # Keep moving straight with minimal steering adjustments
            steer = 0
            throttle = 0.5  # Move at a normal speed

        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = throttle
        control_msg.steer = steer
        return control_msg


    def get_yaw_from_pose(self, pose):
        orientation = pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw

    def publish_view_approximation(self, x, y, z, color,life_time):
        location = carla.Location(x=x, y=-y, z=z)
        rospy.loginfo(f"Spawning debug point at: x={x}, y={y}, z={z}")
        self.world.debug.draw_point(location, size=0.2, color=color, life_time=life_time)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    try:
        vehicle_controller = VehicleController(world)
        vehicle_controller.start_moving()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

