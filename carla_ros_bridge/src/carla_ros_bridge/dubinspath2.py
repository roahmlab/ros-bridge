import rospy
import carla
import numpy as np
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from tf.transformations import euler_from_quaternion

class VehicleController:
    def __init__(self, world):
        rospy.init_node('vehicle_movement_controller')
        self.current_pose = None
        self.pose_received = False
        self.world = world  # Set the CARLA world object

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
                (-62.91, -104.85)
            ]
            waypoint_threshold = 1.0  # Distance threshold to consider waypoint reached
            damping_factor = 1  # Damping factor for smoother steering

            # Visualize the path waypoints as green dots
            for wp in waypoints:
                self.publish_view_approximation(wp[0], wp[1], 0, carla.Color(0, 255, 0),600)  # Green color for path waypoints

            for i in range(len(waypoints)):
                goal = waypoints[i]
                rospy.loginfo(f"Moving towards waypoint {i+1}: x={goal[0]}, y={goal[1]}")
                goal_reached = False

                while not goal_reached and not rospy.is_shutdown():
                    current_position = [self.current_pose.position.x, self.current_pose.position.y]
                    distance_to_goal = np.linalg.norm([goal[0] - current_position[0], goal[1] - current_position[1]])

                    if distance_to_goal < waypoint_threshold:
                        goal_reached = True
                        rospy.loginfo(f"Reached waypoint {i+1}: x={goal[0]}, y={goal[1]}")
                        if i == len(waypoints) - 1:
                            rospy.loginfo("Reached the final waypoint. Stopping the vehicle.")
                            control_msg = CarlaEgoVehicleControl()
                            control_msg.throttle = 0.0
                            control_msg.brake = 1.0
                            pub.publish(control_msg)
                            return

                    target_yaw = np.arctan2(goal[1] - current_position[1], goal[0] - current_position[0])
                    current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
                    yaw_error = target_yaw - current_yaw

                    if yaw_error > np.pi:
                        yaw_error -= 2 * np.pi
                    elif yaw_error < -np.pi:
                        yaw_error += 2 * np.pi

                    steer = np.clip(yaw_error / np.pi, -1.0, 1.0) * damping_factor
                    throttle = 0.5 if abs(steer) < 0.2 else 0.3

                    control_msg = CarlaEgoVehicleControl()
                    control_msg.throttle = throttle
                    control_msg.steer = steer

                    pub.publish(control_msg)
                    rospy.loginfo(f"Published control command: throttle={control_msg.throttle}, steer={control_msg.steer}")

                    # Visualize the current goal waypoint as a red dot
                    self.publish_view_approximation(goal[0], goal[1], 0, carla.Color(255, 0, 0),3.0)  # Red color for current target waypoint

                    rate.sleep()

    def publish_view_approximation(self, x, y, z, color, life_time):
        location = carla.Location(x=x, y=y, z=z)
        rospy.loginfo(f"Spawning debug point at: x={x}, y={y}, z={z}")
        self.world.debug.draw_point(location, size=0.2, color=color, life_time=life_time)

    def get_yaw_from_quaternion(self, quaternion):
        _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    #weather = carla.WeatherParameters(
    #    cloudiness=0.0,
    #    precipitation=0.0,
    #    sun_altitude_angle=70.0
    #)
    #world.set_weather(weather)

    try:
        vehicle_controller = VehicleController(world)
        vehicle_controller.start_moving()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

