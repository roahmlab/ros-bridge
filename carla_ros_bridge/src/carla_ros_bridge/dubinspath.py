#!/usr/bin/env python
import rospy
import sys
egg_path = '/home/carla/CarlaDepotAutomation/carla-0.9.14-py3.8-linux-x86_64.egg'

sys.path.append(egg_path)

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
        self.vehicle = None
        self.wheelbase = 3
        self.lidar_sensors = []
        # Add 8 lidar sensors to the vehicle at the specified locations
        self.add_lidar_sensors()
	
	
    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.pose_received = True
        
    def add_lidar_sensors(self):
        """
        Adds 8 LiDAR sensors at different locations relative to the vehicle.
        """
        LIDAR_SENSOR_POSITIONS = [
            carla.Location(x=-50, y=0, z=50),
            carla.Location(x=-50, y=0, z=-50),
            carla.Location(x=50, y=0, z=50),
            carla.Location(x=50, y=0, z=-50),
            carla.Location(x=0, y=50, z=50),
            carla.Location(x=0, y=50, z=-50),
            carla.Location(x=0, y=-50, z=50),
            carla.Location(x=0, y=-50, z=-50),
        ]

        for i, position in enumerate(LIDAR_SENSOR_POSITIONS):
            # Create the LiDAR sensor
            lidar = carla.sensor.Lidar('MyLidar_' + str(i))
            lidar.set(
                Channels=32,  # Example: 32 channels
                Range=50,  # Example: 50m range
                PointsPerSecond=100000,  # Example: 100,000 points per second
                RotationFrequency=10,  # Example: 10 rotations per second
                UpperFovLimit=10,  # Example: 10 degrees
                LowerFovLimit=-30  # Example: -30 degrees
            )

            # Set position and rotation for each LiDAR sensor
            lidar.set_position(position.x, position.y, position.z)
            lidar.set_rotation(pitch=0, yaw=0, roll=0)

            # Add the LiDAR sensor to the CARLA settings
            self.carla_settings.add_sensor(lidar)
            self.lidar_sensors.append(lidar)
            rospy.loginfo(f"Added LiDAR sensor at {position}")
            
            
    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def start_moving(self):
        if self.vehicle is None:
            rospy.logerr("Vehicle not set. Exiting.")
            return
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

            colors = [
                carla.Color(255, 0, 0),    # Red
                carla.Color(0, 255, 0),    # Green
                carla.Color(0, 0, 255),    # Blue
                carla.Color(255, 255, 0),  # Yellow
                carla.Color(255, 0, 255),  # Magenta
                carla.Color(0, 255, 255),  # Cyan
                carla.Color(255, 165, 0),  # Orange
                carla.Color(128, 0, 128),  # Purple
            ]

            turning_radius = 2.5
            lookahead_distance = 5.0  # Distance ahead to look for the target point
            
            waypoint_threshold = 2.0   # Threshold distance to consider a waypoint reached
            angle_threshold = 0.1      # Threshold angle (in radians) to consider correct orientation

            for i, wp in enumerate(waypoints):
                # Draw a sphere at each waypoint
                sphere_center = carla.Location(x=wp[0], y=wp[1], z=0)  # Center of the sphere
                #self.draw_sphere(sphere_center, radius=1.0, num_points=50)

            for i in range(len(waypoints) - 1):
                start = (self.current_pose.position.x, self.current_pose.position.y, self.get_yaw_from_pose(self.current_pose))
                goal = waypoints[i + 1] + (0,)
                rospy.loginfo(f"Moving towards waypoint {i+1}: x={goal[0]}, y={goal[1]}")

                dubins_path = self.compute_dubins_path(start, goal, turning_radius)
                path_index = 0
                while path_index < len(dubins_path) and not rospy.is_shutdown():
                    target_x, target_y, target_yaw = dubins_path[path_index]
                    control_msg = self.compute_control(target_x, target_y, target_yaw, lookahead_distance)

                    # Use CARLA API to control the vehicle's position and velocity
                    target_location = carla.Location(x=target_x, y=target_y)
                    target_velocity = carla.Vector3D(x=control_msg.throttle, y=0, z=0)
                    
                    self.vehicle.set_target_velocity(target_velocity)
                    self.vehicle.set_transform(carla.Transform(target_location, self.vehicle.get_transform().rotation))
                    
                    # Draw a sphere at the current target point
                    sphere_center = carla.Location(x=target_x, y=target_y, z=0.1)  # Center of the sphere
                    #self.draw_sphere(sphere_center, radius=1.0, num_points=50)

                    #pub.publish(control_msg)
                    rospy.loginfo(f"Published control command: throttle={control_msg.throttle}, steer={control_msg.steer}")

                    current_x = self.current_pose.position.x
                    current_y = self.current_pose.position.y
                    current_yaw = self.get_yaw_from_pose(self.current_pose)
                    distance_to_target = np.linalg.norm([target_x - current_x, target_y - current_y])
                    angle_to_target = abs(target_yaw - current_yaw)
                    path_index += 1


                    rate.sleep()

    def adjust_turning_radius(self, throttle):
        if throttle <= 0.1:
            return 2.554
        elif throttle <= 0.2:
            return 2.595
        elif throttle <= 0.5:
            return 2.969
        elif throttle <= 0.75:
            return 3.300
        elif throttle <= 0.9:
            return 3.488
        else:
            return 3.570

    def determine_throttle(self, goal):
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        distance = np.linalg.norm([goal[0] - current_x, goal[1] - current_y])

        if distance < 5:
            return 0.1
        elif distance < 10:
            return 0.15
        elif distance < 20:
            return 0.2
        elif distance < 30:
            return 0.2
        elif distance < 40:
            return 0.2
        else:
            return 0.2

    def compute_dubins_path(self, start, goal, turning_radius):
        path = dubins.shortest_path(start, goal, turning_radius)
        configurations, _ = path.sample_many(1.0)
        return configurations

    def get_yaw_from_pose(self, pose):
        orientation = pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw

    def compute_control(self, target_x, target_y, target_yaw, lookahead_distance):
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_pose(self.current_pose)

        alpha = np.arctan2(target_y - current_y, target_x - current_x) - current_yaw
        if alpha > np.pi:
            alpha -= 2 * np.pi
        elif alpha < -np.pi:
            alpha += 2 * np.pi

        Lf = lookahead_distance
        steer = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), Lf)
        throttle = 0.2  # This can be adjusted based on distance or other factors

        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = throttle
        control_msg.steer = steer
        return control_msg

    def publish_view_approximation(self, x, y, z, color,life_time):
        location = carla.Location(x=x, y=y, z=z)
        #rospy.loginfo(f"Spawning debug point at: x={x}, y={y}, z={z}")
        self.world.debug.draw_point(location, size=0.1, color=color, life_time=life_time)
        
    def draw_sphere(self, center, radius=1.0, num_points=50):
        """
        Draws a sphere by placing points in a spherical pattern.
        :param center: The center location of the sphere (carla.Location).
        :param radius: The radius of the sphere.
        :param num_points: Number of points to approximate the sphere.
        """
        # Draw the center point
        self.publish_view_approximation(center.x, center.y, center.z, carla.Color(255, 0, 0), -1.0)

        # Generate points around the sphere
        for _ in range(num_points):
            # Spherical coordinates
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)

            # Convert spherical coordinates to Cartesian coordinates
            x = center.x + radius * np.sin(phi) * np.cos(theta)
            y = center.y + radius * np.sin(phi) * np.sin(theta)
            z = center.z + radius * np.cos(phi)

            # Draw point
            self.publish_view_approximation(x, y, z, carla.Color(0, 255, 0), -1.0)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    try:
        vehicle_controller = VehicleController(world)

        # Find the forklift vehicle with the role_name 'hero'
        forklift = None
        for actor in world.get_actors():
            if 'forklift' in actor.type_id:
                forklift = actor
                break

        if forklift:
            vehicle_controller.set_vehicle(forklift)
        else:
            rospy.logerr("No forklift vehicle with role_name 'hero' found.")

        vehicle_controller.start_moving()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
