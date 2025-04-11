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
	
    def draw_sphere(self, center, radius=1.0, num_points=50, color=carla.Color(255, 0, 0), life_time=0):
        """
        Draws a sphere by placing points in a spherical pattern and connects them to form a circle.
        :param center: The center location of the sphere (carla.Location).
        :param radius: The radius of the sphere.
        :param num_points: Number of points to approximate the sphere.
        :param color: The color of the sphere.
        :param life_time: Lifetime of the drawn points.
        """
        positions = []

        # Draw the center point
        self.publish_view_approximation(center.x, center.y, center.z, color, life_time)
        positions.append((center.x, center.y, center.z))

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
            self.publish_view_approximation(x, y, z, color, life_time)
            positions.append((x, y, z))

        # Draw lines between consecutive points to form a sphere-like shape
        for i in range(1, len(positions)):
            self.world.debug.draw_line(
                carla.Location(*positions[i-1]), 
                carla.Location(*positions[i]), 
                thickness=0.01, 
                color=color, 
                life_time=life_time
            )

        # Connect the last point to the first to close the circle
        self.world.debug.draw_line(
            carla.Location(*positions[-1]), 
            carla.Location(*positions[1]), 
            thickness=0.01, 
            color=color, 
            life_time=life_time
        )

        return positions
    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.pose_received = True

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

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
                (-28.96, -105.75),
                (-14.68, -174.61),
                (-86.83, -183.85),
                (-88.23, -15.68),
                (-51.66, -60.93),
                (-10.15, -11.42),
                (-7.34, -120.40),
                (-62.91, -104.85)
            ]

            turning_radius = 2.6  # Adjust as needed
            waypoint_threshold = 1.0  # Distance threshold to consider waypoint reached
            
            # Visualize the path waypoints as green dots
            for wp in waypoints:
                #self.publish_view_approximation(wp[0], wp[1], 0, carla.Color(0, 255, 0),600)  # Green color for path waypoints
                sphere_center = carla.Location(x=wp[0], y=wp[1], z=0.5)  # Center of the sphere
                self.draw_sphere(sphere_center, radius=1.0, num_points=50,color=carla.Color(0, 255, 0),life_time=45)

            for i in range(len(waypoints) - 1):
            
                start = waypoints[i] + (self.get_yaw_from_pose(self.current_pose),)
                goal = waypoints[i + 1] + (0,)  # Orientation at goal can be 0 or any desired orientation
                rospy.loginfo(f"Moving towards waypoint {i+1}: x={goal[0]}, y={goal[1]}")
                rate.sleep()

                dubins_path = self.compute_dubins_path(start, goal, turning_radius)
                
                # Draw spheres along the Dubins path
                all_sphere_positions = []
                for configuration in dubins_path:
                    x, y, yaw = configuration
                    sphere_center = carla.Location(x=x, y=y, z=0.5)
                    positions = self.draw_sphere(sphere_center, radius=1.0, num_points=50, color=carla.Color(157, 0, 255),life_time=10)
                    all_sphere_positions.append(positions)

                for j, configuration in enumerate(dubins_path):
                    x, y, yaw = configuration
                    control_msg = self.compute_control(x, y, yaw)

                    # Update vehicle transform
                    target_location = carla.Location(x, y)
                    target_rotation = carla.Rotation(pitch=0.0, yaw=np.degrees(yaw), roll=0.0)
                    
                    self.vehicle.set_transform(carla.Transform(target_location, target_rotation))
                    
                    pub.publish(control_msg)

                    rate.sleep()

            # Stop the vehicle at the end
            control_msg = CarlaEgoVehicleControl()
            control_msg.throttle = 0.0
            control_msg.steer = 0.0
            pub.publish(control_msg)
            rospy.loginfo("Stopping the vehicle.")
    def compute_dubins_path(self, start, goal, turning_radius):
        path = dubins.shortest_path(start, goal, turning_radius)
        configurations, _ = path.sample_many(1.0)
        return configurations
        
    def get_yaw_from_path(self, start, end):
        return np.arctan2(end[1] - start[1], end[0] - start[0])
    
    def get_yaw_from_pose(self, pose):
        orientation = pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw

    def compute_control(self, target_x, target_y, target_yaw):
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_pose(self.current_pose)

        # Control logic based on Dubins path configuration
        yaw_error = target_yaw - current_yaw
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        elif yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        steer = np.clip(yaw_error / np.pi, -1.0, 1.0) * 0.5  # Damping factor for smoother steering
        distance = np.linalg.norm([target_x - current_x, target_y - current_y])
        throttle = 0.5 if abs(steer) < 0.2 else 0.3

        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = throttle
        control_msg.steer = steer
        return control_msg

    def publish_view_approximation(self, x, y, z, color,life_time):
        location = carla.Location(x=x, y=y, z=z)
        #rospy.loginfo(f"Spawning debug point at: x={x}, y={y}, z={z}")
        self.world.debug.draw_point(location, size=0.02, color=color, life_time=life_time)

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
            rospy.logerr("No forklift vehicle found.")

        vehicle_controller.start_moving()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

