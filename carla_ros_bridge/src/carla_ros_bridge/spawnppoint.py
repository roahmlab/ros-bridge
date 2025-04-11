#!/usr/bin/env python
import sys
egg_path = '/home/carla/CarlaDepotAutomation/carla-0.9.14-py3.8-linux-x86_64.egg'

sys.path.append(egg_path)

import rospy
import carla
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class VehicleController:
    def __init__(self, world):
        rospy.init_node('vehicle_debug_spawner')
        self.current_pose = None
        self.pose_received = False
        self.world = world

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.pose_received = True

    def start(self):
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_callback)
        rate = rospy.Rate(10)  # Check for pose updates at 10 Hz

        while not self.pose_received and not rospy.is_shutdown():
            rospy.loginfo("Waiting for vehicle's current pose...")
            rate.sleep()

        if self.pose_received:
            self.spawn_debug_items()

    def spawn_debug_items(self):
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            z = self.current_pose.position.z
            orientation_q = self.current_pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation_q.x, 
                orientation_q.y, 
                orientation_q.z, 
                orientation_q.w
            ])

            # Forward direction vector
            forward_vector = np.array([np.cos(yaw), np.sin(yaw), 0])

            # Spawn debug items along the vehicle's forward direction
            for i in range(6):  # 5 additional items, 1 meter apart
                spawn_x = x + i * forward_vector[0]
                spawn_y = - (y + i * forward_vector[1])  # Negate y-coordinate
                location = carla.Location(x=spawn_x, y=spawn_y, z=z)
                color = carla.Color(255, 0, 0)  # Red color for visibility
                rospy.loginfo(f"Spawning debug item at: x={spawn_x}, y={spawn_y}, z={z}")
                self.world.debug.draw_point(location, size=0.2, color=color, life_time=10.0)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    try:
        vehicle_controller = VehicleController(world)
        vehicle_controller.start()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

