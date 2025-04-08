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
        self.world = world
        self.vehicle = None
        self.wheelbase = 3
	
    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.pose_received = True

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
            initial_position = self.current_pose.position
            rospy.loginfo(f"Initial Position: x={initial_position.x}, y={initial_position.y}")

            # Stop the vehicle if it's moving
            self.stop_vehicle(pub, rate)

            # Calculate the new target position 5 meters ahead
            current_orientation = self.current_pose.orientation
            current_position = self.current_pose.position
            _, _, current_yaw = euler_from_quaternion([
                current_orientation.x,
                current_orientation.y,
                current_orientation.z,
                current_orientation.w
            ])
            
            target_x = current_position.x + 5.0 * np.cos(current_yaw)
            target_y = current_position.y + 5.0 * np.sin(current_yaw)

            rospy.loginfo(f"Target Position (5m ahead): x={target_x}, y={target_y}")

            # Move the vehicle towards the target position
            self.move_to_target(target_x, target_y, pub, rate)

            final_position = self.current_pose.position
            rospy.loginfo(f"Final Position (after 5m): x={final_position.x}, y={final_position.y}")

            # Turn the vehicle 90 degrees dynamically
            rospy.loginfo("Turning vehicle 90 degrees.")
            target_yaw = current_yaw + np.pi / 2
            self.turn_to_angle(target_yaw, pub, rate)

            # Move 20 meters to the left
            target_x = final_position.x - 40.0 * np.sin(target_yaw)
            target_y = final_position.y + 40.0 * np.cos(target_yaw)

            rospy.loginfo(f"Target Position (20m left): x={target_x}, y={target_y}")
            self.move_to_target(target_x, target_y, pub, rate)

    def move_to_target(self, target_x, target_y, pub, rate):
        while not rospy.is_shutdown():
            current_position = self.current_pose.position
            distance_to_target = np.linalg.norm([target_x - current_position.x, target_y - current_position.y])
            
            if distance_to_target < 0.5:  # Stop if the vehicle is close to the target
                rospy.loginfo("Reached the target position.")
                break

            control_msg = CarlaEgoVehicleControl()
            control_msg.throttle = 0.2  # Set throttle value for movement

            target_yaw = np.arctan2(target_y - current_position.y, target_x - current_position.x)
            current_yaw = self.get_yaw_from_pose(self.current_pose)

            steer_angle = np.arctan2(np.sin(target_yaw - current_yaw), np.cos(target_yaw - current_yaw))
            control_msg.steer = np.clip(steer_angle, -1.0, 1.0)

            pub.publish(control_msg)
            rate.sleep()

        # Stop the vehicle after reaching the target
        self.stop_vehicle(pub, rate)

    def stop_vehicle(self, pub, rate):
        rospy.loginfo("Stopping the vehicle...")
        while not rospy.is_shutdown():
            velocity = self.vehicle.get_velocity()
            speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
            if speed < 0.01:  # Considered stopped
                rospy.loginfo("Vehicle stopped.")
                break

            control_msg = CarlaEgoVehicleControl()
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
            pub.publish(control_msg)
            rate.sleep()

    def turn_to_angle(self, target_yaw, pub, rate):
        while not rospy.is_shutdown():
            current_yaw = self.get_yaw_from_pose(self.current_pose)
            angle_diff = target_yaw - current_yaw

            # Normalize angle difference
            if angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            elif angle_diff < -np.pi:
                angle_diff += 2 * np.pi

            if abs(angle_diff) < 0.05:
                rospy.loginfo("Reached the target yaw.")
                break

            control_msg = CarlaEgoVehicleControl()
            control_msg.throttle = 0.3  # Move forward while turning

            if angle_diff > 0:
                control_msg.steer = 1.0  # Full right steer
            else:
                control_msg.steer = -1.0  # Full left steer

            pub.publish(control_msg)
            rate.sleep()

        # Stop the vehicle after turning
        self.stop_vehicle(pub, rate)

    def get_yaw_from_pose(self, pose):
        orientation = pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    try:
        vehicle_controller = VehicleController(world)

        # Find the vehicle (e.g., a car) with the role_name 'hero'
        vehicle = None
        for actor in world.get_actors():
            if 'vehicle' in actor.type_id:  # Removed role_name check
                vehicle = actor
                break

        if vehicle:
            vehicle_controller.set_vehicle(vehicle)
        else:
            rospy.logerr("No suitable vehicle found.")

        vehicle_controller.start_moving()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

