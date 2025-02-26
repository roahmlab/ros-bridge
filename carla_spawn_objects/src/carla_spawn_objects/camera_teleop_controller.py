#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose, Twist
from transforms3d.euler import euler2quat

class CameraTeleopController:
    def __init__(self):
        rospy.init_node('camera_teleop_controller', anonymous=True)

        # Publisher: the camera's pose topic.
        self.pose_topic = rospy.get_param('~pose_topic', '/carla/map_camera_video/control/set_transform')
        self.pose_pub = rospy.Publisher(self.pose_topic, Pose, queue_size=10)

        # Subscriber: listen for Twist commands on this topic (e.g., published by teleop_twist_keyboard).
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/camera/cmd_vel')
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)

        # Set update rate (Hz) for integrating commands into pose updates.
        self.update_rate = rospy.get_param('~update_rate', 10.0)  # Hz

        # Initialize camera state.
        # For simplicity, we assume the camera has zero roll and pitch.
        self.position = [0.0, 0.0, 10.0]  # [x, y, z] in meters
        self.yaw = 0.0                    # in degrees

        # Latest command velocities (in the camera's local frame).
        # linear_cmd: [vx, vy, vz] (m/s)
        # angular_cmd: yaw rate (rad/s)
        self.linear_cmd = [0.0, 0.0, 0.0]
        self.angular_cmd = 0.0

        self.last_update_time = rospy.Time.now().to_sec()

        # Timer for regularly updating the camera state.
        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.update_state)

        rospy.loginfo("Camera Teleop Controller started.\n"
                      "Listening for Twist commands on topic '%s'", self.cmd_vel_topic)

    def cmd_vel_callback(self, msg):
        # Update the command velocities from the received Twist message.
        self.linear_cmd = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.angular_cmd = msg.angular.z  # We use only the z-component (yaw rate)
        rospy.loginfo("Received cmd_vel: linear=%s, angular=%.2f", self.linear_cmd, self.angular_cmd)

    def update_state(self, event):
        # Compute time step.
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Update position.
        # Convert local linear velocities to global frame.
        # Here we assume the camera's local frame is rotated about z by 'yaw'.
        yaw_rad = math.radians(self.yaw)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        vx_local, vy_local, vz_local = self.linear_cmd
        vx_global = vx_local * cos_yaw - vy_local * sin_yaw
        vy_global = vx_local * sin_yaw + vy_local * cos_yaw

        self.position[0] += vx_global * dt
        self.position[1] += vy_global * dt
        self.position[2] += vz_local * dt

        # Update yaw.
        # Convert angular command (rad/s) to degrees.
        self.yaw += math.degrees(self.angular_cmd) * dt

        # Publish the updated camera pose.
        self.publish_pose()

    def publish_pose(self):
        pose = Pose()
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = self.position[2]

        # Assume roll and pitch are zero.
        roll = 0.0
        pitch = 0.0
        yaw_rad = math.radians(self.yaw)
        quat = euler2quat(roll, pitch, yaw_rad)
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]

        self.pose_pub.publish(pose)
        rospy.loginfo("Published camera pose: pos=(%.2f, %.2f, %.2f), yaw=%.2f",
                      self.position[0], self.position[1], self.position[2], self.yaw)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = CameraTeleopController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
