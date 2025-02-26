#!/usr/bin/env python

import rospy
import math
import sys
import select
import termios
import tty
import threading
from geometry_msgs.msg import Pose
from transforms3d.euler import euler2quat

class KeyboardCameraController:
    def __init__(self):
        rospy.init_node('keyboard_camera_controller', anonymous=True)

        # Publisher for the camera pose
        self.pose_topic = rospy.get_param('~pose_topic', '/carla/map_camera_video/control/set_transform')
        self.pose_depth_topic = rospy.get_param('~depth_pose_topic', '/carla/map_camera_depth/control/set_transform')
        self.pose_pub = rospy.Publisher(self.pose_topic, Pose, queue_size=10)
        self.pose_depth_pub = rospy.Publisher(self.pose_depth_topic, Pose, queue_size=10)

        # Set update rate (Hz)
        self.rate = rospy.get_param('~update_rate', 10)

        # Initialize camera pose state (position, yaw, and pitch).
        self.position = [0.0, 0.0, 3.0]  # x, y, z (meters)
        self.yaw = 0.0                  # yaw in degrees (rotation around z)
        self.pitch = 0.0                # pitch in degrees (rotation up/down)

        # Step sizes for translation (meters) and rotation (degrees)
        self.move_step = 0.25
        self.yaw_step = 1
        self.pitch_step = 0.5
        # Flag to control the keyboard thread
        self.running = True

        # Start a timer to publish the current pose at a fixed rate.
        rospy.Timer(rospy.Duration(1.0/self.rate), self.publish_pose)

        # Start a separate thread to listen for keyboard input.
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        rospy.loginfo(
            "Keyboard Camera Controller started.\n"
            "Use keys:\n"
            "  w/s : increase/decrease x (forward/backward)\n"
            "  a/d : increase/decrease y (left/right)\n"
            "  q/e : increase/decrease z (up/down)\n"
            "  j/l : rotate yaw (left/right)\n"
            "  i/k : increase/decrease pitch (up/down)\n"
            "Press Ctrl-C to exit."
        )

    def keyboard_loop(self):
        # Save terminal settings
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown() and self.running:
                # Check if there is keyboard input (non-blocking)
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == '\x03':  # Ctrl-C
                        break
                    self.process_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
            self.running = False

    def process_key(self, key):
        # Update the camera pose based on key presses.
        # Adjust keys as desired.
        if key == 'w':
            self.position[0] += self.move_step
        elif key == 's':
            self.position[0] -= self.move_step
        elif key == 'a':
            self.position[1] += self.move_step
        elif key == 'd':
            self.position[1] -= self.move_step
        elif key == 'q':
            self.position[2] += self.move_step
        elif key == 'e':
            self.position[2] -= self.move_step
        elif key == 'j':
            self.yaw += self.yaw_step
        elif key == 'l':
            self.yaw -= self.yaw_step
        elif key == 'i':
            self.pitch += self.pitch_step
        elif key == 'k':
            self.pitch -= self.pitch_step

        rospy.loginfo("Updated pose: pos=%s, yaw=%.1f, pitch=%.1f",
                      self.position, self.yaw, self.pitch)

    def publish_pose(self, event):
        # Create and publish a Pose message based on the current state.
        pose = Pose()
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = self.position[2]

        # Convert yaw and pitch (with roll assumed to be zero) to a quaternion.
        roll = 0.0
        pitch_rad = math.radians(self.pitch)
        yaw_rad = math.radians(self.yaw)
        quat = euler2quat(roll, pitch_rad, yaw_rad)
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]

        self.pose_pub.publish(pose)
        self.pose_depth_pub.publish(pose)

    def run(self):
        rospy.loginfo("Starting Keyboard Camera Controller Node")
        rospy.spin()
        self.running = False

if __name__ == '__main__':
    try:
        controller = KeyboardCameraController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
