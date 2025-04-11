#!/usr/bin/env python
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
import keyboard  # This module provides robust key-state detection

class DubinsPathKeyboardControl:
    def __init__(self):
        rospy.init_node('dubinspath_keyboardcontrol', anonymous=True)

        # Publisher for vehicle control commands
        self.control_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',
                                           CarlaEgoVehicleControl, queue_size=10)
        # Subscribe to the ego vehicle's odometry (if needed for additional control logic)
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_callback)
        self.current_pose = None

        # Tunable control parameters:
        self.forward_throttle = rospy.get_param('~forward_throttle', 0.7)
        self.backward_throttle = rospy.get_param('~backward_throttle', -0.3)
        # Lower steering_gain gives a smaller steering command -> larger turning radius
        self.steering_gain = rospy.get_param('~steering_gain', 0.4)

        # Set the control update rate (50 Hz for fast response)
        self.rate = rospy.Rate(50)

        rospy.loginfo(
            "DubinsPathKeyboardControl node started.\n"
            "Hold keys (supports multiple keys simultaneously):\n"
            "  w: move forward\n"
            "  s: move backward\n"
            "  j: turn left\n"
            "  l: turn right\n"
            "Press Ctrl-C to exit."
        )

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose

    def update_control(self):
        # Using the keyboard module to check the state of each key.
        # This method returns True as long as the key is pressed.
        forward_intent = 0.0
        angular_intent = 0.0
        active_keys = []

        if keyboard.is_pressed('w'):
            forward_intent += 1.0
            active_keys.append('w')
        if keyboard.is_pressed('s'):
            forward_intent -= 1.0
            active_keys.append('s')
        if keyboard.is_pressed('j'):
            angular_intent += 1.0
            active_keys.append('j')
        if keyboard.is_pressed('l'):
            angular_intent -= 1.0
            active_keys.append('l')

        control_msg = CarlaEgoVehicleControl()

        if forward_intent != 0.0 or angular_intent != 0.0:
            # Determine throttle based on forward intent.
            if forward_intent > 0:
                control_msg.throttle = self.forward_throttle
                motion = "forward"
            elif forward_intent < 0:
                control_msg.throttle = self.backward_throttle
                motion = "backward"
            else:
                control_msg.throttle = 0.0
                motion = "stationary"

            # Compute steering command with the steering gain.
            control_msg.steer = np.clip(angular_intent * self.steering_gain, -1.0, 1.0)
            if angular_intent > 0:
                turn_direction = "left"
            elif angular_intent < 0:
                turn_direction = "right"
            else:
                turn_direction = "straight"

            rospy.loginfo_throttle(0.5,
                                   "Command: Vehicle moving %s with throttle %.2f, turning %s with steer %.2f; Active keys: %s",
                                   motion, control_msg.throttle, turn_direction, control_msg.steer, active_keys)
        else:
            # No keys are pressed: stop the vehicle.
            control_msg.throttle = 0.0
            control_msg.steer = 0.0
            rospy.loginfo_throttle(1.0,
                                   "Command: Vehicle stopped (no keys pressed). Active keys: %s", active_keys)

        self.control_pub.publish(control_msg)

    def run(self):
        rospy.loginfo("DubinsPathKeyboardControl is running...")
        while not rospy.is_shutdown():
            self.update_control()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = DubinsPathKeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass

