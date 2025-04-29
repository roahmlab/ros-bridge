#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Keyboard tele-operation for a CARLA pedestrian (walker).

ROS 1 node – publishes carla_msgs/CarlaWalkerControl.
"""

import math, sys, select, termios, tty, threading
import rospy
from geometry_msgs.msg import Vector3
from carla_msgs.msg import CarlaWalkerControl

# ------------------------------------------------------------
ROLE_NAME        = rospy.get_param("/walker_role_name", "walker_01")
TOPIC            = "/carla/{}/walker_control_cmd".format(ROLE_NAME)
ROT_STEP_DEG     = 5.0               # A / D   rotate
STRAFE_DIR_DEG   = 90.0              # J / L   side-step
FWD_SPEED        = 1.4               # m/s
BACK_SPEED       = 1.0               # m/s
# ------------------------------------------------------------

class WalkerTeleop(object):
    def __init__(self):
        rospy.init_node("walker_keyboard_control")

        self.pub = rospy.Publisher(TOPIC, CarlaWalkerControl,
                                   queue_size=1)

        self.heading = 0.0           # yaw in rad; 0 = +X (UE) == East
        self.crouch  = False

        self._running = True
        self.kb_thread = threading.Thread(target=self._keyboard_loop)
        self.kb_thread.daemon = True
        self.kb_thread.start()

        rospy.loginfo("""Walker tele-op started  (role_name: %s)
W/S : forward / back
A/D : rotate left / right
J/L : step left / right
C   : toggle crouch
SPACE: jump
Q/ESC: quit
""" % ROLE_NAME)

        self.spin()

    # --------------------------------------------------------

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self._running:
            rate.sleep()

    # --------------------------------------------------------
    # keyboard handling
    def _keyboard_loop(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self._running and not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1)
                    if key in ('\x03', '\x1b', 'q'):     # Ctrl-C / Esc / q
                        self._running = False
                        rospy.signal_shutdown("User exit")
                        break
                    self._process_key(key.lower())
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    # --------------------------------------------------------
    def _process_key(self, k):
        ctrl = CarlaWalkerControl()
        dir_vec = Vector3()

        if k == 'w':                                     # forward
            dir_vec.x =  math.cos(self.heading)
            dir_vec.y =  math.sin(self.heading)
            ctrl.speed = FWD_SPEED

        elif k == 's':                                   # back
            dir_vec.x = -math.cos(self.heading)
            dir_vec.y = -math.sin(self.heading)
            ctrl.speed = BACK_SPEED

        elif k == 'a':                                   # rotate left
            self.heading += math.radians(ROT_STEP_DEG)

        elif k == 'd':                                   # rotate right
            self.heading -= math.radians(ROT_STEP_DEG)

        elif k == 'j':                                   # strafe left
            angle = self.heading + math.radians(+STRAFE_DIR_DEG)
            dir_vec.x = math.cos(angle)
            dir_vec.y = math.sin(angle)
            ctrl.speed = FWD_SPEED

        elif k == 'l':                                   # strafe right
            angle = self.heading + math.radians(-STRAFE_DIR_DEG)
            dir_vec.x = math.cos(angle)
            dir_vec.y = math.sin(angle)
            ctrl.speed = FWD_SPEED

        elif k == 'c':                                   # crouch toggle
            self.crouch = not self.crouch
            ctrl.crouch = self.crouch

        elif k == ' ':                                   # jump
            ctrl.jump = True

        # placeholder: grabbing action would be implemented here
        # elif k == 'g':
        #     publish custom GrabObject message...

        else:
            return                                      # ignore other keys

        ctrl.direction = dir_vec
        self.pub.publish(ctrl)

# ------------------------------------------------------------

if __name__ == "__main__":
    try:
        WalkerTeleop()
    except rospy.ROSInterruptException:
        pass
