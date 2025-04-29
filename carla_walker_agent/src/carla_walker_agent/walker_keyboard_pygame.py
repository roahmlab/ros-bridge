#!/usr/bin/env python
# walker_pygame_control.py
# ----------------------------------------------------------
import math, os, sys
import rospy, pygame
from geometry_msgs.msg import Vector3
from carla_msgs.msg import CarlaWalkerControl

# ---------- parameters ------------------------------------
ROLE_NAME   = rospy.get_param("/walker_role_name", "walker_01")
TOPIC       = "/carla/{}/walker_control_cmd".format(ROLE_NAME)
FWD_SPEED   = 1.4
BACK_SPEED  = 1.0
ROT_STEP    = math.radians(5)       # radians per tick while A/D held
STRAFE_ANG  = math.radians(90)
PUBLISH_HZ  = 20
# -----------------------------------------------------------

class WalkerPygameTeleop:
    def __init__(self):
        rospy.init_node("walker_pygame_control")
        self.pub     = rospy.Publisher(TOPIC, CarlaWalkerControl, queue_size=1)

        # pygame init (headless-safe)
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        pygame.init()
        self.screen = pygame.display.set_mode((320, 80))
        pygame.display.set_caption("Walker Control – press ESC to quit")

        self.heading = 0.0        # yaw (rad)
        self.crouch  = False
        self.clock   = pygame.time.Clock()

    # -------------------------------------------------------
    def spin(self):
        while not rospy.is_shutdown():
            self._process_events()
            self._publish_from_keys()
            self.clock.tick(PUBLISH_HZ)

    # -------------------------------------------------------
    def _process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown("pygame window closed")
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                rospy.signal_shutdown("quit key")

    def _publish_from_keys(self):
        keys = pygame.key.get_pressed()
        ctrl = CarlaWalkerControl()
        dirv = Vector3()

        move_fwd  = keys[pygame.K_w]
        move_back = keys[pygame.K_s]
        rotate_l  = keys[pygame.K_a]
        rotate_r  = keys[pygame.K_d]
        strafe_l  = keys[pygame.K_j]
        strafe_r  = keys[pygame.K_l]

        # heading updates (continuous while key held)
        if rotate_l: self.heading += ROT_STEP
        if rotate_r: self.heading -= ROT_STEP

        # forward/back direction vector
        if move_fwd or move_back:
            sign = 1 if move_fwd else -1
            dirv.x = sign * math.cos(self.heading)
            dirv.y = sign * math.sin(self.heading)
            ctrl.speed = FWD_SPEED if sign == 1 else BACK_SPEED

        # strafing overrides direction
        if strafe_l or strafe_r:
            sign = 1 if strafe_l else -1
            angle = self.heading + sign * STRAFE_ANG
            dirv.x = math.cos(angle)
            dirv.y = math.sin(angle)
            ctrl.speed = FWD_SPEED

        # jump / crouch toggles (edge-trigger)
        if keys[pygame.K_SPACE]:
            ctrl.jump = True
        if keys[pygame.K_c]:
            if not self.crouch:    # toggle on key down only
                self.crouch = True
            ctrl.crouch = self.crouch
        else:
            self.crouch = False

        ctrl.direction = dirv
        self.pub.publish(ctrl)

# -----------------------------------------------------------
if __name__ == "__main__":
    try:
        WalkerPygameTeleop().spin()
    except rospy.ROSInterruptException:
        pass
