#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
pygame tele-operation for a CARLA pedestrian
– streams the front RGB camera and shows live controls.
"""

import math, os, sys, threading, numpy as np, cv2, rospy, pygame
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from carla_msgs.msg import CarlaWalkerControl
from cv_bridge import CvBridge

# ---------- parameters -------------------------------------------------
ROLE_NAME    = rospy.get_param("/walker_role_name", "walker_01")
IMG_TOPIC    = f"/carla/{ROLE_NAME}/rgb_front/image"
CTRL_TOPIC   = f"/carla/{ROLE_NAME}/walker_control_cmd"
FWD_SPEED    = 1.4
BACK_SPEED   = 1.0
ROT_STEP     = math.radians(5)
STRAFE_ANG   = math.radians(90)
HZ           = 20
# -----------------------------------------------------------------------

_KEY_TEXT = [
    "W/S  : forward / back",
    "A/D  : rotate left / right",
    "J/L  : strafe left / right",
    "SPACE: jump",
    "ESC/Q: quit"
]

class WalkerTeleop:
    def __init__(self):
        rospy.init_node("walker_pygame_teleop")

        self.pub  = rospy.Publisher(CTRL_TOPIC, CarlaWalkerControl, queue_size=1)
        self.bridge = CvBridge()

        self.heading = 0.0
        self.image   = None
        self.img_lock= threading.Lock()

        rospy.Subscriber(IMG_TOPIC, Image, self.image_cb, queue_size=1)

        # ── pygame ──────────────────────────────────────────────
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        pygame.init()
        self.font = pygame.font.SysFont("monospace", 14)
        self.screen = pygame.display.set_mode((1280, 1024))
        pygame.display.set_caption("Walker Tele-op  –  ESC to quit")
        self.clock  = pygame.time.Clock()

    # ───────────────────────────────────────────────────────────
    def image_cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)       # pygame wants RGB
        with self.img_lock:
            self.image = cv_img                                # numpy array

    # ───────────────────────────────────────────────────────────
    def spin(self):
        while not rospy.is_shutdown():
            self._process_pygame()
            self.clock.tick(HZ)

    # ───────────────────────────────────────────────────────────
    def _process_pygame(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown("window closed")
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                rospy.signal_shutdown("quit")

        keys = pygame.key.get_pressed()
        self._publish_control(keys)
        self._draw_window(keys)

    # ───────────────────────────────────────────────────────────
    def _publish_control(self, keys):
        ctrl = CarlaWalkerControl()
        dirv = Vector3()

        if keys[pygame.K_a]: self.heading += ROT_STEP
        if keys[pygame.K_d]: self.heading -= ROT_STEP

        if keys[pygame.K_w] or keys[pygame.K_s]:
            sign = 1 if keys[pygame.K_w] else -1
            dirv.x = sign * math.cos(self.heading)
            dirv.y = sign * math.sin(self.heading)
            ctrl.speed = FWD_SPEED if sign == 1 else BACK_SPEED

        if keys[pygame.K_j] or keys[pygame.K_l]:
            sign  = 1 if keys[pygame.K_j] else -1       # left = +1
            angle = self.heading + sign * STRAFE_ANG
            dirv.x = math.cos(angle)
            dirv.y = math.sin(angle)
            ctrl.speed = FWD_SPEED

        if keys[pygame.K_SPACE]:
            ctrl.jump = True

        ctrl.direction = dirv
        self.pub.publish(ctrl)

    # ───────────────────────────────────────────────────────────
    def _draw_window(self, keys):
        with self.img_lock:
            frame = self.image.copy() if self.image is not None else np.zeros((480,640,3), dtype=np.uint8)
        surf = pygame.surfarray.make_surface(np.flipud(frame))           # cv img → Surface
        surf = pygame.transform.rotate(surf, 90)
        self.screen.blit(pygame.transform.scale(surf, self.screen.get_size()), (0,0))

        # semi-transparent overlay for key-map
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        overlay.fill((0,0,0,120))                                        # translucent black
        self.screen.blit(overlay, (0,0))
        
        y = 10
        for line in _KEY_TEXT:
            txt = self.font.render(line, True, (255,255,255))
            self.screen.blit(txt, (10, y))
            y += 18

        # status line
        status = f"heading {math.degrees(self.heading):5.1f}° "
        txt = self.font.render(status, True, (0,255,0))
        self.screen.blit(txt, (10, y+10))

        pygame.display.flip()

# -----------------------------------------------------------------------
if __name__ == "__main__":
    try:
        WalkerTeleop().spin()
    except rospy.ROSInterruptException:
        pass
