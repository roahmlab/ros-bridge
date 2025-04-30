#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
pygame tele-operation for a CARLA camera
– streams the camera feed and allows keyboard control of the camera pose
"""

import os
import math
import threading

import rospy
import numpy as np
import cv2
import pygame
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from transforms3d.euler import euler2quat

# ── parameters (can be overridden via ROS params) ─────────────────────────
IMAGE_TOPIC   = rospy.get_param('~image_topic',   '/carla/map_camera_video/image')
POSE_TOPIC    = rospy.get_param('~pose_topic',    '/carla/map_camera_video/control/set_transform')
LINEAR_SPEED  = rospy.get_param('~linear_speed',  1.0)           # m/s
ALT_SPEED     = rospy.get_param('~alt_speed',     1.0)           # m/s
YAW_RATE      = rospy.get_param('~yaw_rate',      math.radians(45)) # rad/s
HZ            = rospy.get_param('~hz',            20)            # control / display rate

# ── key mappings (teleop_twist_keyboard style) ────────────────────────────
# w/s : forward / back
# a/d : strafe left / right
# i/k : up / down
# j/l : turn left / right
# ESC/Q: quit

class CameraPygameTeleop:
    def __init__(self):
        rospy.init_node('camera_pygame_teleop', anonymous=True)

        # ROS interfaces
        self.bridge    = CvBridge()
        self.pose_pub  = rospy.Publisher(POSE_TOPIC, Pose, queue_size=1)
        rospy.Subscriber(IMAGE_TOPIC, Image, self._image_cb, queue_size=1)

        # camera state
        self.position = [0.0, 0.0, 10.0]  # x, y, z
        self.yaw      = 0.0               # degrees
        self._cmd_lin = [0.0, 0.0, 0.0]    # vx, vy, vz in local frame
        self._cmd_yaw = 0.0               # yaw rate (rad/s)

        self._last_t = rospy.Time.now().to_sec()
        self._image  = None
        self._img_lock = threading.Lock()

        # pygame setup
        os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
        pygame.init()
        self.screen = pygame.display.set_mode((1280, 1024))
        pygame.display.set_caption('Camera Pygame Tele-op')
        self.clock  = pygame.time.Clock()
        self.font   = pygame.font.SysFont('monospace', 14)

        rospy.loginfo('Camera Pygame Teleop initialized')
        rospy.loginfo('Subscribing to image: %s', IMAGE_TOPIC)
        rospy.loginfo('Publishing pose:    %s', POSE_TOPIC)

    def _image_cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        with self._img_lock:
            self._image = cv_img

    def _process_events(self):
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                rospy.signal_shutdown('Pygame window closed')
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q):
                rospy.signal_shutdown('Quit by user')

        keys = pygame.key.get_pressed()
        # reset commands
        vx = vy = vz = yaw_rate = 0.0
        # forward/back
        if keys[pygame.K_w]: vx = LINEAR_SPEED
        if keys[pygame.K_s]: vx = -LINEAR_SPEED
        # strafe
        if keys[pygame.K_a]: vy =  LINEAR_SPEED
        if keys[pygame.K_d]: vy = -LINEAR_SPEED
        # altitude
        if keys[pygame.K_i]: vz =  ALT_SPEED
        if keys[pygame.K_k]: vz = -ALT_SPEED
        # yaw
        if keys[pygame.K_j]: yaw_rate =  YAW_RATE
        if keys[pygame.K_l]: yaw_rate = -YAW_RATE

        self._cmd_lin = [vx, vy, vz]
        self._cmd_yaw = yaw_rate

    def _update_state(self):
        t = rospy.Time.now().to_sec()
        dt = t - self._last_t
        self._last_t = t

        # transform local to global XY
        yaw_rad = math.radians(self.yaw)
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        vx, vy, vz = self._cmd_lin
        gx = vx * c - vy * s
        gy = vx * s + vy * c

        self.position[0] += gx * dt
        self.position[1] += gy * dt
        self.position[2] += vz * dt

        self.yaw += math.degrees(self._cmd_yaw) * dt

        self._publish_pose()

    def _publish_pose(self):
        p = Pose()
        p.position.x, p.position.y, p.position.z = self.position
        quat = euler2quat(0.0, 0.0, math.radians(self.yaw))
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = quat
        self.pose_pub.publish(p)

    def _draw(self):
        with self._img_lock:
            frame = (self._image.copy() if self._image is not None
                     else np.zeros((480, 640, 3), dtype=np.uint8))
        # correct orientation: flip and rotate CCW 90°
        surf = pygame.surfarray.make_surface(np.flipud(frame))
        surf = pygame.transform.rotate(surf, 90)
        self.screen.blit(pygame.transform.scale(surf, self.screen.get_size()), (0,0))

        # overlay instructions
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        overlay.fill((0,0,0,120))
        self.screen.blit(overlay, (0,0))
        lines = [
            'w/s : forward / back',
            'a/d : strafe left / right',
            'i/k : up / down',
            'j/l : turn left / right',
            'ESC/Q: quit'
        ]
        y = 10
        for ln in lines:
            txt = self.font.render(ln, True, (255,255,255))
            self.screen.blit(txt, (10, y)); y += 18

        status = f"pos=({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f})  yaw={self.yaw:.1f}°"
        txt = self.font.render(status, True, (0,255,0))
        self.screen.blit(txt, (10, y+10))

        pygame.display.flip()

    def spin(self):
        while not rospy.is_shutdown():
            self._process_events()
            self._update_state()
            self._draw()
            self.clock.tick(HZ)

if __name__ == '__main__':
    try:
        CameraPygameTeleop().spin()
    except rospy.ROSInterruptException:
        pass
