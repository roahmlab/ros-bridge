#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
pygame tele-op for CARLA ego vehicle with multi-sensor display
– uses Dubins-style control (throttle + steer)
– streams camera, LiDAR, and radar views; switch via number keys
"""

import os
import math
import threading

import rospy
import numpy as np
import cv2
import pygame
from sensor_msgs.msg import Image, PointCloud2
from carla_msgs.msg import CarlaEgoVehicleControl
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

# ── parameters ────────────────────────────────────────────────────────────
ROLE_NAME         = rospy.get_param('~role_name',         'ego_vehicle')
CAMERA_IDS        = rospy.get_param('~camera_ids',        [
                      'rgb_front','rgb_view',
                      'semantic_segmentation_front','depth_front','dvs_front',
                      'lidar','semantic_lidar','radar_front'])
FORWARD_THROTTLE  = rospy.get_param('~forward_throttle',   0.7)
BACKWARD_THROTTLE = rospy.get_param('~backward_throttle', -0.3)
STEERING_GAIN     = rospy.get_param('~steering_gain',      0.4)
LIDAR_RANGE       = rospy.get_param('~lidar_range',      50.0)     # meters
HZ                = rospy.get_param('~hz',                20)

class EgoVehiclePygameTeleop:
    def __init__(self):
        rospy.init_node('ego_vehicle_pygame_teleop', anonymous=True)
        self.bridge      = CvBridge()
        self.control_pub = rospy.Publisher(
            f"/carla/{ROLE_NAME}/vehicle_control_cmd",
            CarlaEgoVehicleControl, queue_size=1)
        # storage for frames/images
        self.frames      = {cam: None for cam in CAMERA_IDS}
        # subscribe to camera and pointcloud topics
        lock = threading.Lock()
        for cam in CAMERA_IDS:
            topic = f"/carla/{ROLE_NAME}/{cam}"
            if cam in ('lidar','semantic_lidar','radar_front'):
                rospy.Subscriber(topic, PointCloud2,
                                 self._make_pc_cb(cam, lock), queue_size=1)
            else:
                rospy.Subscriber(f"{topic}/image", Image,
                                 self._make_image_cb(cam, lock), queue_size=1)
        self.active_cam_idx = 0
        self.throttle       = 0.0
        self.steer          = 0.0
        # pygame setup
        os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
        pygame.init()
        self.screen = pygame.display.set_mode((1280, 1024))
        pygame.display.set_caption('Ego Vehicle Tele-op')
        self.clock  = pygame.time.Clock()
        self.font   = pygame.font.SysFont('monospace', 14)
        rospy.loginfo('EgoVehiclePygameTeleop initialized. Sensors: %s', CAMERA_IDS)

    def _make_image_cb(self, cam, lock):
        def _cb(msg):
            try:
                # handle color images
                if msg.encoding in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
                    cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                else:
                    # handle single-channel: depth, semantic, dvs
                    cv_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    arr = np.array(cv_raw, copy=False)
                    # normalize floats to 0-255
                    if arr.dtype == np.float32 or arr.dtype == np.float64:
                        arr = np.nan_to_num(arr, nan=0.0)
                        arr = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX)
                        arr = arr.astype(np.uint8)
                    else:
                        arr = arr.astype(np.uint8)
                    # grayscale to RGB
                    cv_img = cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
                with lock:
                    self.frames[cam] = cv_img
            except Exception as e:
                rospy.logwarn('Failed to convert image for %s: %s', cam, e)
        return _cb
    def _make_pc_cb(self, cam, lock):
        def _cb(msg):
            # project pointcloud to top-down view
            width, height = 800, 600
            scale = width / (2 * LIDAR_RANGE)
            img = np.zeros((height, width, 3), dtype=np.uint8)
            try:
                for i, point in enumerate(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)):
                    if i >= 2000: break
                    x, y, _ = point
                    px = int(width/2 + y * scale)
                    py = int(height - x * scale)
                    if 0 <= px < width and 0 <= py < height:
                        img[py, px] = (255,255,255)
            except Exception as e:
                rospy.logwarn('PC convert error %s: %s', cam, e)
            with lock:
                self.frames[cam] = img
        return _cb

    def _process_events(self):
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                rospy.signal_shutdown('Pygame window closed')
            elif e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_ESCAPE, pygame.K_q):
                    rospy.signal_shutdown('Quit by user')
                if pygame.K_0 <= e.key <= pygame.K_9:
                    idx = e.key - pygame.K_0
                    if idx < len(CAMERA_IDS):
                        self.active_cam_idx = idx
        keys = pygame.key.get_pressed()
        f = (1 if keys[pygame.K_w] else 0) + (-1 if keys[pygame.K_s] else 0)
        a = (1 if keys[pygame.K_j] else 0) + (-1 if keys[pygame.K_l] else 0)
        self.throttle = FORWARD_THROTTLE if f > 0 else BACKWARD_THROTTLE if f < 0 else 0.0
        self.steer    = np.clip(a * STEERING_GAIN, -1.0, 1.0)

    def _publish_control(self):
        ctrl = CarlaEgoVehicleControl()
        ctrl.throttle = self.throttle if self.throttle >= 0 else 0.0
        ctrl.brake = -self.throttle if self.throttle < 0 else 0.0
        ctrl.steer = self.steer
        self.control_pub.publish(ctrl)

    def _draw(self):
        cam_id = CAMERA_IDS[self.active_cam_idx]
        frame = self.frames.get(cam_id)
        if frame is None:
            frame = np.zeros((600, 800, 3), np.uint8)
        surf = pygame.surfarray.make_surface(np.flipud(frame))
        surf = pygame.transform.rotate(surf, 90)
        surf = pygame.transform.scale(surf, self.screen.get_size())
        self.screen.blit(surf, (0, 0))
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 120))
        self.screen.blit(overlay, (0, 0))
        lines = [
            'w/s: forward/backward',
            'j/l: turn left/right',
            f'0-{len(CAMERA_IDS)-1}: switch sensor view',
            'ESC/Q: quit'
        ]
        y = 10
        for ln in lines:
            txt = self.font.render(ln, True, (255,255,255))
            self.screen.blit(txt, (10, y)); y += 18
        status = f"View [{self.active_cam_idx}]: {cam_id}   Throttle:{self.throttle:.2f}   Steer:{self.steer:.2f}"
        txt2 = self.font.render(status, True, (0,255,0))
        self.screen.blit(txt2, (10, y))
        pygame.display.flip()

    def spin(self):
        rate = rospy.Rate(HZ)
        while not rospy.is_shutdown():
            self._process_events()
            self._publish_control()
            self._draw()
            self.clock.tick(HZ)
            rate.sleep()

if __name__ == '__main__':
    try:
        EgoVehiclePygameTeleop().spin()
    except rospy.ROSInterruptException:
        pass
