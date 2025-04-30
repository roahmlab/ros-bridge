#!/usr/bin/env python

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraVideoRecorder:
    def __init__(self):
        rospy.init_node('camera_video_recorder', anonymous=True)
        
        # Parameters
        # Adjust these as needed
        self.image_topic = rospy.get_param('~image_topic', '/carla/map_camera_video/image')
        self.video_filename = rospy.get_param('~video_filename', 'recorded_video.mp4')
        self.fps = rospy.get_param('~fps', 30)
        self.frame_width = rospy.get_param('~frame_width', 2560)
        self.frame_height = rospy.get_param('~frame_height', 1440)
        
        self.bridge = CvBridge()
        
        # Use 'mp4v' codec for MP4 output
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(self.video_filename, fourcc, self.fps, (self.frame_width, self.frame_height))
        
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.loginfo("CameraVideoRecorder initialized. Recording to %s", self.video_filename)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Check if the size matches what we assumed; if not, adjust dynamically
        if cv_image.shape[1] != self.frame_width or cv_image.shape[0] != self.frame_height:
            rospy.logwarn("Incoming image size (%dx%d) does not match expected (%dx%d). Resizing.",
                          cv_image.shape[1], cv_image.shape[0], self.frame_width, self.frame_height)
            cv_image = cv2.resize(cv_image, (self.frame_width, self.frame_height))
        
        # Write the frame
        self.out.write(cv_image)
    
    def run(self):
        rospy.spin()
        # Release the video writer on shutdown
        self.out.release()
        rospy.loginfo("Video saved to %s", self.video_filename)

if __name__ == '__main__':
    recorder = CameraVideoRecorder()
    recorder.run()

