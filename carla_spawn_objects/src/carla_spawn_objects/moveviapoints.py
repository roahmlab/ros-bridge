#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

def lerp(start, end, t):
    """Linearly interpolate between two values."""
    return start + (end - start) * t

def publish_transform(pub, x, y, z):
    """Publish a TransformStamped message with the given translation and identity rotation."""
    transform_msg = TransformStamped()
    transform_msg.header = Header()
    transform_msg.header.stamp = rospy.Time.now()
    transform_msg.header.frame_id = "world"
    
    transform_msg.transform.translation.x = x
    transform_msg.transform.translation.y = y
    transform_msg.transform.translation.z = z
    
    # Identity quaternion (no rotation)
    transform_msg.transform.rotation.x = 0.0
    transform_msg.transform.rotation.y = 0.0
    transform_msg.transform.rotation.z = 0.0
    transform_msg.transform.rotation.w = 1.0
    
    pub.publish(transform_msg)

def lerp_through_points():
    rospy.init_node('lerp_through_points', anonymous=True)
    pub = rospy.Publisher('/object_transform', TransformStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz publishing rate

    # Define a series of waypoints (x, y, z)
    points = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0)
    ]
    
    segment_duration = 5.0  # seconds per segment between waypoints
    steps = int(segment_duration * 10)  # Number of interpolation steps based on the 10Hz rate

    rospy.loginfo("Starting lerp-through-points publisher...")
    while not rospy.is_shutdown():
        # Loop through each segment in the list of points
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i+1]
            for step in range(steps):
                t = float(step) / steps  # fraction between 0 and 1
                x = lerp(start_point[0], end_point[0], t)
                y = lerp(start_point[1], end_point[1], t)
                z = lerp(start_point[2], end_point[2], t)
                
                publish_transform(pub, x, y, z)
                rate.sleep()

if __name__ == '__main__':
    try:
        lerp_through_points()
    except rospy.ROSInterruptException:
        pass

