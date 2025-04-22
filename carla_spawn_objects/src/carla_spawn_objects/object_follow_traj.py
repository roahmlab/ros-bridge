#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

def move_objects():
    pub = rospy.Publisher('/object_transform', TransformStamped, queue_size=10)
    rospy.init_node('moving_obstacle_pub', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time
        t_wait = 20.0
        if t < 10.0:
            x_position = 0.0
        # Move in straight line along x-axis
        else:    
            x_position = 2.0 -0.5*(t-t_wait)
        
        transform_msg = TransformStamped()
        transform_msg.header = Header()
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.frame_id = "world"
        transform_msg.transform.translation.x = x_position
        transform_msg.transform.translation.y = -2.0
        transform_msg.transform.translation.z = 0.0
        
        # Identity quaternion (no rotation)
        transform_msg.transform.rotation.x = 0.0
        transform_msg.transform.rotation.y = 0.0
        transform_msg.transform.rotation.z = 0.0
        transform_msg.transform.rotation.w = 1.0

        pub.publish(transform_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_objects()
    except rospy.ROSInterruptException:
        pass

