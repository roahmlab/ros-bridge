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
    # while start_time != 0.0:
    #     start_time = rospy.Time.now().to_sec()
    #     rate.sleep()

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time   
        # x_position = 2.0 - 0.5*(t)
        
        start_x = 2.0
        t_wait = 15.0
        if t < t_wait:
            x_position = start_x
        # Move in straight line along x-axis
        else:    
            x_position = start_x - 0.5*(t-t_wait)
        
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

