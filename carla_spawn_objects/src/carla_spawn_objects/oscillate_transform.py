#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

def oscillate_transform():
    pub = rospy.Publisher('/object_transform', TransformStamped, queue_size=10)
    rospy.init_node('oscillate_transform_pub', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time
        # Oscillate in x between -1.0 and 1.0
        x_position = math.sin(t)
        
        transform_msg = TransformStamped()
        transform_msg.header = Header()
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.frame_id = "world"
        transform_msg.transform.translation.x = x_position
        transform_msg.transform.translation.y = 0.0
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
        oscillate_transform()
    except rospy.ROSInterruptException:
        pass

