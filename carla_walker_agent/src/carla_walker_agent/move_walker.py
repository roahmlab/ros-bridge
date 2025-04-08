#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

def publish_waypoints():
    # Initialize the ROS node
    rospy.init_node('walker_waypoint_publisher', anonymous=True)

    # Publishers for waypoints and target speed
    path_pub = rospy.Publisher('/carla/ego_vehicle/waypoints', Path, queue_size=1)
    speed_pub = rospy.Publisher('/carla/ego_vehicle/target_speed', Float64, queue_size=1)

    # Create a Path message
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "map"  # adjust the frame if needed

    # Define two waypoints
    points = [
        (0.0, 0.0, 1.0),     # start at (0,0,1)
        (1000.0, 1000.0, 1.0) # end at (1000,1000,1)
    ]

    # Populate the Path message with PoseStamped messages
    for pt in points:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.position.z = pt[2]
        # Orientation can be left as default (0,0,0,1) for no rotation
        path.poses.append(pose)

    # Wait a bit to ensure publishers are registered
    rospy.sleep(1.0)

    # Publish the path and target speed once
    rospy.loginfo("Publishing waypoints path.")
    path_pub.publish(path)

    target_speed = Float64()
    target_speed.data = 1.0  # target speed of 1
    rospy.loginfo("Publishing target speed: %f", target_speed.data)
    speed_pub.publish(target_speed)

    # Keep the node alive for a while to ensure messages get through
    rospy.sleep(2.0)

if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass

