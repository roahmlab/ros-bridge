#!/usr/bin/env python

import rospy
import json
import os
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf.transformations
import math

def load_objects(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)
    return data.get("objects", [])

def euler_to_quaternion(roll, pitch, yaw):
    # Convert degrees to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)
    q = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
    return q

def main():
    rospy.init_node('static_transform_publisher_node')

    objects_file = rospy.get_param('~objects_definition_file', '')
    if not objects_file or not os.path.exists(objects_file):
        rospy.logerr("Invalid or missing objects_definition_file parameter.")
        return

    objects = load_objects(objects_file)

    static_broadcaster = StaticTransformBroadcaster()

    transforms = []

    for obj in objects:
        obj_type = obj.get("type", "")
        obj_id = obj.get("id", "")
        spawn_point = obj.get("spawn_point", {})

        # Only process sensors with spawn_point
        if "spawn_point" in obj and "map_camera" in obj_type or "map_lidar" in obj_type:
            x = spawn_point.get("x", 0.0)
            y = spawn_point.get("y", 0.0)
            z = spawn_point.get("z", 0.0)
            roll = spawn_point.get("roll", 0.0)
            pitch = spawn_point.get("pitch", 0.0)
            yaw = spawn_point.get("yaw", 0.0)

            q = euler_to_quaternion(roll, pitch, yaw)

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = obj_id + "_tf"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            transforms.append(t)
            rospy.loginfo(f"Publishing static transform for {obj_id}_tf")

    static_broadcaster.sendTransform(transforms)
    rospy.loginfo("All static transforms published.")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

