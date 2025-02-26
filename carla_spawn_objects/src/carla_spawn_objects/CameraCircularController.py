#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Pose
from transforms3d.euler import euler2quat

def slerp(q0, q1, t):
    def normalize_q(q):
        norm = math.sqrt(sum(x*x for x in q))
        return [x/norm for x in q]

    q0 = normalize_q(q0)
    q1 = normalize_q(q1)

    dot = sum(q0[i]*q1[i] for i in range(4))

    # If dot < 0, reverse q1 to avoid taking the long way around the sphere
    if dot < 0.0:
        q1 = [-q for q in q1]
        dot = -dot

    if dot > 0.9995:
        # Very close, just lerp and normalize
        result = [q0[i] + t*(q1[i]-q0[i]) for i in range(4)]
        return normalize_q(result)

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)

    theta = theta_0 * t
    sin_theta = math.sin(theta)

    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0

    result = [
        q0[0]*s0 + q1[0]*s1,
        q0[1]*s0 + q1[1]*s1,
        q0[2]*s0 + q1[2]*s1,
        q0[3]*s0 + q1[3]*s1
    ]
    return result

class CameraWaypointController:
    def __init__(self):
        rospy.init_node('camera_waypoint_controller', anonymous=True)

        # Now each waypoint (except the last) includes a T for the time to reach the next waypoint
        # Example: [x_cm, y_cm, z_cm, roll_deg, pitch_deg, yaw_deg, T]
        # The last waypoint doesn't need T since it's the end of the path.
        raw_waypoints = [
        	[-12273, -22278, 27818, 0.0, 0.0, 90.0, 2.0],        
            [5273, -18278, 28818, 0.0, 0.0, 180.0, 1.0],
            [5273, -12278, 28818, 0.0, 0.0, 180.0, 1.0],
            [5922, -3667, 32653, 0.0, 0.0, 180.0, 1.0],
            [15452, 676, 32653, 0.0, 0.0, 90.0, 1.0],
            [26791, 46, 30461, 0.0, 0.0, 90.0, 1.0],
            [27160, 3617, 28555, 0.0, 0.0, 90.0, 2.0],
            [31570, 3617, 28555, 0.0, 0.0, 90.0, 2.0],
            [37983, 3617, 28555, 0.0, 0.0, 0.0, 2.0],
            [38893, 382, 26901, 0.0, 0.0, -90.0, 3.0],
            [36496, 382, 26901, 0.0, 0.0, 180.0, 3.0],
            [31197, -382, 27201, 0.0, 0.0, 90.0,2.0],
            [36496, -382, 27201, 0.0, 0.0, 0.0, 3.0],
            [31197, -382, 27201, 0.0, 0.0, 0.0, 2.0],
            [36496, -382, 27201, 0.0, 0.0, 270.0]
        ]

        self.positions = []
        self.orientations = []
        self.segment_durations = []  # Store T for each segment

        for i, wp in enumerate(raw_waypoints):
            if i < len(raw_waypoints) - 1:
                x_raw, y_raw, z_raw, roll_deg, pitch_deg, yaw_deg, T = wp
            else:
                # Last waypoint doesn't have T
                x_raw, y_raw, z_raw, roll_deg, pitch_deg, yaw_deg = wp
                T = 0.0  # No next segment
            x = x_raw / 100.0
            y = y_raw / 100.0
            z = z_raw / 100.0
            roll_rad = math.radians(roll_deg)
            pitch_rad = math.radians(pitch_deg)
            yaw_rad = math.radians(yaw_deg)

            yaw_rad_corrected = yaw_rad - math.radians(90)
            quat = euler2quat(roll_rad, pitch_rad, yaw_rad_corrected)
            self.positions.append((x, y, z))
            self.orientations.append(quat)

            # If not last waypoint, store the segment duration T
            if i < len(raw_waypoints) - 1:
                self.segment_durations.append(T)

        self.num_waypoints = len(self.positions)
        self.num_segments = self.num_waypoints - 1

        # Total duration is now the sum of segment durations
        self.total_duration = sum(self.segment_durations)

        self.pose_topic = rospy.get_param('~pose_topic', '/carla/map_camera_video/control/set_transform')
        self.pose_pub = rospy.Publisher(self.pose_topic, Pose, queue_size=10)

        self.finished = False

        # Immediately force camera to first waypoint
        self.publish_pose(self.positions[0], self.orientations[0])
        rospy.sleep(0.5)

        # Start timing after forcing the position
        self.start_time = rospy.Time.now().to_sec()

        self.update_rate = 100.0
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_pose)

    def update_pose(self, event):
        if self.finished:
            return

        current_time = rospy.Time.now().to_sec()
        elapsed = current_time - self.start_time

        if elapsed >= self.total_duration and self.num_segments > 0:
            # End of path: set to last waypoint
            final_pos = self.positions[-1]
            final_quat = self.orientations[-1]
            self.publish_pose(final_pos, final_quat)
            rospy.loginfo("Reached final waypoint.")
            self.timer.shutdown()
            self.finished = True
            return

        # Find the current segment based on elapsed time and segment durations
        segment_index = 0
        cumulative_time = 0.0
        for i, seg_time in enumerate(self.segment_durations):
            if elapsed < cumulative_time + seg_time:
                segment_index = i
                break
            cumulative_time += seg_time
        else:
            # If we didn't break, it means elapsed == total_duration
            segment_index = self.num_segments - 1

        # t is the fraction along the current segment
        segment_time = self.segment_durations[segment_index]
        time_in_segment = elapsed - cumulative_time
        t = time_in_segment / segment_time if segment_time > 0 else 1.0

        p0 = self.positions[segment_index]
        p1 = self.positions[segment_index + 1]
        q0 = self.orientations[segment_index]
        q1 = self.orientations[segment_index + 1]

        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        z = p0[2] + t * (p1[2] - p0[2])
        quat = slerp(q0, q1, t)

        self.publish_pose((x, y, z), quat)

        rospy.loginfo("Interpolating: segment %d/%d, t=%.2f, Pos(%.2f,%.2f,%.2f)",
                      segment_index, self.num_segments, t, x, y, z)

    def publish_pose(self, pos, quat):
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]
        self.pose_pub.publish(pose)

    def run(self):
        rospy.loginfo("Starting Camera Waypoint Controller Node")
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = CameraWaypointController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

