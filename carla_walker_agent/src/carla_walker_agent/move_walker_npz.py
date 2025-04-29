#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

# ----------------------------------------------------------------------
NPZ_FILE            = "/home/carla/PythonExamples/Mcity_trajectories.npz"
ROLE_NAME           = "walker_01"
FRAME_ID            = "map"
SUBSAMPLE_STEP      = 20            # take every 100-th point
Y_OFFSET            = -2.0           # match previous scripts
TARGET_SPEED        = 1.0            # m s⁻¹
# ----------------------------------------------------------------------

def publish_waypoints():
    rospy.init_node("walker_waypoint_publisher", anonymous=True)

    path_pub  = rospy.Publisher(f"/carla/{ROLE_NAME}/waypoints",
                                Path,   queue_size=1, latch=True)
    speed_pub = rospy.Publisher(f"/carla/{ROLE_NAME}/target_speed",
                                Float64, queue_size=1, latch=True)

    # -------- load & preprocess trajectory ---------------------------------
    try:
        data = np.load(NPZ_FILE)
        traj = data["ego_traj"]          # shape (N,3)  ->  x,y,yaw
    except (IOError, KeyError) as e:
        rospy.logfatal(f"Cannot load '{NPZ_FILE}': {e}")
        return

    traj = traj[::SUBSAMPLE_STEP]        # subsample
    traj[:,1] += Y_OFFSET               # y-shift

    # -------- build Path message -------------------------------------------
    path                = Path()
    path.header.stamp   = rospy.Time.now()
    path.header.frame_id= FRAME_ID

    for x, y, _ in traj:
        ps                  = PoseStamped()
        ps.header           = path.header      # same stamp / frame
        ps.pose.position.x  = float(x)
        ps.pose.position.y  = float(y)
        ps.pose.position.z  = 0.4              # walker pelvis height
        path.poses.append(ps)

    rospy.loginfo("Publishing %d waypoints (step=%d) from '%s'",
                  len(path.poses), SUBSAMPLE_STEP, NPZ_FILE)
    path_pub.publish(path)

    # -------- speed ---------------------------------------------------------
    speed_pub.publish(Float64(data=TARGET_SPEED))
    rospy.loginfo("Publishing target speed %.2f m/s", TARGET_SPEED)

    rospy.spin()                          # keep latched topics alive

if __name__ == "__main__":
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass
