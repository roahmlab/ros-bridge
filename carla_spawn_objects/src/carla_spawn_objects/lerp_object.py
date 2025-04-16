#!/usr/bin/env python
import glob
import os
import sys
import time
import argparse
import logging
import sys
egg_path = '/home/carla/CarlaDepotAutomation/carla-0.9.14-py3.8-linux-x86_64.egg'

sys.path.append(egg_path)

import rospy
import carla
import json
import math
import os


from transforms3d.euler import euler2quat
from carla_msgs.msg import CarlaActorList
from carla_msgs.srv import SpawnObject, DestroyObject
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion

# Helper functions to convert between ROS and CARLA data types
def ros_to_carla_transform(ros_transform):
    """Convert a ROS TransformStamped message into a carla.Transform."""
    # Extract translation
    t = ros_transform.transform.translation
    location = carla.Location(x=t.x, y=t.y, z=t.z)
    
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    q = ros_transform.transform.rotation
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    rotation = carla.Rotation(roll=roll * 180.0/3.14159,
                                pitch=pitch * 180.0/3.14159,
                                yaw=yaw * 180.0/3.14159)
    return carla.Transform(location, rotation)

# (Existing functions like get_object_id_map, disable_objects, etc. remain unchanged)
def get_object_id_map(envobjects):
    name_to_ids = {}
    for obj in envobjects:
        if obj.name in name_to_ids:
            name_to_ids[obj.name].append(obj.id)
        else:
            name_to_ids[obj.name] = [obj.id]
    return name_to_ids

def disable_objects(world, object_ids):
    if not object_ids:
        logging.warning("No object IDs provided to disable.")
        return
    world.enable_environment_objects(object_ids, False)
    logging.info("Disabled environment objects with IDs: {}".format(object_ids))

def enable_objects(world, object_ids):
    if not object_ids:
        logging.warning("No object IDs provided to enable.")
        return
    world.enable_environment_objects(object_ids, True)
    logging.info("Enabled environment objects with IDs: {}".format(object_ids))

def spawn_dynamic_actor(world, blueprint_name, transform):
    blueprint = world.get_blueprint_library().find(blueprint_name)
    if not blueprint:
        logging.error("Blueprint '{}' not found.".format(blueprint_name))
        return None
    blueprint.set_attribute('role_name', 'dynamic_mover')
    try:
        actor = world.spawn_actor(blueprint, transform)
        logging.info("Spawned dynamic actor with ID: {}".format(actor.id))
        return actor
    except Exception as e:
        logging.error("Failed to spawn actor: {}".format(e))
        return None

def lerp(start, end, fraction):
    return carla.Location(
        x=start.x + (end.x - start.x) * fraction,
        y=start.y + (end.y - start.y) * fraction,
        z=start.z + (end.z - start.z) * fraction
    )

def move_actor_smoothly(actor, target_location, duration=5.0, steps=50):
    start_transform = actor.get_transform()
    start_location = start_transform.location
    for step in range(1, steps + 1):
        fraction = step / steps
        new_location = lerp(start_location, target_location, fraction)
        new_transform = carla.Transform(new_location, start_transform.rotation)
        actor.set_transform(new_transform)
        time.sleep(duration / steps)
    logging.info("Completed moving actor ID {} to {}.".format(actor.id, target_location))

# ROS callback to update the actor's transform based on incoming transform messages
def transform_callback(msg, args):
    """
    This callback receives TransformStamped messages from ROS and updates the actor's transform.
    The 'args' parameter is expected to be a dictionary containing at least the following keys:
        - 'actor': the CARLA actor to control.
    """
    actor = args.get('actor')
    if not actor:
        rospy.logwarn("No actor set for transform control.")
        return

    new_transform = ros_to_carla_transform(msg)
    try:
        actor.set_transform(new_transform)
        rospy.logdebug("Updated actor {} transform to: {}".format(actor.id, new_transform))
    except Exception as e:
        rospy.logerr("Failed to update actor transform: {}".format(e))

def main():
    argparser = argparse.ArgumentParser(
        description="Control a CARLA actor via ROS transform messages."
    )
    argparser.add_argument('--host', default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', default=2000, type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--spawn-blueprint-name', type=str, required=True,
                           help='Blueprint name for the dynamic actor (e.g., static.prop.Stack_Box_02_Prefab_01)')
    argparser.add_argument('--initial-transform-x', type=float, default=0.0,
                           help='Initial x position for spawning')
    argparser.add_argument('--initial-transform-y', type=float, default=0.0,
                           help='Initial y position for spawning')
    argparser.add_argument('--initial-transform-z', type=float, default=0.0,
                           help='Initial z position for spawning')
    argparser.add_argument('--initial-roll', type=float, default=0.0,
                           help='Initial roll in degrees')
    argparser.add_argument('--initial-pitch', type=float, default=0.0,
                           help='Initial pitch in degrees')
    argparser.add_argument('--initial-yaw', type=float, default=0.0,
                           help='Initial yaw in degrees')
    argparser.add_argument('--ros-transform-topic', type=str, default='/object_transform',
                           help='ROS topic to subscribe for transform messages (default: /object_transform)')
    argparser.add_argument('--wait-for-carla-status', action='store_true', default=False)

    args, unknown = argparser.parse_known_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    # Set up the initial transform for the actor based on command-line parameters
    initial_location = carla.Location(x=args.initial_transform_x,
                                      y=args.initial_transform_y,
                                      z=args.initial_transform_z)
    initial_rotation = carla.Rotation(roll=args.initial_roll,
                                      pitch=args.initial_pitch,
                                      yaw=args.initial_yaw)
    initial_transform = carla.Transform(initial_location, initial_rotation)

    # Initialize ROS node and subscribe to the transform topic
    rospy.init_node('carla_transform_controller', anonymous=True)

    if args.wait_for_carla_status:        
        from collections import deque
        from carla_msgs.msg import CarlaStatus
        timestamps = deque(maxlen=20)
        
        def cb(msg):
            nonlocal timestamps
            timestamps.append(rospy.Time.now().to_sec())
        
        sub = rospy.Subscriber('/carla/status', CarlaStatus, cb)
        rate = rospy.Rate(10)
        stable_since = None

        while not rospy.is_shutdown():
            if len(timestamps) >= 10:
                intervals = [j - i for i, j in zip(timestamps, list(timestamps)[1:])]
                avg_rate = 1.0 / (sum(intervals) / len(intervals)) if intervals else 0.0
                if avg_rate >= 5.0:
                    if stable_since is None:
                        stable_since = time.time()
                    elif time.time() - stable_since >= 1.0:
                        rospy.loginfo("Detected /carla/status at >5Hz for 1s. Continuing.")
                        break
                else:
                    stable_since = None
            rate.sleep()
        sub.unregister()

    # Spawn the dynamic actor at the initial transform
    dynamic_actor = spawn_dynamic_actor(world, args.spawn_blueprint_name, initial_transform)
    if not dynamic_actor:
        logging.error("Failed to spawn the dynamic actor. Exiting.")
        return
    
    def shutdown_hook():
        if dynamic_actor is not None:
            try:
                dynamic_actor.destroy()
                rospy.loginfo("Destroyed dynamic actor ID: {}".format(dynamic_actor.id))
            except Exception as e:
                rospy.logerr("Failed to destroy actor: {}".format(e))

    # The lambda allows us to pass additional arguments to the callback (the actor)
    rospy.Subscriber(args.ros_transform_topic, TransformStamped,
                     callback=lambda msg: transform_callback(msg, {'actor': dynamic_actor}))
    
    rospy.loginfo("ROS node started. Listening on topic '{}' for transform messages.".format(args.ros_transform_topic))

    # Keep the node running until shutdown
    rospy.spin()
    shutdown_hook()
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nOperation cancelled by user.')
    finally:
        print('Done.')

