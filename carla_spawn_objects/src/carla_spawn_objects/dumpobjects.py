#!/usr/bin/env python
import glob
import os
import sys
import time
import argparse
import logging

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def main():
    argparser = argparse.ArgumentParser(
        description="Dump static prop blueprints and spawn them one at a time at (0,0,0), removing them after 2 seconds."
    )
    argparser.add_argument('--host', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', default=2000, type=int, help='TCP port (default: 2000)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    # Filter for static prop blueprints; adjust the filter if needed.
    static_prop_blueprints = blueprint_library.filter("static.prop.*")
    logging.info("Found {} static prop blueprints.".format(len(static_prop_blueprints)))

    # Dump list of static prop blueprints.
    for bp in static_prop_blueprints:
        logging.info("Static prop blueprint: {}".format(bp.id))

    # Define spawn transform at (0,0,0)
    spawn_location = carla.Location(x=0.0, y=0.0, z=0.0)
    spawn_rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    spawn_transform = carla.Transform(spawn_location, spawn_rotation)

    # Spawn each static prop one at a time.
    for bp in static_prop_blueprints:
        logging.info("Spawning static prop: {}".format(bp.id))
        try:
            actor = world.spawn_actor(bp, spawn_transform)
        except Exception as e:
            logging.error("Failed to spawn {}: {}".format(bp.id, e))
            continue

        # Let the prop stay in the scene for 2 seconds.
        time.sleep(2)

        # Remove (destroy) the spawned actor.
        try:
            actor.destroy()
            logging.info("Destroyed static prop: {}".format(bp.id))
        except Exception as e:
            logging.error("Failed to destroy {}: {}".format(bp.id, e))

    logging.info("Finished processing all static props.")

if __name__ == '__main__':
    main()

