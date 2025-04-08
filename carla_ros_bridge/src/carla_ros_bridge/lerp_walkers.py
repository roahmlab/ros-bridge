#!/usr/bin/env python3

import rospy
import math
import time
import random
import sys

# Make sure your CARLA egg is appended
egg_path = '/home/carla/carla/PythonAPI/carla/dist/carla-0.9.14-py3.8-linux-x86_64.egg'
sys.path.append(egg_path)

import carla


def lerp(a, b, t):
    """ Simple linear interpolation between scalars a and b. """
    return a + t * (b - a)


def calculate_yaw(from_pos, to_pos):
    """
    Calculate the yaw angle (in degrees) to face from_pos toward to_pos.
    """
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    yaw = math.atan2(dy, dx)  # Yaw angle in radians
    return math.degrees(yaw)  # Convert to degrees


def main():
    # Connect to CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Get walker blueprint
    blueprint_library = world.get_blueprint_library()
    walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))

    # Get a random spawn point
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("No spawn points available. Exiting.")
        return
    spawn_point = random.choice(spawn_points)

    # Spawn the walker
    walker = world.try_spawn_actor(walker_bp, spawn_point)
    if not walker:
        print("Failed to spawn walker. Exiting.")
        return

    print(f"Walker {walker.id} spawned at {spawn_point.location}")

    # Define waypoints (x, y, z, T)
    waypoints = [
        (spawn_point.location.x, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y, spawn_point.location.z, 3.0),
        (spawn_point.location.x + 10, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y + 10, spawn_point.location.z, 3.0),
        (spawn_point.location.x, spawn_point.location.y, spawn_point.location.z, 3.0)
    ]

    positions = [(x, y, z) for x, y, z, _ in waypoints]
    segment_durations = [T for _, _, _, T in waypoints[:-1]]

    total_duration = sum(segment_durations)
    num_segments = len(segment_durations)

    # Force the walker to the first position
    first_pos = positions[0]
    walker.set_transform(carla.Transform(
        carla.Location(x=first_pos[0], y=first_pos[1], z=first_pos[2]),
        carla.Rotation(yaw=0.0)
    ))
    time.sleep(0.5)

    start_time = time.time()
    update_rate = 10.0  # Hz

    try:
        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            if elapsed >= total_duration and num_segments > 0:
                # Done, snap to last position
                final_pos = positions[-1]
                loc = carla.Location(x=final_pos[0], y=final_pos[1], z=final_pos[2])
                final_yaw = calculate_yaw(positions[-2], positions[-1])
                rot = carla.Rotation(yaw=final_yaw)
                walker.set_transform(carla.Transform(loc, rot))
                print("Reached final waypoint. Exiting loop.")
                break

            # Figure out which segment we are in
            segment_index = 0
            cumulative_time = 0.0
            for i, seg_time in enumerate(segment_durations):
                if elapsed < cumulative_time + seg_time:
                    segment_index = i
                    break
                cumulative_time += seg_time
            else:
                segment_index = num_segments - 1

            # Fraction in current segment
            segment_time = segment_durations[segment_index]
            time_in_segment = elapsed - cumulative_time
            t = time_in_segment / segment_time if segment_time > 0 else 1.0

            # Lerp positions
            p0 = positions[segment_index]
            p1 = positions[segment_index + 1]
            x = lerp(p0[0], p1[0], t)
            y = lerp(p0[1], p1[1], t)
            z = lerp(p0[2], p1[2], t)

            # Calculate yaw to face the direction of movement
            yaw = calculate_yaw(p0, p1)

            loc = carla.Location(x=x, y=y, z=z)
            rot = carla.Rotation(yaw=yaw)
            walker.set_transform(carla.Transform(loc, rot))

            print(f"Segment {segment_index}/{num_segments}, t={t:.2f}, Pos=({x:.2f},{y:.2f},{z:.2f}), Yaw={yaw:.2f}")
            time.sleep(1.0 / update_rate)

    finally:
        # Cleanup
        print("Destroying walker...")
        walker.destroy()


if __name__ == '__main__':
    main()

