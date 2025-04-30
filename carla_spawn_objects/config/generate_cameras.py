#!/usr/bin/env python3

import argparse
import json
import os
import sys

def create_camera(name, spawn, image_size_x, image_size_y, fov, camera_type="rgb"):
    """
    Returns a dictionary representing a single camera sensor (rgb or depth).
    'spawn' is a dictionary with x, y, z, roll, pitch, yaw.
    """
    cam_dict = {
        "type": f"sensor.camera.{camera_type}",  # e.g. sensor.camera.rgb or sensor.camera.depth
        "id": name,
        "spawn_point": {
            "x": spawn["x"],
            "y": spawn["y"],
            "z": spawn["z"],
            "roll": spawn["roll"],
            "pitch": spawn["pitch"],
            "yaw": spawn["yaw"]
        },
        "image_size_x": image_size_x,
        "image_size_y": image_size_y,
        "fov": fov,
        "attached_objects": [
            {
                "type": "actor.pseudo.control",
                "id": "control"
            }
        ]
    }

    # If it's a depth camera, you may want to add extra attributes
    if camera_type == "depth":
        cam_dict.update({
            "lens_circle_falloff": 0.5,
            "lens_circle_multiplier": 1.0,
            "lens_k": 0.1,
            "lens_kcube": 0.1,
            "lens_x_size": 0.05,
            "lens_y_size": 0.05,
            "sensor_tick": 0.05
        })

    return cam_dict

def create_imu(name, spawn):
    """
    Returns a dictionary for an IMU sensor.
    """
    imu_dict = {
        "type": "sensor.other.imu",
        "id": name,
        "spawn_point": {
            "x": spawn["x"],
            "y": spawn["y"],
            "z": spawn["z"],
            "roll": spawn["roll"],
            "pitch": spawn["pitch"],
            "yaw": spawn["yaw"]
        },
        "noise_accel_stddev_x": 0.02,
        "noise_accel_stddev_y": 0.02,
        "noise_accel_stddev_z": 0.02,
        "noise_gyro_bias_x": 0.001,
        "noise_gyro_bias_y": 0.001,
        "noise_gyro_bias_z": 0.001,
        "noise_gyro_stddev_x": 0.01,
        "noise_gyro_stddev_y": 0.01,
        "noise_gyro_stddev_z": 0.01,
        "noise_seed": 123,   # could randomize or handle uniquely
        "role_name": "imu",
        "sensor_tick": 0.01
    }
    return imu_dict


def create_lidar(name, spawn, range, channels, points_per_second, upper_fov, lower_fov, rotation_frequency, noise_stddev):
    """
    Returns a dictionary for a LiDAR sensor with customizable parameters.
    """
    lidar_dict = {
        "type": "sensor.lidar.ray_cast",
        "id": name,
        "spawn_point": {
            "x": spawn["x"],
            "y": spawn["y"],
            "z": spawn["z"],
            "roll": spawn["roll"],
            "pitch": spawn["pitch"],
            "yaw": spawn["yaw"]
        },
        "range": range,
        "channels": channels,
        "points_per_second": points_per_second,
        "upper_fov": upper_fov,
        "lower_fov": lower_fov,
        "rotation_frequency": rotation_frequency,
        "noise_stddev": noise_stddev,
        "attached_objects": [
            {
                "type": "actor.pseudo.control",
                "id": "control"
            }
        ]
    }
    return lidar_dict

def load_lidar_settings(path):
    """
    Load LiDAR spawn settings from a file with validation and default handling.
    """
    default_lidar = {
        "name": "default_lidar",
        "spawn": {"x": 0.0, "y": 0.0, "z": 5.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "range": 100,
        "channels": 32,
        "points_per_second": 576000,
        "upper_fov": 2.81,
        "lower_fov": -89,
        "rotation_frequency": 10,
        "noise_stddev": 0.1
    }

    if not os.path.isfile(path):
        print(f"Error: {path} not found. Using default LiDAR settings.")
        return [default_lidar]

    try:
        with open(path, 'r') as fh:
            lidar_spawns = json.load(fh)

        validated_spawns = []
        for spawn in lidar_spawns:
            # Create a copy of the default and update it with the custom values
            validated_spawn = default_lidar.copy()
            validated_spawn.update(spawn)
            validated_spawns.append(validated_spawn)

        return validated_spawns

    except json.JSONDecodeError:
        print(f"Error: Could not decode JSON from {path}. Using default settings.")
        return [default_lidar]
    except Exception as e:
        print(f"Error loading LiDAR settings: {e}. Using default settings.")
        return [default_lidar]

        
def load_camera_settings(path):
    """
    Load camera spawn settings from a file.
    """
    if not os.path.isfile(path):
        # If we can't load it, return an example list with just 1 camera
        return [{
            "name": "default_camera",
            "spawn": {"x": 0.0, "y": 0.0, "z": 3.0, "roll":0, "pitch":0, "yaw":0},
            "image_size_x": 1280,
            "image_size_y": 720,
            "fov": 90
        }]

    with open(path, 'r') as fh:
        data = json.load(fh)
    return data
    
def generate_lidar_sensors(args, objects_list):
    """
    Generate LiDAR sensors based on the LiDAR settings file.
    """
    lidar_spawns = load_lidar_settings(args.lidar_settings_file)

    for i in range(args.num_lidars):
        spawn_info = lidar_spawns[min(i, len(lidar_spawns) - 1)]
        lidar_obj = create_lidar(
            name=spawn_info["name"],
            spawn=spawn_info["spawn"],
            range=spawn_info["range"],
            channels=spawn_info["channels"],
            points_per_second=spawn_info["points_per_second"],
            upper_fov=spawn_info["upper_fov"],
            lower_fov=spawn_info["lower_fov"],
            rotation_frequency=spawn_info["rotation_frequency"],
            noise_stddev=spawn_info["noise_stddev"]
        )
        objects_list.append(lidar_obj)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--num-stereo', type=int, default=0, help='Number of stereo camera pairs')
    parser.add_argument('--num-mono', type=int, default=0, help='Number of single/mono cameras')
    parser.add_argument('--num-lidars', type=int, default=0, help='Number of LiDAR sensors')
    parser.add_argument('--stereo-offset', type=float, default=0.0, help='Offset between L/R camera for stereo pairs')
    parser.add_argument('--enable-depth', action='store_true', help='Add a depth camera for each camera or stereo pair?')
    parser.add_argument('--enable-imu', action='store_true', help='Add an IMU sensor at the same location as each camera?')
    parser.add_argument('--camera-settings-file', type=str, default='', help='File with camera/spawn settings')
    parser.add_argument('--lidar-settings-file', type=str, default='', help='File with LiDAR spawn settings')
    parser.add_argument('--output', type=str, default='carla_sensors.json', help='Output JSON file name')
    args = parser.parse_args()

    # 1) Load base camera spawn settings from file (or a fallback if file missing).
    #    We'll cycle through them as we create cameras. If user wants more cameras
    #    than lines in the file, we can wrap around or just reuse the last entry.
    camera_settings = load_camera_settings(args.camera_settings_file)

    # 2) Start building the "objects" list with pseudo sensors (traffic lights, objects, etc.)
    objects_list = [

    ]

    # We'll keep a simple function to index into camera_settings safely
    def get_camera_setting(idx):
        if idx < len(camera_settings):
            return camera_settings[idx]
        else:
            # If we run out, just reuse the last setting
            return camera_settings[-1]

    # 3) Generate stereo camera pairs
    stereo_count = args.num_stereo
    for i in range(stereo_count):
        base_settings = get_camera_setting(i)
        name_base = base_settings["name"] if "name" in base_settings else f"stereo_{i+1}"

        # We'll place the 'left' camera exactly at the base location
        # and the 'right' camera offset in X or Y by args.stereo_offset
        # (You can decide which axis to offset: x, y, or a combination).
        # We'll just assume offset in X for demonstration.
        left_spawn = dict(base_settings["spawn"])  # copy
        right_spawn = dict(base_settings["spawn"])  # copy
        right_spawn["x"] = right_spawn["x"] + args.stereo_offset  # offset in x

        # Create left camera (RGB)
        left_cam = create_camera(
            name=f"{name_base}_left",
            spawn=left_spawn,
            image_size_x=base_settings["image_size_x"],
            image_size_y=base_settings["image_size_y"],
            fov=base_settings["fov"],
            camera_type="rgb"
        )
        objects_list.append(left_cam)

        # Create right camera (RGB)
        right_cam = create_camera(
            name=f"{name_base}_right",
            spawn=right_spawn,
            image_size_x=base_settings["image_size_x"],
            image_size_y=base_settings["image_size_y"],
            fov=base_settings["fov"],
            camera_type="rgb"
        )
        objects_list.append(right_cam)

        # Depth camera (optional)
        if args.enable_depth:
            depth_spawn = {
                "x": (left_spawn["x"] + right_spawn["x"]) / 2,
                "y": (left_spawn["y"] + right_spawn["y"]) / 2,
                "z": (left_spawn["z"] + right_spawn["z"]) / 2,
                "roll": left_spawn["roll"],
                "pitch": left_spawn["pitch"],
                "yaw": left_spawn["yaw"]
            }
            depth_cam = create_camera(
                name=f"{name_base}_depth",
                spawn=depth_spawn,
                image_size_x=base_settings["image_size_x"],
                image_size_y=base_settings["image_size_y"],
                fov=base_settings["fov"],
                camera_type="depth"
            )
            objects_list.append(depth_cam)

        # IMU (optional)
        if args.enable_imu:
            imu_spawn = {
                "x": (left_spawn["x"] + right_spawn["x"]) / 2,
                "y": (left_spawn["y"] + right_spawn["y"]) / 2,
                "z": (left_spawn["z"] + right_spawn["z"]) / 2,
                "roll": left_spawn["roll"],
                "pitch": left_spawn["pitch"],
                "yaw": left_spawn["yaw"]
            }
            imu = create_imu(
                name=f"{name_base}_imu",
                spawn=imu_spawn
            )
            objects_list.append(imu)


    # 4) Generate single (mono) cameras
    mono_count = args.num_mono
    for i in range(mono_count):
        base_settings = get_camera_setting(stereo_count + i)
        name_base = base_settings["name"] if "name" in base_settings else f"mono_{i+1}"

        # Create the mono camera
        mono_cam = create_camera(
            name=f"{name_base}_mono",
            spawn=base_settings["spawn"],
            image_size_x=base_settings["image_size_x"],
            image_size_y=base_settings["image_size_y"],
            fov=base_settings["fov"],
            camera_type="rgb"
        )
        objects_list.append(mono_cam)

        # Depth camera (optional)
        if args.enable_depth:
            depth_cam = create_camera(
                name=f"{name_base}_depth",
                spawn=base_settings["spawn"],
                image_size_x=base_settings["image_size_x"],
                image_size_y=base_settings["image_size_y"],
                fov=base_settings["fov"],
                camera_type="depth"
            )
            objects_list.append(depth_cam)

        # IMU (optional)
        if args.enable_imu:
            imu = create_imu(
                name=f"{name_base}_imu",
                spawn=base_settings["spawn"]
            )
            objects_list.append(imu)

    # 5) Generate LiDAR sensors
    # Generate LiDAR sensors
    generate_lidar_sensors(args, objects_list)


    # 6) Build final dictionary
    final_dict = {"objects": objects_list}

    # 7) Output
    with open(args.output, 'w') as out_f:
        json.dump(final_dict, out_f, indent=4)
    print(f"Generated file: {args.output}")


if __name__ == "__main__":
    main()

