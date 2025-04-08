#!/usr/bin/env python
import rospy
import sys
egg_path = '/home/carla/carla/PythonAPI/carla/dist/carla-0.9.14-py3.8-linux-x86_64.egg'

sys.path.append(egg_path)

import carla
import random
import time


def main():
    client = carla.Client('localhost', 10000)
    client.set_timeout(300.0)  # Long timeout for large maps

    try:
        # Get the world and blueprint library
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # Get walker and controller blueprints
        walker_blueprints = blueprint_library.filter('walker.pedestrian.*')
        walker_controller_bp = blueprint_library.find('controller.ai.walker')

        # Decide how many actors to spawn
        num_walkers = random.randint(10, 50)
        print(f"Spawning {num_walkers} walkers.")

        # Percentage of pedestrians running / crossing
        percentage_pedestrians_running = 0.0
        percentage_pedestrians_crossing = 0.0

        # 1. Collect random spawn points
        spawn_points = []
        for _ in range(num_walkers):
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                spawn_points.append(carla.Transform(loc))

        # 2. Spawn the walker actors via batch
        walkers_list = []
        all_ids = []
        walker_speeds = []
        batch = []
        SpawnActor = carla.command.SpawnActor

        for i, transform in enumerate(spawn_points):
            walker_bp = random.choice(walker_blueprints)

            # Make them vulnerable (not invincible)
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')

            # Optionally pick speed.  The recommended_values are often something like [“speed=0”, “speed=1.4”, “speed=2.0”] 
            if walker_bp.has_attribute('speed'):
                # walker_bp.get_attribute('speed').recommended_values -> typically [‘0’, ‘1.39’, ‘2.78’] for some walkers
                if random.random() < percentage_pedestrians_running:
                    speed = walker_bp.get_attribute('speed').recommended_values[2]  # running
                else:
                    speed = walker_bp.get_attribute('speed').recommended_values[1]  # walking
                walker_speeds.append(speed)
            else:
                # If no speed available, fallback
                walker_speeds.append("1.4")

            batch.append(SpawnActor(walker_bp, transform))

        results = client.apply_batch_sync(batch, True)

        # Filter out any spawns that failed
        valid_results = []
        for i, res in enumerate(results):
            if res.error:
                print(f"Failed to spawn walker {i}: {res.error}")
            else:
                walkers_list.append({"id": res.actor_id})
                valid_results.append(res)

        # 3. Spawn controllers for each valid walker
        batch = []
        for i, _ in enumerate(valid_results):
            batch.append(
                SpawnActor(
                    walker_controller_bp,
                    carla.Transform(),  # Controllers can be spawned with an empty transform
                    walkers_list[i]["id"]
                )
            )
        results_controller = client.apply_batch_sync(batch, True)

        # Add controller IDs to walkers_list
        for i, res in enumerate(results_controller):
            if res.error:
                print(f"Failed to spawn controller for walker {i}: {res.error}")
            else:
                walkers_list[i]["con"] = res.actor_id

        # 4. Collect all IDs and get them as actors
        for w in walkers_list:
            all_ids.append(w["con"])
            all_ids.append(w["id"])
        all_actors = world.get_actors(all_ids)

        # Wait a tick to ensure the client has the updated transforms
        world.tick()

        # 5. Start each AI controller and set their destinations
        # Also optionally control how many can cross
        world.set_pedestrians_cross_factor(percentage_pedestrians_crossing)

        for i in range(0, len(all_ids), 2):
            controller = all_actors[i]     # even index
            walker = all_actors[i + 1]     # odd index
            controller.start()
            controller.go_to_location(world.get_random_location_from_navigation())
            try:
                speed = float(walker_speeds[i // 2])
            except ValueError:
                speed = 1.4
            controller.set_max_speed(speed)

        print(f"Spawned {len(walkers_list)} walkers with AI control. Press Ctrl+C to exit.")

        # Keep alive so the AI can keep controlling them
        while True:
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("Simulation interrupted. Cleaning up...")
    finally:
        # Clean up
        print("Destroying walkers...")
        # Stop controllers first
        actors_to_destroy = []
        world_actors = world.get_actors(all_ids)
        for i in range(0, len(all_ids), 2):
            controller = world_actors[i]
            walker = world_actors[i+1]
            if controller.is_alive:
                controller.stop()
            actors_to_destroy.append(controller.id)
            actors_to_destroy.append(walker.id)

        client.apply_batch([carla.command.DestroyActor(x) for x in actors_to_destroy])
        print("Cleanup complete. Exiting.")


if __name__ == "__main__":
    main()
