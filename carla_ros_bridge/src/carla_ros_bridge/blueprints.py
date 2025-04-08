import carla

def main():
    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world and blueprint library
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # Filter and list all vehicle blueprints
    print("Available vehicle blueprints:")
    for blueprint in blueprint_library.filter('vehicle.*'):
        print(blueprint.id)

if __name__ == '__main__':
    main()
