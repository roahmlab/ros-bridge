import os
import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=get_package_share_directory(
                'carla_spawn_objects') + '/config/map_objects.json'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_sensors_only',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_id',
            default_value='control'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'launch/carla_spawn_objects.launch.py')
            ),
            launch_arguments={
                'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file'),
                'spawn_sensors_only': launch.substitutions.LaunchConfiguration('spawn_sensors_only')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
