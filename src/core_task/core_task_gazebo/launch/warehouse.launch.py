#!/usr/bin/env python3
# Brings up the AWS RoboMaker small-warehouse world with a TurtleBot3 Waffle Pi.
#
# Self-contained env so it runs on a clean machine (graded reproducibility):
#   - Sources /usr/share/gazebo/setup.sh (captures its GAZEBO_* exports) so
#     GAZEBO_RESOURCE_PATH includes gazebo's core media dir; without it gzclient
#     aborts with a null rendering::Camera ("Gazebo/shadow_caster" not found).
#   - Prepends this package's models/ to GAZEBO_MODEL_PATH so the warehouse
#     model:// URIs resolve.
#   - Forces TURTLEBOT3_MODEL=waffle_pi (camera + LIDAR).
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TB3_MODEL = 'waffle_pi'


def _nvidia_available():
    """True if an NVIDIA GPU is usable, so gzclient can render the mesh-heavy
    warehouse on the dedicated GPU instead of freezing the integrated one.
    Set CORE_TASK_USE_NVIDIA=0 to force off (e.g. broken offload)."""
    override = os.environ.get('CORE_TASK_USE_NVIDIA')
    if override is not None:
        return override not in ('0', 'false', 'False', '')
    import shutil
    if not shutil.which('nvidia-smi'):
        return False
    try:
        return subprocess.run(['nvidia-smi', '-L'], capture_output=True,
                              timeout=5).returncode == 0
    except Exception:
        return False


def _source_gazebo_setup():
    """Equivalent of `source /usr/share/gazebo/setup.sh`: run it in a subshell
    and return the GAZEBO_* variables it exports. Empty dict if not installed."""
    setup = '/usr/share/gazebo/setup.sh'
    if not os.path.isfile(setup):
        return {}
    out = subprocess.run(
        ['bash', '-c', 'source "%s" && env' % setup],
        capture_output=True, text=True).stdout
    return {
        k: v for k, v in (ln.split('=', 1) for ln in out.splitlines() if '=' in ln)
        if k.startswith('GAZEBO_')
    }


def generate_launch_description():
    pkg_share = get_package_share_directory('core_task_gazebo')
    tb3_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='6.561')
    y_pose = LaunchConfiguration('y_pose', default='2.182')
    yaw = LaunchConfiguration('yaw', default='3.138337')  # face -x, down the aisle
    world = os.path.join(
        pkg_share, 'worlds', 'no_roof_small_warehouse.world')

    # Replicate `source /usr/share/gazebo/setup.sh`.
    gz = _source_gazebo_setup()
    env_actions = [SetEnvironmentVariable(k, v) for k, v in gz.items()]

    # Render on the dedicated NVIDIA GPU when present; the warehouse meshes
    # freeze Intel integrated graphics. No-op on machines without NVIDIA.
    if _nvidia_available():
        env_actions += [
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        ]

    # Prepend the warehouse models onto whatever setup.sh set (or the live env).
    model_base = gz.get('GAZEBO_MODEL_PATH', os.environ.get('GAZEBO_MODEL_PATH', ''))
    model_path = os.path.join(pkg_share, 'models')
    if model_base:
        model_path += os.pathsep + model_base

    env_actions += [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
    ]
    # Fallback if setup.sh was missing: guarantee gazebo's core media is present.
    if 'GAZEBO_RESOURCE_PATH' not in gz:
        env_actions.append(SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            '/usr/share/gazebo-11' + os.pathsep
            + os.environ.get('GAZEBO_RESOURCE_PATH', '')))

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # Spawn directly (stock spawn_turtlebot3.launch.py exposes no yaw arg).
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', 'turtlebot3_' + TB3_MODEL, 'model.sdf')
    spawn_turtlebot = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=[
            '-entity', TB3_MODEL, '-file', urdf_path,
            '-x', x_pose, '-y', y_pose, '-z', '0.01', '-Y', yaw,
        ])

    ld = LaunchDescription()
    for action in env_actions:
        ld.add_action(action)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    return ld
