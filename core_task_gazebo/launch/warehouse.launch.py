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
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TB3_MODEL = 'waffle_pi'


def _nvidia_available():
    """Report whether an NVIDIA GPU is usable, so gzclient can render the
    mesh-heavy warehouse on the dedicated GPU instead of freezing the
    integrated one. Set CORE_TASK_USE_NVIDIA=0 to force off (e.g. broken
    offload)."""
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


# The camera link pose in the stock waffle_pi SDF. It is level (no pitch) and
# only 9 cm off the floor, so a full-height person is out of frame closer than
# ~3.8 m - and follow drives the robot right up to the target. Tilting the
# camera up keeps the target's body in frame during the approach. The pose value
# is shared with an inertial element, so we match the <sensor> line that follows
# it to hit only the camera.
_CAM_POSE = '<pose>0.076 0.0 0.093 0 0 0</pose>'
_CAM_ANCHOR = '\n      <sensor name="camera" type="camera">'


def _namespaced(sdf, namespace):
    """Namespace every plugin and prefix the TF frames, for a second robot.

    Without this, two stock TurtleBot3s collide on topics (cmd_vel, scan, odom)
    and on TF (both broadcast odom->base_footprint). Injecting a <namespace>
    into each plugin's <ros> block moves the topics under /<namespace>, and
    prefixing odom / base_footprint / base_scan keeps the two robots' TF trees
    apart. Robot 1 is left un-namespaced so the rest of the stack (Nav2, the
    controller, perception) keeps working unchanged.
    """
    ns = namespace if namespace.startswith('/') else '/' + namespace
    prefix = namespace.strip('/')
    # Namespace every plugin, and redirect TF to /<ns>/tf. The diff_drive
    # plugin hardcodes the global /tf and ignores <namespace>, so without the
    # remap robot 2's odom TF lands on /tf while its namespaced Nav2 reads
    # /robot2/tf - and the costmaps never find robot2/odom. The remap is a
    # no-op on the plugins that publish no TF.
    inject = ('<ros>\n            <namespace>%s</namespace>'
              '\n            <remapping>/tf:=/%s/tf</remapping>'
              % (ns, prefix))
    out = sdf.replace('<ros>', inject)
    # Prefix ONLY the plugin TF-config tags, not link names or joint <parent>/
    # <child> refs - those must keep pointing at the model's real link names.
    for tag, val in (('robot_base_frame', 'base_footprint'),
                     ('odometry_frame', 'odom'),
                     ('frame_name', 'base_scan')):
        out = out.replace('<%s>%s</%s>' % (tag, val, tag),
                          '<%s>%s/%s</%s>' % (tag, prefix, val, tag))
    return out


def _patched_robot_sdf(sdf_path, range_m, camera_pitch, namespace=''):
    """Return a waffle_pi SDF with the laser range and camera pitch applied.

    Stock values (range 3.5, pitch 0, no namespace) touch nothing and return
    the original path; any change is written to a temp copy so the system model
    is never modified. Mesh model:// URIs resolve via GAZEBO_MODEL_PATH
    regardless of file location. camera_pitch is radians; negative tilts the
    camera up. namespace, if set, isolates a second robot (see _namespaced).
    """
    with open(sdf_path) as f:
        sdf = f.read()
    patched = sdf

    if range_m not in ('3.5', '3.50', '3.500000'):
        after = patched.replace('<max>3.5</max>', '<max>%s</max>' % range_m)
        if after == patched:
            raise RuntimeError(
                'laser <max>3.5</max> not found in %s (upstream SDF changed)'
                % sdf_path)
        patched = after

    if abs(float(camera_pitch)) > 1e-6:
        tilted = '<pose>0.076 0.0 0.093 0 %s 0</pose>' % camera_pitch
        after = patched.replace(_CAM_POSE + _CAM_ANCHOR, tilted + _CAM_ANCHOR)
        if after == patched:
            raise RuntimeError(
                'camera pose anchor not found in %s (upstream SDF changed)'
                % sdf_path)
        patched = after

    # Keep LiDAR visual enabled in Gazebo so it's visible on the robot.
    # Comment out the replacement below to hide it again if it causes performance issues.
    lidar_visual = ('      <visual name="lidar_sensor_visual">\n'
                    '        <pose>-0.064 0 0.121 0 0 0</pose>\n'
                    '        <geometry>\n'
                    '          <mesh>\n'
                    '            <uri>model://turtlebot3_common/meshes/sensors/lds.stl</uri>\n'
                    '            <scale>0.001 0.001 0.001</scale>\n'
                    '          </mesh>\n'
                    '        </geometry>\n'
                    '        <material>\n'
                    '          <ambient>0.2 0.2 0.2 1.0</ambient>\n'
                    '          <diffuse>0.2 0.2 0.2 1.0</diffuse>\n'
                    '        </material>\n'
                    '      </visual>\n')
    # patched = patched.replace(lidar_visual, '')  # Disabled: keep LiDAR visible

    if namespace:
        patched = _namespaced(patched, namespace)

    if patched == sdf:
        return sdf_path
    out = os.path.join(
        tempfile.gettempdir(),
        'turtlebot3_%s_lidar%s_cam%s_ns%s.sdf'
        % (TB3_MODEL, range_m, camera_pitch, namespace.strip('/') or 'none'))
    with open(out, 'w') as f:
        f.write(patched)
    return out


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
    # The stock SDF hard-codes the 3.5 m LDS-01, which can't see across the
    # warehouse aisles for SLAM. Default to the 12 m LDS-02 the current TB3
    # actually ships; override with CORE_TASK_LIDAR_RANGE (e.g. 3.5 for LDS-01).
    # Keep core_task_mapping's slam max_laser_range in sync with this.
    lidar_range = os.environ.get('CORE_TASK_LIDAR_RANGE', '12.0')
    # Tilt the camera up ~20 deg (-0.35 rad) so the low 9 cm camera frames a
    # standing person during the follow approach. Override with
    # CORE_TASK_CAMERA_PITCH (0 restores the stock level camera).
    camera_pitch = os.environ.get('CORE_TASK_CAMERA_PITCH', '-0.35')
    urdf_path = _patched_robot_sdf(
        os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                     'models', 'turtlebot3_' + TB3_MODEL, 'model.sdf'),
        lidar_range, camera_pitch)
    spawn_turtlebot = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=[
            '-entity', TB3_MODEL, '-file', urdf_path,
            '-x', x_pose, '-y', y_pose, '-z', '0.01', '-Y', yaw,
        ])

    # Optional second robot for the multi-agent challenge. Namespaced under
    # /robot2 with a prefixed TF tree so it does not collide with robot 1;
    # drive it via /robot2/cmd_vel. Off by default so single-robot demos are
    # unchanged - enable with robot2:=true.
    robot2_arg = DeclareLaunchArgument(
        'robot2', default_value='false',
        description='Spawn a second robot under /robot2.')
    sdf2 = _patched_robot_sdf(
        os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                     'models', 'turtlebot3_' + TB3_MODEL, 'model.sdf'),
        lidar_range, camera_pitch, namespace='robot2')
    spawn_robot2 = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        condition=IfCondition(LaunchConfiguration('robot2')),
        arguments=[
            '-entity', 'robot2', '-file', sdf2,
            '-x', '0.0', '-y', '0.0', '-z', '0.01', '-Y', '0.0',
        ])

    # Robot 2's own robot_state_publisher: publishes robot2/base_footprint ->
    # robot2/base_link -> robot2/base_scan/... so Nav2 can place the lidar. Runs
    # under the robot2 namespace (reads /robot2/joint_states, no name clash with
    # robot 1's RSP) with a robot2/ frame prefix. The stock tb3 RSP launch can't
    # do this - it is global and unprefixed - so the node is built here.
    with open(os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'urdf', 'turtlebot3_' + TB3_MODEL + '.urdf')) as f:
        robot2_desc = f.read()
    robot2_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='robot2', name='robot_state_publisher', output='screen',
        condition=IfCondition(LaunchConfiguration('robot2')),
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot2_desc,
            'frame_prefix': 'robot2/',
        }],
        # Publish robot 2's link TF to /robot2/tf so it lines up with the
        # namespaced Nav2 (which reads /robot2/tf), matching the diff_drive
        # remap above. Otherwise these frames land on the global /tf.
        remappings=[('/tf', '/robot2/tf'), ('/tf_static', '/robot2/tf_static')])

    ld = LaunchDescription()
    ld.add_action(robot2_arg)
    for action in env_actions:
        ld.add_action(action)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    ld.add_action(spawn_robot2)
    ld.add_action(robot2_rsp)
    return ld
