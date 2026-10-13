import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def _default_config():
    config = os.path.join(
        get_package_share_directory('rbf_ntrip_driver'),
        'config',
        'rbf_ntrip_driver.param.yaml'
    )

    # rbf_ntrip_driver.param.yaml is gitignored (contains credentials); when it
    # is absent from the install space, fall back to the deploy config mount.
    if not os.path.exists(config):
        config = os.path.join(
            os.environ.get('ROBIONE_CONFIG_DIR', '/root/config'),
            'rbf_ntrip_driver.param.yaml'
        )
    return config


def _build(context, *args, **kwargs):
    # Empty means "not given": the caller (e.g. the gnss wrapper launch) forwards the argument
    # unconditionally, so an empty value has to fall back here rather than in the declaration.
    config = LaunchConfiguration('rbf_ntrip_driver_param_path').perform(context) or _default_config()

    container = ComposableNodeContainer(
    name='rbf_ntrip_driver',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='rbf_ntrip_driver',
            plugin='rbf_ntrip_driver::NtripDriver',
            name='rbf_ntrip_driver_node',
            # allow_substs: a config may read the caster credentials from the
            # environment through $(env ...) instead of storing them.
            parameters=[ParameterFile(config, allow_substs=True)],
            extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )
    return [container]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'rbf_ntrip_driver_param_path',
            default_value='',
            description='parameter file; empty uses the package config or $ROBIONE_CONFIG_DIR',
        ),
        OpaqueFunction(function=_build),
    ])
