"""Snowbot execution mode selection; never starts low-level drivers or motors."""
from pathlib import Path
import uuid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from bindu_contracts.runtime_mode import resolve_mode
from bindu_runtime.teleop_deployment import TARGET_STREAM_LIMIT_DEFAULTS, target_stream_limits, execution_backend


def assemble(context):
    value = lambda name, default='': LaunchConfiguration(name, default=default).perform(context)
    if value('position_protocol'):
        raise ValueError('SNOWBOT_POSITION_PROTOCOL_REMOVED: session_v3 is fixed; remove position_protocol')
    if value('target_stream_brake_lead_s'):
        raise ValueError('SNOWBOT_BRAKE_LEAD_MOVED: set execution.target_stream_brake_lead_s in parameters_config')
    legacy = value('simulation')
    if legacy not in ('', 'true', 'false'):
        raise ValueError('RUNTIME_SIMULATION_ALIAS_INVALID')
    mode = resolve_mode(value('runtime_mode'), None if not legacy else legacy == 'true', default='real')
    if value('gripper_config') and mode != 'real':
        raise ValueError('GRIPPER_REQUIRES_REAL_MODE')
    backend = execution_backend(mode,value('device_backend'),value('execution_mode','position'))
    enabled = value('command_enabled', 'false')
    if enabled not in ('true', 'false'):
        raise ValueError('SNOWBOT_COMMAND_ENABLED_INVALID')
    if mode == 'simulation' and enabled == 'true':
        raise ValueError('COMMAND_ENABLED_IS_REAL_ONLY')
    recording = value('recording_mode', 'off')
    if recording not in ('off', 'compact', 'normal'):
        raise ValueError('INVALID_RECORDING_MODE')
    share = Path(get_package_share_directory('snowbot_bringup'))
    profile = value('profile') or str(share/'config'/(
        'examples/snowbot/snowbot_vendor_position.json' if mode == 'real' else 'examples/snowbot/snowbot_shovel_sim.json'))
    common = {'profile': profile, 'runtime_mode': mode, 'run_id': value('run_id', 'manual')}
    parameters = {**common}
    if mode == 'real':
        parameters.update(command_enabled=enabled == 'true', vendor_namespace=value('vendor_namespace', '/'),
            execution_mode=value('execution_mode', 'position'),
            **{key: value(key) for key in ('arm_receiver_node', 'servo_receiver_node', 'mid_receiver_node', 'gripper_config', 'parameters_config')})
        limits={key:value(key,str(default)) for key,default in TARGET_STREAM_LIMIT_DEFAULTS.items()}
        parameters.update(target_stream_limits(limits))
    else:
        parameters['device_backend'] = backend
    from snowbot_hardware.pdo_reader_trace import adapter_trace_env
    return [Node(package='snowbot_hardware' if mode == 'real' else 'bindu_runtime', executable='snowbot_adapter' if mode == 'real' else 'execution',
                 namespace=value('namespace', 'bindu_snowbot'), additional_env=adapter_trace_env() if mode=='real' else {}, output='screen', parameters=[parameters]),
            Node(package='bindu_runtime', executable='recorder', namespace=value('namespace', 'bindu_snowbot'),
                 output='screen', parameters=[{**common, 'device_backend': backend,
                     'recording_mode': recording, 'output': value('output', '/tmp/bindu-runs')}])]


def generate_launch_description():
    defaults = {'runtime_mode': '', 'simulation': '', 'device_backend': '', 'profile': '',
                'namespace': 'bindu_snowbot', 'run_id': uuid.uuid4().hex, 'vendor_namespace': '/',
                'arm_receiver_node': '', 'servo_receiver_node': '', 'mid_receiver_node': '',
                'command_enabled': 'false', 'recording_mode': 'off', 'output': '/tmp/bindu-runs', 'parameters_config': '', 'gripper_config': '', 'execution_mode': 'position'}
    defaults.update({key:str(value) for key,value in TARGET_STREAM_LIMIT_DEFAULTS.items()})
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=assemble)])
