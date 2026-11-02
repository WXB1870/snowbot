"""Snowbot gripper policy attached to the generic bimanual tool node."""
from bindu_runtime.bimanual_tool_node import BimanualToolNode
from bindu_runtime.common import spin
from snowbot_hardware.gripper import validate_config, JOINTS


def load_grippers(config, profile, mode):
    from snowbot_hardware.parameters import select_mode_config
    return validate_config(select_mode_config(config,mode), profile), dict(JOINTS)


def load_tool(node, config):
    from rcl_interfaces.msg import ParameterDescriptor
    from snowbot_hardware.parameters import load_parameters, apply_tool_timing
    node.runtime_parameters = load_parameters(node.declare_parameter(
        'parameters_config', '', descriptor=ParameterDescriptor(read_only=True)).value or None)
    return apply_tool_timing(config, node.runtime_parameters)


class SnowbotToolNode(BimanualToolNode):
    def __init__(self):
        super().__init__(gripper_config_loader=lambda config,profile: load_grippers(config,profile,self.runtime_mode),
                         tool_config_loader=load_tool)


def main(args=None):
    spin(SnowbotToolNode, args)


if __name__ == '__main__':
    main()
