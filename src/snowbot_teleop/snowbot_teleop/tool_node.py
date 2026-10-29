"""Snowbot gripper policy attached to the generic bimanual tool node."""
from bindu_runtime.bimanual_tool_node import BimanualToolNode
from bindu_runtime.common import spin
from snowbot_hardware.gripper import validate_config, JOINTS


def load_grippers(config, profile):
    return validate_config(config, profile), dict(JOINTS)


class SnowbotToolNode(BimanualToolNode):
    def __init__(self):
        super().__init__(gripper_config_loader=load_grippers)


def main(args=None):
    spin(SnowbotToolNode, args)


if __name__ == '__main__':
    main()
