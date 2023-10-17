import rclpy
from rclpy.node import Node
from .keyboard_listener import KeyboardListener
from .command_publisher import CommandPublisher
import collections

PUNCTUATION_REMAPPING = {
    'comma': ',',
    'period': '.',
    'lessthan': '<',
    'greaterthan': '>',
}


class KeyboardTeleop(Node):
    def __init__(self):
        Node.__init__(self, 'metro_teleop', allow_undeclared_parameters=True,
                      automatically_declare_parameters_from_overrides=True)

        logger = self.get_logger()

        self.bindings = collections.defaultdict(dict)
        for param in self._parameters:
            if param.startswith('bindings'):
                _, key, component = param.split('.')
                key = PUNCTUATION_REMAPPING.get(key, key)
                value = self.get_parameter(param).value
                self.bindings[key][component] = value

        for key in self.bindings:
            logger.info(f'Key {key}:')
            for component, value in self.bindings[key].items():
                logger.info(f'  {component}: {value}')

        self.cmd_pub = CommandPublisher(self)
        self.jl = KeyboardListener(self.key_cb)

    def key_cb(self, key):
        components = self.bindings[key]
        self.cmd_pub.publish_scaled_command(**components)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = KeyboardTeleop()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
