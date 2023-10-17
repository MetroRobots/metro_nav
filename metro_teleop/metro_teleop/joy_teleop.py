import rclpy
from rclpy.node import Node
from .joystick_listener import JoyListener
from .command_publisher import CommandPublisher


class JoyTeleop(Node):
    def __init__(self):
        Node.__init__(self, 'metro_teleop')

        logger = self.get_logger()
        self.declare_parameter('enable', '')
        self.enable_button = self.get_parameter('enable').value
        if self.enable_button:
            logger.info(f'Enable Button: {self.enable_button}')

        self.axes = {}

        for component in ['linear', 'angular']:
            for dimension in 'xyz':
                twist_name = f'{component}_{dimension}'
                axis_p = f'{twist_name}_axis'
                self.declare_parameter(axis_p, '')
                axis_name = self.get_parameter(axis_p).value
                if axis_name:
                    logger.info(f'{component.title()} {dimension.title()} Axis: {axis_name}')
                    self.axes[axis_name] = twist_name

        self.zero_command = False
        self.cmd_pub = CommandPublisher(self)
        self.jl = JoyListener(self, self.joy_cb)

    def joy_cb(self, axes, buttons):
        if self.enable_button and self.enable_button not in buttons:
            if not self.zero_command:
                self.cmd_pub.publish_command()
                self.zero_command = True
            return

        args = {}
        for axis_name, twist_name in self.axes.items():
            args[twist_name] = axes[axis_name]
        self.cmd_pub.publish_scaled_command(**args)
        self.zero_command = False


def main(args=None):
    try:
        rclpy.init(args=args)
        node = JoyTeleop()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
