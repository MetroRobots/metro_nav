from sensor_msgs.msg import Joy
import rclpy


class JoyListener:
    def __init__(self, node, callback):
        self.node = node
        self.callback = callback

        self.node.declare_parameter('axis_names', rclpy.Parameter.Type.STRING_ARRAY)
        if self.node.has_parameter('axis_names'):
            self.axis_names = self.node.get_parameter('axis_names').value
        else:
            self.axis_names = None

        self.node.declare_parameter('button_names', rclpy.Parameter.Type.STRING_ARRAY)

        if self.node.has_parameter('button_names'):
            self.button_names = self.node.get_parameter('button_names').value
        else:
            self.button_names = None

        self.node.declare_parameter('joy_topic', 'joy')
        topic = self.node.get_parameter('joy_topic').value
        self.joy_subscriber = self.node.create_subscription(Joy, topic, self.joy_callback, 1)

    def joy_callback(self, msg):
        axes = {}
        buttons = set()

        if self.axis_names:
            for name, value in zip(self.axis_names, msg.axes):
                axes[name] = value
        else:
            for i, value in enumerate(msg.axes):
                axes[i] = value

        if self.button_names:
            for name, value in zip(self.button_names, msg.buttons):
                if value:
                    buttons.add(name)
        else:
            for i, value in enumerate(msg.axes):
                if value:
                    buttons.add(i)

        self.callback(axes, buttons)
