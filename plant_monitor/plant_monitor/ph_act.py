import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class PhActuator(Node):
    def __init__(self, name: str = 'ph_act'):
        super().__init__(name)
        self.logger = self.get_logger()
        self.log_level = 20
        self.logger.set_level(self.log_level)
        self.create_subscription(Float32, 'ph', self.callback, 10)

    def callback(self, data: Float32):
        val = data.data
        log_message = None
        if val < 0:
            log_message = f'acidifying by {abs(round(val, 1))}'
        elif val > 0:
            log_message = f'basifying by {abs(round(val, 1))}'
        if log_message:
            self.logger.info(log_message)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = PhActuator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()