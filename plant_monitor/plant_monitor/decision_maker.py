import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

class DecisionMaker(Node):
    def __init__(self, name: str = 'decision_maker'):
        super().__init__(name)
        self.logger = self.get_logger()
        self.log_level = 20
        self.logger.set_level(self.log_level)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('expected_temp', 22),
                ('expected_light', 500),
                ('expected_ph', 5.5),
                ('expected_humidity', 80),

                ('threshold_temp', 2),
                ('threshold_light', 20),
                ('threshold_ph', 5.5),
                ('threshold_humidity', 3),
            ])

        # Retrieve parameters or set default values for expected values and thresholds
        expected_temp = self.get_parameter('expected_temp')
        expected_light = self.get_parameter('expected_light')
        expected_ph = self.get_parameter('expected_ph')
        expected_humidity = self.get_parameter('expected_humidity')

        threshold_temp = self.get_parameter('threshold_temp')
        threshold_light = self.get_parameter('threshold_light')
        threshold_ph = self.get_parameter('threshold_ph')
        threshold_humidity = self.get_parameter('threshold_humidity')

        # Store expected values and thresholds in lists for easy access
        self.normal_values = [expected_temp.value, expected_light.value, expected_ph.value, expected_humidity.value]
        self.thresholds = [threshold_temp.value, threshold_light.value, threshold_ph.value, threshold_humidity.value]

        # Create publishers for actuators
        self.pub_heat = self.create_publisher(Float32, 'heat', 10)
        self.pub_light = self.create_publisher(Float32, 'light', 10)
        self.pub_ph = self.create_publisher(Float32, 'ph', 10)
        self.pub_water = self.create_publisher(Float32, 'water', 10)

        # Store publisher instances in a list for easy access
        self.pub_list = [self.pub_heat, self.pub_light, self.pub_ph, self.pub_water]
        self.decision_names = ['heat', 'light', 'ph', 'water']

        # Create a subscription to sensor data topic
        self.create_subscription(Float32MultiArray, 'sensor_data', self.callback, 10)

    def decide(self, data: Float32MultiArray):
        '''
        Responsible for determining actions based on the received sensor data.
        It returns a list of decisions indicating actions to be taken for each parameter.
        Depends on given parameters of expected values and thresholds'''
        num_array = [0] * 4
        for i in range(4):
            if data.data[i] > self.normal_values[i] + self.thresholds[i]:
                num_array[i] = abs(self.normal_values[i] - data.data[i])
            elif data.data[i] < self.normal_values[i] - self.thresholds[i]:
                num_array[i] = -1 * abs(self.normal_values[i] - data.data[i])
        return num_array

    def callback(self, data: Float32MultiArray):
        decisions = self.decide(data)
        # limiting the precision of float values before publishing them can help to reduce unnecessary data and maintain consistency.
        for i in range(4):
            if decisions[i] > 0:
                # If the decision is to increase the parameter, publish the decision value to the corresponding actuator topic
                if self.decision_names[i] == 'heat':
                    self.pub_heat.publish(Float32(data=round(decisions[i], 2)))
                    self.logger.info('temperature too high, cooling down')
                if self.decision_names[i] == 'light':
                    self.pub_light.publish(Float32(data=round(decisions[i], 2)))
                    self.logger.info('light too high, decreasing light')
                if self.decision_names[i] == 'ph':
                    self.pub_ph.publish(Float32(data=round(decisions[i], 2)))
                    self.logger.info('ph too high, acidifying')
                if self.decision_names[i] == 'water':
                    self.pub_water.publish(Float32(data=round(decisions[i], 2)))
                    self.logger.info('humidity too low, watering')
            elif decisions[i] == -1:
                # If the decision is to decrease the parameter, publish the decision value to the corresponding actuator topic
                if self.decision_names[i] == 'heat':
                    self.pub_heat.publish(Float32(data=round(decisions[i], 2)))
                    self.logger.info('temperature too low, heating up')
                if self.decision_names[i] == 'light':
                    self.pub_light.publish(Float32(data=round(decisions[i], 2)))
                    self.logger.info('light too low, increasing light')
                if self.decision_names[i] == 'ph':
                    self.pub_ph.publish(Float32(data=round(decisions[i], 2)))
                    self.logger.info('ph too low, alkalizing')
                if self.decision_names[i] == 'water':
                    self.pub_water.publish(Float32(data=round(decisions[i], 2)))
                    self.logger.info('humidity too high, stopping watering')
            else:
                self.pub_list[i].publish(Float32(data=0.0))


def main(args=None):
    try:
        rclpy.init(args=args)
        node = DecisionMaker('decision_maker')
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()