import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray # working with array dtype
import numpy as np



class SensorManager(Node):
    def __init__(self, name: str = 'sensor_manager'):
        super().__init__(name)  
        self.pub = self.create_publisher(Float32MultiArray, 'sensor_data', 10)
        self.declare_parameter("rate", 1.0)
        self.rate = self.get_parameter("rate").value

        self.timer_publisher = self.create_timer(self.rate, self.callback)
        self.logger = self.get_logger()
        self.logger.set_level(20) # show info on logger
        self.mean = [22, 500, 6.0, 50]  # Temperature (ºC), Light (µmol/s/m²), pH, Humidity (%)
        self.deviation = [4, 200, 0.5, 20]  # Standard deviation for each parameter
        self.i = 0

    # generator for the sensor data, mesuring temperature, light, ph, humidity, each value will be a sine wave with noise
    # in real life cases this will be input from sensors, but since this is a simualtion we'll create our own array of states
    def sensor_data(self):
        while True:
            yield [self.mean[i] + self.deviation[i] * np.sin(self.i / 10) + (np.random.normal(0, 1) / 10) * self.deviation[i] for i in range(4)]

    def callback(self):
        data = next(self.sensor_data())
        msg = Float32MultiArray(data=data)
        self.last_state = msg
        self.pub.publish(msg)
        self.i += 1
        self.logger.info(f"[temp (ºC), light (µmol/s/m²), ph, humidity (%)] - {data}")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = SensorManager('sensor_manager')
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

