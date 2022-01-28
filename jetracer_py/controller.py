import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from waveshare.nvidia_racer import NvidiaRacecar


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        self.subscription_throttle = self.create_subscription(Float32, "throttle", self.throttle_callback, 1000)
        self.subscription_steering = self.create_subscription(Float32, "steering", self.steering_callback, 1000)
        self.car = NvidiaRacecar()
        self.car.steering_gain = 0
        self.car.steering_offset = 0
        self.car.throttle_gain = 1
        self.car.throttle = 0.0
        self.car.steering = 0.0

    def steering_callback(self, msg):
        self.car.steering = msg.data
        self.get_logger().info('Set steering to %f' % msg.data)

    def throttle_callback(self, msg):
        self.car.throttle = msg.data
        self.get_logger().info('Set throttle to %f' % msg.data)

    def destroy_node(self):
        self.get_logger().info('Shutting down controller node.')
        self.car.steering = 0.0
        self.car.throttle = 0.0
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    controller.get_logger().info('Controller ready.')
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()
