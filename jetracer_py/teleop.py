import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

settings = termios.tcgetattr(sys.stdin)


# noinspection PyBroadException
class Teleop(Node):
    def __init__(self):
        super().__init__('Teleop')
        self.publisher_throttle = self.create_publisher(Float32, "throttle", 1000)
        self.publisher_steering = self.create_publisher(Float32, "steering", 1000)
        self.rate = self.create_rate(10)  # 10 Hz

        try:
            while 42:
                key = self.get_key()
                if key == 'w':
                    self.publisher_throttle.publish(Float32(1.0))
                elif key == 'a':
                    self.publisher_steering.publish(Float32(1.0))
                elif key == 's':
                    self.publisher_throttle.publish(Float32(-1.0))
                elif key == 'd':
                    self.publisher_steering.publish(Float32(-1.0))
                elif key == 'i':
                    self.publisher_steering.publish(Float32(0.0))
                elif key == 'o':
                    self.publisher_throttle.publish(Float32(0.0))
                self.rate.sleep()
        except Exception as e:
            self.get_logger().info(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    @staticmethod
    def get_key():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    teleop.get_logger().info('Controller ready.')
    print("Controller ready.")
    rclpy.spin(teleop)

    teleop.destroy_node()
    rclpy.shutdown()
