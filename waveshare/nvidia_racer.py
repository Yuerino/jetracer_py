# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of
# the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from .racer import Racecar
import traitlets
from adafruit_servokit import ServoKit


class NvidiaRacecar(Racecar):
    i2c_address = traitlets.Integer(default_value=0x40)
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0)
    steering_channel = traitlets.Integer(default_value=0)
    throttle_gain = traitlets.Float(default_value=0.8)
    throttle_channel = traitlets.Integer(default_value=1)

    def __init__(self, *args, **kwargs):
        super(NvidiaRacecar, self).__init__(*args, **kwargs)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]

    @traitlets.observe('steering')
    def _on_steering(self, change):
        self.steering_motor.throttle = change['new'] * self.steering_gain + self.steering_offset

    @traitlets.observe('throttle')
    def _on_throttle(self, change):
        self.throttle_motor.throttle = change['new'] * self.throttle_gain
# noinspection PyProtectedMember
# class NvidiaRacecar(Racecar):
#     i2c_address1 = traitlets.Integer(default_value=0x40)
#     i2c_address2 = traitlets.Integer(default_value=0x60)
#     steering_gain = traitlets.Float(default_value=-0.65)
#     steering_offset = traitlets.Float(default_value=0)
#     steering_channel = traitlets.Integer(default_value=0)
#     throttle_gain = traitlets.Float(default_value=0.8)
#
#     def __init__(self, *args, **kwargs):
#         super(NvidiaRacecar, self).__init__(*args, **kwargs)
#         self.kit = ServoKit(channels=16, address=self.i2c_address1)
#         self.motor = ServoKit(channels=16, address=self.i2c_address2)
#         self.motor._pca.frequency = 1600
#         self.steering_motor = self.kit.continuous_servo[self.steering_channel]
#
#     @traitlets.observe('steering')
#     def _on_steering(self, change):
#         self.steering_motor.throttle = change['new'] * self.steering_gain + self.steering_offset
#
#     @traitlets.observe('throttle')
#     def _on_throttle(self, change):
#         if change['new'] > 0:
#             self.motor._pca.channels[0].duty_cycle = int(0xFFFF * (change['new'] * self.throttle_gain))
#             self.motor._pca.channels[1].duty_cycle = 0xFFFF
#             self.motor._pca.channels[2].duty_cycle = 0
#             self.motor._pca.channels[3].duty_cycle = 0
#             self.motor._pca.channels[4].duty_cycle = int(0xFFFF * (change['new'] * self.throttle_gain))
#             self.motor._pca.channels[7].duty_cycle = int(0xFFFF * (change['new'] * self.throttle_gain))
#             self.motor._pca.channels[6].duty_cycle = 0xFFFF
#             self.motor._pca.channels[5].duty_cycle = 0
#         else:
#             self.motor._pca.channels[0].duty_cycle = int(-0xFFFF * (change['new'] * self.throttle_gain))
#             self.motor._pca.channels[1].duty_cycle = 0
#             self.motor._pca.channels[2].duty_cycle = 0xFFFF
#             self.motor._pca.channels[3].duty_cycle = int(-0xFFFF * (change['new'] * self.throttle_gain))
#             self.motor._pca.channels[4].duty_cycle = 0
#             self.motor._pca.channels[7].duty_cycle = int(-0xFFFF * (change['new'] * self.throttle_gain))
#             self.motor._pca.channels[6].duty_cycle = 0
#             self.motor._pca.channels[5].duty_cycle = 0xFFFF
