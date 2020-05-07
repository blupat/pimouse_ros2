#!/usr/bin/env python3
#
# =======================================================================
#   @file   motors.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class Motors(Node):

    __slots__ = (
        '_subscriber', '_service_on', '_service_off', '_is_on', '_using_cmd_vel',
        '_last_time', '_timer')

    def __init__(self):
        super().__init__('motors')
        self._subscriber = self.create_subscription(Twist, 'cmd_vel', self.callback_cmd_vel, 1)
        self._service_on = self.create_service(Trigger, 'motor_on', self.callback_on)
        self._service_off = self.create_service(Trigger, 'motor_off', self.callback_off)
        self.set_power(False)
        self._using_cmd_vel = False
        self._last_time = time.time()
        self._timer = self.create_timer(0.1, self.timer_callback)

    def set_power(self, onoff=False):
        en = "/dev/rtmotoren0"
        try:
            with open(en, 'w') as f:
                f.write("1\n" if onoff else "0\n")
            self._is_on = onoff
            return True
        except:
            self.get_logger().error("cannot write to " + en)
        
        return False

    def _set_raw_freq(self, left_hz, right_hz):
        if not self._is_on:
            self.get_logger().error("not enpowered")
            return
        try:
            with open("/dev/rtmotor_raw_l0", "w") as lf, open("/dev/rtmotor_raw_r0", "w") as rf:
                lf.write(str(int(round(left_hz))) + "\n")
                rf.write(str(int(round(right_hz))) + "\n")
        except:
            self.get_logger().error("cannot write to rtmotor_raw_*")

    def callback_cmd_vel(self, message):
        forward_hz = 80000.0 * message.linear.x / (9 * math.pi)
        rot_hz = 400.0 * message.angular.z / math.pi
        self._set_raw_freq((forward_hz - rot_hz), (forward_hz + rot_hz))
        self._using_cmd_vel = True
        self._last_time = time.time()

    def callback_on(self, request, response):
        response.success = self.set_power(True)
        response.message = "ON" if self._is_on else "OFF"
        return response

    def callback_off(self, request, response):
        response.success = self.set_power(False)
        response.message = "ON" if self._is_on else "OFF"
        return response

    def timer_callback(self):
        if self._using_cmd_vel and (time.time() - self._last_time) >= 1.0:
            self._set_raw_freq(0, 0)
            self._using_cmd_vel = False


def main(args=None):
    rclpy.init(args=args)

    motors = Motors()

    rclpy.spin(motors)

    motors.set_power(False)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
