#!/usr/bin/env python3
#
# =======================================================================
#   @file   lightsensors.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import rclpy
from rclpy.node import Node
from pimouse_msgs.msg import LightSensorValues


class LightSensors(Node):

    __slots__ = ('_publisher', '_lightsensors_period', '_timer')

    def __init__(self):
        super().__init__('lightsensors')
        self._publisher = self.create_publisher(LightSensorValues, 'lightsensors')
        self._lightsensors_period = self.get_period()
        self._timer = self.create_timer(self._lightsensors_period, self.timer_callback)

    def timer_callback(self):
        is_enable = self.get_parameter_or('lightsensors_enable', alternative_value=True)
        if is_enable:
            devfile = '/dev/rtlightsensor0'
            try:
                with open(devfile, 'r') as f:
                    data = f.readline().split()
                    data = [ int(e) for e in data ]
                    d = LightSensorValues()
                    d.right_forward = data[0]
                    d.right_side = data[1]
                    d.left_side = data[2]
                    d.left_forward = data[3]
                    d.sum_all = sum(data)
                    d.sum_forward = data[0] + data[3]
                    self._publisher.publish(d)
            except IOError:
                self.get_logger().info("cannot write to " + devfile)

        period = get_period()
        if period != self._lightsensors_period:
            self._lightsensors_period = period
            self._timer.destroy()
            self._timer = self.create_timer(self._lightsensors_period, self.timer_callback)

    def get_period(self):
        period = self.get_parameter_or('lightsensors_period', alternative_value=0.1)
        try:
            if period <= 0.0:
                raise Exception()
        except Exception:
            self.get_logger().info("value error: lightsensors_period")
            period = 0.1
        return period

def main(args=None):
    rclpy.init(args=args)

    lightsensors = LightSensors()

    rclpy.spin(lightsensors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lightsensors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
