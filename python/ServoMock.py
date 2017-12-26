"""
================================================
ABElectronics ServoPi 16-Channel PWM Servo Driver

Requires python smbus to be installed
================================================
"""

import re
import time
import math
import platform

class Servo(object):
    """
    Servo class for controlling RC servos with the Servo PWM Pi Zero
    """
    __pwm = None
    __lowpos = 0
    __highpos = 4095
    __frequency = 50
    __channel = 1

    def __init__(self, address=0x40, low_limit=1.0, high_limit=2.0):
        """
        init object with i2c address, default is 0x40 for ServoPi board
        """

        self.set_frequency(50)
        self.set_low_limit(low_limit)
        self.set_high_limit(high_limit)

        print(self.__lowpos)
        print(self.__highpos)

    def move(self, channel, position, steps=250):
        """
        set the position of the servo
        """
        if channel >= 1 and channel <= 16:
            self.__channel = channel
        else:
            raise ValueError('move: channel out of range')

        if steps < 0 or steps > 4095:
            raise ValueError('move: steps out of range')

        if position >= 0 and position <= steps:
            pwm_value = (((float(self.__highpos)-float(self.__lowpos)) /
                          steps) * float(position)) + self.__lowpos

            print(pwm_value)
        else:
            raise ValueError('move: channel out of range')

    def set_low_limit(self, low_limit):
        """
        Set the low limit in milliseconds
        """
        self.__lowpos = int(4096.0 * (low_limit / 1000.0) * self.__frequency)

        if (self.__lowpos < 0) or (self.__lowpos > 4095):
            raise ValueError('set_low_limit: value out of range')

    def set_high_limit(self, high_limit):
        """
        Set the high limit in milliseconds
        """
        self.__highpos = int(4096.0 * (high_limit / 1000.0) * self.__frequency)

        if (self.__highpos < 0) or (self.__highpos > 4095):
            raise ValueError('set_high_limit: value out of range')

    def set_frequency(self, freq):
        """
        Set the PWM frequency
        """
        return

    def output_disable(self):
        """
        disable output via OE pin
        """
        return

    def output_enable(self):
        """
        enable output via OE pin
        """
        return


