import psoc
import numpy


def get_default_wheel(wheel_num):
    return Wheel(wheel_num)


def get_default_wheel_list():
    result = []
    for i in range(3):
        result.append(get_default_wheel(i))
    return result

def set_motor_speed(speed_list):
    psoc.set_motor_speeds(speed_list)

def power_off():
    psoc.disengage()


class Wheel(object):
    def __init__(self, wheel_num):
        self.wheel_num = wheel_num

    def get_speed(self):
        """
        return : rotations /sec
        """
        psoc.get_speed(self.wheel_num)

    def set_motor_speed(self, speed):
        """
        """
        print "inside set_motor_speed"
        print "wheel number", self.wheel_num, speed
        psoc.set_motor_speed(self.wheel_num, speed)
        psoc.spin()

    def set_debug(self, debug):
        psoc.set_debug(debug)
