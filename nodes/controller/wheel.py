import psoc
import numpy


def get_default_wheel(wheel_num):
    return Wheel(wheel_num)

def get_default_wheel_list():
    result = []
    for i in range(3):
        result.append(get_default_wheel(i))
    return result

def spin():
    psoc.spin()

class Wheel(object):
    def __init__(self, wheel_num):
        self.wheel_num = wheel_num
        # self.linear_velocity = linear_velocity
        # self.wheel_spin = wheel_spin = 0
        psoc.setPID(self.wheel_num, 1, 1, 800) # 30,000, 80,000

    def get_speed(self):
        """
        return : rotations /sec
        """
        psoc.get_speed(self.wheel_num)

    def set_motor_speed(self, speed):
        """
        """
        psoc.set_motor_speed(self.wheel_num, speed)

