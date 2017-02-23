import wheel

class Robot(object):
    def __init__(self, velocity=(0, 0), angular_velocity=0,
    wheels=wheel.get_defualt_wheel_list(), position=0, theta=0):
        """
        wheels : [Wheel(), Wheel(), Wheel()]
        """
        self.velocity = velocity
        self.angular_velocity = angular_velocity # omega, w
        if len(wheels) != 3:
            throw ValueError("Three wheels not initialzied")
        self.wheels = wheels
        self.position = position
        self.theta = theta
    
    def move_in_square(self, side_len):
        pass

    def center_up(self):
        self.spin()

    def spin(self, degrees=360):
        for wheel in self.wheels:
            wheel.rotate(degrees)

