
class Robot(object):
    def __init__(self, velocity=(0, 0), angular_velocity=0, *args):
        """
        TODO: make
        args : list of Wheel objects to be used as wheels
        """
        self.velocity = (0, 0)
        self.angular_velocity = 0 # omega, w
        if len(args) != 3:
            throw Exception("Three wheels not initialzied")
        self.wheels = [args[i] for i in range(3)]
    
    def move_in_square(self, side_len):
        pass

    def center_up(self):
        pass

    def spin(self):
        pass

