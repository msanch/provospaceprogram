def get_default_wheel():
    pass    

def get_default_wheel_list():
    result = []
    for i in range(3):
        result.append(get_default_wheel())
    return result


class Wheel(object):
    def __init__(self, vector_from_robot_center, linear_velocity, wheel_spin=0):
        pass

    def get_speed(self):
        """
    
        """
        pass

    def spin(self, wheel_velocity):
        pass

