
def get_default_wheel_list():
    result = []
    for i in range(3):
        result.append(get_default_wheel())
    return result


class Wheel(object):
    def __init__(self, vector_from_robot_center, wheel_spin=0, linear_velocity):
        pass

