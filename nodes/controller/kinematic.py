import numpy

def _get_rotation_matrix(theta):
    matrix = [
        [numpy.cos(theta), numpy.sin(theta), 0], 
        [-1 * numpy.sin(theta), numpy.cos(theta), 0],
        [0, 0, 1]
    ]
    return numpy.matrix(matrix)

def _rotation(w1, rotation_matrix):
    return rotation_matrix * w1

# FIXME MICHAEL : I need the radius of each wheel from the center of the body
# in meters
#
#             Back
#              |
#              |  3
#              |
#             / \
#          2 /   \ 1
#           /     \
# Front(b/c a kicker could go here)
WHEEL_DISTANCE_FROM_CENTER = 0.0381
# TODO Keep this function, but use it to generate a constant, then use that
# hard-coded constant
def _get_center_of_robot_to_wheel_vectors():
    result = [
        (WHEEL_DISTANCE_FROM_CENTER * 0.86602540378443871, # numpy.cos(-30*numpy.pi/180)
            WHEEL_DISTANCE_FROM_CENTER * -0.49999999999999994, 0), # numpy.sin(-30*numpy.pi/180)
        (WHEEL_DISTANCE_FROM_CENTER * -0.49999999999999978, # numpy.cos(-120*numpy.pi/180) 
            WHEEL_DISTANCE_FROM_CENTER * -0.86602540378443871, 0), # numpy.cos(-120*numpy.pi/180)
        (0, WHEEL_DISTANCE_FROM_CENTER, 0)
    ]
    return numpy.matrix(result)

# FIXME MICHAEL : I need the radius of the wheels in meters
RADIUS_OF_WHEELS = 0.004
RHO = RADIUS_OF_WHEELS
def get_wheel_spin_vector(theta, wheel_spin):
    """
    theta : robot body
    wheel_spin_direction : [-x,0,y] - 3 elements for each 
                           wheel. Rotation/sec
    return wheel_spin_direction : [(x,y,0), (x,y,0), (x,y,0)]
    where x,y are functions of the robot theta and wheel direction
    """
    _get_center_of_robot_to_wheel_vectors()    

def _get_m(theta, wheel_speed_list):
    """
    given: 
        RHO - Wheel size - CONSTANT - Float?
        R_VECTOR - Location of wheels on body - matrix 
    theta : robot degree turned
    wheel_speed_list : list of wheel speeds in rotations/sec
    """
    s = get_wheel_spin_vectors(theta, wheel_speed_list)
    harry_matrix = 
    result = (1 / RHO) * harry_matrix
    return

def _convert_radians_to_rotations_per_second(omega_list):
    """
    omega_list : [omega, omega, omega] where omega is in radians/sec
    returns : [alpha, alpha, alpha] where alpha is rotations/sec
    """
    result = []
    for omega in omega_list:
        result.append(omega / (2 * numpy.pi))
    return result

def _vector_matrix_to_list(matrix):
    """
    matrix : numpy.matrix
    return : list()
    """
    return matrix.T.tolist()[0]

def get_wheel_speeds(wheel_speed_list, theta, desired_velocities):
    """
    wheel_speed_list : [a,b,c] where a,b,c are 
                       floats representing rot/sec
    theta : of robot
    desired_velocities : (vx,vy,omega)
    return : [omega, omega, omega] : rotations / second
    """
    m = _get_m(theta, wheel_speed_list)
    r = _get_rotation_matrix(theta)
    desired_velocities_matrix = numpy.matrix(desired_velocities).T
    matrix_result = m * r * desired_velocities_matrix
    list_result = _vector_matrix_to_list(matrix_result)
    result = _convert_radians_to_rotations_per_second(list_result)
    return result

# def get_world_velocity(theta, angular_velocity_list):
#     """
#     theta : of robot - to be used in rotation matrix
#     angular_velocity_list : (omega, omega, omega) where omega is rotations/sec
#     """
#     r = _get_rotation_matrix(theta)
#     r_t = r.T
#     m_inv = 
#     angular_velocity_rotations_per_second_vector
#     result_vector = r_t * m_inv * angular_velocity_rotations_per_second_vector
#     return result_vector.T.tolist()[0]

def main():
    get_world_velocity()
    w1 = numpy.matrix("1;2;3")
    print rotation(w1, get_rotation_matrix(numpy.pi))
    help(rotation(w1, get_rotation_matrix(numpy.pi)))

if __name__ == "__main__":
    main()

