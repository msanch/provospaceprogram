import numpy


class Kinematic():
    WHEEL_DISTANCE_FROM_CENTER_1 = 0.073
    WHEEL_DISTANCE_FROM_CENTER_2 = 0.084

    def __init__(self, psp_num):
        """
        :param psp_num: int 1 or 2
        """
        self.WHEEL_1_DISTANCE_FROM_CENTER = self.WHEEL_DISTANCE_FROM_CENTER_1 if psp_num == 1 else self.WHEEL_DISTANCE_FROM_CENTER_2
        self.WHEEL_2_DISTANCE_FROM_CENTER = self.WHEEL_DISTANCE_FROM_CENTER_1 if psp_num == 1 else self.WHEEL_DISTANCE_FROM_CENTER_2
        self.WHEEL_3_DISTANCE_FROM_CENTER = self.WHEEL_DISTANCE_FROM_CENTER_1 if psp_num == 1 else self.WHEEL_DISTANCE_FROM_CENTER_2
        self.center_of_robot_to_wheel_vectors = self.__get_center_of_robot_to_wheel_vectors()
        self.speed_matrix = self.__create_speed_matrix()

    @staticmethod
    def _get_rotation_matrix(theta):
        matrix = (
            (numpy.cos(theta), numpy.sin(theta), 0),
            (-1 * numpy.sin(theta), numpy.cos(theta), 0),
            (0, 0, 1)
        )
        return numpy.matrix(matrix)

    @staticmethod
    def _rotation(w1, rotation_matrix):
        return rotation_matrix * w1

    def get_xy_correction(self, theta, x_coordinate, y_coordinate):
        original_vector = numpy.matrix((x_coordinate,
                                        y_coordinate,
                                        0)).T
        rotation_matrix = self._get_rotation_matrix(-0.5*theta)
        rotated_vector = self._rotation(original_vector, rotation_matrix)
        rotated_list = rotated_vector.tolist()
        x_rotated = rotated_list[0][0]
        y_rotated = rotated_list[1][0]
        return (x_rotated, y_rotated)

    #
    #             Back
    #              |
    #              |  1            --y
    #              |              | x
    #             / \
    #          2 /   \ 3
    #           /     \
    # Front(b/c a kicker could go here)
    def __get_center_of_robot_to_wheel_vectors(self):
        """Called at init to prevent processing everytime"""
        result = (
            (-self.WHEEL_1_DISTANCE_FROM_CENTER, 0, 0),
            (self.WHEEL_2_DISTANCE_FROM_CENTER  * numpy.cos(-60*numpy.pi/180),
                self.WHEEL_2_DISTANCE_FROM_CENTER * numpy.sin(-60*numpy.pi/180), 0),
            (self.WHEEL_3_DISTANCE_FROM_CENTER * numpy.cos(60*numpy.pi/180),
                self.WHEEL_3_DISTANCE_FROM_CENTER * numpy.sin(60*numpy.pi/180), 0)
        )
        return numpy.matrix(result)

    @staticmethod
    def __create_speed_matrix():
        """
        (
            (x,y)
            (x,y)
            (x,y)
        )
        """
        result = (
            (numpy.cos(270*numpy.pi/180), numpy.sin(270*numpy.pi/180)),
            (numpy.cos( 30*numpy.pi/180), numpy.sin( 30*numpy.pi/180)),
            (numpy.cos(150*numpy.pi/180), numpy.sin(150*numpy.pi/180))
        )
        return result

    def get_wheel_spin_vector(self):
        """
        wheel_spin_direction : [-2.3,0,1.5] - 3 elements for each
                               wheel. Rotation/sec
        return wheel_spin_direction : matrix((x,y,0), (x,y,0), (x,y,0))
        where x,y are functions of the robot theta and wheel direction
        """
        r = self.center_of_robot_to_wheel_vectors
        s = self.speed_matrix
        wheel_tupple_matrix = (
            (s[0][0], s[0][1], s[0][1] * r[0].T[0] - (s[0][0] * r[0].T[1])),
            (s[1][0], s[1][1], s[1][1] * r[1].T[0] - (s[1][0] * r[1].T[1])),
            (s[2][0], s[2][1], s[2][1] * r[2].T[0] - (s[2][0] * r[2].T[1]))
        )
        wheel_matrix = numpy.matrix(wheel_tupple_matrix)
        return wheel_matrix

    RADIUS_OF_WHEELS = 0.030
    RHO = RADIUS_OF_WHEELS

    def _get_m(self):
        """
        given:
            RHO - Wheel size - CONSTANT - Float?
            R_VECTOR - Location of wheels on body - matrix
        wheel_speed_list : list of wheel speeds in rotations/sec
        """
        s = self.get_wheel_spin_vector()
        result = (1 / self.RHO) * s
        return result

    @staticmethod
    def _convert_radians_to_rotations_per_second(omega_list):
        """
        omega_list : [omega, omega, omega] where omega is in radians/sec
        returns : [alpha, alpha, alpha] where alpha is rotations/sec
        """
        result = []
        for omega in omega_list:
            result.append(omega / (2 * numpy.pi))
        return result

    @staticmethod
    def _vector_matrix_to_list(matrix):
        """
        matrix : numpy.matrix
        return : list()
        """
        return matrix.T.tolist()[0]

    def get_desired_wheel_speeds(self, theta, desired_velocities):
        """
        theta : of robot
        desired_velocities : (vx,vy,omega)
        return : [omega, omega, omega] : rotations / second
        """
        m = self._get_m()
        r = self._get_rotation_matrix(theta)
        desired_velocities_matrix = numpy.matrix(desired_velocities).T
        matrix_result = m * r * desired_velocities_matrix
        list_result = self._vector_matrix_to_list(matrix_result)
        result = self._convert_radians_to_rotations_per_second(list_result)
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
    pass
    # get_world_velocity()
    # w1 = numpy.matrix("1;2;3")
    # print rotation(w1, get_rotation_matrix(numpy.pi))
    # help(rotation(w1, get_rotation_matrix(numpy.pi)))


if __name__ == "__main__":
    main()

