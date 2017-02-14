import numpy

def get_rotation_matrix(theta):
    return [[numpy.cos(theta), numpy.sin(theta), 0], [-1 * numpy.sin(theta), numpy.cos(theta), 0],
        [0, 0, 1]]

def rotation(w1, rotation_matrix):
    return rotation_matrix * w1

def main():
    w1 = numpy.matrix("1;2;3")
    print(rotation(w1, get_rotation_matrix(5)))

if __name__ == "__main__":
    main()

