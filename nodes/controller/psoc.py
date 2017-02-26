import struct
import serial

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None)

SAMPLE_RATE = 50 #samples per second
PULSES_PER_ROTATION = 4955 #Old motors

motor_1_speed = 0
motor_2_speed = 0
motor_3_speed = 0

def spin():
    ser.write('s')
    writeFloat(motor_1_speed)
    writeFloat(motor_2_speed)
    writeFloat(motor_3_speed)

def writeFloat(f):
    ser.write(struct.pack('>i', int(f*1000)))

def readFloat():
    return float(struct.unpack('>i', ser.read(4))[0])/1000

def setPower(p1, p2, p3):
    ser.write('p')
    writeFloat(p1)
    writeFloat(p2)
    writeFloat(p3)

def set_motor_speed(motor_num, speed):
    if motor_num == 1:
        global motor_1_speed
        motor_1_speed = speed
    elif motor_num == 2:
        global motor_2_speed
        motor_2_speed = speed
    elif motor_num == 3:
        global motor_3_speed
        motor_3_speed = speed

def setPID(motor, p, i, qpps): #use motor = 0 to set all motors
    ser.write('k')
    ser.write(str(motor))
    writeFloat(p)
    writeFloat(i)
    writeFloat(qpps)

def setT(period_ms, tau_ms):
    ser.write('t')
    writeFloat(period_ms)
    writeFloat(tau_ms)

def getSpeed(wheel_num=0):
    """
    wheel_num : default value of 0, if 1-3 it will return that motors value
    return : list with either all three values in it or the desired value
    """
    ser.write('v')
    wheel = [None, None, None, None]
    wheel[1] = [readFloat()]
    wheel[2] = [readFloat()]
    wheel[3] = [readFloat()]
    wheel[0] = [wheel[1][0], wheel[2][0], wheel[3][0]]
    return wheel[wheel_num]


def getEncoderCount():
    ser.write('e')
    return readFloat(), readFloat(), readFloat()

def disengage():
    ser.write('d')

setT(20, 50)

