import struct
import serial
import sys
import time

class FakeSerial():
    def read(self, *args, **kwargs):
        pass

    def write(self, *args, **kwargs):
        pass

ser = FakeSerial()

def initialize(debug, psp_num):
    global ser
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) if not debug else FakeSerial()
    print "Set PIDS"
    setT(20, 50)
    set_pids(psp_num)

SAMPLE_RATE = 50  # samples per second
PULSES_PER_ROTATION = 4955  # Old motors


def writeFloat(f):
    ser.write(struct.pack('>i', int(f*1000)))


def readFloat():
    return float(struct.unpack('>i', ser.read(4))[0])/1000


def setPower(p1, p2, p3):
    ser.write('p')
    writeFloat(p1)
    writeFloat(p2)
    writeFloat(p3)


def set_pids(psp_num):
    """
    Robot 1:
        Speed avg 34194.9464 35653.5624 34796.385
    Robot 2:
	Speed avg 370.3572 385.2274 380.7416
    """
    pid_1 = 34194.9464 if psp_num == 1 else 370.3572
    pid_2 = 35653.5624 if psp_num == 1 else 385.2274
    pid_3 = 34796.385  if psp_num == 1 else 380.7416
    setPID(1, 1, 5.0, pid_1)
    setPID(2, 1, 5.0, pid_2)
    setPID(3, 1, 5.0, pid_3)


def setSpeed(s1, s2, s3):
	ser.write('s')
	writeFloat(s1)
	writeFloat(s2)
	writeFloat(s3)


def set_motor_speeds(speeds):
    setSpeed(speeds[0]*PULSES_PER_ROTATION, speeds[1]*PULSES_PER_ROTATION, speeds[2]*PULSES_PER_ROTATION)


def setPID(motor, p, i, qpps):  # use motor = 0 to set all motors
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

