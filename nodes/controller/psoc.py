import serial
import struct


class FakeSerial():
    def read(self, *args, **kwargs):
        pass

    def write(self, *args, **kwargs):
        pass


class PSoC():
    def __init__(self, debug, psp_num):
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) if not debug else FakeSerial()
        print "Set PIDS"
        self.PULSES_PER_ROTATION = 4955 if psp_num == 1 else 116.9
        self.set_t(20, 50)
        self.set_pids(psp_num)

    def write_float(self, f):
        self.ser.write(struct.pack('>i', int(f*1000)))

    def read_float(self):
        return float(struct.unpack('>i', self.ser.read(4))[0])/1000

    def set_pids(self, psp_num):
        """
        Robot 1:
            Speed avg 34194.9464 35653.5624 34796.385
        Robot 2:
        Speed avg 370.3572 385.2274 380.7416
        """
        pid_1 = 34194.9464 if psp_num == 1 else 370.3572
        pid_2 = 35653.5624 if psp_num == 1 else 385.2274
        pid_3 = 34796.385  if psp_num == 1 else 380.7416
        self.set_pid(1, 1, 5.0, pid_1)
        self.set_pid(2, 1, 5.0, pid_2)
        self.set_pid(3, 1, 5.0, pid_3)

    def set_power(self, p1, p2, p3):
        self.ser.write('p')
        self.write_float(p1)
        self.write_float(p2)
        self.write_float(p3)

    def set_motor_speeds(self, speeds):
        self.ser.write('s')
        self.write_float(speeds[0]*self.PULSES_PER_ROTATION)
        self.write_float(speeds[1]*self.PULSES_PER_ROTATION)
        self.write_float(speeds[2]*self.PULSES_PER_ROTATION)

    def set_pid(self, motor, p, i, qpps):  # use motor = 0 to set all motors
        self.ser.write('k')
        self.ser.write(str(motor))
        self.write_float(p)
        self.write_float(i)
        self.write_float(qpps)

    def set_t(self, period_ms, tau_ms):
        self.ser.write('t')
        self.write_float(period_ms)
        self.write_float(tau_ms)

    def get_speed(self, wheel_num=0):
        """
        wheel_num : default value of 0, if 1-3 it will return that motors value
        return : list with either all three values in it or the desired value
        """
        self.ser.write('v')
        wheel = [None, None, None, None]
        self.wheel[1] = (self.read_float())
        self.wheel[2] = (self.read_float())
        self.wheel[3] = (self.read_float())
        self.wheel[0] = (wheel[1][0], wheel[2][0], wheel[3][0])
        return wheel[wheel_num]

    def get_encoder_count(self):
        self.ser.write('e')
        return self.read_float(), self.read_float(), self.read_float()

    def disengage(self):
        self.ser.write('d')
