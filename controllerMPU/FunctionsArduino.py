import serial


class FunctionsArduino:
    def __init__(self, COM, baud):
        self.arduino = serial.Serial(COM, baud)
        self.i = 0
        self.aux = True
        self.dictionary = None

        # Reset Arduino
        print('Reset Arduino')
        self.arduino.write(b'2')

    def ascii2int(self, value):
        return value - 48

    def values(self, arr, x10):
        ex = []
        value = 0
        sig = 1
        for i in range(x10 - 1, -1, -1):
            ex.append(10 ** i)
        for i in range(0, len(arr) - 1):
            if arr[i + 1] >= 0:
                value += arr[i + 1] * ex[i]
            else:
                sig = -1
        return sig * value / 100

    def conver_serial_msg(self, arduino_msg):
        # arduino_msg = b'#0#1876#2536#30072#163#425#52\r\n'
        msg = {
            'num_mpu': 0,
            'roll': 0,
            'pitch': 0,
            'yaw': 0
        }
        ini = []
        pos = []
        msg_arr = []

        for item in arduino_msg:
            if item == 35:
                ini.append(chr(item))
            elif item == 45:
                ini.append(-1)
            elif 48 <= item <= 57:
                ini.append(self.ascii2int(item))
            elif item == 70:
                return msg

        for i in range(0, len(ini)):
            if ini[i] == '#':
                pos.append(i)
        pos.append(len(ini))

        for i in range(0, len(pos) - 1):
            msg_arr.append(self.values(ini[pos[i]:pos[i + 1]], pos[i + 1] - pos[i] - 1))

        msg['num_mpu'] = msg_arr[0]
        msg['roll'] = msg_arr[1]
        msg['pitch'] = msg_arr[2]
        msg['yaw'] = msg_arr[3]

        return msg

    def get_dictionary(self):
        return self.dictionary

    def get_gyroscope(self):
        rawString = self.arduino.readline()
        # print(rawString)
        if self.i == 7:
            print('Start Arduino')
            self.arduino.write(b'1')
        if self.i >= 30:
            self.dictionary = self.conver_serial_msg(rawString)
            self.aux = False
        if self.aux is True:
            self.i += 1
