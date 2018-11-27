import math
import serial
import time


def ascii2int(value):
    return value - 48


def values(arr, x10):
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


def conver_serial_msg(arduino_msg):
    # arduino_msg = b'#0#1876#2536#30072#163#425#52\r\n'
    msg = {
        'num_mpu': 0,
        'yaw': 0,
        'pitch': 0,
        'roll': 0,
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
            ini.append(ascii2int(item))
        elif item == 70:
            return msg

    for i in range(0, len(ini)):
        if ini[i] == '#':
            pos.append(i)
    pos.append(len(ini))

    for i in range(0, len(pos) - 1):
        msg_arr.append(values(ini[pos[i]:pos[i + 1]], pos[i + 1] - pos[i] - 1))

    msg['num_mpu'] = msg_arr[0]
    msg['yaw'] = msg_arr[1]
    msg['pitch'] = msg_arr[2] - 3.03
    msg['roll'] = msg_arr[3] - 5.96

    return msg


arduino = serial.Serial('COM3', 115200)
i = 0
aux = True
dictionary = None
gyroScale = 131
t = time.time()
j = 1

while 1:
    rawString = arduino.readline()
    if i >= 30:
        dictionary = conver_serial_msg(rawString)
        if dictionary['yaw'] != 0 and dictionary['pitch'] != 0 and dictionary['roll'] != 0:
            print(dictionary)
        aux = False
    if aux is True:
        i += 1

arduino.close()
