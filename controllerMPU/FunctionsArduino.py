import serial


class FunctionsArduino:
    def __init__(self, COM):
        self.arduino = serial.Serial(COM, 9600)

    def get_gyroscope(self):
        rawString = self.arduino.readline()
        # print(rawString)
        # for item in rawString:
        #     print(item)

        return rawString
