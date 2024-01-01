#     |\__/,|   (`\
#   _.|o o  |_   ) )
# -(((---(((--------

import serial

class SensorHandler:
    def __init__(self, arduino_baud = 250000, arduino_port = 'COM6'):
        self.port = arduino_port
        self.baudrate = arduino_baud
        self.arduino = 0
        self.IR_command = 'IR'
        self.IMUg_command = 'Gyro'

    def connectArduino(self):
        self.arduino = serial.Serial(self.port, self.baudrate, timeout=1)
    
    def disconnectArduino(self):
        self.arduino.close()
    
    def getDistance(self):
        self.arduino.write(self.IR_command.encode())
        rdistance = self.arduino.readline().decode().strip()
        if rdistance == '':
            rdistance = self.arduino.readline().decode().strip()
        return float(rdistance)
    
    def getDPS(self):
        self.arduino.write(self.IMUg_command.encode())
        rDPS = self.arduino.readline().decode().strip()
        if rDPS == '':
            rDPS = self.arduino.readline().decode().strip()
        dps = rDPS.split(",")
        xdps = float(dps[0])
        ydps = float(dps[1])
        zdps = float(dps[2])
        return xdps,ydps,zdps

def dps2AX(dps):
    AX_speed = dps/0.666
    return round(AX_speed)

def AX2dps(AX_speed):
    dps = AX_speed*0.666
    return dps

def dps2secs(dps, dist = 180): # Usign 180 as the defeault distance for the caracterization to be able to capture the velocity at the right moment
    time_secs = dist/dps
    return time_secs