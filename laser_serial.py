#pyserial

import serial
import io


class LaserIO(object):


    def __init__(self, port_name = "/dev/ttyUSB0"):


        self.port_name = port_name
        self.ser = serial.Serial(self.port_name,9600,timeout = 1)

        self.ser_io = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, 1),  
                               newline = '\r',
                               line_buffering = True)
        self.normal_mode()
    

    def normal_mode(self):
        print("normal mode")
        self.ser_io.write("R0\r")
        out = self.ser_io.readline()
        print(out)

    def measure(self):
        '''
        Get laser head measurement through serial
        '''
        self.ser_io.write('M0\r')
        out = self.ser_io.readline()
        print(out)

        i = out.rfind("M0,") + 3
        if i == -1:
            print("M0 measurement not found")
            return None
        j = out[i:].find(",")
        print(i)
        head_val =  out[i:i+j]
        print(head_val)

        if self.isfloat(head_val):
            distance  = float(head_val)
            return distance 
        else:
            print("Laser head out of range")
            return None

    def isfloat(self,value):
        try:
            float(value)
            return True
        except ValueError:
            return False

    def close(self):
        self.ser.close()
        return None


        
