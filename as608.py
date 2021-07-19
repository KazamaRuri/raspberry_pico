from machine import UART,Pin
import utime

cmd_getimage   = b'\xEF\x01\xFF\xFF\xFF\xFF\x01\x00\x03\x01\x00\x05'
cmd_genchar    = b'\xEF\x01\xFF\xFF\xFF\xFF\x01\x00\x04\x02\x01\x00\x08'
cmd_search     = b'\xEF\x01\xFF\xFF\xFF\xFF\x01\x00\x08\x04\x01\x00\x00\x00\x64\x00\x72'
cmd_enroll     = b'\xEF\x01\xFF\xFF\xFF\xFF\x01\x00\x03\x10\x00\x14'
cmd_identify   = b'\xEF\x01\xFF\xFF\xFF\xFF\x01\x00\x03\x11\x00\x15'
cmd_deletchar  = b'\xEF\x01\xFF\xFF\xFF\xFF\x01\x00\x07\x0C\x00\x00\x00\x01\x00\16' # page number sum
cmd_emptychar  = b'\xEF\x01\xFF\xFF\xFF\xFF\x01\x00\x03\x0D\x00\x11'

class fingerprint:
    def __init__(self):
        self.uart = UART(id=0,baudrate=57600,bits=8,parity=None,stop=1,tx=Pin(0),rx=Pin(1))
        utime.sleep_ms(500)

    def write(self,cmd):
        self.uart.write(cmd)
    
    def recv(self):
        return self.uart.read()

    def GetImage(self):
        self.write(cmd_getimage)
        utime.sleep(1)
        data = self.recv()
        return data[9:10]

    def GenChar(self):
        self.write(cmd_genchar)
        utime.sleep(1)
        data = bytes()
        data = self.recv()
        return data[9:10]

    def Search(self):
        self.write(cmd_search)
        utime.sleep(1)
        data = bytes()
        data = self.recv()
        return data

    def Enroll(self):
        self.write(cmd_enroll)
        utime.sleep(1)
        data = bytes()
        data = self.recv()
        return data

    def Identify(self):
        self.write(cmd_identify)
        utime.sleep(1)
        data = bytes()
        data = self.recv()
        return data

    def DeletChar(self):
        self.write(cmd_deletchar)
        utime.sleep(1)
        data = bytes()
        data = self.recv()
        return data
    
    def EmptyChar(self):
        self.write(cmd_emptychar)
        utime.sleep(1)
        data = bytes()
        data = self.recv()
        return data        
    
if __name__ == '__main__':
    fp = fingerprint()
    print(fp.EmptyChar())
    