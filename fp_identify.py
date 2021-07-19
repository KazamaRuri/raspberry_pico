from as608 import fingerprint
from servo import Servos
from ssd1306 import SSD1306_I2C
import utime

fp = fingerprint()
oled = SSD1306_I2C(128, 64)

def ByteToHex( bins ):
    return ''.join( [ "%02X" % x for x in bins ] ).strip()

def display(test):
    oled.text(test, 0, 0)
    oled.show()
    
if __name__ == '__main__':
    while True:
        
        data_GetImage = fp.GetImage()
        if(data_GetImage == b'\x02'):
            display("Please Hold")
        elif(data_GetImage == b'\x00'):
            data_Identify = fp.Identify()
            if(data_Identify[9:10] == b'\x00'):
                user = data_Identify[10:12]
                mark = data_Identify[12:14]
                oled.text("Success",0,15)
                oled.text("User:",0,25)
                oled.text("Mark",0,35)
                oled.text(ByteToHex(user),50,25)
                oled.text(ByteToHex(mark),50,35)
                oled.show()
                utime.sleep(2)
                oled.init_display()
            else:
                oled.init_display()
                display("Pease Retry")
                utime.sleep_ms(500)
                oled.init_display()
        else:
            oled.init_display()
            display("Error")
            utime.sleep_ms(500)
            oled.init_display()
            
