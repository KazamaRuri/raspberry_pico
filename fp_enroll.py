from as608 import fingerprint
from servo import Servos
from ssd1306 import SSD1306_I2C
import utime

fp = fingerprint()
oled = SSD1306_I2C(128, 64)

def ByteToHex( bins ):
    return ''.join( [ "%02X" % x for x in bins ] ).strip()

def display(test):
    oled.text("enroll",40,0)
    oled.text(test, 0, 15)
    oled.show()

if __name__ == '__main__':
    while True:
        data_GetImage = fp.GetImage()
        if(data_GetImage == b'\x02'):
            display("Please Hold")

        elif(data_GetImage == b'\x00'):
            data_GenChar = fp.GenChar()
            if(data_GenChar == b'\x00'):
                data_Search = fp.Search()
                if(data_Search[9:10] == b'\x00'):
                    user = data_Search[10:12]
                    mark = data_Search[12:14]
                    oled.text("has been exist",0,25)
                    oled.text("User:",0,35)
                    oled.text("Mark",0,45)
                    oled.text(ByteToHex(user),50,35)
                    oled.text(ByteToHex(mark),50,45)
                    oled.show()
                    utime.sleep(2)
                    oled.init_display()
                else:
                    oled.text("enrolling",0,25)
                    data_Enroll = fp.Enroll()
                    if(data_Enroll[9:10] == b'\x00'):
                        user = data_Enroll[10:12]
                        oled.text("enroll succeed",0,35)
                        oled.text("User:",0,45)
                        oled.text(ByteToHex(user),50,45)
                        oled.show()
                        utime.sleep(2)
                        oled.init_display()
                    
                    else:
                        oled.init_display()
                        display("enroll error")
                        utime.sleep_ms(500)
                        oled.init_display()
            else:
                oled.init_display()
                display("gen not succeed")
                utime.sleep_ms(500)
                oled.init_display()
        else:
            oled.init_display()
            display("error")
            utime.sleep_ms(500)
            oled.init_display()