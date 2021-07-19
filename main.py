from machine import SoftI2C, Pin
# 导入SSD1306驱动模块
from ssd1306 import SSD1306_I2C


def ByteToHex( bins ):
    return ''.join( [ "%02X" % x for x in bins ] ).strip()

if __name__ == '__main__':
    # 初始化SoftI2C
    # OLED屏幕的scl连接到树莓派PICO的GPIO0, sda连接到GPIO1

    # oled = SSD1306_I2C(width, height, i2c, addr)
    # width:屏幕宽
    # height: 屏幕高
    # i2c:已定义的I2C对象
    oled = SSD1306_I2C(128, 64) #OLED显示屏初始化：128*64分辨率,OLED的I2C地址是0x3c
    # OLED显示的字符串，横坐标和纵坐标
    b = ByteToHex(b'\xe9\x80\x86\xe7\x81\xab')
    
    print(b)
    oled.text(b, 0, 0)
    # OLED显示
    oled.show()