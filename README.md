# 树莓派 PICO教程（MicroPython）

#### 1 树莓派 PICO 简介

##### 1.1 简介

　　`Raspberry Pi Pico`是具有灵活数字接口的低成本，高性能微控制器板。它集成了`Raspberry Pi`自己的`RP2040`微控制器芯片，运行速度高达133 MHz的双核`Arm Cortex M0 +`处理器，嵌入式264KB SRAM和2MB板载闪存以及26个多功能GPIO引脚。对于软件开发，可以使用`Raspberry Pi`的C / C ++ SDK或MicroPython。
　　
##### 1.2 配置

|                      树莓派 PICO配置                      |
| :-------------------------------------------------------: |
|               双核 Arm Cortex-M0 + @ 133MHz               |
|       2 个 UART、2 个 SPI 控制器和 2 个 I2C 控制器        |
|           芯片内置 264KB SRAM 和 2MB 的板载闪存           |
|                      16 个 PWM 通道                       |
|        通过专用 QSPI 总线支持最高 16MB 的片外闪存         |
|                  USB 1.1 主机和设备支持                   |
|                        DMA 控制器                         |
| 8 个树莓派可编程 I/O（PIO）状态机，用于自定义外围设备支持 |
|         30 个 GPIO 引脚，其中 4 个可用作模拟输入          |
|    支持 UF2 的 USB 大容量存储启动模式，用于拖放式编程     |

##### 1.3 引脚图

![](https://i.loli.net/2021/07/18/RjMdhA4oSWrCcwF.png)

##### 1.4 尺寸

![img](https://i.loli.net/2021/07/18/3StZqFDkLrKIcuj.png)

#### 2 安装

##### 2.1 烧录固件

1. 点击[rp2-pico-latest.uf2](https://micropython.org/download/rp2-pico/rp2-pico-latest.uf2)链接下载UF2文件
2. 按住BOOTSEL键不放，将Pico插入电脑的USB串口，电脑上会弹出一个新的U盘文件夹，把刚刚下载的UF2文件拖拽到文件夹中，树莓派 PICO将会自动重启，此时，固件烧录完成。

##### 2.2 安装IDE（Thonny IDE）

1. 进入[Thonny IDE](https://thonny.org/)下载软件，最好下载最新版的，否则可能不支持树莓派 PICO；
2. 安装Thonny，安装完成后打开Thonny软件，打开工具->设置-> 解释器，选择`MicroPython(Raspberry Pi Pico)`解释器，并在串口处选择树莓派PICO的串口号（如果板子已经连接在电脑上，软件一般会自动检测串口号）
3. 重启软件，可以看到软件左下方显示了树莓派PICO中的文件；

> 如果没有显示左侧文件树的话可以勾选 视图->文件

##### 2.3 离线运行程序

![新建文件](https://i.loli.net/2021/07/18/4qMT6yKkj5piLDm.png)

　　新建文件，编写完代码后，按住`ctrl+s`将该文件保存在树莓派PICO上，并命名为`main.py`(一定要加后缀`.py`)，下次树莓派PICO通电时便会自动运行`main.py`中的程序。

#### 3 基础

##### 3.01 点亮板载LED灯

```python
复制代码123456789101112PYTHONfrom machine import Pin

if __name__ == '__main__':
    # 构建led对象
    # 板载LED灯连接与引脚25相连
    # LED = Pin(id, mode, pull)
    # id:PICO引脚编号
    # mode:输入输出方式，有Pin.IN(输入)和Pin.OUT(输出)两种
    # pull:上下拉电阻配置，有None(无上下拉电阻)、Pin.PULL_UP(上拉电阻)和Pin.PULL_DOWN(下拉电阻)三种
    LED = Pin(25, Pin.OUT)
    # 高电平点亮
    LED.value(1)
```

##### 3.02 板载LED闪烁

```python
复制代码1234567891011121314PYTHONfrom machine import Pin
from utime import sleep
import utime

led = Pin(25, Pin.OUT)

if __name__ == '__main__':
    while True:
        # led点亮
        led.value(1)
        utime.sleep_ms(1000)
        # led熄灭
        led.value(0)
        utime.sleep_ms(1000)
```

##### 3.03 LED流水灯

1. LED发光二极管正负极区分

- 一般引脚长的一端为正极，引脚短的为负极
- 看发光二极管内部，支架大的为负极，支架小的为负极

1. 电路连线图

![img](https://pic.imgdb.cn/item/60ab83b6ce272128a6d2ad56.png)

![img](https://pic.imgdb.cn/item/60ab83a0ce272128a6d226b2.jpg)

1. 代码

```python
from machine import Pin
import utime

# 定义LED引脚数组
leds = [Pin(i,Pin.OUT) for i in range(0,5)]

if __name__ == '__main__':
    while True:
        # 依次点亮
        for n in range(0,5):
            leds[n].value(1)
            utime.sleep_ms(200)
        # 依次熄灭
        for n in range(0,5):
            leds[n].value(0)
            utime.sleep_ms(100)
```

![img](https://pic.imgdb.cn/item/60ab83a0ce272128a6d22746.gif)

##### 3.04 按键实验

1. 四角按键图片

![img](https://pic.imgdb.cn/item/60ab86bcce272128a6e71d47.jpg)

1. 四角按键怎么连接

![img](https://pic.imgdb.cn/item/60ab86bcce272128a6e71d73.jpg)

默认按键未按下的情况下，12相连接，34相连接；当按下按键时，1234才相连接。

1. 电路接线图

![img](https://pic.imgdb.cn/item/60ab89e3ce272128a6fc8124.png)

![img](https://i.loli.net/2021/07/18/rKLxl3k2PFRWqfO.jpg)

1. 代码

```python
from machine import Pin
import utime

# 配置按键
# key = machine.Pin(id, mode, pull)
# id:树莓派Pico引脚编号
# mode:输入输出方式，有Pin.IN(输入)和Pin.OUT(输出)两种
# pull:上下拉电阻配置，有None(无上下拉电阻)、Pin.PULL_UP(上拉电阻)和Pin.PULL_DOWN(下拉电阻)三种
key = Pin(0, Pin.IN, Pin.PULL_UP)

if __name__ == '__main__':
    while True:
        # print(key.value())
        if key.value() == 0:
            # 等待一段时间，防止抖动
            utime.sleep_ms(100)
            if key.value() == 0:
                print('The button is pressed')
```

> 按键消抖可以参考[https://baike.baidu.com/item/%E6%8C%89%E9%94%AE%E6%B6%88%E6%8A%96](https://baike.baidu.com/item/按键消抖)

##### 3.05 外部中断(改进3.04 按键实验)

1. 什么是外部中断

　　外部中断是单片机实时地处理外部事件的一种内部机制。当某种外部事件发生时，单片机的中断系统将迫使CPU暂停正在执行的程序，转而去进行中断事件的处理；中断处理完毕后．又返回被中断的程序处，继续执行下去。[[3\]](https://www.cnblogs.com/Elijah-Z/p/14833517.html#fn3)

1. 外部中断的作用

- 节省CPU资源

1. 代码实现

　　在3.04 按键实验中，检测按键是否被按下采用的是在主程序中写死循环的办法，假如这个按键被按下的频率十分低（一天只有几次被按下），采用死循环的方法将会浪费大量的CPU资源，而采用外部中断的方式检测按键是否被按下将大大节省CPU资源。

```python
from machine import Pin
import utime

#配置按键
key = Pin(0, Pin.IN, Pin.PULL_UP)

def external_interrupt(key):
    # 消除抖动
    utime.sleep_ms(100)
    # 再次判断按键是否被按下
    if key.value() == 0:
        print('The button is pressed')

if __name__ == '__main__':
    # KEY.irq(handler,trigger)
    # handler:中断执行的回调函数
    # trigger:触发中断的方式，分别为Pin.IRQ_FALLING(下降沿触发)、
    # Pin.IRQ_RISING(上升沿触发)、Pin.IRQ_LOW_LEVEL(低电平触发)和
    # Pin.IRQ_HIGH_LEVEL(高电平触发)四种
    # 定义中断，下降沿触发
    key.irq(external_interrupt, Pin.IRQ_FALLING)
```

##### 3.06 定时器中断(改进3.02 板载LED闪烁)

1. 什么是定时器中断

- 定时器中断是由单片机中的定时器溢出而申请的中断，即设定一个时间，到达这个时间后就会产生中断

1. 代码

通过设置定时器中断使树莓派PICO板载LED每隔两秒闪烁一次

```python
from machine import Pin, Timer

# 创建LED对象
led=Pin(25, Pin.OUT)

# 闪烁回调函数
def twinkle(tim):
    # toggle方法:LED状态翻转
    led.toggle()

if __name__ == '__main__':
    # 构建定时器
    tim = Timer()
    # tim.init(period, mode, callback)
    # period:周期时间(单位为ms)
    # mode:工作模式，有Timer.ONE_SHOT(执行一次)和Timer.PERIODIC(周期性执行)两种
    # callback:定时器中断的回调函数
    tim.init(period=2000, mode=Timer.PERIODIC, callback=twinkle)
```

##### 3.07 PWM 脉冲宽度调制(实现板载LED呼吸灯)

1. 什么是PWM

　　脉冲宽度调制是一种模拟控制方式，根据相应载荷的变化来调制晶体管基极或MOS管栅极的偏置，来实现晶体管或MOS管导通时间的改变，从而实现开关稳压电源输出的改变。这种方式能使电源的输出电压在工作条件变化时保持恒定，是利用微处理器的数字信号对模拟电路进行控制的一种非常有效的技术。脉冲宽度调制是利用微处理器的数字输出来对模拟电路进行控制的一种非常有效的技术，广泛应用在从测量、通信到功率控制与变换的许多领域中。[[4\]](https://www.cnblogs.com/Elijah-Z/p/14833517.html#fn4)

1. 代码

```python
from machine import Pin, Timer, PWM
import utime

led = PWM(Pin(25))
# 设置频率值
led.freq(1000)

led_value = 0
# led以5%增长/减少的速度变化亮度
led_space = 5

if __name__ == '__main__':
    while True:
        led_value += led_space
        if led_value >= 100:
            led_value = 100
            led_space = -5
        elif led_value <= 0:
            led_value = 0
            led_space = 5
        # 设置占空比，需在0-65535之间
        led.duty_u16(int(led_value * 500))
        utime.sleep_ms(100)
```

![img](https://pic.imgdb.cn/item/60aba708ce272128a6dc80f6.gif)

##### 3.08 I2C总线(使用SSD1306 OLED屏幕)

1. I2C总线简介

　　I2C总线是由Philips公司开发的一种简单、双向二线制同步串行总线。它只需要两根线即可在连接于总线上的器件之间传送信息。I2C由 2 条线组成：SDA（串行数据线）和SCL（串行时钟线），都是双向I/O线。[[5\]](https://www.cnblogs.com/Elijah-Z/p/14833517.html#fn5)

1. SSD1306 OLED简介

　　SSD1306是一款带控制器的用于OLED点阵图形显示系统的单片CMOS OLED/PLED驱动器。它由128个SEG（列输出）和64个COM（行输出）组成。该芯片专为共阴极OLED面板设计。
  SSD1306内置对比度控制器、显示RAM（GDDRAM）和振荡器，以此减少了外部元件的数量和功耗。该芯片有256级亮度控制。数据或命令由通用微控制器通过硬件选择的6800/8000系通用并行接口、I2C接口或串行外围接口发送。该芯片适用于许多小型便携式应用，如手机副显示屏、MP3播放器和计算器等。[[6\]](https://www.cnblogs.com/Elijah-Z/p/14833517.html#fn6)[[7\]](https://www.cnblogs.com/Elijah-Z/p/14833517.html#fn7)

1. 电路连线图

![img](https://pic.imgdb.cn/item/60aba900ce272128a6edcaa5.png)

1. 代码

> ssd1306.py下载地址： https://elijah.lanzoui.com/iJ13fpnq6je
>
> 下载完成后，在Thonny软件左侧的文件窗口内找到这个文件，右键点击文件，选择上载到选项，文件即可传输到树莓派PICO上

```python
from machine import SoftI2C, Pin
# 导入SSD1306驱动模块
from ssd1306 import SSD1306_I2C

if __name__ == '__main__':
    # 初始化SoftI2C
    # OLED屏幕的scl连接到树莓派PICO的GPIO0, sda连接到GPIO1
    i2c = SoftI2C(scl=Pin(0), sda=Pin(1))
    # oled = SSD1306_I2C(width, height, i2c, addr)
    # width:屏幕宽
    # height: 屏幕高
    # i2c:已定义的I2C对象
    oled = SSD1306_I2C(128, 64, i2c) #OLED显示屏初始化：128*64分辨率,OLED的I2C地址是0x3c
    # OLED显示的字符串，横坐标和纵坐标
    oled.text("Hello World!", 0, 0)
    # OLED显示
    oled.show()
```

<img src="https://pic.imgdb.cn/item/60abaa43ce272128a6fa7be5.jpg" alt="img" style="zoom:50%;" />

#### 4 传感器程序

##### 4.1 温度传感器(DS18B20)

DS18B20是常用的数字温度传感器，其输出的是数字信号，具有体积小，硬件开销低，抗干扰能力强，精度高的特点。

- 测温范围: -55℃～+125℃，固有测温误差1℃
- 工作电源: 3.0~5.5V/DC
- 单总线驱动，只占用一个IO口

```python
复制代码1234567891011121314151617181920212223242526PYTHONimport machine, onewire, ds18x20, time, utime

# 使用GPIO0口传输数据
# 将DS18B20的VCC端连接到树莓派PICO的3V3(OUT)端
# 将DS18B20的数据端连接到树莓派PICO的GPIO0口
# 将DS18B20的GND端连接到树莓派PICO的GND端
pin = machine.Pin(0)
sensor = ds18x20.DS18X20(onewire.OneWire(pin))

# 扫描是否存在DS18B20设备
roms = sensor.scan()
print('Found a ds18x20 device')

# 获取温度数据
def detect_tem():
    while True:
      sensor.convert_temp()
      for rom in roms:
        # 打印出温度值
        # 第一个打印出来的数值可能不太准确，从第二条数据开始才会显示出正常数据
        print("{:.3f}".format(sensor.read_temp(rom)))  
      utime.sleep_ms(2000)
      
# 程序入口
if __name__ == '__main__':
    detect_tem()
```

##### 4.2 温湿度传感器

> DHT22.py文件下载地址： https://elijah.lanzoui.com/iFueapnq6id
>
> 文件上传方法参考3.08 I2C总线

###### 4.2.1 DHT11

DHT11是一款有已校准数字信号输出的温湿度传感器。 其精度湿度±5%RH， 温度±2℃，量程湿度5-95%RH， 温度0-+50℃。[[8\]](https://www.cnblogs.com/Elijah-Z/p/14833517.html#fn8)

```python
复制代码123456789101112131415161718192021222324PYTHONfrom machine import Pin
from DHT22 import DHT22
import utime

pin = Pin(0,Pin.IN,Pin.PULL_UP)
# 创建dht11对象
# 将DHT11的VCC端连接到树莓派PICO的3V3(OUT)端
# 将DHT11的数据端连接到树莓派PICO的GPIO0口
# 将DHT11的GND端连接到树莓派PICO的GND端
dht_sensor=DHT22(pin, dht11=True)

# 循环函数
def detection():
    while True:
        T, H = dht_sensor.read()
        if T is None:
            print("sensor error")
        else:
            print("{}'C  {}%".format(T, H))
        utime.sleep_ms(2000)

# 程序入口
if __name__ == '__main__':    
    detection()
```

###### 4.2.1 DHT22

　　DHT22也称AM2302，是一款含有已校准数字信号输出的温湿度复合传感器，湿度量程范围0-99.9%RH，精度±2%RH，而温度量程范围是-40℃-80℃，精度±0.5℃。[[9\]](https://www.cnblogs.com/Elijah-Z/p/14833517.html#fn9)

```python
复制代码123456789101112131415161718192021222324PYTHONfrom machine import Pin
from DHT22 import DHT22
import utime

pin = Pin(0,Pin.IN,Pin.PULL_UP)
# 创建dht11对象
# 将DHT11的VCC端连接到树莓派PICO的3V3(OUT)端
# 将DHT11的数据端连接到树莓派PICO的GPIO0口
# 将DHT11的GND端连接到树莓派PICO的GND端
dht_sensor=DHT22(pin, dht11=False)

# 循环函数
def detection():
    while True:
        T, H = dht_sensor.read()
        if T is None:
            print("sensor error")
        else:
            print("{:.2f}'C  {:.2f}%".format(T, H))
        utime.sleep_ms(2000)

# 程序入口
if __name__ == '__main__':    
    detection()
```

