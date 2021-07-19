from pca9685 import PCA9685
from machine import I2C, Pin
import math

i2c = I2C(id=0, sda=Pin(16), scl=Pin(17))

class Servos:
    def __init__(self, i2c=i2c, address=0x40, freq=50, min_us=500, max_us=2500, degrees=180):
        self.period = 1000000 / freq
        self.min_duty = self._us2duty(min_us)
        self.max_duty = self._us2duty(max_us)
        self.degrees = degrees
        self.freq = freq
        self.pca9685 = PCA9685(i2c, address)
        self.pca9685.freq(freq)

    def _us2duty(self, value):
        return int(4096 * value / self.period)

    def deg(self, index, degrees=None, radians=None, us=None, duty=None):
        span = self.max_duty - self.min_duty
        if degrees is not None:
            duty = self.min_duty + span * degrees / self.degrees
        elif radians is not None:
            duty = self.min_duty + span * radians / math.radians(self.degrees)
        elif us is not None:
            duty = self._us2duty(us)
        elif duty is not None:
            pass
        else:
            return self.pca9685.duty(index)
        duty = min(self.max_duty, max(self.min_duty, int(duty)))
        self.pca9685.duty(index, duty)

    def release(self, index):
        self.pca9685.duty(index, 0)