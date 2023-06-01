import ad7291
import board
import time

# create I2C bus
i2c = board.I2C()   # uses board.SCL and board.SDA

"""
initialize an ad7291
this example has 1 active channel
the 1st index in active_channels is set to true
indicating that the 1st channel is active
"""
ad = ad7291.AD7291(i2c, number_of_active_channels=1,
                   active_channels=[False, True, False,
                                    False, False, False,
                                    False, False])

while True:
    print(ad.read_voltage)
    time.sleep(1)