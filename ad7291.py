"""
Circuit Python Drivers for the AD7291 SAR ADC

* Author(s): Thomas Damiani

Implementation Notes
=====================

**Software dependencies**

* adafruit Circuit Python firmware (8.1 +):
    https://github.com/adafruit/circuitpython/releases
* Adafruit's bys Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

from adafruit_register.i2c_struct import Struct, UnaryStruct
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit, RWBit

try:
    import typing
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.1"
__repo__ = "https://github.com/PyCubed-Mini/CircuitPython_AD7291.git"

# data registers
_COMMAND_REGISTER = 0x00
_VOLTAGE_CONVERSION = 0x01
_T_SENSE_CONVERSION_RESULT = 0x02
_T_SENSE_AVERAGE_RESULT = 0x03
_CH0_DATA_HIGH = 0x04
_CH0_DATA_LOW = 0x05
_CH0_HYSTERESIS = 0X06
_CH1_DATA_HIGH = 0x07
_CH1_DATA_LOW = 0x08
_CH2_DATA_HIGH = 0x0A
_CH2_DATA_LOW = 0x0B
_CH3_DATA_HIGH = 0x0D
_CH3_DATA_LOW = 0x0E
_CH4_DATA_HIGH = 0x10
_CH4_DATA_LOW = 0x11
_CH5_DATA_HIGH = 0x13
_CH5_DATA_LOW = 0x14
_CH6_DATA_HIGH = 0x16
_CH6_DATA_LOW = 0x17
_CH7_DATA_HIGH = 0x19
_CH7_DATA_LOW = 0x1A
_T_SENSE_DATA_HIGH = 0x1C
_T_SENSE_DATA_LOW = 0x1D


_DEFAULT_ADDRESS = 0x2F

# Command Register
# 16 bits, 15-8 are CH0 to CH7,


class AD7291:
    """Driver for the AD7291 SAR ADC"""

    # command register as shown in the table
    # RWBits(num_bits, register_address, lowest_bit, register_width=1,
    #        lsb_first=True)

    active_channels = RWBits(8, _COMMAND_REGISTER, 8,
                             register_width=2, lsb_first=False)
    tsense = RWBit(_COMMAND_REGISTER, 7, register_width=2,
                   lsb_first=False)
    noise_dealy = RWBit(_COMMAND_REGISTER, 5, register_width=2,
                        lsb_frist=False)

    def __init__(self, i2c: I2C, addr: int = _DEFAULT_ADDRESS,
                 number_of_active_channels: int = 0,
                 active_channels: list = [False] * 8,
                 enable_temp_conversions: bool = False) -> None:

        self.i2c_device = I2CDevice(i2c, addr)
        """
        convert bool list to integer representing which channels
        should be active

        Properties
        -----------------

        i2c: the i2c bus being used
        addr: the i2c address of the ad7291 in your hardware

        active_channels: a bool list of length 8 representing which
        voltage input channels of the ad7291 should be read.
        """
        self.channels = self.channel_list_to_bits(active_channels)
        self.tsense = enable_temp_conversions

        # initialize a buffer to interact with i2c
        self.num_active_channels = number_of_active_channels
        self.buf = bytearray(2)

    def channel_list_to_bits(channel_list: list = [False] * 8) -> int:
        """
        takes the input list, should be a bool list, and converts it
        into an integer that will be passed into bits D15-D8 of
        command register
        """
        result = 0
        for i, channel in enumerate(channel_list):
            if channel:

                result += 1 << i
        return result & ((i << 8) - 1)

    @property
    def read_from_voltage(self):
        """Initialize return list"""
        res = [None] * self.num_active_channels

        """want to read from the voltage conversion register"""
        self.buf[0] = _VOLTAGE_CONVERSION

        # writes to buffer that we want the voltage conversion register
        with self.i2c_device as i2c:
            i2c.write(self.buf, end=1)

        """
        Voltages are returned sequentially, so we readinto for
        each channel we have activated.

        Not Currently working, not sure why. Only reads one channel
        """
        for i in range(self.num_active_channels):
            with self.i2c_device as i2c:
                i2c.readinto(self.buf)

            # the channel that the voltage in is coming from
            channel = self.buf[0] >> 4 & ((1 << 4) - 1)  # D[15:12]

            # the conversion from the voltage read
            voltage = (self.buf[0] & ((1 << 4) - 1))     # D[11:8]
            voltage << 8                                 # shift to make room
            voltage += self.buf[1]                       # D[7-0]
            res[i] = (channel, voltage)
        return res
