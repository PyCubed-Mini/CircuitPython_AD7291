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

from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_struct import Struct, UnaryStruct
from micropython import const

try:
    import typing

    from busio import I2C
    from typing_extensions import Literal
except ImportError:
    pass

__version__ = "0.0.1"
__repo__ = "https://github.com/PyCubed-Mini/CircuitPython_AD7291.git"

# data registers as shown on page 16 of datasheet
_COMMAND_REGISTER = const(0x00)
_VOLTAGE_CONVERSION = const(0x01)
_T_SENSE_CONVERSION_RESULT = const(0x02)
_T_SENSE_AVERAGE_RESULT = const(0x03)
_CH0_DATA_HIGH = const(0x04)
_CH0_DATA_LOW = const(0x05)
_CH0_HYSTERESIS = const(0X06)
_CH1_DATA_HIGH = const(0x07)
_CH1_DATA_LOW = const(0x08)
_CH1_HYSTERESIS = const(0x09)
_CH2_DATA_HIGH = const(0x0A)
_CH2_DATA_LOW = const(0x0B)
_CH2_HYSTERESIS = const(0x0C)
_CH3_DATA_HIGH = const(0x0D)
_CH3_DATA_LOW = const(0x0E)
_CH3_HYSTERESIS = const(0x0F)
_CH4_DATA_HIGH = const(0x10)
_CH4_DATA_LOW = const(0x11)
_CH4_HYSTERESIS = const(0x12)
_CH5_DATA_HIGH = const(0x13)
_CH5_DATA_LOW = const(0x14)
_CH5_HYSTERESIS = const(0x15)
_CH6_DATA_HIGH = const(0x16)
_CH6_DATA_LOW = const(0x17)
_CH6_HYSTERESIS = const(0x18)
_CH7_DATA_HIGH = const(0x19)
_CH7_DATA_LOW = const(0x1A)
_CH7_HYSTERESIS = const(0X1B)
_T_SENSE_DATA_HIGH = const(0x1C)
_T_SENSE_DATA_LOW = const(0x1D)
_T_SENES_HYSTERESIS = const(0X1E)


_DEFAULT_ADDRESS = 0x2F


class AD7291:
    """Driver for the AD7291 SAR ADC"""

    def __init__(self, i2c: I2C, addr: int = _DEFAULT_ADDRESS,
                 number_of_active_channels: int = 0,
                 active_channels: list = [False] * 8,
                 enable_temp_conversions: bool = False,
                 enable_noise_delay: bool = False) -> None:
        """
        convert bool list to integer representing which channels
        should be active

        Properties
        -----------------

        i2c: the i2c bus being used
        addr: the i2c address of the ad7291 in your hardware

        active_channels: a bool list of length 8 representing which
        voltage input channels of the ad7291 should be read. Index
        in this list corresponds to channel number. Index 0 is CH0,
        index 1 is CH1 and so on
        """
        self.i2c_device = I2CDevice(i2c, addr)

        self.active_channels = active_channels
        """
        set the channel bits in the command register to correspond
        with the active_channels list
        """

        self.channels = self.channel_list_to_bits(active_channels)
        self.tsense = enable_temp_conversions
        self.noise = enable_noise_delay
        self.num_active_channels = number_of_active_channels

        self.settings = 0x00
        if self.tsense:
            print("tsense enabled")
            self.settings += 1 << 7
        if self.noise:
            self.settings += 1 << 5

        # initialize a buffer to interact with i2c
        self.buf = bytearray(2 * self.num_active_channels)

        with self.i2c_device as device:
            self.buf[0] = _COMMAND_REGISTER
            self.buf[1] = self.channels
            self.buf[2] = self.settings

            device.write(self.buf, end=3)

    def channel_list_to_bits(self, channel_list: list = [False] * 8) -> int:
        """
        takes the input list, should be a bool list, and converts it
        into an integer that will be passed into bits D15-D8 of
        command register
        """
        result = 0
        for i, channel in enumerate(channel_list):
            if channel:
                result += 1 << (7 - i)        # set ith channel bit to 1
        return result & ((i << 8) - 1)

    def get_channel(self,
                    channel: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8],
                    polarity: Literal["low", "high"]) -> None:
        """
        will get the data register address
        """

        """
        Circuit python does not support switch case statements, so I'm using a
        dictionary instead
        """
        dict = {
            "low": {
                0: _CH0_DATA_LOW,
                1: _CH1_DATA_LOW,
                2: _CH2_DATA_LOW,
                3: _CH3_DATA_LOW,
                4: _CH4_DATA_LOW,
                5: _CH5_DATA_LOW,
                6: _CH6_DATA_LOW,
                7: _CH7_DATA_LOW,
                8: _T_SENSE_DATA_LOW,
            },
            "high": {
                0: _CH0_DATA_HIGH,
                1: _CH1_DATA_HIGH,
                2: _CH2_DATA_HIGH,
                3: _CH3_DATA_HIGH,
                4: _CH4_DATA_HIGH,
                5: _CH5_DATA_HIGH,
                6: _CH6_DATA_HIGH,
                7: _CH7_DATA_HIGH,
                8: _T_SENSE_DATA_HIGH,
            },
        }
        self.buf[0] = dict[polarity][channel]

    def set_channel_upper_limit(self,
                                channel: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8],
                                value: int) -> None:
        """
        sets the DATA_HIGH register for the channel you specify.
        """
        if 0 <= channel <= 8:
            RuntimeError("invalid channel")
        if 0 <= value <= (1 << 12) - 1:
            RuntimeError("invalid limit")

        # get the channel we want to use
        self.get_channel_limit(channel, "high")

        # set the buffer to be the value
        self.buf[1] = (value >> 8) & ((1 << 8) - 1)
        self.buf[2] = value & ((1 << 8) - 1)

        with self.i2c_device as i2c:
            i2c.write(self.buf, end=3)

    def set_channel_lower_limit(self,
                                channel: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8],
                                value: int) -> None:
        """
        sets the DATA_LOW register for the channel you specify.
        """
        if 0 <= channel <= 8:
            RuntimeError("invalid channel")
        if 0 <= value <= (1 << 12) - 1:
            RuntimeError("invalid limit")

        # get desired channel
        self.get_channel_limit(channel, "low")

        # set the buffer to input value
        self.buf[1] = (value >> 8) & ((1 << 8) - 1)
        self.buf[2] = value & ((1 << 8) - 1)

        with self.i2c_device as i2c:
            i2c.write(self.buf, end=2)

    def get_channel_limits(self,
                           channel: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8]
                           ) -> tuple:
        """
        reads from the data high and low regoisters for a channel. Is returned
        in a tuple of (DATA_LOW, DATA_HIGH).
        """

        ch_high = self.get_channel(channel, "high")
        ch_low = self.get_channel(channel, "low")

        # read from the high channel
        self.buf[0] = ch_high
        with self.i2c_device as i2c:
            i2c.write(self.buf, end=1)

            i2c.readinto(self.buf, end=2)

        # convert reading to a value
        high = (self.buf[0] << 8) + self.buf[1]

        # read from the low channel
        self.buf[0] = ch_low
        with self.i2c_device as i2c:
            i2c.write(self.buf, end=1)

            i2c.readinto(self.buf, end=2)

        # convert reading to a value
        low = (self.buf[0] << 8) + self.buf[1]
        return (low, high)

    @property
    def read_from_voltage(self):
        """Initialize return list"""
        res = [None] * self.num_active_channels

        self.buf[0] = _VOLTAGE_CONVERSION

        with self.i2c_device as i2c:
            i2c.write(self.buf, end=1)

        """
        Voltages are returned sequentially, so we readinto using our
        entire buffer of (2 * num_active_channel) bytes
        """
        with self.i2c_device as i2c:
            i2c.readinto(self.buf)

        """
        For each 2 byte sequence, we first get bits 15-12 which represent the
        channel address, we then get bits 11-0 which represent the converted
        voltage read.
        """
        for i in range(0, 2 * self.num_active_channels, 2):
            channel = (self.buf[i] >> 4) & ((1 << 4) - 1)    # D[12:16]
            if not 0 <= channel <= 7:
                RuntimeError("Channel returned is not a voltage channel (0-7)")

            # the conversion from the voltage read
            voltage = (self.buf[i]) & ((1 << 4) - 1)         # D[8:12]
            voltage = voltage << 8                                     # shift to make room
            voltage += self.buf[i + 1]                       # D[0:8]
            res[int(i/2)] = (channel, voltage)

        return res

    @property
    def read_temperature_conversion(self):
        """
        Reads the latest temperature conversion off of the temperatuve
        conversion register. Temperature conversions are done every ~5ms
        so it is not gauranteed to be a new measurement if retrieved
        multiple times within a 5ms window.
        """
        if not self.settings >> 7:
            RuntimeError("Temperature sensor not enabled")

        self.buf[0] = _T_SENSE_CONVERSION_RESULT
        with self.i2c_device as i2c:
            i2c.write(self.buf, end=1)

            i2c.readinto(self.buf, end=2)

        channel = (self.buf[0] >> 4) & ((1 << 4) - 1)   # D[12:16]
        if (channel != 8):
            RuntimeError("Channel returned is not Temperature Channel (8)")

        temperature = (self.buf[0]) & ((1 << 4) - 1)    # D[8:12]
        temperature = temperature << 8                                # shifts
        temperature += self.buf[1]                      # D[0:8]

        if temperature > 4096:
            return (4096 - temperature)/4
        else:
            return temperature/4
