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
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_bit import ROBit, RWBit

try:
    import typing
    from typing_extensions import Literal
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

    def __init__(self, i2c: I2C, addr: int = _DEFAULT_ADDRESS) -> None:
        self.i2c_device = I2CDevice(i2c, addr)
