#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64

import time
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
import threading
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus.register_write_message import (WriteMultipleRegistersResponse,WriteSingleRegisterResponse)
from pymodbus.payload import BinaryPayloadBuilder, Endian, BinaryPayloadDecoder


client = ModbusClient(method='rtu', port='/dev/ttyUSB0', stopbits=1, parity='N', baudrate=115200, timeout=0.5)
connection = client.connect()



data14=client.write_register(0x7C, 0x9E, unit= 0x01)
data14=client.write_register(0x7C, 0x9E, unit= 0x02)

time.sleep(5)
data14=client.write_register(0x7C, 0x9F, unit= 0x01)
data14=client.write_register(0x7C, 0x9F, unit= 0x02)
# time.sleep(5)
# data14=client.write_register(0x7C, 0x9E, unit= 0x01)
# data14=client.write_register(0x7C, 0x9E, unit= 0x02)



# data14=client.write_register(0x7C, 0xD8, unit= 0x01)

# data11=client.write_registers(0x2E, 10, unit= 0x01)
# data12=client.write_register(0x2F, 10, unit= 0x01)
# data13=client.write_register(0x30,500, unit=0x01)

# data14=client.write_register(0x7C, 0x96, unit= 0x01)


# data24=client.write_register(0x7C, 0xD8, unit= 0x02)

# data21=client.write_registers(0x2E, 10, unit= 0x02)
# data22=client.write_register(0x2F, 10, unit= 0x02)
# data23=client.write_register(0x30,500, unit=0x02)

# data24=client.write_register(0x7C, 0x96, unit= 0x02)

# time.sleep(4)
# data13=client.write_register(0x30,0, unit=0x01)
# data23=client.write_register(0x30,0, unit=0x02)

# data14=client.write_register(0x7C, 0x9F, unit= 0x01)
# data14=client.write_register(0x7C, 0x9F, unit= 0x02)

# data14=client.write_register(0x7C, 0x96, unit= 0x01)
# data14=client.write_register(0x7C, 0x96, unit= 0x02)





