
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
import time
import threading
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
#from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus.register_write_message import (WriteMultipleRegistersResponse,WriteSingleRegisterResponse)
from pymodbus.payload import BinaryPayloadBuilder, Endian, BinaryPayloadDecoder

v11=0
v22=0

client = ModbusClient(method='rtu', port='/dev/ttyUSB0', stopbits=1, parity='N', baudrate=115200, timeout=0.05)
connection = client.connect()

data11=client.write_registers(0x2E, 50, unit= 0x01)
data12=client.write_register(0x2F, 50, unit= 0x01)
data13=client.write_register(0x30,0, unit=0x01)
data14=client.write_register(0x7C, 0x96, unit= 0x01)

data21=client.write_registers(0x2E, 50, unit= 0x02)
data22=client.write_register(0x2F, 50, unit= 0x02)
data23=client.write_register(0x30,0, unit=0x02)
data24=client.write_register(0x7C, 0x96, unit= 0x02)

def callback(data):
    dulieu = data.data
    a = dulieu.split(",")
    v1 = float(a[0])
    v1 = round(v1, 0)
    v2 = float(a[1])
    v2 = round(v2, 0)
    global v11
    global v22
    v11 = int(v1)
    v22 = int(v2)

def guidulieu():
    # Gui gia tri van toc den dong co 1
    builder1 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    builder1.reset()
    builder1.add_16bit_int(int(v11))
    payload1 = builder1.to_registers()
    data13 = client.write_register(0x30, payload1[0], unit=0x01)

    # Gui gia tri van toc den dong co 2
    builder2 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    builder2.reset()
    builder2.add_16bit_int(int(v22))
    payload2 = builder2.to_registers()
    data23 = client.write_register(0x30, payload2[0], unit=0x02)

    if v11 == 0 and v22 == 0:
        data13 = client.write_register(0x30, 0, unit=0x01)
        data23 = client.write_register(0x30, 0, unit=0x02)

    print("done", v11, v22)

def listenermodbus():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('modbus', String, callback)

def main():
    listenermodbus()
    while True:
        guidulieu()
        time.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
    	pass