# Copyright (c) [2025-2035] [Xinzhe Tang/JiangNan University]. All rights reserved.
# Licensed under the [GLP] (see LICENSE or <https://www.gnu.org/licenses/>).

import serial.tools.list_ports
import time

class HelmholtzDriver:
    def __init__(self, port='COM6', baud_rate=1000000):
        self.ser = None
        self.port = port
        self.baud_rate = baud_rate
    def SerialPortDetect(self):
        ports_list = list(serial.tools.list_ports.comports())
        match len(ports_list):
            case 0:
                print("无串口设备。")
                return False
            case 1:
                print(list(ports_list)[0][0], list(ports_list)[0][1])
                self.port = list(ports_list)[0][0]
                return True
            case _:
                print("可用的串口设备如下：")
                for comport in ports_list:
                    print(list(comport[0]), list(comport[1]))
                self.port = input("请选择串口：")
                return True
    def OpenSerialPort(self):
        self.ser = serial.Serial(self.port, self.baud_rate)
        if self.ser.is_open:
            print('串口打开成功')
        else:
            print('串口打开失败')
    def CloseSerialPort(self):
        self.ser.close()
        if self.ser.isOpen():
            print('串口未关闭')
        else:
            print('串口已关闭')
    def Write(self, message):
        self.ser.write(message)
    def Read(self, length):
        return self.ser.read(length)
    def Enable(self, id, enable):
         self.Write(CalTx(0x00, enable, id, 0x06))
         if 0 < id < 8:
              RX = self.Read(8)
              if Crc16_MODBUS(RX[:6]) == int.from_bytes(RX[6:], byteorder='little'):
                   if RX[:6] == bytes([id, 0x06, 0x00, 0x00, 0x00, enable]):
                        print('Enable')
                        return True
              return False
    def SetPWM(self, id, pwm):
         self.Write(CalTx(0x01, pwm, id, 0x06))
         if 0 < id < 8:
              RX = self.Read(8)
              if Crc16_MODBUS(RX[:6]) == int.from_bytes(RX[6:], byteorder='little'):
                   expected_response = bytes([id, 0x06, 0x00, 0x01, (pwm >> 8) & 0xFF, pwm & 0xFF])
                   if RX[:6] == expected_response:
                        print('SetPWM')
                        return True
              return False
    def AppPWM(self, id):
         self.Write(CalTx(0x02, 0x01, id, 0x06))
         if 0 < id < 8:
              RX = self.Read(8)
              if Crc16_MODBUS(RX[:6]) == int.from_bytes(RX[6:], byteorder='little'):
                   if RX[:6] == bytes([id, 0x06, 0x00, 0x02, 0x00, 0x01]):
                        print('AppPWM')
                        return True
              return False
    def ReadCurrent(self, id):
         if 0 < id < 8:
              self.Write(CalTx(0x05, 1, id, 0x03))
              RX = self.Read(7)
              if Crc16_MODBUS(RX[:5]) == int.from_bytes(RX[5:], byteorder='little'):
                   current_raw = (RX[3] << 8) | RX[4]
                   if current_raw >= 0x8000:
                        current = current_raw - 0x10000
                   else:
                        current = current_raw
                   return current
              return False
         else:
              print('WrongID')
              return False
    def ReadVoltage(self, id):
         if 0 < id < 8:
              self.Write(CalTx(0x06, 1, id, 0x03))
              RX = self.Read(7)
              if Crc16_MODBUS(RX[:5]) == int.from_bytes(RX[5:], byteorder='little'):
                   current_raw = (RX[3] << 8) | RX[4]
                   if current_raw >= 0x8000:
                        current = current_raw - 0x10000
                   else:
                        current = current_raw
                   return current*10
              return False
         else:
              print('WrongID')
              return False
def Crc16_MODBUS(datas):
    crc = 0xFFFF
    poly = 0xA001
    for byte in datas:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    return crc
def CalTx(register_address, value, id, function_code):
     modbus_frame = [
          id,
          function_code,
          (register_address >> 8) & 0xFF,
          register_address & 0xFF,
          (value >> 8) & 0xFF,
          value & 0xFF
     ]
     crc = Crc16_MODBUS(modbus_frame)
     modbus_frame.append(crc & 0xFF)
     modbus_frame.append((crc >> 8) & 0xFF)
     return bytes(modbus_frame)


driver = HelmholtzDriver()
if driver.SerialPortDetect():
    driver.OpenSerialPort()

    driver.Enable(0, 1)
    time.sleep(0.001)
    driver.SetPWM(1, 1000)
    driver.AppPWM(1)
    time.sleep(1)
    print('current', driver.ReadCurrent(1))
    time.sleep(0.001)
    print('voltage', driver.ReadVoltage(1))
    time.sleep(0.001)
    driver.Enable(0, 0)

    driver.CloseSerialPort()