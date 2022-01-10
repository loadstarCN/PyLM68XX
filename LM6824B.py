#!/usr/bin/env python
# -*- coding:utf-8 -*-

'''
2017@Olive Software 
LM6824B读卡器Python驱动程序

'''

import glob,platform,serial,struct,binascii,time,sys
import serial.tools.list_ports

#计算BCC异或校验最后一位 输入格式[0x00,0x01,0x02....]
def bcc(data):
	if len(data) >1:
		ret=data[0]
		for id in range(len(data)-1):
			ret^=data[id+1]
		return ret
	else:
		return None

#BCC异或校验算法 输入格式'0001020304...'
def check_bcc(data):
	if data[-1]==bcc(data[0:-1]):
		return True
	else:
		return False

# 整形列表转换为hex string
def int2hex_str(data):
	return binascii.b2a_hex(str(bytearray(data)))

class RFIDReader(serial.Serial):
	"""LM6824A读卡器Python驱动类"""

	def __init__(self, name, timeout=0.1):
		super(RFIDReader, self).__init__(name,timeout=timeout)
		self.ID=self.get_reader_id()

	def print_info(self):
		print('#'*40)
		print('序列号:{}'.format(binascii.hexlify(self.ID)))
		print('端口:{}'.format(self.port))
		print('波特率:{}'.format(self.baudrate))
		print('数据位:{}'.format(self.bytesize))
		print('校验位:{}'.format(self.parity))
		print('停止位:{}'.format(self.stopbits))
		print('读超时:{}'.format(self.timeout))
		print('写超时:{}'.format(self.writeTimeout))
		print('软件流控:{}'.format(self.xonxoff))
		print('硬件流控:{}'.format(self.rtscts))
		print('硬件流控:{}'.format(self.dsrdtr))
		print('字符间隔超时:{}'.format(self.interCharTimeout))
		print('#'*40)
	
	def check_return_bytes(self,data):

		print('校验返回数据:{}'.format(binascii.hexlify(data)))

		if len(data)<1:
			print("空数据!")
			return False

		if not (data[0] ==0x55 and data[1] ==0x55):
			print("数据起始位格式错误!")
			return False

		if not check_bcc([c for c in data]):
			print("数据校验位错误!")
			return False

		if not data[6]==0x00:
			print("命令操作返回结果代码为错误!")
			return False

		print('数据校验成功!')
		return True

	def send_msg(self,message):
		self.write(message)
		print('-'*40)
		print('发送命令:{}'.format(binascii.hexlify(message)))

	#读取读卡器芯片 EEPROM，安全地址范围为:0030-007F。访问其它地址可能引起系统异常
	def read_eeprom(self,address,length):

		_value=None

		_data=[0x55,0x55]			#命令头
		_data.extend([0x00])		#保留字
		_data.extend([0x00])		#地址字
		_data.extend([0x00,0x06])	#长度字 从命令字到最后一个字节
		_data.extend([0x01,0x07])	#命令字
		_data.extend([address//256,address%256])		#读卡器射频芯片内部EEPROM的字节地址:0030，大端
		_data.append(length)			#要读取数据的长度:01
		_data.append(bcc(_data))	#校验字
		
		msg=struct.pack(str(len(_data))+'B',*_data)
		try:
			self.send_msg(msg)
			#ret=self.read(13)
			ret=self.readall()
			if self.check_return_bytes(ret):
				_value=ret[7:-1]
				print('成功读取卡芯片EEPROM起始地址:{} 数据:{}'.format(hex(address),binascii.hexlify(_value)))
			self.flushInput()
		except Exception as ex:
			print(ex)

		return _value

	#写读卡芯片 EEPROM，安全地址范围为:0030-007F。访问其它地址可能引起系统异常
	#date=[1,2,3,4..] 或[0x01,0x02,0x03,0x04...]
	def write_eeprom(self,address,data):
		_value=False
		_data=[0x55,0x55]			#命令头
		_data.extend([0x00])		#保留字
		_data.extend([0x00])		#地址字
		_data.extend([(len(data)+6)//256,(len(data)+6)%256])	#长度字 从命令字到最后一个字节
		_data.extend([0x01,0x08])	#命令字
		_data.extend([address//256,address%256])	#读卡器射频芯片内部EEPROM的字节地址:0030，大端
		_data.append(len(data))		#要写入数据的长度:01
		_data.extend(data)		#写入的数据
		_data.append(bcc(_data))	#校验字

		try:
			msg=struct.pack(str(len(_data))+'B',*_data)
			self.send_msg(msg)
			#ret=ser.read(13)
			ret=self.readall()
			if self.check_return_bytes(ret):
				_value=True
				
				print('成功写读卡芯片EEPROM起始地址:{} 数据:{}'.format(hex(address), data))
	
			self.flushInput()
		except Exception as ex:
			print(ex)

		return _value

	#关闭蜂鸣器
	def close_sound(self):
		msg=struct.pack('9B',0x55,0x55,0x00,0x00,0x00,0x03,0x01,0x05,0x07)
		self.send_msg(msg)
		#ret=self.read(8)
		ret=self.readall()
		if self.check_return_bytes(ret):
			print('关闭蜂鸣器')
		self.flushInput()
	
	#打开蜂鸣器
	def open_sound(self):
		msg=struct.pack('9B',0x55,0x55,0x00,0x00,0x00,0x03,0x01,0x04,0x06)
		self.send_msg(msg)
		#ret=self.read(8)
		ret=self.readall()
		if self.check_return_bytes(ret):
			print('打开蜂鸣器')
		self.flushInput()

	#关闭自动寻卡 55 55 00 00 00 03 01 03 01
	def close_search(self):
		msg=struct.pack('9B',0x55,0x55,0x00,0x00,0x00,0x03,0x01,0x03,0x01)
		self.send_msg(msg)
		#ret=self.read(8)
		ret=self.readall()
		if self.check_return_bytes(ret):
			print('关闭自动寻卡')
		self.flushInput()

	#启动自动寻卡 55 55 00 00 00 03 01 01 03
	def open_search(self):
		msg=struct.pack('9B',0x55,0x55,0x00,0x00,0x00,0x03,0x01,0x01,0x03)
		self.send_msg(msg)
		#ret=self.read(8)
		ret=self.readall()
		if self.check_return_bytes(ret):
			print('启动自动寻卡')
		self.flushInput()

	#获取读卡器序列号 55 55 00 00 00 03 0A 01 08
	def get_reader_id(self):
		_value=None
		msg=struct.pack('9B',0x55,0x55,0x00,0x00,0x00,0x03,0x0A,0x01,0x08)
		self.send_msg(msg)
		#ret=self.read(12)
		ret=self.readall()
		if self.check_return_bytes(ret):
			_value=ret[7:-1]
			print('获取读卡器序列号:{}'.format(binascii.hexlify(_value)))
		else:
			print('没有读取到读卡器序列号')
		self.flushInput()
		return _value

	#14443A 复合寻卡
	def multiple_14443A_search_card(self):
		_value=None
		msg=struct.pack('9B',0x55,0x55,0x00,0x00,0x00,0x03,0x02,0x04,0x05)
		self.send_msg(msg)
		#ret=self.read(12)
		ret=self.readall()
		if self.check_return_bytes(ret):
			_value=ret[7:-1]
			print('14443A 复合寻卡:{}'.format(binascii.hexlify(_value)))
		else:
			print('没有读取到卡片')
		self.flushInput()
		return _value


	#M1 复合读块 相对地址模式
	def multiple_M1_read_card(self,section,block=0x01):
		_value=None
		_data=[0x55,0x55,0x00,0x00,0x00,0x0E,0x03,0x07,0x00,0x00,section,block,0x60,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
		_data.append(bcc(_data))
		msg=struct.pack(str(len(_data))+'B',*_data)
		self.send_msg(msg)
		#ret=self.read(20)
		ret=self.readall()
		if self.check_return_bytes(ret):
			_value=ret[7:-1]
			print('M1复合读块成功 扇区地址:{} 块地址:{} 数据:{}'.format(section,block,binascii.hexlify(_value)))
		self.flushInput()

		return _value
	
	#M1 复合写块 相对地址模式 
	#date=[1,2,3,4..] 或[0x01,0x02,0x03,0x04...]
	def multiple_M1_write_card(self,section,block,data):
		_value=False
		_data=[0x55,0x55,0x00,0x00,0x00,0x1E,0x03,0x07,0x01,0x00,section,block,0x60,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
		data.extend([0]*16)
		_data.extend(data)
		_data.append(bcc(_data))

		msg=struct.pack(str(len(_data))+'B',*_data)
		self.send_msg(msg)
		#ret=ser.read(20)
		ret=self.readall()

		if self.check_return_bytes(ret):
			_value=True
			print('M1复合写块成功,扇区地址:{} 块地址:{} 数据:{}'.format(section,block,data))

		self.flushInput()
		
		return _value


	@staticmethod 
	def get_list():
		reader_list=[]
		usb_list=[]
		print("当前操作系统:"+platform.system())

		if platform.system() =="Windows":
			usb_list = []
			com_list=list(serial.tools.list_ports.comports())
			for p in com_list:
				usb_list.append(p[0])

		elif platform.system()=='Darwin':
			usb_list=glob.glob(r'/dev/tty.usbmodem*')

		else:
			usb_list=glob.glob(r'/dev/ttyACM*')
		try:
			for usb in usb_list:
				ser=RFIDReader(usb,timeout=0.1)
				if ser.ID:
					reader_list.append(ser)

		except Exception as ex:
			print(ex)
		
		print('读卡器列表:{}'.format(reader_list))
		return reader_list


if __name__ == "__main__":

	ser_list = RFIDReader.get_list()
	if len(ser_list)>0:
		ser=ser_list[0]
		# ser.print_info()
		# ser.read_eeprom(0x30,16)
		ser.write_eeprom(0x30,[0x01]*16)
		# ser.read_eeprom(0x30,16)
	else:
		print("没有发现串口设备，程序退出")
		