# -*- coding: UTF-8 -*-
# 开发作者   ：lau
# 开发时间   ：2022/6/21
# 文件名称   ：
# 开发工具   ：
# 文件说明   ：此文件用于封装串口函数
import time

# 导入外部库
import serial
# 导入内部库

class SerialClass:
    def __init__(self):
        """初始化属性"""
        # 端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等  妙算=> portx = "/dev/ttyTHS2"  工控机

        self.portx = "/dev/ttyUSB1"
        #需要给予权限sudo chmod 777 /dev/ttyACM0
        # 波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
        self.bps = 115200
        # 超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
        self.timex = 0.01
        try:
            self.ser = serial.Serial(self.portx, self.bps, timeout=self.timex)
        except:
            pass
            try :
                self.ser = serial.Serial("/dev/ttyUSB0", self.bps, timeout=self.timex)
            except:
                pass
            pass
        self.len = 5
        self.mycolor='blue'    # "blue/"red"
        if self.mycolor == 'blue':
            self.enemycolor='red'
        elif self.mycolor == 'red':
            self.enemycolor = 'blue'
        self.shootSpeed = 15
        self.shootpattern='armor'  # 'armor'/'rune'
        self.getdata = []
        self.getdata_fifo = []
        #self.read_log=1

    def read(self):
        try:
            data = self.ser.read()  # 读取串口数据
            data = hex(ord(data))

            if data == "0xaa":
                while True:
                    if data == "0xbb":
                        self.getdata.append(data)
                        self.getdata_fifo = self.getdata
                        self.getdata = []
                        print(self.getdata_fifo)
                        # print(type(self.getdata_fifo[2]),self.getdata_fifo[2])
                        #self.read_log = self.read_log+1
                        break
                    self.getdata.append(data)
                    data = self.ser.read()  # 读取串口数据
                    data = hex(ord(data))
                pass
            if self.getdata_fifo[2] == "0x1":
                self.mycolor == "red"
                self.enemycolor = "blue"
            if self.getdata_fifo[2] == "0x0":
                self.mycolor == "blue"
                self.enemycolor = "red"
        except:
            pass


    def serial_read_thread(self):
        while True:
            self.read()
            # time.sleep(0.01)

    def write(self, yaw, pitch, fire):
        try:
            # ========数据格式处理=========
            yaw = int(100*(yaw + 90))
            pitch = int(100*(pitch + 90))
            if yaw < 10000:
                str_yaw = "0"+str(yaw)
            else:
                str_yaw = str(yaw)
            if pitch < 10000:
                str_pitch = "0" + str(pitch)
            else:
                str_pitch = str(pitch)

            # ========数据格式处理=========
            # 开火是“F” ，不开火是“N”
            send_datas = 'A'+str_yaw+str_pitch + fire
            self.ser.write(str(send_datas).encode('utf-8'))
            print("已发送数据:", send_datas)
        except Exception as exc:
            print("发送异常", exc)

    def write_position(self, x, y):
        try:
            # ========数据格式处理=========
            # 默认赛场上的x，y（单位m）不超过正负30m，这里取2位整数，两位小数
            x=x+30 #大于30是正数，小于30是负数
            y=y+30
            x=round(x,2) #保留两位小数
            y=round(y,2)
            str_x = str(int(y * 100))
            str_y = str(int(y * 100))
            # ========数据格式处理=========
            send_datas = 'B'+ str_x+str_y
            self.ser.write(str(send_datas).encode('utf-8'))
            print("已发送数据:", send_datas)
        except Exception as exc:
            print("发送异常", exc)