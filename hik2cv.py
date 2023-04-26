# -- coding: utf-8 --

import sys
import numpy as np
import cv2
import time

sys.path.append("./MvImport")
from MvCameraControl_class import *

class hik2cv_class():
    def __init__(self):
        """初始化属性name和age"""
        self.deviceList = MV_CC_DEVICE_INFO_LIST()
        self.tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        self.image = 0
        self.image_fifo = 0
        self.image_read_log = True
        # ch: 枚举设备 | en:Enum device
        # nTLayerType[IN] 枚举传输层 ，pstDevList[OUT] 设备列表
        self.Enum_device(self.tlayerType, self.deviceList)

        # 获取相机和图像数据缓存区
        self.cam, self.data_buf, self.nPayloadSize = self.enable_device(0, self.deviceList, self.tlayerType)  # 选择第一个设备

    def Enum_device(self, tlayerType, deviceList):
        """
        ch:枚举设备 | en:Enum device
        nTLayerType [IN] 枚举传输层 ，pstDevList [OUT] 设备列表
        """
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            sys.exit()

        if deviceList.nDeviceNum == 0:
            print("find no device!")
            sys.exit()

        print("Find %d devices!" % deviceList.nDeviceNum)

        for i in range(0, deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
                print("\ngige device: [%d]" % i)
                # 输出设备名字
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                    strModeName = strModeName + chr(per)
                print("device model name: %s" % strModeName)
                # 输出设备ID
                nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
                nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
                nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
                nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
                print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
            # 输出USB接口的信息
            elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                print("\nu3v device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                    if per == 0:
                        break
                    strModeName = strModeName + chr(per)
                print("device model name: %s" % strModeName)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("user serial number: %s" % strSerialNumber)

    def enable_device(self, nConnectionNum, deviceList, tlayerType):
        """
        设备使能
        :param nConnectionNum: 设备编号
        :return: 相机, 图像缓存区, 图像数据大小
        """
        # ch:创建相机实例 | en:Creat Camera Object
        cam = MvCamera()

        # ch:选择设备并创建句柄 | en:Select device and create handle
        # cast(typ, val)，这个函数是为了检查val变量是typ类型的，但是这个cast函数不做检查，直接返回val
        stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            print("create handle fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:打开设备 | en:Open device
        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print("open device fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
            nPacketSize = cam.MV_CC_GetOptimalPacketSize()
            if int(nPacketSize) > 0:
                ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
                if ret != 0:
                    print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
            else:
                print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)

        # ch:设置触发模式为off | en:Set trigger mode as off
        ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            print("set trigger mode fail! ret[0x%x]" % ret)
            sys.exit()

        # 从这开始，获取图片数据
        # ch:获取数据包大小 | en:Get payload size
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
        # MV_CC_GetIntValue，获取Integer属性值，handle [IN] 设备句柄
        # strKey [IN] 属性键值，如获取宽度信息则为"Width"
        # pIntValue [IN][OUT] 返回给调用者有关相机属性结构体指针
        # 得到图片尺寸，这一句很关键
        # payloadsize，为流通道上的每个图像传输的最大字节数，相机的PayloadSize的典型值是(宽x高x像素大小)，此时图像没有附加任何额外信息
        ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
        if ret != 0:
            print("get payload size fail! ret[0x%x]" % ret)
            sys.exit()

        nPayloadSize = stParam.nCurValue

        # ch:开始取流 | en:Start grab image
        ret = cam.MV_CC_StartGrabbing()
        if ret != 0:
            print("start grabbing fail! ret[0x%x]" % ret)
            sys.exit()
        #  返回获取图像缓存区。
        data_buf = (c_ubyte * nPayloadSize)()
        #  date_buf前面的转化不用，不然报错，因为转了是浮点型
        return cam, data_buf, nPayloadSize

    def get_image(self,data_buf, nPayloadSize, cam):
        """
        获取图像
        :param data_buf:
        :param nPayloadSize:
        :return: 图像
        """
        # 输出帧的信息
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        # void *memset(void *s, int ch, size_t n);
        # 函数解释:将s中当前位置后面的n个字节 (typedef unsigned int size_t )用 ch 替换并返回 s
        # memset:作用是在一段内存块中填充某个给定的值，它是对较大的结构体或数组进行清零操作的一种最快方法
        # byref(n)返回的相当于C的指针右值&n，本身没有被分配空间
        # 此处相当于将帧信息全部清空了
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))

        # 采用超时机制获取一帧图片，SDK内部等待直到有数据时返回，成功返回0
        ret = self.cam.MV_CC_GetOneFrameTimeout(self.data_buf, self.nPayloadSize, stFrameInfo, 1000)
        if ret == 0:
            # print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
            #     stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
            self.image = np.asarray(self.data_buf)
            self.image = self.image_control(data=self.image, stFrameInfo=stFrameInfo)
            self.image_fifo = self.image
            self.image_read_log = True
        else:
            print("no data[0x%x]" % ret)

        # image = np.asarray(data_buf)  # 将c_ubyte_Array转化成ndarray得到（3686400，）
        # image = image.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, -1))  # 根据自己分辨率进行转化
        return self.image_fifo

    # 显示图像
    def image_show(image):

        image = cv2.resize(image, (600, 400), interpolation=cv2.INTER_AREA)
        cv2.imshow("image", image)
        k = cv2.waitKey(1) & 0xff

    # 需要显示的图像数据转换
    def image_control(self, data, stFrameInfo):
        if stFrameInfo.enPixelType == 17301505:
            image = data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
            # image_show(image=image)
        elif stFrameInfo.enPixelType == 17301514:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            image = cv2.cvtColor(data, cv2.COLOR_BAYER_GB2RGB)
            # image_show(image=image)
        elif stFrameInfo.enPixelType == 35127316:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            # image_show(image=image)
        elif stFrameInfo.enPixelType == 34603039:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            image = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_Y422)
            # image_show(image=image)
        return image

    def close_device(cam, data_buf):
        """
        关闭设备
        :param cam:
        :param data_buf:
        """
        # ch:停止取流 | en:Stop grab image
        ret = cam.MV_CC_StopGrabbing()
        if ret != 0:
            print("stop grabbing fail! ret[0x%x]" % ret)
            del data_buf
            sys.exit()

        # ch:关闭设备 | Close device
        ret = cam.MV_CC_CloseDevice()
        if ret != 0:
            print("close deivce fail! ret[0x%x]" % ret)
            del data_buf
            sys.exit()

        # ch:销毁句柄 | Destroy handle
        ret = cam.MV_CC_DestroyHandle()
        if ret != 0:
            print("destroy handle fail! ret[0x%x]" % ret)
            del data_buf
            sys.exit()

        del data_buf

    def get_img_thread(self):
        while True:
            self.image_fifo = self.get_image(self.data_buf, self.nPayloadSize, self.cam)
            # cv2.imshow('1', self.image)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     print('程序终止')
            #     break
            #time.sleep(0.01)
if __name__ == "__main__":
    cam=hik2cv_class()
    while 1:
        timestart=time.time()  # calculate fps
        image = cam.get_image(cam.data_buf, cam.nPayloadSize, cam.cam)
        cv2.imshow('1',cam.image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('程序终止')
            break
        timeend = time.time()  # calculate fps
        timewhile = timeend-timestart  # calculate fps
        fps=1/timewhile  # calculate fps
        print(fps)
        # loop_time = cv2.getTickCount() - loop_start
        # total_time = loop_time / (cv2.getTickFrequency())  # 使用getTickFrequency()更加准确
        # running_FPS = int(1 / total_time)  # 帧率取整
        # print("running_FPS:", running_FPS)
