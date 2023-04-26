# -*- coding: UTF-8 -*-
# 开发作者   ：lau
# 开发时间   ：2022/6/23
# 文件名称   ：
# 开发工具   ：
# 文件说明   ：此文件为整个视觉文件的主程序
# =============导入外部库==========================
import struct
import numpy as np
import math
import cv2
import threading
# =============导入内部库==========================
import hik2cv  # 非海康相机请注释
import usb2cv
import DataConnect
import globalVar
import TrackKF_2D
import get_pose
from globalVar import *
globalVar._init()
import DataParam
import ArmorDetector
import PointTransformation

# =============对象初始化===========================
DataParamClass = DataParam.DataParameter()  # 程序参数
if DataParamClass.camera == "hikvision":
    cam = hik2cv.hik2cv_class()  # 海康相机初始化 如果不是海康相机要注释掉
    pass
elif DataParamClass.camera == "usb":
    cam = usb2cv.usb2cv_class(DataParamClass.CamNum)  # usb相机初始化
    pass
data_ser = DataConnect.SerialClass()  # 串口数据初始化
kalman = TrackKF_2D.KalmanFilter()  # 卡尔曼滤波器初始化
pose = get_pose.get_pose()  # 读取t265里程计数据初始化
ArmorDetector = ArmorDetector.ArmorDetector()
PointTrans = PointTransformation.PointTrans()

# =============线程定义==========================
thread1 = threading.Thread(target=cam.get_img_thread, daemon=True)
thread1.start()
thread1.join(1)

# thread2 = threading.Thread(target=pose.get_pose(), daemon=True)
# thread2.start()
# thread2.join(1)

# =============开始循环===========================
while 1:
    # 串口读取
    data_ser.read()

    # 防止一图多次读取
    if cam.image_read_log == True:
        img = cam.image_fifo
        cam.image_read_log = False
    else:
        continue

    # 视觉识别
    if data_ser.shootpattern == 'armor':
        ArmorDetector.ArmorDetectTask(img, data_ser.enemycolor)
    if data_ser.shootpattern == 'rune':
        # 此处continue是因为符文还没写，写完后删除continue
        continue

    # 坐标系转换
    PointTrans.Pixel2Camera(ArmorDetector.center_result, [ArmorDetector.center_x,ArmorDetector.center_y])

    # 串口发送
    data_ser.write(PointTrans.yaw, PointTrans.pitch, ArmorDetector.IfFire)

    # 发送导航绝对位置信息
    # data_ser.write(pose.pose_x, pose.pose_y)

    # 画图
    cv2.putText(ArmorDetector.InfoPlate, "yaw : " + str(PointTrans.yaw), (10, 220),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                [255, 255, 255])
    cv2.putText(ArmorDetector.InfoPlate, "pitch : " + str(PointTrans.pitch), (10, 260),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                [255, 255, 255])
    ArmorDetector.img = cv2.resize((ArmorDetector.img[:, :, 0]+ArmorDetector.img[:, :, 1]+ArmorDetector.img[:, :, 2])/3, (640, 480))
    ArmorDetector.dst = cv2.resize(ArmorDetector.dst, (640, 480))
    ArmorDetector.InfoPlate =cv2.resize(ArmorDetector.InfoPlate, (640, 480))
    merged_frame = np.hstack((ArmorDetector.img, ArmorDetector.dst,ArmorDetector.InfoPlate))
    cv2.imshow('Merged Frames', merged_frame)
    cv2.imshow("img", img)


    # =============终止条件：键盘输入q键======================
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print('程序终止')
        break



