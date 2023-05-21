import numpy as np


class DataParameter():
    def __init__(self):

# -------------相机参数-------------------------------
        # 使用相机型号：海康/支持opencv直接调用的usb相机
        self.camera = "hikvision"  # hikvision/usb/
        self.CamNum = 1
        # 相机内参矩阵
        self.CamParaMatrix = np.zeros((3, 3))
        self.CamParaMatrix[0, 0] = 1869.074197
        self.CamParaMatrix[0, 2] = 664.6998242
        self.CamParaMatrix[1, 1] = 1867.898354
        self.CamParaMatrix[1, 2] = 518.0525069
        self.CamParaMatrix[2, 2] = 1


        # self.CamParaMatrix[0, 0] = 1572.4
        # self.CamParaMatrix[1, 1] = 1572.4
        # self.CamParaMatrix[0, 2] = 655.0
        # self.CamParaMatrix[1, 2] = 503.4
        # self.CamParaMatrix[2, 2] = 1
        # 相机畸变参数
        self.CamDistCoeffs = np.float32([-0.163116073, 0.255155351, 0, 0])
        # 是否加入卡尔曼预测
        self.IfKalman = True
        # 卡尔曼预测系数（KP越大，预测越远）
        self.kalmanKP = 0.5
        # 哨兵像素距离开火阈值
        self.FireDistance = 200
# -------------图像处理---------------------
        # 二值化阈值，取0-255之间
        self.ThresholdValue = 210
        # 膨胀矩阵大小
        self.OpenRect = (1, 1)
        # 腐蚀矩阵大小
        self.CloseRect = (2, 2)
# ------------装甲板识别函数------------------

        # --------灯条筛选------------------
        # 长边/短边 > LgRatio
        self.LgRatio = 2
        # 灯条像素面积 > LgArea
        self.LgArea = 4
        # 灯条倾斜角度 > LgAngle
        self.LgAngle = 45

        # -----灯条匹配筛选---------------
        # （两个灯条的高度差） <= highRshort *（灯条短边）
        self.highRshort = 3
        # 灯条长边 * XRlongMinB<= 两个灯条的间距 <= 灯条长边 * XRlongMaxB 大装甲
        self.XRlongMaxB = 6.5
        self.XRlongMinB = 3.4
        # 灯条长边 * XRlongMinS<= 两个灯条的间距 <= 灯条长边 * XRlongMaxS 小装甲
        self.XRlongMaxS = 3
        self.XRlongMinS = 1
        # 两个灯条的短边之差 <= DiffRMax * 两个灯条的短边最大值
        self.DiffRMaxS = 4
        # 两个灯条的长边之差) <= DiffRMaxL *（w1, w2的最大值）两个灯条的长边最大值
        self.DiffRMaxL = 0.5
        # 两个灯条的角度差  <= 6
        self.DiffAngle = 13
        # 装甲板长宽比<HeightRWidth
        self.HeightRWidth = 3

        # ------枪口抬升-----------------
        # 枪口抬升像素值与距离比值
        self.Distance_K = 115
        # 枪口抬升像素值限幅
        self.RiseValue = 400
# ------------装甲板识别函数------------------
# ------------由于串口的特殊性，请前往DataConnect.py修改------
