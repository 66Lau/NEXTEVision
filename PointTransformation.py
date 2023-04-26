import DataParam
import math
import numpy as np
import cv2


class PointTrans:
    def __init__(self):
        self.DataParam = DataParam.DataParameter()  # 程序参数
        self.center = [0, 0]
        self.distance = 0
        self.yaw = 0
        self.pitch = 0

    def Pixel2Camera(self,center, cenpoint):
        if not center == '---not find---':
            x_distance = center[0] - cenpoint[0]
            y_distance = -(center[1] - cenpoint[1])
            print(x_distance,y_distance)
            self.distance = (x_distance, y_distance)
            # 像素坐标转换为转角，详见： https://blog.csdn.net/u010750137/article/details/97646798
            pts = np.float32([center[0], center[1]])
            # K = np.zeros((3, 3))  # 内参
            # K[0, 0] = 1572.4
            # K[1, 1] = 1572.4
            # K[0, 2] = 655.0
            # K[1, 2] = 503.4
            # K[2, 2] = 1
            # distCoeffs = np.float32([-0.313818281448022, 0.106042482565769, 0, 0])  # 畸变
            undistort_pts = cv2.undistortPoints(pts, self.DataParam.CamParaMatrix, self.DataParam.CamDistCoeffs,
                                                P=self.DataParam.CamParaMatrix)  # 矫正后的装甲板中心坐标

            fx = self.DataParam.CamParaMatrix[0, 0]
            fy = self.DataParam.CamParaMatrix[1, 1]
            cx = self.DataParam.CamParaMatrix[0, 2]
            cy = self.DataParam.CamParaMatrix[1, 2]

            self.yaw = math.atan((undistort_pts[0, 0, 0] - cx) / fx) * 360 / math.pi / 2
            self.pitch = math.atan((cy - undistort_pts[0, 0, 1]) / fy) * 360 / math.pi / 2

        if center == '---not find---':
            self.yaw = 0
            self.pitch = 0