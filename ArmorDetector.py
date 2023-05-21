# -*- coding: UTF-8 -*-
# 开发作者   ：lau
# 开发时间   ：2022/6/21  18:11
# 文件名称   ：vision_main.PY
# 开发工具   ：vsc
# 文件说明   ：此文件用于封装摄像头处理函数
import time

# 导入外部库
import numpy as np
import math
import cv2
# 导入内部库

import DataParam
import TrackKF_2D

class ArmorDetector:
    def __init__(self):
        self.img = None
        self.binary = None
        self.dst = None
        self.center = None
        self.IfFire = "N"
        self.center_y = int(0)
        self.center_x = int(0)
        self.center_result = [0,0]
        self.DataParam = DataParam.DataParameter()  # 程序参数
        self.kalman = TrackKF_2D.KalmanFilter()  # 卡尔曼滤波器初始化
        self.distance = 0
        self.InfoPlate = np.zeros((640, 480))



    def read_morphology(self,binary):
        """
        对图片进行形态学处理
        输入：二值化图片
        输出：进行形态学处理后的二值化图片
        """
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.DataParam.OpenRect)  # 去除多余噪声
        dst = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 9))  # 提取垂直直线,去除横向噪声
        # dst = cv2.morphologyEx(dst, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.DataParam.CloseRect)  # 填充物体中的小洞
        dst = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # dst = cv2.erode(dst, kernel)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # dst = cv2.dilate(dst, kernel)

        return dst


    def img_process(self,img, colour):
        """
         对图片进行通道过滤，二值化处理，形态学处理
         输入：原图像，敌方队伍颜色
         输出：处理后的二值化图像
         """
        blue, green, red = cv2.split(img)  # 分离通道，在opencv中图片的存储通道为BGR非RBG
        if colour == 'blue':
            input_colour = blue
        elif colour == 'red':
            input_colour = red
        ret2, binary = cv2.threshold(input_colour, self.DataParam.ThresholdValue, 255, 0)  # 二值化
        dst = self.read_morphology(binary)
        self.center_y = img.shape[0]/2
        self.center_x = img.shape[1]/2

        return binary, dst


    def find_contours(self,binary, frame):  # find contours and main screening section
        '''寻找装甲板的轮廓

        寻找距离和角度合适的灯条，然后寻找装甲板，对装甲板中心进行标定，并在原图中绘出。

        参数：binary：形态学处理过后的图片 frame：原始图像
        返回值： None
        '''

        contours, heriachy = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 首先找到处理后图像的所有轮廓
        length = len(contours)
        data_list = []
        first_data = []
        second_data1 = []
        second_data2 = []
        self.InfoPlate = np.zeros((640, 480))
        c = 0
        d = 0
        if length > 0:
            print("---founding---")
            for i, contour in enumerate(contours):
                data_dict = dict()
                # print("countour", contour)
                area = cv2.contourArea(contour)
                rect = cv2.minAreaRect(contour)  # 将轮廓转化为矩形，获取特征
                rx, ry = rect[0]  # 中心坐标
                # cv2.circle(frame, (int(rx),int(ry)), 2, (0, 0, 255), -1)
                rw = rect[1][0]
                rh = rect[1][1]  # 宽高
                if rw == 0:
                    rw = 1
                if rh == 0:
                    rh = 1
                z = rect[2]

                coor = cv2.boxPoints(rect)
                x1 = coor[0][0]
                y1 = coor[0][1]
                x2 = coor[1][0]
                y2 = coor[1][1]
                x3 = coor[2][0]
                y3 = coor[2][1]
                x4 = coor[3][0]
                y4 = coor[3][1]  # 矩形4个点的坐标

                # if i >= 1:
                data_dict["area"] = area
                data_dict["rx"] = rx
                data_dict["ry"] = ry
                data_dict["rh"] = rh
                data_dict["rw"] = rw
                data_dict["z"] = z
                data_dict["x1"] = x1
                data_dict["y1"] = y1
                data_dict["x2"] = x2
                data_dict["y2"] = y2
                data_dict["x3"] = x3
                data_dict["y3"] = y3
                data_dict["x4"] = x4
                data_dict["y4"] = y4  # 用字典存入矩形的各个信息
                data_dict["coor"] = coor
                data_list.append(data_dict)
            # =============== 装甲板灯条筛选==================================================
            for i in range(len(data_list)):

                data_rh = data_list[i].get("rh", 0)
                data_rw = data_list[i].get("rw", 0)
                data_area = data_list[i].get("area", 0)
                data_angle = data_list[i].get("z", 0)
                # print(data_rh,data_rw,data_area,data_angle)
                # time.sleep(3)

                light_conditions1 = (float(data_rw / data_rh) >= self.DataParam.LgRatio) \
                                    and (data_area >= self.DataParam.LgArea) \
                                    and (abs(abs(data_angle) - 90) < self.DataParam.LgAngle)
                light_conditions2 = (float(data_rh / data_rw) >= self.DataParam.LgRatio) \
                                    and (data_area >= self.DataParam.LgArea) \
                                    and (abs(abs(data_angle) - 0) < self.DataParam.LgAngle)
                """
                灯条筛选条件：
                1.长边/短边（实测大概在7.5左右）
                2.灯条面积，一般用于筛选过远装甲板
                3.灯条与垂直方向的角度               
                """

                if light_conditions1:
                    first_data.append(data_list[i])
                    cv2.line(frame, (int(data_list[i].get("x1", 0)), int(data_list[i].get("y1", 0))),
                             (int(data_list[i].get("x3", 0)), int(data_list[i].get("y3", 0))), (0, 255, 0), 2, 1)
                elif light_conditions2:  # 通过矩阵的形状大小角度等进行判断，删除不合理的矩阵
                    data_list[i].update(rh=data_rw)
                    data_list[i].update(rw=data_rh)
                    first_data.append(data_list[i])
                    cv2.line(frame, (int(data_list[i].get("x1", 0)), int(data_list[i].get("y1", 0))),
                             (int(data_list[i].get("x3", 0)), int(data_list[i].get("y3", 0))), (0, 255, 0), 2, 1)

                else:
                    # print("装甲板灯条检测不通过")
                    pass
            # =============== 装甲板灯条筛选==================================================
            # print("装甲板数量"+str(len(first_data)))

            for i in range(len(first_data)):
                # cv.circle(frame, (int(first_data[i].get("rx")), int(first_data[i].get("ry"))), 5, (0, 0, 255), -1)
                # cv.circle(frame, (int(first_data[i]["x1"]), int(first_data[i]["y1"])), 2, (0, 0, 255), -1)
                # cv.circle(frame, (int(first_data[i]["x2"]), int(first_data[i]["y2"])), 2, (0, 0, 255), -1)
                # cv.circle(frame, (int(first_data[i]["x3"]), int(first_data[i]["y3"])), 2, (0, 0, 255), -1)
                # cv.circle(frame, (int(first_data[i]["x4"]), int(first_data[i]["y4"])), 2, (0, 0, 255), -1)
                c = i + 1
                while c < len(first_data):
                    # 判断两个矩形是否平行，如果平行则说明很可能是装甲板两边的灯条
                    data_ryi = float(first_data[i].get("ry", 0))
                    data_ryc = float(first_data[c].get("ry", 0))
                    data_rhi = float(first_data[i].get("rh", 0))
                    data_rhc = float(first_data[c].get("rh", 0))
                    data_rxi = float(first_data[i].get("rx", 0))
                    data_rxc = float(first_data[c].get("rx", 0))
                    data_rwi = float(first_data[i].get("rw", 0))  # i装甲板-长边
                    data_rwc = float(first_data[c].get("rw", 0))  # c装甲板-长边
                    data_rai = float(first_data[i].get("z", 0))  # angle-i
                    data_rac = float(first_data[c].get("z", 0))  # angle-c
                    # print("------")

                    # print(data_rai)
                    # rint("------")
                    print("装甲板间距:" + str(abs(data_rxi - data_rxc)))
                    print("灯条长度:" + str((data_rwi + data_rwc) / 2))

                    if (abs(data_ryi - data_ryc) <= self.DataParam.highRshort * ((data_rhi + data_rhc) / 2)) \
                            and ((abs(data_rxi - data_rxc) <= self.DataParam.XRlongMaxB * ((data_rwi + data_rwc) / 2)) and (abs(data_rxi - data_rxc) >= self.DataParam.XRlongMinB * ((data_rwi + data_rwc) / 2))
                                 or (abs(data_rxi - data_rxc) <= self.DataParam.XRlongMaxS * ((data_rwi + data_rwc) / 2)) and (abs(data_rxi - data_rxc) >= self.DataParam.XRlongMinS * ((data_rwi + data_rwc) / 2))) \
                            and (abs(data_rhi - data_rhc) <= self.DataParam.DiffRMaxS * max(data_rhi, data_rhc)) \
                            and (abs(data_rwi - data_rwc) <= self.DataParam.DiffRMaxL * max(data_rwi, data_rwc)) \
                            and ((abs(data_rai - data_rac) <= self.DataParam.DiffAngle) or (abs((abs(data_rai - data_rac) - 90)) <= self.DataParam.DiffAngle)):
                        '''
                        y1-y2 （两个灯条的高度差）   <=  h1，h2均值（灯条短边）
                        x1-x2 （两个灯条的间距）     <=  w1，w2均值（灯条长边） big/small amor experience:big=38/169
                        x1-x2 （两个灯条的间距）     <=  w1，w2均值（灯条长边） big/small amor
                        h1-h2  (两个灯条的短边之差)  <=  4（h1,h2的最大值）两个灯条的短边最大值
                        w1-w2  (两个灯条的长边之差)  <=  0.5（w1,w2的最大值）两个灯条的长边最大值
                        ra1-ra2 (两个灯条的角度差) <=6 or(两个灯条距离90度的角度差--主要为了防止0度和90度跳变)
                        '''


                        second_data1.append(first_data[i])
                        second_data2.append(first_data[c])  # 将平行的矩形成对存入
                        # print("装甲板矩形测试通过")
                    c = c + 1

        for i in range(len(second_data2)):
            cv2.circle(frame, (int(second_data2[i]["rx"]), int(second_data2[i]["ry"])), 5, (0, 0, 255), -1)
            cv2.circle(frame, (int(second_data2[i]["x1"]), int(second_data2[i]["y1"])), 2, (0, 0, 255), -1)
            cv2.circle(frame, (int(second_data2[i]["x2"]), int(second_data2[i]["y2"])), 2, (0, 0, 255), -1)
            cv2.circle(frame, (int(second_data2[i]["x3"]), int(second_data2[i]["y3"])), 2, (0, 0, 255), -1)
            cv2.circle(frame, (int(second_data2[i]["x4"]), int(second_data2[i]["y4"])), 2, (0, 0, 255), -1)

        dataList_c = []
        dataList_d = []  # 距离
        if len(second_data1):  # second_data1中储存了所有可能的的装甲板
            dataRange = []
            dataList_c.clear()  # 用于储存筛选后的装甲板位置中心
            for i in range(len(second_data1)):

                rectangle_x1 = int(second_data1[i]["x1"])
                rectangle_y1 = int(second_data1[i]["y1"])
                rectangle_x2 = int(second_data2[i]["x3"])
                rectangle_y2 = int(second_data2[i]["y3"])

                if abs(rectangle_y1 - rectangle_y2) <= self.DataParam.HeightRWidth * (abs(rectangle_x1 - rectangle_x2)):
                    # 判断所认为的装甲板高宽比
                    # TODO: 可能需要删掉
                    # global point1_1x, point1_1y, point1_2x, point1_2y, point1_3x, point1_3y, point1_4x, point1_4y
                    # global point2_1x, point2_1y, point2_2x, point2_2y, point2_3x, point2_3y, point2_4x, point2_4y

                    point1_1x = second_data1[i]["x1"]
                    point1_1y = second_data1[i]["y1"]
                    point1_2x = second_data1[i]["x2"]
                    point1_2y = second_data1[i]["y2"]
                    point1_3x = second_data1[i]["x3"]
                    point1_3y = second_data1[i]["y3"]
                    point1_4x = second_data1[i]["x4"]
                    point1_4y = second_data1[i]["y4"]

                    point2_1x = second_data2[i]["x1"]
                    point2_1y = second_data2[i]["y1"]
                    point2_2x = second_data2[i]["x2"]
                    point2_2y = second_data2[i]["y2"]
                    point2_3x = second_data2[i]["x3"]
                    point2_3y = second_data2[i]["y3"]
                    point2_4x = second_data2[i]["x4"]
                    point2_4y = second_data2[i]["y4"]
                    midpoint1, midpoint2 = self.Rectangle2Light((point1_1x,point1_1y), (point1_2x,point1_2y), (point1_3x,point1_3y), (point1_4x,point1_4y))
                    midpoint3, midpoint4 = self.Rectangle2Light((point2_1x,point2_1y), (point2_2x,point2_2y), (point2_3x,point2_3y), (point2_4x,point2_4y))
                    print(midpoint1, midpoint2)
                    cv2.line(frame, midpoint1, midpoint2, (0, 225, 100), 2, 1)
                    cv2.line(frame, midpoint3, midpoint4, (0, 225, 100), 2, 1)


                    self.plot_armor(frame, second_data1[i]["coor"], second_data2[i]["coor"])
                    cv2.putText(frame, "target1:", (rectangle_x2, rectangle_y2 - 5), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, [255, 255, 255])

                    # center = (int((point2_2x + point1_4x) / 2),
                    #           int((point2_2y + point1_4y) / 2))  # 在装甲板中心标点。
                    center = (int((
                                              point1_1x + point1_2x + point1_3x + point1_4x + point2_1x + point2_2x + point2_3x + point2_4x) / 8),
                              int((
                                              point1_1y + point1_2y + point1_3y + point1_4y + point2_1y + point2_2y + point2_3y + point2_4y) / 8))

                    # print("中心像素坐标"+str(center))
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)  # 画出重心
                    cv2.circle(frame, (int(self.center_x), int(self.center_y)), 3, (0, 0, 255), -1)  # 画出图像中心

                    dataList_c.append(center)
                    """
                    这里要加灯条宽度======================这里距离不准确=========================
                    """
                    print("面积", (second_data2[i]["rh"]*second_data2[i]["rw"]+second_data1[i]["rh"]*second_data1[i]["rw"]))  # 取当前装甲板的灯条的面积作为判别距离的标准
                    # distance = (1 / (second_data2[i]["rh"]*second_data2[i]["rw"]+second_data1[i]["rh"]*second_data1[i]["rw"])) * 20000 / 80
                    s=(second_data2[i]["rh"]*second_data2[i]["rw"]+second_data1[i]["rh"]*second_data1[i]["rw"])
                    distance = 18.73*(s**(-0.3572))
                    print("距离", distance)
                    self.distance = round(distance, 2)
                    distance_rise = self.distance * self.DataParam.Distance_K
                    print(distance_rise)
                    if distance_rise >= self.DataParam.RiseValue:
                        distance_rise = self.DataParam.RiseValue
                    cv2.putText(frame, 'distance is' + str(distance) + " m",
                                (int((point2_2x + point1_4x) / 2), int((point2_2y + point1_4y) / 2) + 70),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1.5, [255, 255, 255])
                    # -============================= 这里距离不准确 ================================
                    dataList_d.append(distance_rise)
                    high = (rectangle_y1, rectangle_y2)
                    dataRange.append(high)


                else:  # 未通过装甲板检验的装甲板
                    center = '---not find---'
                    print(center)
                    cv2.circle(frame, (self.center_x, self.center_y), 3, (0, 255, 0), -1)  # 画出图像中心
            if not (dataList_c == []):  # 最优装甲板选取与画图
                center_list = []
                center_list = dataList_c
                best_point, best_point_rise = self.find_best_armor(center_list, dataList_d)
                center = best_point
                cv2.circle(frame, center, 20, (244, 244, 120), -1)  # 未枪口抬升的装甲板中心
                cv2.putText(frame, 'best armor point is' + str(best_point), best_point, cv2.FONT_HERSHEY_SIMPLEX,
                            1.5, [255, 255, 255])
                # center = (center[0],int(center[1]-5)) # no rise
                center = (center[0], int(center[1] - best_point_rise))  # bullet rise
                cv2.putText(self.InfoPlate, str(len(dataList_c))+" armors was found", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            [255, 255, 255])
                cv2.putText(self.InfoPlate, "distance : " + str(self.distance), (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            [255, 255, 255])
                cv2.putText(self.InfoPlate, "position of armor : " + str(self.center), (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            [255, 255, 255])
                cv2.putText(self.InfoPlate, "Final position : " + str(self.center_result), (10, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            [255, 255, 255])



        else:  # 图像中未发现可能装甲板的情况
            center = '---not find---'
            print(center)
            cv2.circle(frame, (int(self.center_x), int(self.center_y)), 3, (0, 255, 0), -1)  # 画出图像中心
            cv2.putText(self.InfoPlate, "No armor was found", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [255, 255, 255])
            dataList_c.clear()

        data_list.clear()
        return center



    def find_best_armor(self,center_list, rise_diatance):
        distance_list = []
        for ii in range(len(center_list)):
            distance_x = abs(center_list[ii][0] - self.center_x)
            distance_y = abs(center_list[ii][1] - self.center_y)
            distance_list.append(distance_x + distance_y)
        best_ii = distance_list.index(min(distance_list))
        best_point = center_list[best_ii]
        best_point_rise = rise_diatance[best_ii]
        return best_point, best_point_rise

    def plot_armor(self,frame, coor0, coor1):
        point = [coor0[0], coor0[1], coor0[2], coor0[3], coor1[0], coor1[1], coor1[2], coor1[3]]
        for point_i in point:
            for point_j in point:
                cv2.line(frame, (int(point_i[0]),int(point_i[1])), (int(point_j[0]),int(point_j[1])), (0, 255, 0), 1, 1)


    def ArmorDetectTask(self, img, enemycolor):
        self.img = img
        self.binary, self.dst = self.img_process(self.img, enemycolor)
        self.center = self.find_contours(self.dst, self.img)  # 寻找装甲板 返回最优的中心装甲板坐标
        if not self.center == '---not find---':
            delta_x, delta_y = self.kalman.track(self.center[0], self.center[1])
            # 画出图像中心画出卡尔曼预测后的装甲板中心
            cv2.circle(self.img, (int((delta_x) * self.DataParam.kalmanKP + self.center[0]),
                                  int((delta_y) * self.DataParam.kalmanKP + self.center[1])),
                                  20, (120, 120, 0),-1)
            if self.DataParam.IfKalman:
                self.center_result[0] = int(delta_x)*self.DataParam.kalmanKP + self.center[0]
                self.center_result[1] = int(delta_y)*self.DataParam.kalmanKP + self.center[1]
            else:
                self.center_result[0] = self.center[0]
                self.center_result[1] = self.center[1]

            x_distance = self.center[0] - self.center_x
            y_distance = -(self.center[1] - self.center_y)
            self.distance = (x_distance, y_distance)
            if math.sqrt(self.distance[0]**2 + self.distance[1]**2) <= self.DataParam.FireDistance:
                self.IfFire = "F"
                print("开火F")
                cv2.putText(self.InfoPlate, "Fire or not : " + str(self.IfFire), (10, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            [255, 255, 255])

            else:
                self.IfFire = "N"
                print("熄火N")

        if self.center == '---not find---':
            self.center_result[0] = self.center_x
            self.center_result[1] = self.center_y
            self.IfFire = "N"
            print("熄火N")


    def Rectangle2Light(self,Recpoint1,Recpoint2,Recpoint3,Recpoint4):
        """
        此函数用来将灯条的四个点转换成长边的两点
        输入：矩形的四个点的坐标（x，y）
        输出：穿过矩形短边的长边两点
        """

        def calculate_midpoint(point1, point2):
            # 计算两点之间的中点
            x1, y1 = point1
            x2, y2 = point2
            midpoint = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            return midpoint

        # 计算边的长度
        edge1_length = self.calculate_distance(Recpoint1, Recpoint2)
        edge2_length = self.calculate_distance(Recpoint2, Recpoint3)
        edge3_length = self.calculate_distance(Recpoint3, Recpoint4)
        edge4_length = self.calculate_distance(Recpoint4, Recpoint1)

        # 找到两条最短的边
        shortest_edges = sorted([(edge1_length, Recpoint1, Recpoint2),
                                 (edge2_length, Recpoint2, Recpoint3),
                                 (edge3_length, Recpoint3, Recpoint4),
                                 (edge4_length, Recpoint4, Recpoint1)])

        # 找到两条最短边的中点
        midpoint1 = calculate_midpoint(shortest_edges[0][1], shortest_edges[0][2])
        midpoint2 = calculate_midpoint(shortest_edges[1][1], shortest_edges[1][2])

        return midpoint1, midpoint2

    def calculate_distance(self, point1, point2):
        # 计算两点之间的距离（欧几里德距离）
        x1, y1 = point1
        x2, y2 = point2
        distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        return distance