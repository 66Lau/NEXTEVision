import cv2
import numpy as np



class KalmanFilter(object):

    def __init__(self):

        self.kalman = cv2.KalmanFilter(4, 2) # 4：状态数，包括（x，y，dx，dy）坐标及速度（每次移动的距离）；2：观测量，能看到的是坐标值
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32) # 系统测量矩阵
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],      np.float32) # 状态转移矩阵
        self.kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)*0.03 # 系统过程噪声协方差
        self.current_measurement = np.array((2, 1), np.float32)
        self.last_measurement = np.array((2, 1), np.float32)
        self.current_prediction = np.zeros((2, 1), np.float32)
        self.last_prediction = np.zeros((2, 1), np.float32)
        self.error_frame = 0

    def track(self,x,y):
        self.last_prediction = self.current_prediction # 把当前预测存储为上一次预测
        self.last_measurement = self.current_measurement # 把当前测量存储为上一次测量

        if abs(self.last_measurement[0] - x) > 64 or abs(self.last_measurement[1] - y) > 48: 
                self.error_frame = self.error_frame + 1
        else :
                pass

        if x ==0 and y == 0 : 
                self.error_frame = self.error_frame + 1
        else :
                pass

        if self.error_frame < 5 and self.error_frame > 0  :
            self.current_measurement = np.array([[np.float32(self.last_prediction[0])], [np.float32(self.last_prediction[1])]])

        else:
            self.current_measurement = np.array([[np.float32(x)], [np.float32(y)]]) # 当前测量
            self.error_frame = 0

        # print("error:",self.error_frame)
        self.kalman.correct(self.current_measurement) # 用当前测量来校正卡尔曼滤波器
        self.current_prediction = self.kalman.predict() # 计算卡尔曼预测值，作为当前预测

        lmx, lmy = self.last_measurement[0], self.last_measurement[1] # 上一次测量坐标
        cmx, cmy = self.current_measurement[0], self.current_measurement[1] # 当前测量坐标
        lpx, lpy = self.last_prediction[0], self.last_prediction[1] # 上一次预测坐标
        cpx, cpy = self.current_prediction[0], self.current_prediction[1] # 当前预测坐标

        delta_x = cpx-cmx
        delta_y = cpy-cmy
        if delta_x >25:
            delta_x = 25
        if delta_y > 25:
            delta_y = 25

        return delta_x, delta_y

        

if __name__ == '__main__':
     global current_measurement,  last_measurement, current_prediction, last_prediction

     last_measurement = current_measurement = np.array((2, 1), np.float32)

     last_prediction = current_prediction = np.zeros((2, 1), np.float32)
     kalman = cv2.KalmanFilter(4, 2) # 4：状态数，包括（x，y，dx，dy）坐标及速度（每次移动的距离）；2：观测量，能看到的是坐标值
     kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32) # 系统测量矩阵
     kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],      np.float32) # 状态转移矩阵
     kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)*0.03 # 系统过程噪声协方差
     yaw = np.array([110,112,114,116,118,120,122,124,126,128,130,132,134,136,138,140,142,144,0,0,148,0,0,0,156],np.float)
     pitch = np.array([110,112,114,116,118,120,122,124,126,128,130,132,134,136,138,140,142,144,0,0,148,0,0,0,156],np.float)
     a = KalmanFilter()

     for i in range(len(yaw)):
        nx,ny = a.track(yaw[i],pitch[i])
        print("x:")
        print(int(nx))
        print("y:")
        print(int(ny))


