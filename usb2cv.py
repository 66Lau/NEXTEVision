import cv2

class usb2cv_class():
    def __init__(self,CamNum):
        self.cap = cv2.VideoCapture(CamNum)
        self.CamNum = CamNum
        self.ret = None
        self.image_fifo = None
        self.image_read_log = False

    def get_img_thread(self):
        cap = cv2.VideoCapture(self.CamNum)
        while True:
            # 读取帧
            self.ret, self.image_fifo = cap.read()
            self.image_read_log = True
            # 按‘q'退出
            if cv2.waitKey(1) and 0xFF == ord('q'):
                break
