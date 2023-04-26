import time
class get_pose():
    def __init__(self):
        self.pose_x=0
        self.pose_y=0
        self.pose_z=0
        self.pose_path='/home/lau/ros_project/slam_ws/pose.txt'


    def get_pose(self):
        while True:
            time.sleep(0.2)
            try:
                with open(self.pose_path, 'r') as file:
                    lines = file.readlines()
                    self.pose_x = float(lines[0].strip())
                    self.pose_y = float(lines[1].strip())
                    self.pose_z = float(lines[2].strip())

                print("X_坐标：", self.pose_x)
                print("Y_坐标：", self.pose_y)
                print("Z_坐标：", self.pose_z)
                file.close()
            except:
                continue