# Armor detection using classical methods in cv in RM
---
## Introduction  
This code was specifically designed for the RoboMaster robot competition and serves as my personal learning record. Python was chosen as the main programming language for its fast development efficiency and learning convenience. Please note that due to limited personal energy and time, some of the code may be difficult to read and not fully optimized. Your understanding is greatly appreciated

---
## Directory
* [1. Main Functions](#1main-functions)
* [2. Effect Display](#2effect-display)
* [3. Environment](#3environment)
* [4. Frame](#4frame)
* [5. Scheme](#5scheme)
* [6. Communication Protocol](#6communication-protocol)
* [7. Configuration and Debugging](#7configuration-and-debugging)
* [8. To Conclude](#8to-conclude)
---

## 1.Main Functions
|Functions     |     |
| ------- | ------ |
|Armor Detector| Find armor of enemy robot |
|Rune Detector| Still need to be updated-2023 |
|Angle Transformation| get the Angle of aim point |
|Camera Driver| The industrial camera driver of Hikvision |
|Serial| Communicate with stm32 |
|Parameter Configuration| Adjusting internal parameter 
---
## 2.Effect Display
### Gyro strike(slow motion)
This is a clip of great effect, but the real effect will fluctuate
<center>
<video width="320" height="240" controls>
    <source src="DisplayVideo\sloth.mp4" type="video/mp4">
</video>
</center>

[点击观看视频演示](DisplayVideo/sloth.MP4)
 
### Gyro strike(Kill infantry of 100 HP in 5 seconds)
 <center>
<video width="320" height="240" controls>
    <source src="DisplayVideo\gyro.mp4" type="video/mp4">
</video>
</center>

### Moving object strike
 <center>
<video width="480" height="240" controls>
    <source src="DisplayVideo\move.mp4" type="video/mp4">
</video>
</center>



 
---
## 3.Environment
### Hardware
 <center>

|Hardware|Version|
|---|---|
|MiniPC|iru-i5 8250u|
|Industrial Camera|MV-CA013-A0UMUC|
|Lens|6mm|

 </center>

### Software
 <center>

|Software|Version|
|---|---|
|OS|Ubuntu18.04|
|Library|pyserial3.5|
|Library|opencv-python4.6.0.66|
 </center>

---
## 4.Frame
### document tree
```

│  ArmorDetector.py (装甲板检测)
│  DataConnect.py (上下位机通讯)
│  DataParam.py (内部参数设置)
│  get_pose.py (获取里程计信息)
│  globalVar.py 
│  hik2cv.py (海康相机SDK)
│  hikcam_param.txt (相机参数)
│  launch.py (启动文件)
│  PointTransformation.py (角度解算)
│  readme.md
│  TrackKF_2D.py (卡尔曼)
│  usb2cv.py (普通相机)
|      
├─MvImport (海康相机库)
├─TestImage
├─DisplayVideo


```
### Flow Chart
Waiting for update 

---
## 5.Scheme 
### Armor identification  

1. **Img preprocessing**  


2. **Light strip detection**  
 

3. **Light strip matching**  
 
4. **digital recognition**  


5. **Target Armor selection**  

### Angle solution  
### Distance solution
### Muzzle elevation

---


## 6.Communication Protocol

### This version was wrote in hurry, the code of communication would be improved later.
---
## 7.Configuration and Debugging

### the debugging arrangements was packed in DataParam.py, where could debug the code more convenient.
---
## 8.To Conclude
### The code was written in a hurry by me and lacked manpower and experience at the time of writing, So there are many deficiencies, which might be advanced in the future.
