# pnp位姿估计
## 简介
采用已标定好的二维相机采集红外标靶图片，并提取标靶上940nm红外灯图像，计算相机相对于标靶的位姿。

## 功能
位姿信息包括（世界坐标系原点为标靶左上，正对标靶，右侧为x正，上方为y正，后为z正）。

接口采用海康扳机相机。

数据采用tcp通信进行传输。

![demo image](516.jpg)