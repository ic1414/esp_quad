

## 物料清单: 
### 无人机:
F450 机架

2212 1000kv 电机(我的电机最大15A)

9047 propeller

Blhelis 30A ESC 

2200mAh 35C lipo (建议选>35C)

飞控

### 飞控
ESP32 dev v1

MPU6050

Buck converter (optional)

NRF24l01 (dip)

resistor

pcb 大小:50*70mm

pcb 洞直径4mm， 圆心距离板子边缘也是4mm

#### 安装
esp32，mpu 和 nrf 都是在一个平面上的，要用到排母。具体看下方图片。

![Snipaste_2022-02-12_21-18-44](https://user-images.githubusercontent.com/93729382/153728822-b4021f53-91a1-41fd-ba63-0a54a1d2fdcd.png)

![Snipaste_2022-02-12_21-18-16](https://user-images.githubusercontent.com/93729382/153728832-dda35325-d8dc-4069-884d-cacaf6c28adc.png)



## 建议
用降压模块将电池的3s降到5v再接到PCB的2s输入口给飞控供电。

你可能需要给你的桨叶动平衡。可以用一些胶带。
