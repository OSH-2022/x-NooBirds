## 自动驾驶小汽车的设想

#### 综述：可行性毋庸置疑，就是实现而已，上限取决于我们的脑洞，很好拓展。

1. **功能设定**

   1. 首先是自动运行功能，有以下两种设想

      1. 贴和现实

      比如：有交通主干道一般的虚实结合的线做引导，在线内运动并自动规避路上的障碍物；

      2. 特定指令

      比如：向前移动10m等具体的指令，车辆自动规划路径行进，路途中遇到障碍物就自动重新规划；

      （也可以考虑采用**超声波避障**）

      

   2. 有关添加射击功能

      携带一小的炮筒，配合弹簧发射，指定落点或相应的目标，可以在打击之中自动修正误差，调整方向、俯角以及力度（弹簧压缩长度），直至实现精确打击。

      (在考虑可否将**图像处理（Python）与超声波定位（C）结合**，实现定位的精准化，再辅助修正)

      

2. **预备条件**

   1. 树莓派 + Raspbian操作系统（待定）

   2. **神经网络**

      Python3 + Python-Opencv + Pygame + Numpy + Threading

      (其他的搭配也可选，如训练模块也可采用：Keras + Tensorflow 等）

   3. 环境管理

      Python3 —— 涉及到多版本的Python环境切换，推荐使用Anaconda管理

   4. 超声波定位模块（待定）

      C or Python

   

3. **参考项目**

   自动驾驶：

   1. [GitHub - tomatozgithub/RPi_autoDrive_opencv_python](https://github.com/tomatozgithub/RPi_autoDrive_opencv_python)
   2. [(3条消息) 【记录】本科毕设：基于树莓派的智能小车设计（使用Tensorflow + Keras 搭建CNN卷积神经网络 使用端到端的学习方法训练CNN）_Jack__Ni的博客-CSDN博客_基于树莓派的毕业设计](https://blog.csdn.net/Jack__Ni/article/details/120738969)
   3. [Timthony/self_drive: 基于树莓派的自动驾驶小车，利用树莓派和tensorflow实现小车在赛道的自动驾驶。（Self-driving car based on raspberry pi（tensorflow）） (github.com)](https://github.com/Timthony/self_drive)
   4. 超声波避障（C Language）：[ZevveZ/stm32_car: 超声波避障小车 (github.com)](https://github.com/ZevveZ/stm32_car)