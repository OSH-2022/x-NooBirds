### 有关仿真平台以及硬件实体的调研

#### 许元元 PB20511899

------

#### 一、建模与仿真平台：

1. **自动驾驶仿真**
   		在仿真场景中，普通场景下的自动驾驶算法已经比较完善，目前市面上存在大量基于游戏引擎开发的仿真软件，比如《地平线》系列，其贴近真实的物理引擎以及成效确实非常吸引眼球，但从务实的角度来看，确实有很大的局限性。

   ​		突破难点在于一些极端场景（corner cases），由于极端场景在现实中可遇不可求，利用仿真平台可以便捷生成。所以业界共识是加大仿真测试在自动驾驶测试中的占比。目前自动驾驶算法测试大约90%通过仿真平台完成，9%在测试场完成，1%通过实际路测完成。[1] 无人驾驶汽车真实上路后所要面临的外部环境是复杂多变的。通过利用仿真模拟软件可以检查算法，也可以训练无人车面对不同场景下的感知、决策等算法。

   ​		2020年6月，WP.29世界车辆法规协调论坛通过了世界上首个针对Level3级自动驾驶车辆有约束力的法规—- UN Regulation on Automated Lane Keeping Systems (ALKS)，此条款从2021年1月正式开始生效。此法规着重描述了此自动驾驶功能与多支柱法之间的关系，提出了具体的道路测试、场地测试、仿真测试、审核与验证具体的映射关系，进一步确立了多支柱法中不同条件的交互关系。此外针对安全及相应问题，提出了以下几点要求:

   1. 发生碰撞情况下的紧急情况的处理机制，需要利用仿真进行相应测试；
   2. 系统失效条件下，要求驾驶员取回控制权及相应条件；
   3. 提出系统移交条件以及驾驶员无回应时的系统保护的风险条件;
   4. 安装车辆驾驶员就位识别系统，识别驾驶员就位以及其控制意图识别;
   5. 车辆车载自动驾驶信息存储系统，以及其相应信息记录机制;

   ​		从技术上分，仿真平台主要有两种：第一种是基于合成数据对环境、感知以及车辆进行模拟，这种模拟器主要用于控制与规划算法的初步开发上；第二种是基于真实数据的回放以测试无人驾驶不同部件的功能及性能。

   ​		显然我们选择的方向更倾向于前者。

   ​		补充资料：

   ​		[谈无人驾驶仿真](https://zhuanlan.zhihu.com/p/153979389)

   
   
   ​		**插图：车企主要采用的仿真软件**
   
   
   ​	
   
2. **考察指标**

   ​	首先要符合我们的预设计提出的要求，即可闭环控制的实时操作系统。而且考虑无人车在运行的过程中，可以有差速控制，转向角度精细控制，电机可以正反转来做调头等功能，主要是在于精确控制。

   ​	所以要满足我们应用的以下要求：

   1. **实时性**，下达指令同步，可拓展性，稳定性；

   2. **精确性**，能够达到我们定角度转动，精细修正等要求；

   3. **闭环控制**，能够模拟反馈性闭环控制；

      

3. **初步调研**—平台的选择

   1. **建模：**

      尽量在网上寻找对应模型的数据参数或者相似的模型，我的考虑是尽量根据硬件来采用相应的仿真平台，从而可以获得准确的模型数据，有助于我们后续的仿真操作；

      >  如果寻找不到或难以完成要求再考虑使用软件搭建模型

   2. **预仿真：**

      即数学推导模型的仿真：

      **Simulink** [Get a free trial](https://www.mathworks.com/products/simulink.html)

      **模型搭建 & 运行仿真**

      **实例项目：**[小车建模仿真](https://www.guyuehome.com/17929)

      

      **插图：Simulink**

      

      

   3. **仿真：**

      1. [**CarSim**](https://www.carsim.com/products/carsim/)：

         

         **插图：CarSim**

           

         **仿真器简介：**是 Mechanical Simulation 公司开发的强大的动力学仿真软件（其实是一个包）

          被世界各国的主机厂和供应商所广泛使用。 CarSim 针对四轮汽车， 轻型卡车。

         **安装流程：**
   
         ​	收费.jpg，我就没有安装并实践
      
         **其他测评报告：**
      
         1. **CarSim 模型在计算机上运行的速度可以比实时快 10 倍，模拟结果高度逼近真实车辆。**可以仿真车辆对驾驶员控制， 3D 路面及空气动力学输入的响应， 主要用来预测和仿真汽车整车的操纵稳定性、 制动性、 平顺性、 动力性和经济性。 
         2. **CarSim 自带标准的 Matlab/Simulink 接口， 可以方便的与 Matlab/Simulink 进行联合仿真**， 用于控制算法的开发。
         3. **同时在仿真时可以产生大量数据结果用于后续使用 Matlab 或者 Excel 进行分析或可视化。** 
         4. CarSim 同时提供了 **RT 版本**， 可以支持主流的 HIL 测试系统， 如 dSpace 和 NI 的系统， 方便的联合进行 HIL 仿真。

         

      2. [**Apollo**](https://apollo.auto/developer_cn.html)：

         **仿真器简介：**由百度-阿波罗开发的驾驶仿真平台。

         可以支持一定功能的在线仿真，主要仍是个人电脑安装运行的仿真器。
   
         **Github源码：**[apollo](https://link.zhihu.com/?target=https%3A//github.com/ApolloAuto/apollo)
      
         **安装流程：**

         1. 在线的跟着用户指南走就好了；
         2. 电脑安装的流程参考[官方文档](https://link.zhihu.com/?target=https%3A//github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide.md)；
         3. 因为要通过docker安装，为了防止各种奇奇怪怪的报错，最好还是把docker运行权限改一下。[相关指南](https://link.zhihu.com/?target=https%3A//www.markjour.com/article/docker-no-root.html)

         **个人体验以及其他测评报告：**

         ​		刚开始进入Apollo官网眼前一亮，一个界面简约的现代化仿真软件跃然于眼前。不过实际了解其功能，更多的是参考了其他业界人士测评的报告，普遍的看法是其距离成为一款成熟的仿真器，还有很长的路要走。

         ​		官方提供的场景比较少，大型地图屈指可数。这一点对于我们要求也不是太高。只是它的文档支持不够完善，封装得比较严实。不过改地图，放npc的自由设置空间还是有的，总体而言还在成长阶段，对于我们这种还没有过多接触过无人驾驶行业的学生来讲，我的感觉是不推荐！

         ​		（但也可以试一试，如果学校可以报销我们一套几w的硬件设备×）

         

      3. **RoboMaster (DJI Education Hub)**：

         **仿真器简介：**

          **运行** DJI_Education_Hub_x86_Installer.exe **安装**

         **安装流程：**根据官网教程操作即可

         **总结：**

         1. **总体推荐**，实验室现有容易获取的实体平台（大疆战甲和机器人）；

         2. 支持图形化界面编程 & Python编程；

         3. 生态较为健康，并且文档支持很齐全；

         4. 接口完整（S1和EP系列均是）：

            1. **S1系列**

               [大疆Python API列表](https://github.com/program-in-chinese/overview/wiki/大疆Python-API列表)
   
               [机甲大师S1主要api汇总-大疆社区 (dji.com)](https://bbs.dji.com/forum.php?mod=viewthread&tid=227127)
      
            2. **EP系列**
      
               1. 开放 DJI 官方 SDK，支持超过 50 个可编程传感器接口；
               2. 支持外接Arduino、树莓派等第三方开源硬件，且能利用自身电池为它们供电；
               3. [开发者文档](https://robomaster-dev.readthedocs.io/zh_CN/latest/)
               4. [模块编程手册](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/dji%20terra%20%E6%96%87%E4%BB%B6%E4%B8%8A%E4%BC%A0/%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%20RoboMaster%20EP%20%E7%BC%96%E7%A8%8B%E6%A8%A1%E5%9D%97%E6%89%8B%E5%86%8C%20V2.0.pdf)

            

      4. [**Airsim**](https://link.zhihu.com/?target=https%3A//github.com/microsoft/AirSim)

         **仿真器简介：**由微软开发的仿真器，基于虚幻引擎。主要面向无人机仿真，也提供了驾驶仿真的接口。适用于驾驶决策仿真任务。输出的数据模态包括GPS，IMU，RGB图像，深度图，语义分割，红外相机，事件相机，点云（部分功能额外配置）。

         **安装流程：**
   
         见[Welcome to AirSim](https://link.zhihu.com/?target=https%3A//microsoft.github.io/AirSim/)
      
         **调研仿真器特点：**

         1. 安装流程简单，直接跟着官网教程走，下载封装好的包。
         2. 延续了微软一向的风格，封装过于严密，自定义地图，NPC，天气都不可行。
         3. 接口调用比较简单，容易上手。
   
         4. 更适合Windows系统用户。Github上提供的地图适用于驾驶仿真很少，但是AirSim可以从[Unreal Engine Market](https://link.zhihu.com/?target=https%3A//www.unrealengine.com/marketplace/en-US/store)中获取更多可用于自动驾驶的地图，其中绝大多数都只兼容Windows。
      
         
      
      5. **其他可选项**
      
         Unity、华为自动驾驶云服务 Octopus、SUMO、Cognata等
   
   
   
   

#### 二、硬件实体

**这里我给出两套方案：**

1. **基于特定仿真平台采购并使用其现有的的硬件设备**

   这里特别推荐大疆开发者平台，因为已知在实验室可当即获取的大疆机器人，RoboMaster等内容，且其生态和各项条件都较好；

   只是不知道仿真平台对于实时性这一方面的应答情况如何。

   **硬件：**大疆机器人/RoboMaster + 已有或自主采购的其他配件（见hlh和wtm调研）

   

2. **自行使用软件建模+仿真**

   请示过老师意见后有如下设计：

   1. 尽量避免自己建模

      设计时和采购时，如果不是特别必要，尽量注意找网上现有的或接近的模型，减少自行建模的成分；

   2. 仿真操作选取以上平台之一进行，主要推荐是：

      Simulink + CarSim

   **硬件：**

   1. **电机**：

      最重要的一环，调研以及参照老师意见后，判断**带有编码器的6v以内驱动电压的电机都还比较安全；**

      想用较好的建议是：大疆M3508电机，是大疆机器人的标配电机

   2. **其他硬件：**

      （见hlh和wtm调研）
   
   3. **Sample**
   
      >  自己组装有点low啊说实话，太掉价了
   
      
      
      **插图：车企主要采用的仿真软件**
      
      
   

#### 四、其他收获

1. **控制逻辑图**

    

   **插图：逻辑控制图**

   

2.  **其他建议：**（收集）

   1. 电池盒是后面放上去的，想要用三节18650达到12V左右的电压，电路上先用模块搭一下，电源出来12V电压使用 LM2596S-5.0V 的**稳压到5v**给树莓派供电（之前手上有L7805，两块并联都不行，发烫导致树莓派不断重启）；
   2. 买较大风扇放在电路顶端，给树莓派、稳压、电机驱动等的散热。



#### 三、References

[1] 2021中国自动驾驶仿真研究报告

[2] 中国无人驾驶仿真技术蓝皮书2020