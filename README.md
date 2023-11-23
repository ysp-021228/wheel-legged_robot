# 目录

- [轮腿机器人相关说明](#轮腿机器人相关说明-a-name1-sectiona)
  - [整机图片](#整机图片-a-name1-section-1a)
  - [正方向定义](#正方向定义-a-name1-section-2a)
  - [底盘结构体从属关系](#底盘结构体从属关系-a-name1-section-3a)
  - [控制方式说明](#控制方式说明-a-name1-section-4a)
- [项目系统与功能设计](#项目系统与功能设计-a-name2-sectiona)
  - [硬件设计](#硬件设计-a-name2-section-2a)
  - [软件设计](#软件设计-a-name2-section-3a)
    - [系统架构](#系统架构-a-name2-section-3-1a)
    - [运行流程](#运行流程-a-name2-section-3-2a)
    - [底盘任务流程](#底盘任务流程-a-name2-section-3-3a)
    - [离线检测任务流程](#离线检测任务流程-a-name2-section-3-4a)
  - [功能设计](#功能设计-a-name2-section-4a)


# 轮腿机器人相关说明 <a name="1-section"></a>
## 整机图片 <a name="1-section-1"></a>
<center>

![GitHub Logo](picture/f1f453ee73a97a37a0b2060098c7de4.jpg)
</center>

## 正方向定义 <a name="1-section-2"></a>
<center>

![GitHub Logo](picture/Positive_direction_specification.jpg)
</center>

## 底盘结构体从属关系 <a name="1-section-3"></a>
<center>

![GitHub Logo](picture/底盘结构体.png)
</center>

## 控制方式说明 <a name="1-section-4"></a>
机器人左右两腿皆从左视图参照正方向定义中控制两腿，参考示意图：
<center>

![GitHub Logo](picture/Control_specification.jpg)
</center>

# 项目系统与功能设计 <a name="2-section"></a>
## 机械设计 <a name="2-section-1"></a>
<center>

![GitHub Logo](picture/腿部设计.jpg)
</center>

五连杆机器人腿是一种常见的机械结构，具有一些优势，特别适用于复杂的应用场景。  

**稳定性：** 五连杆结构可以提供较好的稳定性，使得机器人在行走或运动时更加平稳。这对于需要在不同地形或工作环境中移动的机器人尤为重要。  
**适应性强**： 五连杆机器人腿的设计使得机器人能够适应不同高度和倾斜角度的地面。这种灵活性使得机器人能够在多样化的环境中行动，从而拓展了其应用领域。  
**精准控制**： 五连杆结构提供了较好的关节控制，使得机器人能够在移动和定位过程中实现更为精准的运动。这对于执行复杂任务或避开障碍物非常有帮助。  
**有效载荷能力**： 由于五连杆机器人腿的结构稳定，它通常具有较高的有效载荷能力。这使得机器人能够携带更多的设备、传感器或负载，扩展了其应用范围。  
**节省空间**： 五连杆腿结构通常能够以较小的空间实现较大的运动范围，这对于有限空间内的机器人应用尤为重要，如在狭窄的通道中行走或执行任务。  
**可编程性**： 五连杆机器人腿的关节可编程性较高，使得开发人员可以更容易地实现各种运动模式和行为，从而适应不同的任务需求。  

总体而言，五连杆机器人腿的优势在于其结构稳定性、适应性强、精准控制和较高有效载荷能力。

### 机械参数 <a name="2-section-1-1"></a>


#### 尺寸相关

|   部件名称    |  数值  | 单位 |
|:---------:|:----:|:--:|
|    杆L1    | 0.11 | m  |
|    杆L2    | 0.18 | m  |
|    杆L3    | 0.18 | m  |
|    杆L4    | 0.11 | m  |
|    杆L5    | 0.09 | m  |
|   驱动轮半径   | 0.05 | m  |
| 底盘绕Z轴旋转半径 | 0.1577 | m  |
| 大腿趴下机械限位  | 0.10472 | rad  |


#### 质量相关

|   部件名称    |  数值  | 单位 |
|:---------:|:----:|:--:|
|    整车    | 4.635 | kg  |
|    轮子    | 0.183 | kg  |
|    单腿    | 0.715 | kg  |
|    车体    | 2.839 | kg  |

## 硬件设计 <a name="2-section-2"></a>
### 整机硬件框图
<center>

![GitHub Logo](picture/硬件框图.png)
</center>
整车电源由电池提供，电池直接连接中心板再分流到各电机以及主控板，可以有效保护主控板。机器人主要执行器都是用CAN通信，整机一共使用2个CAN网络。  

轮子驱动电机电子调速器CAN总线直接连接主控板，接入CAN1网络；关节电机发送扩展帧优先级较低，为了避免丢失数据，将四个关节电机单独接入CAN2网络，并通过中心板连接主控板。

## 软件设计 <a name="2-section-3"></a>
### 系统架构 <a name="2-section-3-1"></a>
机器人的主控程序采用相同的驱动层和整体架构。主控程序以 FreeRTOS 作为片上操作系统，实现多线程间的调度和通信。FreeRTOS 操作系统是一个轻量级的操作系统，提供了线程管理、时间管理、内存管理等。
<center>

![GitHub Logo](picture/系统架构.png)
</center>
系统以FreeRTOS为片上操作系统，底层的外设驱动涉及 UART、CAN、IIC、GPIO、SPI、FLASH等。顶层应用层包含 Task 和 Module 两方面，Task 中主要为与系统运行相关的线程，Module 主要为系统运行过程所需要使用的算法(控制算法、解算算法、滤波算法等)和工具（遥控器、传感器等）。

### 运行流程 <a name="2-section-3-2"></a>
<center>

![GitHub Logo](picture/运行流程.png)
</center>

### 底盘任务流程 <a name="2-section-3-3"></a>
<center>

![GitHub Logo](picture/底盘任务流程.png)
</center>
底盘运行任务开始后，会进行一段时间延时，再进行底盘相关变量的初始化。初始化完毕后进入循环体。循环体中，chassis_info_update()更新底盘陀螺仪和电机相关数据；chassis_ctrl_info_get()解析遥控发送的底盘控制信息；chassis_control()根据底盘控制命令对底盘进行运动学正逆解算，并进行LQR和VMC计算出电机的期望转矩，并根据电机说明转换为期望电流；chassis_device_offline_handle()检测底盘相关设备的是否离线，若离线，则使相关电机失能，防止底盘失控；chassis_motors_can_send()发送can数据控制电机。  

腿部控制处理中，chassis_forward_kinematics()先结算处虚拟腿姿态；chassis_K_matrix_fitting()根据虚拟腿长进行LQR的K矩阵拟合；off_ground_detection()根据腿部支持力判断是否离线以及进行离线处理；jump_handle()在接到跳跃命令之后进行跳跃处理；state_variable_reference相关函数进行状态变量的期望、反馈、误差获取并在motors_torque_cal()中计算电机期望转矩；vmc_inverse_solution()进行运动学和动力学逆解；最后fn_cal()进行支持力计算。

### 离线检测任务流程 <a name="2-section-3-4"></a>
<center>

![GitHub Logo](picture/离线监测任务流程.png)
</center>
detect_task是用来检测离线设备的任务。任务开始时，detect_device_init()对被检设备进行初始化，设置离线判断的时间阈值并默认设备离线。初始化完毕后进入循环体。每次循环开始会初始化最高警告等级和离线设备数量，接着检测每个设备离线间隔时间是否超过对应设备的间隔阈值，超过的标记为离线，并且记录最高警告等级。offline_remind()中根据最高离线等级和离线设备数量，利用蜂鸣器和RGB灯发出提醒信息。

## 功能设计 <a name="2-section-4"></a>
### 移动控制
机器人可以在保持机体Pitch轴稳定的情况下进行移动和旋转运动。
<center>

![GitHub Logo](picture/control/移动控制.jpg)
</center>

### 自由度控制
Pitch、Yaw、Roll三轴自由度调节，并配合腿部控制完成Z轴升降。
<center>

![GitHub Logo](picture/control/自由度控制.jpg)
</center>

### 适应性控制
复杂路面上的自适应，能完成Roll轴自稳，下台阶冲击抵抗，转向重心偏移。
<center>

![GitHub Logo](picture/control/适应性控制.jpg)
</center>

### 操作控制
能在一个遥控器上实现所有操作，能够检测机器人是否离地，能稳定自启。
<center>

![GitHub Logo](picture/control/操作控制.jpg)
</center>

### 抗干扰能力
遇到持续外力和突发冲撞能保持稳定。
<center>

![GitHub Logo](picture/control/抗干扰能力.jpg)
</center>

### 跳跃
能够稳定跳跃，实现原地跳跃和向前跳跃。
<center>

![GitHub Logo](picture/control/原地跳跃.jpg)
![GitHub Logo](picture/control/跳上台阶.jpg)
</center>


<center>


</center>