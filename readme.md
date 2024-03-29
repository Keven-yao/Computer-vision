# Computer-vision
一、需求分析

本项目基于RoboMaster机器人辅助瞄准的功能需求而提出。辅助瞄准既通过机器视觉识别和追踪敌方机器人装甲模块，从而实现云台自动跟随，激光自动瞄准。
期望实现的功能如下：
1.	自动识别目标；
2.	建立机器人系统坐标系（基坐标->底盘、工具坐标->发射机构、工件坐标->敌方装甲）；
3.	识别距离，并计算仰角；
4.	通过串口实现上位机与单片机的通信，上位机发送目标作别给单片机；
5.	目标移动时可以视觉能够立马跟随，并偏转云台。
二、概要设计
（一）装甲识别
1.利用颜色权重进行颜色分割，灰度二值化，将灰度二值图与颜色分割图做差，去除干扰，并对最终二值图进行形态学及滤波操作。必要情况可搭配滤光片辅助。
2.寻找轮廓并进行椭圆拟合得到疑似灯条集合。
3.灯条两两组合得到疑似装甲。
4.装甲集合进行二次筛选去除粘连装甲，如图2.1所示其中序号2为正确装甲序号1和3为错误装甲，二次筛选利用共用灯条等信息去除错误装甲。第三次筛选装甲集合，将装甲集合安装与上一帧识别装甲中心距排序，对于最近目标判断中心距是否满足一定确定阈值，满足则确认目标连续与上一帧目标相同，否则判断目标装甲号码，相同则目标连续否则切换目标。切换目标时将装甲集合按灯条高度排序，依次识别装甲号码，期间可搭配一定策略，如跳过工程机器人，当目标号码符合可击打要求时确定击打当前目标，并记录装甲号码。

 ![image](https://user-images.githubusercontent.com/55519437/138230944-f7ab97e6-d092-4ced-b2a1-b7e8f06f15d2.png)
图2.1

（二）跟随打击

视觉识别到装甲后将装甲相对于云台的坐标信息发送给stm32，stm32收到后，对坐标进行轨迹规划，选用的是三次多项式轨迹规划，轨迹规划后得到电机转角，采用PID控制，电机进行响应，当电机转到目标值是，启动打击目标。
PID控制如图2.2：

![image](https://user-images.githubusercontent.com/55519437/138231168-25f0d341-4844-4d23-b5fe-7aa0f01ddcce.png)
 
图2.2

（三）电控
云台控制、云台旋转坐标的计算，利用获得的目标点坐标与当前坐标的误差来确定云台需要旋转的方向和角度，如图2.3。

![image](https://user-images.githubusercontent.com/55519437/138231233-291e1dc6-af7c-4086-823c-e5ef88335f14.png)

图2.3

三、详细设计

1.具体视觉算法流程图

![image](https://user-images.githubusercontent.com/55519437/138231276-9ca99b18-cb6d-48ab-82f0-2b950bb42d62.png)

图3.1
2. 初始数据与图片的获取及处理
2.1．检测出LED灯带的位置，检测的方法：图像二值化，提取轮廓，拟合成旋转矩形，判断旋转矩形的中心是否接近白色，若接近白色，则说明是LED等待所在位置，然后利用检测出的LED灯带的旋转矩形的是否是平行关系来检测装甲。以一帧图像为例，说明其检测过程，原图片如图3.2.1。
2.2.亮度调整。每个通道的数值-120，小于零=0，大于255则=255，用于突出LED灯带所在区域，如图3.2.2。

![image](https://user-images.githubusercontent.com/55519437/138231317-c356ee13-3222-4298-b902-a534c5c91c4d.png)![image](https://user-images.githubusercontent.com/55519437/138231338-3ad2ea68-7744-4c9c-8ea6-f4c0621d7142.png)	 
图3.2.1	图3.2.2

2.3.二值化。将上图RGB，三个通道分离，R值减去G值，若大于25，则在二值图像中为255，否则为0.二值化后可以看到断断续续的LED灯区域的轮廓，如图3.2.3。
2.4.膨胀之后（如图3.2.4），将轮廓较为平滑，连通区域变得粗重。
![image](https://user-images.githubusercontent.com/55519437/138788159-5aed8c22-bedf-40a0-bb64-03e783ed4a33.png)![image](https://user-images.githubusercontent.com/55519437/138788166-6fde81b8-40b3-4f48-9f78-c9b9f15ece5a.png)

图3.2.3	图3.2.4
2.5.腐蚀操作后见下图，先膨胀在经腐蚀相当于闭运算，有去除空洞，使得轮廓变 得平滑的功能，如图3.2.5。
2.6.提取轮廓后，如图3.2.6。
![image](https://user-images.githubusercontent.com/55519437/138788193-86b05951-10d6-46a8-b0b6-5600c9bf8a29.png)![image](https://user-images.githubusercontent.com/55519437/138788204-ef6d08ac-b9fd-4899-88f3-df254d70b622.png)

图3.2.5	图3.2.6
2.7.遍历所有轮廓，找出满足条件（轮廓要大于10个像素点）的轮廓，拟合成旋转矩形，然后在判断该旋转矩形的中心[Math Processing Error]5×55\times{5}的像素块（要从二值化之前的图像看）是否接近白色，如何是则说明可能是装甲LED灯带所在区域，将其添加至一个向量vEllipse中，为下一步定位装甲做准备。如图3.2.7，展示了这些旋转矩形：
![image](https://user-images.githubusercontent.com/55519437/138788217-938b6091-ef2b-40ac-a384-d430d10f2aad.png)

图3.2.7
3.检测装甲的位置
在以上旋转矩形中，两两判断，根据下载原则定位装甲：
（1）两个矩形近似平行，即旋转矩形的角度之差接近。
（2）两个旋转矩形的宽和高应该相差不大。
然后求出装甲的宽度和高度，高度取LED旋转矩形height的平均值，宽度等于两个旋转矩形的中心距离。最后标出装甲位置，如图3.3。
![image](https://user-images.githubusercontent.com/55519437/138788227-ce8055c5-e8e0-42c0-a07a-b0551a7c737b.png)

图3.3
4.与电子控制的通信
使用linux QT usb转TTl进行串口通信，如图3.4：
![image](https://user-images.githubusercontent.com/55519437/138788242-7bffc366-72d5-4624-a090-1a243c284637.png)

图3.4
5.角度解算
装甲检测与神符检测输出皆为一个旋转矩形，用于描述目标所在图像中位置。为了输出真实的角度信息，需对目标进行位置解算，由于知道目标的真实大小以及相机的标定参数，则可以利用PnP解算目标相对于相机的位置，再利用相机的安装信息，可以将摄像机坐标系与云台坐标系进行转换，最终计算出云台到达目标所需的转角，如图3.5：
![image](https://user-images.githubusercontent.com/55519437/138788252-9a33822d-4ed2-4f70-85d4-caeb6ee78bf7.png)

图3.5
炮管偏移：
将目标转换至云台坐标系后，由于炮管在云台的Y轴上存在偏移，因此云台的转角计算时需考虑该偏移的影响。（如上图所示，若不考虑炮管偏移，则云台转角位α，若考虑则转角应为θ）
重力影响：
为了消除重力带来的子弹弹道改变，需要在原本的角度上向上提高一个角度，该角度与子弹速度、目标的距离相关。
四、开发实现
1.PID调节：
PID算法使用率比较频繁，在底盘驱动、云台驱动、射速控制等等均有涉及，故为最重要的算法之一。主要由电控组进行设计：
![image](https://user-images.githubusercontent.com/55519437/138788262-1edb3a43-3e00-4606-a3fc-9ffe00e86bc8.png)

图4.1
2.视觉识别算法：
用于识别能量机关、辅助瞄准、哨兵视觉识别。主要由算法组进行设计。首先检测出LED灯带的位置，检测的方法：图像二值化，提取轮廓，拟合成旋转矩形，判断旋转矩形的中心是否接近白色，若接近白色，说明是LED等待所在位置，然后利用检测出的LED灯带的旋转矩形的是否是平行关系来检测装甲。
![image](https://user-images.githubusercontent.com/55519437/138788268-82855916-2e5d-43b8-b46d-bc1e7d6c2ca8.png)

图4.2.1
寻找灯条位置：寻找符合灯条形状、颜色的所有矩形区域，并保存目标数据
![image](https://user-images.githubusercontent.com/55519437/138788277-4008a872-c2b1-4757-9077-00eb90db8b66.png)

图4.2.2
检测装甲，框出装甲位置并通过串口发送坐标数据
![image](https://user-images.githubusercontent.com/55519437/138788286-a4b6f980-5f05-4428-b278-b9fcec7255ec.png)

图4.2.3
显示视频图像
![image](https://user-images.githubusercontent.com/55519437/138788290-affb25c8-40b4-4f4c-8c9c-b94522658dba.png)

图4.2.4
清理内存
![image](https://user-images.githubusercontent.com/55519437/138788302-60f9ed05-0c28-4d86-b1c3-0b4e64d35169.png)

图4.2.5
五.系统测试
通过测试检验可达到3m内百分百识别。
![image](https://user-images.githubusercontent.com/55519437/138788314-6606767d-1d8c-44be-8331-cb41a1a73c9d.png)

图5.1
![image](https://user-images.githubusercontent.com/55519437/138788324-b0455e45-1a7e-462b-9bac-1182ba2918de.png)![image](https://user-images.githubusercontent.com/55519437/138788331-41b9c98a-b17a-44d6-ae74-7570e3c19939.png)

图5.2	图5.3
注意：由于没能模拟实际赛场的环境，不能保证在将来可以直接运用算法至比赛中。
六.总结
此次为期两周的机器人实训，是对本团队潜力的进一步锻炼，也是一种考验。通过这个项目，我们将课内外所学的理论知识结合到了实际运用中来，从中获得了诸多收获，学到了新技能，提高了动手能力。 
