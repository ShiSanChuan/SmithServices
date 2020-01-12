# SmithServices
加油啊，猩红爱丽丝

<img src="doc/maj86-jvk9u.gif">

## environment test
- Ubuntu 16.04
- GNU 5.4.0
- Opencv 3.3.1
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) 0.6
- [ceres](http://ceres-solver.org/installation.html)

## build
~~需要google求解器~~
```shell

```
```shell
mkdir build &  cd build
cmake ..
make
```

### 虚拟串口
因为测试用的CH340G为半双工，不能同发同受，可以用虚拟串口测试
```shell
sudo apt-get install socat
socat -d -d pty,raw,echo=0 pty,raw,echo=0

```

## run
### 配置文件 param.yaml
- map_width : 地图宽
- map_length : 地图长
- BALLON_num : 气球个数 
- AIMUAV_num : 目标机个数
- Listen_port : 服务串口
- Simulate_port : 仿真串口
- UAV_filed : UAV视野
- UAV_speed : UAV速度
- start_point : UAV起始坐标
- first_point ：UAV第一次设定坐标
- end_point ： UAV返回坐标
- Simulate_speed : 仿真目标机速度
- ballon_point : 气球坐标

```shell
./bin/RunService param.yaml 2>/dev/null
```
## 通讯协议
- 使用16字节包为一个数据包
- CMD:前4bit为状态，后4bit为标识 三架飞机分别为 

```
typedef enum:unsigned char{
	ROBOT_MODE_IN_INIT    = 0x00,//初始化
	ROBOT_MODE_IN_TAKEOFF = 0x01,//自动起飞
	ROBOT_MODE_IN_MOVETO  = 0x02,//飞到一个点
	ROBOT_MODE_IN_LINE    = 0x03,//一条线飞
	ROBOT_MODE_IN_ARCH    = 0x04,//弓字型飞
	ROBOT_MODE_IN_CATCH   = 0x05,//抓气球
	ROBOT_MODE_IN_STAB    = 0x06,//刺气球
	ROBOT_MODE_IN_RETURN  = 0x07,//返回
	ROBOT_MODE_IN_LOST    = 0xfe,//时延太高丢失
	ROBOT_MODE_IN_EMPTY   = 0xff,//空状态	
}Status;
typedef enum:unsigned char
{
	UAV1 = 0x00,
	UAV2 = 0x01,
	UAV3 = 0x02,
	AIM = 0x03,
	Service = 0x04
}Marker;
```
| |Head|CMD|UAV-x|Position|End|
| --- | --- | --- | --- | --- | --- |
|ROBOT_MODE_IN_INIT | ff |00|UAVx|XYZ| 0d |
|ROBOT_MODE_IN_TAKEOFF | ff |01|UAVx|XYZ| 0d |
|ROBOT_MODE_IN_MOVETO | ff |02|UAVx|XYZ| 0d |
|ROBOT_MODE_IN_LINE | ff |03|UAVx|XYZ| 0d |
|ROBOT_MODE_IN_ARCH | ff |04|UAVx|XYZ| 0d |
|ROBOT_MODE_IN_CATCH | ff |05|UAVx|XYZ| 0d |
|ROBOT_MODE_IN_STAB | ff |06|UAVx|XYZ| 0d |
|ROBOT_MODE_IN_RETURN | ff |07|UAVx|XYZ| 0d |
|ROBOT_MODE_IN_LOST | ff |fe|UAVx|XYZ| 0d |
|ROBOT_MODE_IN_EMPTY | ff |ff|UAVx|XYZ| 0d |

## todo
- ~~合并两个飞机代码~~
- 添加TCP通信方式
- ~~解决飞机启动飞行问题结合大疆wiki~~
- 解决数传问题（或者通过遥控器控制）
- ~~增加代码注释，方便修改问题~~
- 两架机协同扎球

