import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import pandas as pd

with open("../datalog.csv") as f:
	datalog = pd.read_table(f,sep="\t");

UAV1= datalog[:][["UAV1X","UAV1Y","UAV1Z","UAV1situation","UAV1_t"]]
UAV1.columns = ["X","Y","Z","situation","time"]
UAV2= datalog[:][["UAV2X","UAV2Y","UAV2Z","UAV2situation","UAV2_t"]]
UAV2.columns = ["X","Y","Z","situation","time"]
UAV3= datalog[:][["UAV3X","UAV3Y","UAV3Z","UAV3situation","UAV3_t"]]
UAV3.columns = ["X","Y","Z","situation","time"]

# print(UAV1.head())
# print(UAV2.head())
# print(UAV3.head())
#显示路径
def showdata():
	fig = plt.figure(figsize=(16,4))
	ax1 = fig.add_subplot(141,projection='3d')
	ax1.set_xlim(0,100);
	ax1.set_ylim(0,40);
	ax1.set_zlim(0,20);
	ax1.set_title("UAV1 Motion trajectory");
	ax1.set_xlabel("x");
	ax1.set_ylabel("y");
	ax1.set_zlabel("z");
	ax2 = fig.add_subplot(142,projection='3d')
	ax2.set_xlim(0,100);
	ax2.set_ylim(0,40);
	ax2.set_zlim(0,20);
	ax2.set_xlabel("x");
	ax2.set_ylabel("y");
	ax2.set_zlabel("z");
	ax2.set_title("UAV2 Motion trajectory");
	ax3 = fig.add_subplot(143,projection='3d')
	ax3.set_xlim(0,100);
	ax3.set_ylim(0,40);
	ax3.set_zlim(0,20);
	ax3.set_xlabel("x");
	ax3.set_ylabel("y");
	ax3.set_zlabel("z");
	ax3.set_title("UAV3 Motion trajectory");
	ax4 = fig.add_subplot(144,projection='3d')
	ax4.set_xlim(0,100);
	ax4.set_ylim(0,40);
	ax4.set_zlim(0,20);
	ax4.set_title("UAVs Motion trajectory");
	ax4.set_xlabel("x");
	ax4.set_ylabel("y");
	ax4.set_zlabel("z");

	ax1.plot3D(UAV1["X"],UAV1["Y"],UAV1["Z"],'gray')
	ax2.plot3D(UAV2["X"],UAV2["Y"],UAV2["Z"],'gray')
	ax3.plot3D(UAV3["X"],UAV3["Y"],UAV3["Z"],'gray')

	ax4.plot3D(UAV1["X"],UAV1["Y"],UAV1["Z"],'red',label='UAV1')
	ax4.plot3D(UAV2["X"],UAV2["Y"],UAV2["Z"],'green',label='UAV2')
	ax4.plot3D(UAV3["X"],UAV3["Y"],UAV3["Z"],'blue',label='UAV3')

	plt.show()

#路迹轨道图
def showpath():
	with open("../archpath_0.csv") as f:
		archpath_0 = pd.read_table(f,sep="\t");
	with open("../archpath_1.csv") as f:
		archpath_1 = pd.read_table(f,sep="\t");
	with open("../archpath_2.csv") as f:
		archpath_2 = pd.read_table(f,sep="\t");

	fig2 = plt.figure(figsize=(5,4))
	ax  = fig2.add_subplot(111,projection='3d')
	ax.set_xlim(0,100);
	ax.set_ylim(0,40);
	ax.set_zlim(0,20);
	ax.set_title("UAV1 Motion trajectory");
	ax.set_xlabel("x");
	ax.set_ylabel("y");
	ax.set_zlabel("z");
	ax.plot3D(archpath_0["X"],archpath_0["Y"],archpath_0["Z"],'gray')
	ax.plot3D(archpath_1["X"],archpath_1["Y"],archpath_1["Z"],'gray')
	ax.plot3D(archpath_2["X"],archpath_2["Y"],archpath_2["Z"],'gray')

	plt.show()

#统计数据结果
S = {0x00:"ROBOT_MODE_IN_INIT",0x01:"ROBOT_MODE_IN_TAKEOFF",0x02:"ROBOT_MODE_IN_MOVETO",\
	0x03:"ROBOT_MODE_IN_LINE",0x04:"ROBOT_MODE_IN_ARCH",0x05:"ROBOT_MODE_IN_CATCH",\
	0x06:"ROBOT_MODE_IN_STAB",0x07:"ROBOT_MODE_IN_RETURN",0xfe:"ROBOT_MODE_IN_LOST",\
	0xff:"ROBOT_MODE_IN_EMPTY"};
#UAV1 起飞时间 线性搜索时间 抓捕时间 弓字型搜索时间 刺球时间 总时间
def caldata(UAV):
	last = 0;
	for i in range(1,UAV.iloc[:,0].size):
		if(UAV["situation"][i]!=UAV["situation"][i-1] ):
			for key in S:
				if(UAV["situation"][i-1] == key ):
					print(S[key],UAV["time"][i-1])
					# last = UAV["time"][i];
	
	print("UAV time ",UAV["time"][UAV.iloc[:,0].size-1]);
print("UAV1 time")
caldata(UAV1)
print("UAV2 time")
caldata(UAV2)
print("UAV3 time")
caldata(UAV3)
