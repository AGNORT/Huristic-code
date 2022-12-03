#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <algorithm>
#include <time.h>
#include <random>

extern std::ofstream outPf;

/*********************************参数定义****************************************/
extern int cusNum;			//客户点数目
extern int pointNum;		//所有节点的数目
extern int carNum;			//卡车数目
extern float Q;			//卡车额定载重量
extern float alpha;		//卡车实载率
extern float C;			//卡车行驶的变动成本
extern float V;			//卡车行驶的速度
extern float M;			//一个足够大的常数
extern float tau;			//惩罚成本系数

/**********************************集合定义***************************************/
extern std::vector<float> q;				//客户点的需求量
extern std::vector<float> s;				//客户点的服务时间
extern std::vector<float> e;				//客户点的早时间窗
extern std::vector<float> l;				//客户点的晚时间窗
extern std::vector<std::vector<float>> Dis;		//客户点之间的距离矩阵

/******************************TS相关定义************************************/
//extern std::vector<std::vector<int>> Tabu;	//设置禁忌表<路径索引，客户点编号>不允许某个客户点被插入到某条路径中，防止重复搜索

extern float GBestCost;						//记录全局最优解


/*最优解相关定义*/
//一条车辆路径
class VRut
{
public:
	std::vector<int> Ruts;			//当前最优路径
	std::vector<float> AtTimes;		//当前最优路径对应的车辆出发时间
	float Loads;					//车辆的载重量
	float Cost;						//车辆路径的成本
	//float RTimes;					//剩余等待时间集合 不考虑等待时间成本
	VRut()
	{//初始化路径
		Ruts.push_back(0);
		AtTimes.push_back(0);
		Loads = 0;
		Cost = 0;
	}
	VRut(float C)
	{
		Cost = C;
	}
};
extern std::vector<VRut> Solution;

/**********************************函数声明***************************************/
//读取数据
void ReadData();

//创建初始解
void CrtInitSol();

//判断车辆能否返回场站
bool BackToDpt(VRut &vR, int &j);

//将某一个客户点插入到某一条路径中的某一个位置，判断路径可行性
bool JgeRutValy(int cus, int pos, VRut & vR);

//将某一个客户点插入到某一条路径中的某一个位置，更新插入点之后的客户点的到达时间，并判断路径可行性
bool UpDateTime(int cus, int pos, VRut & vR);

//任意选择一条路径中的任意一个客户点
int ChosOneCus(std::vector<VRut> &resSolution);

//随机加贪婪插入修复
std::vector<VRut> Insert(int cus, std::vector<VRut> &resSolution);

//计算将客户点插入到某条路径的某个位置的变动成本
float CalCost(int cus, int pos, VRut &vR);

//创建一条新路径
void CrtNewRut(int cus, std::vector<VRut> &preSolution);

//计算当前解的成本
float sumCost(std::vector<VRut> &preSolution);

//整条路径破坏和修复
void SingleRdel(std::vector<VRut> &resSolution);

//2-opt intra-route优化，随机选择某条路径中的两个客户点进行互换，之后判断互换之后时间窗约束
std::vector<VRut> IntraOpt2();

//1-opt inter-route优化，随机选择一个客户点，在当前解所有的路径中寻找最优位置进行插入，若不能插入，则新建一条路径
std::vector<VRut> InterOpt1();

//2-opt inter-route优化，随机选择某两条路径中的两个客户点进行互换，之后判断互换之后的时间窗和载重量约束
std::vector<VRut> InterOpt2();

//选择邻域操作算子
std::vector<VRut> ChsNbOpt(int i);