#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <algorithm>
#include <time.h>
#include <random>
#include <iomanip>


#define MAXTABU 5		//设置最大禁忌步长

extern std::ofstream outPf;

/*********************************参数定义****************************************/
extern int cusNum;			//客户点数目
extern int pointNum;		//所有节点的数目
extern int carNum;			//卡车数目
extern float Q;			//卡车额定载重量
extern float altra;		//卡车实载率
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

/*******************************ACO相关定义*************************************/
extern int antNum;						//每个蚁群中蚂蚁的个数
extern float alpha;						//概率选择函数参数alpha
extern float beta;						//概率选择函数参数beta
extern float rho;						//信息素消失速率rho
extern float bestE;						//局部最优路径的信息素更新权重
extern std::vector<float> antCost;		//记录蚁群中每个蚂蚁的成本
extern std::vector<std::vector<float>> phero;		//信息素矩阵
extern std::vector<std::vector<float>> eta;			//启发式信息矩阵

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
};
extern std::vector<VRut> Solution;
extern std::vector<std::vector<VRut>> antColn;	//蚁群


/**********************************函数声明***************************************/
//读取数据
void ReadData();

//初始化信息素矩阵
void InitPherom();

//初始化蚁群位置
void InitAC();

//为每一个蚂蚁构建解方案
void ConstructSol(std::vector<VRut> & preAnt);

//计算每一个蚂蚁的成本
float CalCost(std::vector<VRut> preAnt);

//更新信息素信息
void UpdatePherom(int locBIdx);

//更新最优解信息素信息
void UpdateBestPherom();

//LS优化
void LS_Opt();