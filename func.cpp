#include "header.h"

using namespace std;

ofstream outPf;

/*********************************参数定义****************************************/
int cusNum;			//客户点数目
int pointNum;		//所有节点的数目
int carNum;			//卡车数目
float Q;			//卡车额定载重量
float altra;		//卡车实载率
float C;			//卡车行驶的变动成本
float V;			//卡车行驶的速度
float M;			//一个足够大的常数
float tau;			//惩罚成本系数

/**********************************集合定义***************************************/
vector<float> q(pointNum);		//客户点的需求量
vector<float> s(pointNum);		//客户点的服务时间
vector<float> e(pointNum);		//客户点的早时间窗
vector<float> l(pointNum);		//客户点的晚时间窗
vector<vector<float>> Dis(pointNum, vector<float>(pointNum, 0));		//客户点之间的距离矩阵

/******************************最优解相关定义************************************/
vector<VRut> Solution;			//一个解方案-一个蚂蚁

/*******************************ACO相关定义*************************************/
int antNum;						//每个蚁群中蚂蚁的个数
vector<vector<VRut>> antColn;	//蚁群
vector<float> antCost;			//记录蚁群中每个蚂蚁的成本
float alpha = 3;				//概率选择函数参数alpha
float beta = 2;					//概率选择函数参数beta
float rho = 0.01;				//信息素消失速率rho
float bestE = 2;				//局部最优路径的信息素更新权重
vector<vector<float>> phero;	//信息素矩阵
vector<vector<float>> eta;		//启发式信息矩阵

float GBestCost = 0;			//记录全局最优解

//读取数据
void ReadData()
{
	ifstream inPut("E:/桌面文件/研究生/精确算法/VRPTW_BCP/陈师姐算例/r111-25.txt");
	outPf.open("r110-25.txt");
	if (!inPut.is_open())
	{
		cerr << "读入文件打开文件失败！" << endl;
		exit(1);
	}
	if (!outPf.is_open())
	{
		cerr << "输出文件打开失败！" << endl;
		exit(1);
	}

	//导入参数信息
	inPut >> cusNum >> carNum >> Q >> altra >> C >> V >> M >> tau;
	pointNum = cusNum + 1;
	q.resize(pointNum, 0); s.resize(pointNum, 0); e.resize(pointNum, 0); l.resize(pointNum, 0); Dis.resize(pointNum);
	for (int i = 0; i < pointNum; ++i)
		Dis[i].resize(pointNum, 0);
	for (int i = 0; i < pointNum; ++i)
		inPut >> q[i];
	for (int i = 0; i < pointNum; ++i)
		inPut >> s[i];
	for (int i = 0; i < pointNum; ++i)
		inPut >> e[i];
	for (int i = 0; i < pointNum; ++i)
		inPut >> l[i];
	for (int i = 0; i < pointNum; ++i)
		for (int j = 0; j < pointNum; ++j)
			inPut >> Dis[i][j];

	antNum = cusNum;		//设定蚁群中蚂蚁的数量为客户点的个数
	GBestCost = M;			//定义全局最优解
	inPut.close();
}

//检查某条路径添加某个客户点之后是否可行
bool JgeVal(const VRut & preRut, int preNode)
{
	int bNode = preRut.Ruts.back();
	float atTime = preRut.AtTimes.back() + s[bNode] + Dis[bNode][preNode] / V;
	//检查时间窗
	if (atTime > l[preNode])
		return false;
	//检查载重量
	float weight = preRut.Loads + q[preNode];
	if (weight > Q)
		return false;
	return true;
}

//每一条路径添加一个客户点之后，更新路径的相关信息
bool UpdateInfo(VRut & preRut, int preNode)
{
	int bNode = preRut.Ruts.back();
	float atTime = preRut.AtTimes.back() + s[bNode] + Dis[bNode][preNode] / V;
	float weight = preRut.Loads + q[preNode];
	//检查时间窗 载重量
	if (!JgeVal(preRut, preNode))
		return false;

	//若载重量时间窗均满足要求，则更新路径信息
	preRut.Ruts.push_back(preNode);
	preRut.AtTimes.push_back(max(atTime,e[preNode]));
	preRut.Loads = weight;
	preRut.Cost += Dis[bNode][preNode];

	return true;
}

//初始化信息素矩阵和启发式信息矩阵
void InitPherom()
{
	phero.resize(pointNum);
	eta.resize(pointNum);
	for (int i = 0; i < pointNum; ++i)
	{
		phero[i].resize(pointNum, 0);
		eta[i].resize(pointNum, 0);
	}

	for (int i = 0; i < pointNum; ++i)
	{
		for (int j = 0; j < pointNum; ++j)
		{
			phero[i][j] = 1;
			if(Dis[i][j] != 0)
				eta[i][j] = 1 / Dis[i][j];
		}
	}
}

//初始化蚁群
void InitAC()
{
	vector<int> cusSet;
	for (int i = 1; i <= cusNum; ++i)
		cusSet.push_back(i);
	random_shuffle(cusSet.begin(),cusSet.end());

	for (int i = 0; i < antNum; ++i)
	{
		vector<VRut> t(1);
		antColn.push_back(t);
		for (int j = 0; j < (int)antColn[i].size(); ++j)
			UpdateInfo(antColn[i][j], cusSet[i]);
	}
}

//选择没有访问过的客户点集合
vector<int> FinduvCSet(const vector<VRut> & preAnt)
{
	//记录已经访问的客户点集合
	vector<int> vCusSet;
	for (int i = 0; i < (int)preAnt.size(); ++i)
	{
		for (int j = 1; j < (int)preAnt[i].Ruts.size(); ++j)
			vCusSet.push_back(preAnt[i].Ruts[j]);
	}
	vector<int> uvCusSet;//记录未访问的客户点集合
	for (int i = 1; i <= cusNum; ++i)
	{
		if(find(vCusSet.begin(),vCusSet.end(), i) == vCusSet.end())
			uvCusSet.push_back(i);
	}
	return uvCusSet;
}

//在未访问的客户点集合中找可行域客户点集合
vector<int> FindN(const VRut & preRut, const vector<int> &uvCusSet)
{
	vector<int> NSet;
	for (int i = 0; i < (int)uvCusSet.size(); ++i)
	{
		if (JgeVal(preRut, uvCusSet[i]))
			NSet.push_back(uvCusSet[i]);
	}
	return NSet;
}

//计算概率
float CalProb(int bNode, int pNode, const vector<int> &NSet)
{
	//计算分母
	float dSum = 0,		//记录分母的值
		mSum = 0;		//记录分子的值
	for (int i = 0; i < (int)NSet.size(); ++i)
		dSum += pow(phero[bNode][NSet[i]], alpha)*pow(eta[bNode][NSet[i]],beta);//累加分母

	mSum = pow(phero[bNode][pNode], alpha)*pow(eta[bNode][pNode], beta);		//计算分子
	
	if (mSum == 0 || dSum == 0) return 0;
	return mSum / dSum;
}

//选择应该拓展的客户点
int ChsNode(int bNode, const vector<int> &NSet)
{
	float bestProb = 0;		//记录最优概率(max)
	int bestNode = -1;		//记录最优客户点
	for (int i = 0; i < (int)NSet.size(); ++i)
	{
		float tP = CalProb(bNode, NSet[i], NSet);
		if (tP >= bestProb-1e-6)
		{
			bestProb = tP;
			bestNode = NSet[i];
		}

	}
	if (bestNode == -1)
	{//错误处理
		cout << "probalisity calculation error！" << endl;
		float tP = CalProb(bNode, NSet.back(), NSet);

		getchar();
	}
	return bestNode;
}

//为每一个蚂蚁构建解方案
void ConstructSol(vector<VRut> & preAnt)
{
	//选择出当前蚂蚁没有访问的客户点集合
	vector<int> uvCusSet = FinduvCSet(preAnt);

	for (int i = 0; i < (int)preAnt.size(); ++i)
	{
		if (!uvCusSet.size())
		{
			if (preAnt.back().Ruts.back() != 0)
				UpdateInfo(preAnt.back(), 0);
			return;
		}

		while (true)
		{//重复为当前路径插入客户点
			int bNode = preAnt[i].Ruts.back();
			if (!uvCusSet.size())
			{
				UpdateInfo(preAnt[i], 0);
				break;
			}

			//寻找当前路径最后一个客户点的可行域客户点集合
			vector<int> NSet = FindN(preAnt[i], uvCusSet);
			if (!NSet.size())
			{//若当前路径不能到达任意其余客户点，则返回场站,并且随机选择一个未访问的客户点新建一条路径
				UpdateInfo(preAnt[i], 0);
				preAnt.push_back(VRut());
				int randIdx = rand() % uvCusSet.size();
				UpdateInfo(preAnt.back(), uvCusSet[randIdx]);
				uvCusSet.erase(uvCusSet.begin() + randIdx);
				break;
			}

			//利用概率选择函数选择应该拓展的客户点
			int preCus = ChsNode(bNode, NSet);
			UpdateInfo(preAnt[i], preCus);


			uvCusSet.erase(find(uvCusSet.begin(), uvCusSet.end(),preCus));//删除已经访问的客户点
		}
	}

}

//计算每一个蚂蚁的成本
float CalCost(vector<VRut> preAnt)
{
	float cost = 0;
	for (int i = 0; i < (int)preAnt.size(); ++i)
		cost += preAnt[i].Cost;
	return cost;
}

//更新信息素信息
void UpdatePherom(int locBIdx)
{
	//先计算信息素消退效应
	for (int i = 0; i < pointNum; ++i)
		for (int j = 0; j < pointNum; ++j)
			phero[i][j] = (1 - rho)*phero[i][j];
	
	//再计算当前AC的影响
	for (int i = 0; i < antNum; ++i)
	{
		int rNum = (int)antColn[i].size();
		if (i != locBIdx)
		{//普通路径信息素更新
			for (int j = 0; j < rNum; ++j)
			{
				for (int k = 0; k < (int)antColn[i][j].Ruts.size()-1; ++k)
					phero[k][k + 1] += 1/antCost[i];//100
			}
		}
		else
		{//局部最优路径信息素更新
			for (int j = 0; j < rNum; ++j)
			{
				for (int k = 0; k < (int)antColn[i][j].Ruts.size() - 1; ++k)
					phero[k][k + 1] += bestE / antCost[i];//100
			}
		}
	}
}

//更新最优解信息素信息
void UpdateBestPherom()
{
	int rNum = (int)Solution.size();
	float bestCost = CalCost(Solution);
	for (int i = 0; i < rNum; ++i)
	{
		for (int k = 0; k < (int)Solution[i].Ruts.size() - 1; ++k)
			phero[k][k + 1] += bestE / bestCost;//100
	}
}

