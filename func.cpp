#include "header.h"

using namespace std;

ofstream outPf;

/*********************************参数定义****************************************/
int cusNum;			//客户点数目
int pointNum;		//所有节点的数目
int carNum;			//卡车数目
float Q;			//卡车额定载重量
float alpha;		//卡车实载率
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
vector<VRut> g_Solution;			//全局最优解方案

/******************************TS相关定义************************************/
vector<vector<int>> Tabu;		//设置禁忌表

float GBestCost = 0;				//记录全局最优解

//读取数据
void ReadData()
{
	ifstream inPut("E:/桌面文件/研究生/精确算法/VRPTW_BCP/陈师姐算例/r111-25.txt");
	outPf.open("E:/桌面文件/研究生/精确算法/VRPTW_BCP/输出文件/r111-25.txt");
	//ifstream inPut("E:/桌面文件/研究生/精确算法/VRPTW_BCP/测试算例/VRPTW_18.txt");
	//outPf.open("E:/桌面文件/研究生/精确算法/VRPTW_BCP/输出文件/VRPTW-18.txt");
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
	inPut >> cusNum >> carNum >> Q >> alpha >> C >> V >> M >> tau;
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


	inPut.close();
}

//创建初始解
void CrtInitSol()
{//使用顺序插入的方法生成初始解
	int j = 1;
	for (int i = 0; i < carNum; ++i)
	{
		VRut vR;				//新建一条路径
		for (; j <= cusNum; ++j)
		{
			//判断路径可行性
			if (!JgeRutValy(j, (int)vR.Ruts.size(), vR))
			{
				BackToDpt(vR, j);
				break;
			}
			else
			{
				vR.Cost += Dis[j-1][j];
				vR.Ruts.push_back(j);
				vR.Loads += q[j];
			}
		}

		if (j > cusNum)
		{
			if (BackToDpt(vR, j))
			{
				g_Solution.push_back(vR);
				break;//能返回场站则初始解生成完毕，否则将最后一个客户点重新生成一条路径
			}
		}
		g_Solution.push_back(vR);
	}

}

//判断车辆能否返回场站
bool BackToDpt(VRut &vR, int &j)
{//让车辆返回场站
	float atTime = vR.AtTimes.back() + s[vR.Ruts.back()] + Dis[vR.Ruts.back()][0] / V;
	if (atTime < l[0])
	{//车辆可以返回场站
		vR.AtTimes.push_back(atTime);
		vR.Ruts.push_back(0);
		vR.Cost += Dis[j-1][0];
		return true;
	}
	else
	{//不能返回场站，则去掉一个客户点，之后再返回场站
		vR.Ruts.pop_back();
		atTime = vR.AtTimes.back() + s[vR.Ruts.back()] + Dis[vR.Ruts.back()][0] / V;
		vR.AtTimes.push_back(atTime);
		vR.Ruts.push_back(0);
		vR.Cost += Dis[j-2][0];
		--j;
		return false;
	}
}

//将某一个客户点插入到某一条路径中的某一个位置，判断路径可行性
bool JgeRutValy(int cus, int pos, VRut & vR)
{
	//首先判断载重量
	float newLoad = vR.Loads + q[cus];
	if (newLoad > Q) return false;

	//判断时间窗
	if(!UpDateTime(cus, pos, vR)) return false;
	return true;
}

//将某一个客户点插入到某一条路径中的某一个位置，更新插入点之后的客户点的到达时间，并判断路径可行性
bool UpDateTime(int cus, int pos, VRut & vR)
{
	//首先判断是否会超过新插入客户点的晚时间窗
	int lastCus = vR.Ruts[pos - 1];
	float atTime = vR.AtTimes[pos - 1] + s[lastCus] + Dis[lastCus][cus] / V;
	if (atTime > l[cus]) return false;//若超过晚时间窗，则无需更新

	atTime = max(atTime, e[cus]);	  //不考虑等待时间成本，车辆能够不超过晚时间窗到达就行

	//若是插入到尾部，则直接更新新客户点的到达时间
	if (pos == (int)vR.AtTimes.size())
	{
		vR.AtTimes.push_back(atTime);
		return true;
	}
	else
	{//插入到路径中间的情况,使用递归进行计算
		if (vR.Ruts[pos] != cus)
		{
			vR.Ruts.insert(vR.Ruts.begin() + pos, cus);
			vR.AtTimes.insert(vR.AtTimes.begin() + pos, atTime);
		}
		else
			vR.AtTimes[pos] = atTime;
		//更新后续客户点的到达时间
		int i = pos + 1;

		if (i == vR.Ruts.size())
			return true;
		
		if(!UpDateTime(vR.Ruts[i], i, vR))		//递归计算
			return false;					

		return true;
	}
}

//任意选择一条路径中的任意一个客户点
int ChosOneCus(vector<VRut> &tempSol)
{
	int rIdx = rand() % (int)tempSol.size(),						//随机选择一条路径
		cIdx = (rand() % ((int)tempSol[rIdx].Ruts.size() - 2)) + 1,//随机选择一个非场站的客户点
		objCus = tempSol[rIdx].Ruts[cIdx];

	//更新禁忌表
	if (Tabu.size() == MAXTABU)
		Tabu.erase(Tabu.begin());
	Tabu.push_back({ rIdx, tempSol[rIdx].Ruts[cIdx] });			//哪条路径中的某选中的客户点


	tempSol[rIdx].Ruts.erase(tempSol[rIdx].Ruts.begin() + cIdx);		//将选中的客户点删掉
	tempSol[rIdx].AtTimes.erase(tempSol[rIdx].AtTimes.begin() + cIdx);//将对应的到达时间删除
	if (tempSol[rIdx].Ruts.size() == 2)								//若路径中没有客户点了，则将路径也删除
	{
		//更新禁忌表中的路径索引
		for (int i = 0; i < (int)Tabu.size(); ++i)
		{
			if (Tabu[i][0] == rIdx)
				Tabu.erase(Tabu.begin() + i);//删除对应的Tabu记录
			else if (Tabu[i][0] > rIdx)
				--Tabu[i][0];				 //遇到大于rIdx的路径索引需要进行递减
		}
		tempSol.erase(tempSol.begin() + rIdx);

	}
	
	return objCus;
}

//插入信息结构体
struct InsertInfo
{
	float cost; //插入成本
	int rutIdx; //插入路径
	int cusIdx; // 插入位置
	bool tabuFlag = false;//记录当前修复方式是否违反禁忌表，若是，则为true
};

//插入信息结构体排序
bool Mycomp(InsertInfo &info1, InsertInfo &info2)
{
	return info1.cost < info2.cost;
}

//将客户点插入路径中
void InsertCusToRut(vector<VRut> &tSolution, int preRIdx, int preCusIdx, int cus, const float &preInstCost)
{
	tSolution[preRIdx].Ruts.insert(tSolution[preRIdx].Ruts.begin() + preCusIdx, cus);
	tSolution[preRIdx].Loads += q[cus];
	tSolution[preRIdx].Cost += preInstCost;
	UpDateTime(cus, preCusIdx, tSolution[preRIdx]);
}

//随机加贪婪插入修复
void Insert(int cus, vector<VRut> &tempSol)
{
	//在当前解方案中随机选择3/5的路径进行贪婪插入对比
	int n = (int)tempSol.size();				//当前解方案的路径数
	int rCnt = (int)floor(5.0*n/5.0);
	vector<int> rIdx;
	for (int i = 0; i < n; ++i)
		rIdx.push_back(i);

	random_device rd;
	mt19937 engine2(rd());
	shuffle(rIdx.begin(), rIdx.end(), engine2);		//随机打乱解方案序列
	

	int bestRIdx = -1,
		bestCIdx = -1;
	float bestCost = M;
	for (int i = 0; i < rCnt; ++i)
	{//遍历每一条路径
		for (int j = 0; j < tempSol[i].Ruts.size() - 1; ++j)
		{//遍历每一个插入位置
			float insertCost = CalCost(cus, j + 1, tempSol[i]);
			if (insertCost < bestCost)
			{
				bestCost = insertCost;
				bestRIdx = i;
				bestCIdx = j + 1;
			}
		}
	}
	//将客户点插入到最优路径的最优位置
	if (bestRIdx != -1)
	{//若能够找到某条路径进行插入
		//检查当前操作是否在禁忌表中
		for (int i = 0; i < (int)Tabu.size(); ++i)
		{
			if (Tabu[i][0] == bestRIdx && Tabu[i][1] == cus)
			{
				//检查是否禁忌操作可以产生优于当前最优解的解
				vector<VRut> tSolution = tempSol;	//临时记录最优解
				tSolution[bestRIdx].Ruts.insert(tSolution[bestRIdx].Ruts.begin() + bestCIdx, cus);
				tSolution[bestRIdx].Loads += q[cus];
				tSolution[bestRIdx].Cost += bestCost;
				UpDateTime(cus, bestCIdx, tSolution[bestRIdx]);
				float currCost = SumCost(tempSol);	//计算当前解
				if (currCost < GBestCost)
				{//若是，则将禁忌操作释放，并记录最优解
					Tabu.erase(Tabu.begin() + i);
					tempSol = tSolution;
					return;
				}
				else
				{//若不是，则选择一条禁忌表之外的路径，将客户点随机插入
					CrtNewRut(cus, tempSol);
					return;
				}
			}
		}
		//若操作不在禁忌表中
		tempSol[bestRIdx].Ruts.insert(tempSol[bestRIdx].Ruts.begin() + bestCIdx, cus);
		tempSol[bestRIdx].Loads += q[cus];
		tempSol[bestRIdx].Cost += bestCost;
		UpDateTime(cus, bestCIdx, tempSol[bestRIdx]);
	}
	else//新建一条路径
		CrtNewRut(cus, tempSol);
}

//计算将客户点插入到某条路径的某个位置的变动成本
float CalCost(int cus, int pos, VRut &vR)
{
	//首先判断载重量
	float newLoad = vR.Loads + q[cus];
	if (newLoad > Q) return M;					//路径不可行，变动成本设为极大值

	//判断时间窗
	VRut tvR = vR;
	if (!UpDateTime(cus, pos, tvR)) return M;	//路径不可行，变动成本设为极大值

	//计算变动成本	
	int pCus = vR.Ruts[pos - 1],						//记录插入点之前的客户点
		fCus = vR.Ruts[pos];							//记录插入点之后的客户点
	float oCost = Dis[pCus][fCus],								//记录路径之前的成本
		vCost = Dis[pCus][cus] + Dis[cus][fCus] - oCost;//记录变动成本
	return vCost;
}

//创建一条新路径
void CrtNewRut(int cus, vector<VRut> &tempSol)
{
	VRut newVr;
	newVr.Ruts.push_back(cus);
	newVr.Ruts.push_back(0);
	newVr.AtTimes.push_back(e[cus]);
	newVr.AtTimes.push_back(e[cus]+s[cus]+Dis[cus][0]/V);
	newVr.Loads += q[cus];
	newVr.Cost = 2 * Dis[0][cus];
	tempSol.push_back(newVr);
}

//计算当前解的成本
float SumCost(vector<VRut> &tempSol)
{
	float sCost = 0;
	for (int i = 0; i < (int)tempSol.size(); ++i)
		sCost += tempSol[i].Cost;
	return sCost;
}

//选择当前解中客户点最多的路径
int ChsMRut(vector<VRut> &tempSol)
{
	int rutIdx = 0;
	int maxSize = 0;
	for (int i = 0; i < (int)tempSol.size(); ++i)
	{
		if (tempSol.size() > maxSize)
		{
			maxSize = (int)tempSol.size();
			rutIdx = i;
		}
	}
	return rutIdx;
}

//选择当前解中客户点最少的路径
int ChSRut(vector<VRut> &tempSol)
{
	int rutIdx = 0;
	int minSize = (int)M;
	for (int i = 0; i < (int)tempSol.size(); ++i)
	{
		if (tempSol.size() < minSize)
		{
			minSize = (int)tempSol.size();
			rutIdx = i;
		}
	}
	return rutIdx;
}

//整条路径破坏和修复
void SingleRdel(vector<VRut> &tempSol)
{
	//选择客户点最少的路径
	int rIdx = rand()% tempSol.size();

	vector<int> cusSet;
	for (int i = 1; i < (int)tempSol[rIdx].Ruts.size() - 1; ++i)
		cusSet.push_back(tempSol[rIdx].Ruts[i]);
	//更新禁忌表路径索引
	for (int i = 0; i < (int)Tabu.size(); ++i)
	{
		if (Tabu[i][0] == rIdx)
			Tabu.erase(Tabu.begin() + i);//删除对应的Tabu记录
		else if (Tabu[i][0] > rIdx)
			--Tabu[i][0];				 //遇到大于rIdx的路径索引需要进行递减
	}
	tempSol.erase(tempSol.begin() + rIdx);

	
	for (int i = 0; i < (int)cusSet.size(); ++i)
		Insert(cusSet[i], tempSol);

	//判断最优解成本
	float currCost = SumCost(tempSol);
	if (currCost < GBestCost)
	{
		GBestCost = currCost;
		g_Solution = tempSol;
		cout << currCost << endl;
	}
}

