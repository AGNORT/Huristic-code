#include "header.h"

using namespace std;

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
vector<VRut> Solution;			//解方案

/******************************TS相关定义************************************/
//vector<vector<int>> Tabu;		//设置禁忌表

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
				vR.Cost += Dis[j - 1][j];
				vR.Ruts.push_back(j);
				vR.Loads += q[j];
			}
		}

		if (j > cusNum)
		{
			if (BackToDpt(vR, j))
			{
				Solution.push_back(vR);
				break;//能返回场站则初始解生成完毕，否则将最后一个客户点重新生成一条路径
			}
		}
		Solution.push_back(vR);
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
		vR.Cost += Dis[j - 1][0];
		return true;
	}
	else
	{//不能返回场站，则去掉一个客户点，之后再返回场站
		vR.Ruts.pop_back();
		atTime = vR.AtTimes.back() + s[vR.Ruts.back()] + Dis[vR.Ruts.back()][0] / V;
		vR.AtTimes.push_back(atTime);
		vR.Ruts.push_back(0);
		vR.Cost += Dis[j - 2][0];
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
	if (!UpDateTime(cus, pos, vR)) return false;
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

		if (!UpDateTime(vR.Ruts[i], i, vR))		//递归计算
			return false;

		return true;
	}
}

//任意选择一条路径中的任意一个客户点
int ChosOneCus(vector<VRut> &resSolution)
{
	int rIdx = rand() % (int)resSolution.size(),						//随机选择一条路径
		cIdx = (rand() % ((int)resSolution[rIdx].Ruts.size() - 2)) + 1,//随机选择一个非场站的客户点
		objCus = resSolution[rIdx].Ruts[cIdx];

	////更新禁忌表
	//if (Tabu.size() == MAXTABU)
	//	Tabu.erase(Tabu.begin());
	//Tabu.push_back({ rIdx, Solution[rIdx].Ruts[cIdx] });			//哪条路径中的某选中的客户点


	resSolution[rIdx].Ruts.erase(resSolution[rIdx].Ruts.begin() + cIdx);		//将选中的客户点删掉
	resSolution[rIdx].AtTimes.erase(resSolution[rIdx].AtTimes.begin() + cIdx);//将对应的到达时间删除
	if (resSolution[rIdx].Ruts.size() == 2)								//若路径中没有客户点了，则将路径也删除
	{
		////更新禁忌表中的路径索引
		//for (int i = 0; i < (int)Tabu.size(); ++i)
		//{
		//	if (Tabu[i][0] == rIdx)
		//		Tabu.erase(Tabu.begin() + i);//删除对应的Tabu记录
		//	else if (Tabu[i][0] > rIdx)
		//		--Tabu[i][0];				 //遇到大于rIdx的路径索引需要进行递减
		//}

		resSolution.erase(resSolution.begin() + rIdx);

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

//随机加贪婪插入修复
vector<VRut> Insert(int cus, vector<VRut> &resSolution)
{
	//在当前解方案中随机选择3/5的路径进行贪婪插入对比
	int n = (int)resSolution.size();				//当前解方案的路径数
	int rCnt = (int)floor(5.0*n / 5.0);
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
		for (int j = 0; j < (int)resSolution[i].Ruts.size() - 1; ++j)
		{//遍历每一个插入位置
			float insertCost = CalCost(cus, j + 1, resSolution[i]);
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

		////检查当前操作是否在禁忌表中
		//for (int i = 0; i < (int)Tabu.size(); ++i)
		//{
		//	if (Tabu[i][0] == bestRIdx && Tabu[i][1] == cus)
		//	{
		//		//检查是否禁忌操作可以产生优于当前最优解的解
		//		vector<VRut> tSolution = Solution;	//临时记录最优解
		//		tSolution[bestRIdx].Ruts.insert(tSolution[bestRIdx].Ruts.begin() + bestCIdx, cus);
		//		tSolution[bestRIdx].Loads += q[cus];
		//		tSolution[bestRIdx].Cost += bestCost;
		//		UpDateTime(cus, bestCIdx, tSolution[bestRIdx]);
		//		float currCost = sumCost();	//计算当前解
		//		if (currCost < GBestCost)
		//		{//若是，则将禁忌操作释放，并记录最优解
		//			Tabu.erase(Tabu.begin() + i);
		//			Solution = tSolution;
		//			return;
		//		}
		//		else
		//		{//若不是，则选择一条禁忌表之外的路径，将客户点随机插入
		//			CrtNewRut(cus);
		//			return;
		//		}
		//	}
		//}
		//若操作不在禁忌表中
		resSolution[bestRIdx].Ruts.insert(resSolution[bestRIdx].Ruts.begin() + bestCIdx, cus);
		resSolution[bestRIdx].Loads += q[cus];
		resSolution[bestRIdx].Cost += bestCost;
		UpDateTime(cus, bestCIdx, resSolution[bestRIdx]);
	}
	else//新建一条路径
		CrtNewRut(cus, resSolution);
	return resSolution;
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
void CrtNewRut(int cus, vector<VRut> &preSolution)
{
	VRut newVr;
	newVr.Ruts.push_back(cus);
	newVr.Ruts.push_back(0);
	newVr.AtTimes.push_back(e[cus]);
	newVr.AtTimes.push_back(e[cus] + s[cus] + Dis[cus][0] / V);
	newVr.Loads += q[cus];
	newVr.Cost = 2 * Dis[0][cus];
	preSolution.push_back(newVr);
}

//计算当前解的成本
float sumCost(vector<VRut> &preSolution)
{
	float sCost = 0;
	for (int i = 0; i < (int)preSolution.size(); ++i)
		sCost += preSolution[i].Cost;
	return sCost;
}

//选择当前解中客户点最多的路径
int ChsMRut()
{
	int rutIdx = 0;
	int maxSize = 0;
	for (int i = 0; i < (int)Solution.size(); ++i)
	{
		if ((int)Solution.size() > maxSize)
		{
			maxSize = (int)Solution.size();
			rutIdx = i;
		}
	}
	return rutIdx;
}

//选择当前解中客户点最少的路径
int ChSRut()
{
	int rutIdx = 0;
	int minSize = (int)M;
	for (int i = 0; i < (int)Solution.size(); ++i)
	{
		if ((int)Solution.size() < minSize)
		{
			minSize = (int)Solution.size();
			rutIdx = i;
		}
	}
	return rutIdx;
}

//整条路径破坏和修复
void SingleRdel(vector<VRut> &resSolution)
{
	//选择客户点最少的路径
	int rIdx = rand() % resSolution.size();

	vector<int> cusSet;
	for (int i = 1; i < (int)resSolution[rIdx].Ruts.size() - 1; ++i)
		cusSet.push_back(resSolution[rIdx].Ruts[i]);
	////更新禁忌表路径索引
	//for (int i = 0; i < (int)Tabu.size(); ++i)
	//{
	//	if (Tabu[i][0] == rIdx)
	//		Tabu.erase(Tabu.begin() + i);//删除对应的Tabu记录
	//	else if (Tabu[i][0] > rIdx)
	//		--Tabu[i][0];				 //遇到大于rIdx的路径索引需要进行递减
	//}
	resSolution.erase(resSolution.begin() + rIdx);


	for (int i = 0; i < (int)cusSet.size(); ++i)
		Insert(cusSet[i], resSolution);

	//判断最优解成本
	float currCost = sumCost(resSolution);
	if (currCost < GBestCost)
	{
		GBestCost = currCost;
		Solution = resSolution;
		cout << "Single Opt: " << currCost << endl;
	}
}

//选择出当前解路径中客户点数量大于2个的客户点数量，即总节点数量大于4个
vector<int> ChsRutM2()
{
	vector<int> rutIdx;
	int solNum = (int)Solution.size();
	for (int i = 0; i < solNum; ++i)
	{
		if ((int)Solution[i].Ruts.size() >= 4)
			rutIdx.push_back(i);
	}
	return rutIdx;
}

//计算某条路径的成本
float CalRutCost(const vector<int> &rut)
{
	int rSize = (int)rut.size();
	float rCost = 0;
	for (int i = 1; i < rSize; ++i)
		rCost += Dis[rut[i-1]][rut[i]];
	return rCost;
}

//计算某条路径的载重量
float CalRutWgt(const vector<int> &rut)
{
	float rWgt = 0;
	int rSize = (int)rut.size();
	for (int i = 1; i < rSize-1; ++i)
		rWgt += q[rut[i]];
	return rWgt;
}

//2-opt intra-route优化，随机选择某条路径中的两个客户点进行互换，之后判断互换之后时间窗约束
vector<VRut> IntraOpt2()
{
	vector<VRut> resSolution = Solution;

	vector<int> rutIdx = ChsRutM2();
	if (!rutIdx.size()) return vector<VRut>();//没有可以进行2 intra-route操作的路径

	int rIdx = rutIdx[rand() % rutIdx.size()];		//选择出用于操作的路径索引
	vector<int> tSet;
	for (int i = 1; i <= (int)resSolution[rIdx].Ruts.size() - 2; ++i)
		tSet.push_back(i);

	random_device rd;
	mt19937 engine2(rd());
	shuffle(tSet.begin(), tSet.end(), engine2);

	int cIdx1 =min(tSet[0],tSet[1]), cIdx2 = max(tSet[0], tSet[1]);	//选择出用于互换的两个客户点的索引
	int cus1 = resSolution[rIdx].Ruts[cIdx1], cus2 = resSolution[rIdx].Ruts[cIdx2];

	VRut tRut = resSolution[rIdx];
	tRut.Ruts[cIdx2] = cus1;
	tRut.Ruts.erase(tRut.Ruts.begin()+cIdx1);
	int rSize1 = (int)tRut.Ruts.size();
	tRut.AtTimes.erase(tRut.AtTimes.begin()+cIdx1);

	if (!UpDateTime(cus2, cIdx1, tRut))
	{
		/*
		int rSize2 = (int)tRut.Ruts.size();
		
		//若时间窗不满足要求则将选出的客户点新建两条路径，并将用于操作的路径进行更新
		VRut tRut = resSolution[rIdx];
		if ((int)tRut.Ruts.size() == 4)
			resSolution.erase(resSolution.begin() + rIdx);//若操作的路径只有两个客户点，则直接将路径删除即可
		else
		{
			tRut.Ruts.erase(tRut.Ruts.begin() + cIdx2);
			tRut.AtTimes.erase(tRut.AtTimes.begin() + cIdx2);
			if (rSize2 > rSize1)
			{//UpDateTime中更新了路径序列才需要将cIdx1位置的客户点删去
				tRut.Ruts.erase(tRut.Ruts.begin() + cIdx1);
				tRut.AtTimes.erase(tRut.AtTimes.begin() + cIdx1);
			}
			int tCus = tRut.Ruts[1];
			tRut.Ruts.erase(tRut.Ruts.begin() + 1);
			tRut.AtTimes.erase(tRut.AtTimes.begin() + 1);
			UpDateTime(tCus,1,tRut);		
			tRut.Loads = CalRutWgt(tRut.Ruts);
			tRut.Cost = CalRutCost(tRut.Ruts);
			resSolution[rIdx] = tRut;

		}
		//新建路径
		CrtNewRut(cus1, resSolution);
		CrtNewRut(cus2, resSolution);
		return resSolution;
		*/
		resSolution.push_back(VRut(1000*M));
		return resSolution;
	}
		
	tRut.Cost = CalRutCost(tRut.Ruts);		   //更新路径成本

	resSolution[rIdx] = tRut;
	return resSolution;
}

//某个解中某路径删除一个点之后插入另外一个点
void DelInst(vector<VRut>&resSolution, int rIdx, int cIdx, int instCus)
{
	
	VRut tRut = resSolution[rIdx];
	int cus = resSolution[rIdx].Ruts[cIdx];

	tRut.Ruts.erase(tRut.Ruts.begin() + cIdx);
	int rSize1 = (int)tRut.Ruts.size();
	tRut.AtTimes.erase(tRut.AtTimes.begin() + cIdx);
	if (!UpDateTime(instCus, cIdx, tRut))
	{
		/*
		int rSize2 = (int)tRut.Ruts.size();
		if (rSize2 > rSize1)
		{
			tRut.Ruts.erase(tRut.Ruts.begin() + cIdx);
			tRut.AtTimes.erase(tRut.AtTimes.begin() + cIdx);
		}
		if (tRut.Ruts.size() == 2)
			resSolution.erase(resSolution.begin() + rIdx);
		int tCus = tRut.Ruts[1];
		tRut.Ruts.erase(tRut.Ruts.begin() + 1);
		tRut.AtTimes.erase(tRut.AtTimes.begin() + 1);
		UpDateTime(tCus, 1, tRut);
		tRut.Loads -= q[cus];
		tRut.Cost = CalRutCost(tRut.Ruts);
		resSolution[rIdx] = tRut;

		CrtNewRut(instCus, resSolution);
		*/
		resSolution.push_back(VRut(1000 * M));
		
	}
	else
	{
		tRut.Loads = tRut.Loads - q[cus] + q[instCus];
		tRut.Cost = CalRutCost(tRut.Ruts);
		resSolution[rIdx] = tRut;
	}
}

//2-opt inter-route优化，随机选择某两条路径中的两个客户点进行互换，之后判断互换之后的时间窗和载重量约束
vector<VRut> InterOpt2()
{
	vector<VRut> resSolution = Solution;
	vector<int> tSet;
	for (int i = 0; i < (int)resSolution.size(); ++i)
		tSet.push_back(i);

	random_device rd;
	mt19937 engine2(rd());
	shuffle(tSet.begin(), tSet.end(), engine2);

	int rIdx1 = tSet[0], rIdx2 = tSet[1];						  //选出两条路径的索引

	int cIdx1 = rand() % (resSolution[rIdx1].Ruts.size() - 2) + 1,//选出用于交换的客户点的索引
		cIdx2 = rand() % (resSolution[rIdx2].Ruts.size() - 2) + 1,
		cus1 = resSolution[rIdx1].Ruts[cIdx1],
		cus2 = resSolution[rIdx2].Ruts[cIdx2];

	VRut tRut1 = resSolution[rIdx1],
		tRut2 = resSolution[rIdx2];

	DelInst(resSolution, rIdx1, cIdx1, cus2);
	DelInst(resSolution, rIdx2, cIdx2, cus1);


	return resSolution;
}

//1-opt inter-route优化，随机选择一个客户点，在当前解所有的路径中寻找最优位置进行插入，若不能插入，则新建一条路径
vector<VRut> InterOpt1()
{
	vector<VRut> resSolution = Solution;

	//任意选择一条路径中的任意一个客户点
	int cus = ChosOneCus(resSolution);
	//随机加贪婪修复
	return Insert(cus, resSolution);
}

//选择邻域操作算子
vector<VRut> ChsNbOpt(int i)
{
	switch (i)
	{
	case 0:
		return InterOpt1();
		break;
	case 1:
		return InterOpt2();
		break;
	case 2:
		return IntraOpt2();
		break;
	}
}