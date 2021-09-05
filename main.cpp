#include "header.h"

using namespace std;

int main()
{
	//种一个随机种子
	srand(unsigned int(time(NULL)));

	//读取数据
	ReadData();

	//ACO优化
	int gloTime = 5000;		//全局优化次数
	int seqTime = 0;		//记录连续没有优化的次数
	InitPherom();			//初始化信息素矩阵和启发式信息矩阵

	for (int i = 0; i < gloTime; ++i)
	{
		InitAC();				//初始化蚁群
		float locBestCost = M;	//记录局部最优解
		int locBestIdx = -1;	//记录局部最优解的索引
		for (int j = 0; j < antNum; ++j)
		{//对于蚁群中每个蚂蚁分别构造解方案
			ConstructSol(antColn[j]);

			float preCost = CalCost(antColn[j]);
			antCost.push_back(preCost);
			if (preCost < GBestCost)
			{//记录全局最优解
				seqTime = 0;
				GBestCost = preCost;
				Solution = antColn[j];
				cout << "ACO Find better solution : "<< GBestCost << endl;
			}
			if (preCost < locBestCost)
			{//记录局部最优解
				locBestCost = preCost;
				locBestIdx = j;
			}
			++seqTime;
		}

		//更新信息素
		if (seqTime > 500)
		{//若连续100次没有优化，则重置信息素
			phero.clear();
			InitPherom();
			seqTime = 0;
			//对当前最优解进行LS优化
			LS_Opt();
			UpdateBestPherom();
		}
		else
			UpdatePherom(locBestIdx);



		//清空上次优化得到的蚁群解方案
		antColn.clear();
		antCost.clear();

	}
	


	return 0;
}