#include "header.h"

using namespace std;

int main()
{
	//种一个随机种子
	srand(unsigned int(time(NULL)));
	
	//读取数据
	ReadData();
	//构建初始解
	CrtInitSol();

	//TS优化
	int gloTime = 5000,			//定义全局优化次数
		locTime = 200;			//定义局部优化次数
	GBestCost = SumCost(g_Solution);		//计算初始解，初始化最优解

	//计算初始SA温度
	double initT = -0.005*GBestCost /log(0.5), tk = initT;

	for (int i = 0; i < gloTime; ++i)
	{
		vector<VRut> LocalSol = g_Solution;

		for (int j = 0; j < locTime; ++j)
		{
			//任意选择一条路径中的任意一个客户点
			int cus = ChosOneCus(LocalSol);
			//随机加贪婪修复
			Insert(cus, LocalSol);

			float currCost = SumCost(LocalSol);
			if (currCost < GBestCost)
			{
				GBestCost = currCost;
				g_Solution = LocalSol;
				cout << currCost << endl;
			}
			else 
			{//使用模拟退火接收较差的解
				int pr = rand() % 101;//随机概率
				double gap = GBestCost - currCost;
				double pSA = exp(-gap/ tk);
				if (pSA * 100 <= pr)
				{//接受较次的解用于优化
					;
				}
				else
					LocalSol = g_Solution;//不接受次优解，仍用最优解进行优化
			}
		}
		//降温
		tk *= 0.99975;
		if (tk <= initT / 3.0)
		{//重新初始化温度
			initT = -0.005*GBestCost / log(0.5);
			tk = initT;
		}
		//单条路径破坏修复
		SingleRdel(LocalSol);
	}

	return 0;
}