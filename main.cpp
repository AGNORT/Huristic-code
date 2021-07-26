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
	GBestCost = sumCost();		//计算初始解，初始化最优解

	for (int i = 0; i < gloTime; ++i)
	{
		for (int j = 0; j < locTime; ++j)
		{
			//任意选择一条路径中的任意一个客户点
			int cus = ChosOneCus();
			//随机加贪婪修复
			Insert(cus);

			float currCost = sumCost();
			if (currCost < GBestCost)
			{
				GBestCost = currCost;
				cout << currCost << endl;
			}
		}
		//单条路径破坏修复
		SingleRdel();
	}

	return 0;
}
