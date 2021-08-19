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

	GBestCost = sumCost(Solution);//记录初始最优解

	int nbNum = 3,			//邻域算子的数量
		gIteNum = 500,		//规定外部迭代次数
		lIteNum = 100,		//规定每个邻域操作算子执行的次数
		i = 0;
	while (i < gIteNum)
	{
		int k = 1;
		while (k < nbNum)
		{
			//使用第k中邻域操作算子来优化当前解
			int j = 0;
			float lOptCost = 1000*M;//记录局部最优解
			while (j < lIteNum)
			{
				vector<VRut> resSol = ChsNbOpt(k);
				float preCost = sumCost(resSol);
				//int pRatio = rand() % 101;
				if (preCost < GBestCost)
				{
					Solution = resSol;
					GBestCost = preCost;
					lOptCost = -1;			//此轮无需更新局部最优解
					cout << "VNS Opt: " << preCost << endl;
				}

				if (lOptCost > 0 && preCost < lOptCost)
					lOptCost = preCost;
				
				++j;
			}

			//内循环结束之后，若没有对解进行优化，则执行LS进行局部优化
			//if (lOptCost > 0 && lOptCost < 1000 * M)
			//{//执行LS
			for (int i = 0; i < lIteNum; ++i)
			{
				vector<VRut> tSol = InterOpt1();
				float tCost = sumCost(tSol);
				if (tCost < GBestCost)
				{
					Solution = tSol;
					GBestCost = tCost;
					cout << "LS Opt: " << tCost << endl;
				}
			}
			//}
			//else		//若对解进行了优化，则重新从那个第一个邻域操作算子开始执行优化
			//	break;
			if (lOptCost < 0)
				k = 0;
			
			++k;
		}

		//整条路径进行破坏
		vector<VRut> tSol = Solution;
		SingleRdel(tSol);

		++i;
	}
	return 0;
}