#include "header.h"

using namespace std;


int main()
{
	//��һ���������
	srand(unsigned int(time(NULL)));

	//��ȡ����
	ReadData();
	//������ʼ��
	CrtInitSol();

	GBestCost = sumCost(Solution);//��¼��ʼ���Ž�

	int nbNum = 3,			//�������ӵ�����
		gIteNum = 500,		//�涨�ⲿ��������
		lIteNum = 100,		//�涨ÿ�������������ִ�еĴ���
		i = 0;
	while (i < gIteNum)
	{
		int k = 1;
		while (k < nbNum)
		{
			//ʹ�õ�k����������������Ż���ǰ��
			int j = 0;
			float lOptCost = 1000*M;//��¼�ֲ����Ž�
			while (j < lIteNum)
			{
				vector<VRut> resSol = ChsNbOpt(k);
				float preCost = sumCost(resSol);
				//int pRatio = rand() % 101;
				if (preCost < GBestCost)
				{
					Solution = resSol;
					GBestCost = preCost;
					lOptCost = -1;			//����������¾ֲ����Ž�
					cout << "VNS Opt: " << preCost << endl;
				}

				if (lOptCost > 0 && preCost < lOptCost)
					lOptCost = preCost;
				
				++j;
			}

			//��ѭ������֮����û�жԽ�����Ż�����ִ��LS���оֲ��Ż�
			//if (lOptCost > 0 && lOptCost < 1000 * M)
			//{//ִ��LS
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
			//else		//���Խ�������Ż��������´��Ǹ���һ������������ӿ�ʼִ���Ż�
			//	break;
			if (lOptCost < 0)
				k = 0;
			
			++k;
		}

		//����·�������ƻ�
		vector<VRut> tSol = Solution;
		SingleRdel(tSol);

		++i;
	}
	return 0;
}