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

	//TS�Ż�
	int gloTime = 5000,			//����ȫ���Ż�����
		locTime = 200;			//����ֲ��Ż�����
	GBestCost = SumCost(g_Solution);		//�����ʼ�⣬��ʼ�����Ž�

	//�����ʼSA�¶�
	double initT = -0.005*GBestCost /log(0.5), tk = initT;

	for (int i = 0; i < gloTime; ++i)
	{
		vector<VRut> LocalSol = g_Solution;

		for (int j = 0; j < locTime; ++j)
		{
			//����ѡ��һ��·���е�����һ���ͻ���
			int cus = ChosOneCus(LocalSol);
			//�����̰���޸�
			Insert(cus, LocalSol);

			float currCost = SumCost(LocalSol);
			if (currCost < GBestCost)
			{
				GBestCost = currCost;
				g_Solution = LocalSol;
				cout << currCost << endl;
			}
			else 
			{//ʹ��ģ���˻���սϲ�Ľ�
				int pr = rand() % 101;//�������
				double gap = GBestCost - currCost;
				double pSA = exp(-gap/ tk);
				if (pSA * 100 <= pr)
				{//���ܽϴεĽ������Ż�
					;
				}
				else
					LocalSol = g_Solution;//�����ܴ��Ž⣬�������Ž�����Ż�
			}
		}
		//����
		tk *= 0.99975;
		if (tk <= initT / 3.0)
		{//���³�ʼ���¶�
			initT = -0.005*GBestCost / log(0.5);
			tk = initT;
		}
		//����·���ƻ��޸�
		SingleRdel(LocalSol);
	}

	return 0;
}