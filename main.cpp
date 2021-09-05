#include "header.h"

using namespace std;

int main()
{
	//��һ���������
	srand(unsigned int(time(NULL)));

	//��ȡ����
	ReadData();

	//ACO�Ż�
	int gloTime = 5000;		//ȫ���Ż�����
	int seqTime = 0;		//��¼����û���Ż��Ĵ���
	InitPherom();			//��ʼ����Ϣ�ؾ��������ʽ��Ϣ����

	for (int i = 0; i < gloTime; ++i)
	{
		InitAC();				//��ʼ����Ⱥ
		float locBestCost = M;	//��¼�ֲ����Ž�
		int locBestIdx = -1;	//��¼�ֲ����Ž������
		for (int j = 0; j < antNum; ++j)
		{//������Ⱥ��ÿ�����Ϸֱ���ⷽ��
			ConstructSol(antColn[j]);

			float preCost = CalCost(antColn[j]);
			antCost.push_back(preCost);
			if (preCost < GBestCost)
			{//��¼ȫ�����Ž�
				seqTime = 0;
				GBestCost = preCost;
				Solution = antColn[j];
				cout << "ACO Find better solution : "<< GBestCost << endl;
			}
			if (preCost < locBestCost)
			{//��¼�ֲ����Ž�
				locBestCost = preCost;
				locBestIdx = j;
			}
			++seqTime;
		}

		//������Ϣ��
		if (seqTime > 500)
		{//������100��û���Ż�����������Ϣ��
			phero.clear();
			InitPherom();
			seqTime = 0;
			//�Ե�ǰ���Ž����LS�Ż�
			LS_Opt();
			UpdateBestPherom();
		}
		else
			UpdatePherom(locBestIdx);



		//����ϴ��Ż��õ�����Ⱥ�ⷽ��
		antColn.clear();
		antCost.clear();

	}
	


	return 0;
}