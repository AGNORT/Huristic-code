#include "header.h"

using namespace std;

#include "header.h"

using namespace std;

ofstream outPf;

/*********************************��������****************************************/
int cusNum;			//�ͻ�����Ŀ
int pointNum;		//���нڵ����Ŀ
int carNum;			//������Ŀ
float Q;			//�����������
float alpha;		//����ʵ����
float C;			//������ʻ�ı䶯�ɱ�
float V;			//������ʻ���ٶ�
float M;			//һ���㹻��ĳ���
float tau;			//�ͷ��ɱ�ϵ��

/**********************************���϶���***************************************/
vector<float> q(pointNum);		//�ͻ����������
vector<float> s(pointNum);		//�ͻ���ķ���ʱ��
vector<float> e(pointNum);		//�ͻ������ʱ�䴰
vector<float> l(pointNum);		//�ͻ������ʱ�䴰
vector<vector<float>> Dis(pointNum, vector<float>(pointNum, 0));		//�ͻ���֮��ľ������

/******************************���Ž���ض���************************************/
vector<VRut> Solution;			//�ⷽ��

/******************************TS��ض���************************************/
//vector<vector<int>> Tabu;		//���ý��ɱ�

float GBestCost = 0;				//��¼ȫ�����Ž�

//��ȡ����
void ReadData()
{
	ifstream inPut("E:/�����ļ�/�о���/��ȷ�㷨/VRPTW_BCP/��ʦ������/r111-25.txt");
	outPf.open("E:/�����ļ�/�о���/��ȷ�㷨/VRPTW_BCP/����ļ�/r111-25.txt");
	//ifstream inPut("E:/�����ļ�/�о���/��ȷ�㷨/VRPTW_BCP/��������/VRPTW_18.txt");
	//outPf.open("E:/�����ļ�/�о���/��ȷ�㷨/VRPTW_BCP/����ļ�/VRPTW-18.txt");
	if (!inPut.is_open())
	{
		cerr << "�����ļ����ļ�ʧ�ܣ�" << endl;
		exit(1);
	}
	if (!outPf.is_open())
	{
		cerr << "����ļ���ʧ�ܣ�" << endl;
		exit(1);
	}

	//���������Ϣ
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

//������ʼ��
void CrtInitSol()
{//ʹ��˳�����ķ������ɳ�ʼ��
	int j = 1;
	for (int i = 0; i < carNum; ++i)
	{
		VRut vR;				//�½�һ��·��
		for (; j <= cusNum; ++j)
		{
			//�ж�·��������
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
				break;//�ܷ��س�վ���ʼ��������ϣ��������һ���ͻ�����������һ��·��
			}
		}
		Solution.push_back(vR);
	}

}

//�жϳ����ܷ񷵻س�վ
bool BackToDpt(VRut &vR, int &j)
{//�ó������س�վ
	float atTime = vR.AtTimes.back() + s[vR.Ruts.back()] + Dis[vR.Ruts.back()][0] / V;
	if (atTime < l[0])
	{//�������Է��س�վ
		vR.AtTimes.push_back(atTime);
		vR.Ruts.push_back(0);
		vR.Cost += Dis[j - 1][0];
		return true;
	}
	else
	{//���ܷ��س�վ����ȥ��һ���ͻ��㣬֮���ٷ��س�վ
		vR.Ruts.pop_back();
		atTime = vR.AtTimes.back() + s[vR.Ruts.back()] + Dis[vR.Ruts.back()][0] / V;
		vR.AtTimes.push_back(atTime);
		vR.Ruts.push_back(0);
		vR.Cost += Dis[j - 2][0];
		--j;
		return false;
	}
}

//��ĳһ���ͻ�����뵽ĳһ��·���е�ĳһ��λ�ã��ж�·��������
bool JgeRutValy(int cus, int pos, VRut & vR)
{
	//�����ж�������
	float newLoad = vR.Loads + q[cus];
	if (newLoad > Q) return false;

	//�ж�ʱ�䴰
	if (!UpDateTime(cus, pos, vR)) return false;
	return true;
}

//��ĳһ���ͻ�����뵽ĳһ��·���е�ĳһ��λ�ã����²����֮��Ŀͻ���ĵ���ʱ�䣬���ж�·��������
bool UpDateTime(int cus, int pos, VRut & vR)
{
	//�����ж��Ƿ�ᳬ���²���ͻ������ʱ�䴰
	int lastCus = vR.Ruts[pos - 1];
	float atTime = vR.AtTimes[pos - 1] + s[lastCus] + Dis[lastCus][cus] / V;
	if (atTime > l[cus]) return false;//��������ʱ�䴰�����������

	atTime = max(atTime, e[cus]);	  //�����ǵȴ�ʱ��ɱ��������ܹ���������ʱ�䴰�������

	//���ǲ��뵽β������ֱ�Ӹ����¿ͻ���ĵ���ʱ��
	if (pos == (int)vR.AtTimes.size())
	{
		vR.AtTimes.push_back(atTime);
		return true;
	}
	else
	{//���뵽·���м�����,ʹ�õݹ���м���
		if (vR.Ruts[pos] != cus)
		{
			vR.Ruts.insert(vR.Ruts.begin() + pos, cus);
			vR.AtTimes.insert(vR.AtTimes.begin() + pos, atTime);
		}
		else
			vR.AtTimes[pos] = atTime;
		//���º����ͻ���ĵ���ʱ��
		int i = pos + 1;

		if (i == vR.Ruts.size())
			return true;

		if (!UpDateTime(vR.Ruts[i], i, vR))		//�ݹ����
			return false;

		return true;
	}
}

//����ѡ��һ��·���е�����һ���ͻ���
int ChosOneCus(vector<VRut> &resSolution)
{
	int rIdx = rand() % (int)resSolution.size(),						//���ѡ��һ��·��
		cIdx = (rand() % ((int)resSolution[rIdx].Ruts.size() - 2)) + 1,//���ѡ��һ���ǳ�վ�Ŀͻ���
		objCus = resSolution[rIdx].Ruts[cIdx];

	////���½��ɱ�
	//if (Tabu.size() == MAXTABU)
	//	Tabu.erase(Tabu.begin());
	//Tabu.push_back({ rIdx, Solution[rIdx].Ruts[cIdx] });			//����·���е�ĳѡ�еĿͻ���


	resSolution[rIdx].Ruts.erase(resSolution[rIdx].Ruts.begin() + cIdx);		//��ѡ�еĿͻ���ɾ��
	resSolution[rIdx].AtTimes.erase(resSolution[rIdx].AtTimes.begin() + cIdx);//����Ӧ�ĵ���ʱ��ɾ��
	if (resSolution[rIdx].Ruts.size() == 2)								//��·����û�пͻ����ˣ���·��Ҳɾ��
	{
		////���½��ɱ��е�·������
		//for (int i = 0; i < (int)Tabu.size(); ++i)
		//{
		//	if (Tabu[i][0] == rIdx)
		//		Tabu.erase(Tabu.begin() + i);//ɾ����Ӧ��Tabu��¼
		//	else if (Tabu[i][0] > rIdx)
		//		--Tabu[i][0];				 //��������rIdx��·��������Ҫ���еݼ�
		//}

		resSolution.erase(resSolution.begin() + rIdx);

	}

	return objCus;
}

//������Ϣ�ṹ��
struct InsertInfo
{
	float cost; //����ɱ�
	int rutIdx; //����·��
	int cusIdx; // ����λ��
	bool tabuFlag = false;//��¼��ǰ�޸���ʽ�Ƿ�Υ�����ɱ����ǣ���Ϊtrue
};

//������Ϣ�ṹ������
bool Mycomp(InsertInfo &info1, InsertInfo &info2)
{
	return info1.cost < info2.cost;
}

//�����̰�������޸�
vector<VRut> Insert(int cus, vector<VRut> &resSolution)
{
	//�ڵ�ǰ�ⷽ�������ѡ��3/5��·������̰������Ա�
	int n = (int)resSolution.size();				//��ǰ�ⷽ����·����
	int rCnt = (int)floor(5.0*n / 5.0);
	vector<int> rIdx;
	for (int i = 0; i < n; ++i)
		rIdx.push_back(i);

	random_device rd;
	mt19937 engine2(rd());
	shuffle(rIdx.begin(), rIdx.end(), engine2);		//������ҽⷽ������

	int bestRIdx = -1,
		bestCIdx = -1;
	float bestCost = M;
	for (int i = 0; i < rCnt; ++i)
	{//����ÿһ��·��
		for (int j = 0; j < (int)resSolution[i].Ruts.size() - 1; ++j)
		{//����ÿһ������λ��
			float insertCost = CalCost(cus, j + 1, resSolution[i]);
			if (insertCost < bestCost)
			{
				bestCost = insertCost;
				bestRIdx = i;
				bestCIdx = j + 1;
			}
		}
	}
	//���ͻ�����뵽����·��������λ��
	if (bestRIdx != -1)
	{//���ܹ��ҵ�ĳ��·�����в���

		////��鵱ǰ�����Ƿ��ڽ��ɱ���
		//for (int i = 0; i < (int)Tabu.size(); ++i)
		//{
		//	if (Tabu[i][0] == bestRIdx && Tabu[i][1] == cus)
		//	{
		//		//����Ƿ���ɲ������Բ������ڵ�ǰ���Ž�Ľ�
		//		vector<VRut> tSolution = Solution;	//��ʱ��¼���Ž�
		//		tSolution[bestRIdx].Ruts.insert(tSolution[bestRIdx].Ruts.begin() + bestCIdx, cus);
		//		tSolution[bestRIdx].Loads += q[cus];
		//		tSolution[bestRIdx].Cost += bestCost;
		//		UpDateTime(cus, bestCIdx, tSolution[bestRIdx]);
		//		float currCost = sumCost();	//���㵱ǰ��
		//		if (currCost < GBestCost)
		//		{//���ǣ��򽫽��ɲ����ͷţ�����¼���Ž�
		//			Tabu.erase(Tabu.begin() + i);
		//			Solution = tSolution;
		//			return;
		//		}
		//		else
		//		{//�����ǣ���ѡ��һ�����ɱ�֮���·�������ͻ����������
		//			CrtNewRut(cus);
		//			return;
		//		}
		//	}
		//}
		//���������ڽ��ɱ���
		resSolution[bestRIdx].Ruts.insert(resSolution[bestRIdx].Ruts.begin() + bestCIdx, cus);
		resSolution[bestRIdx].Loads += q[cus];
		resSolution[bestRIdx].Cost += bestCost;
		UpDateTime(cus, bestCIdx, resSolution[bestRIdx]);
	}
	else//�½�һ��·��
		CrtNewRut(cus, resSolution);
	return resSolution;
}

//���㽫�ͻ�����뵽ĳ��·����ĳ��λ�õı䶯�ɱ�
float CalCost(int cus, int pos, VRut &vR)
{
	//�����ж�������
	float newLoad = vR.Loads + q[cus];
	if (newLoad > Q) return M;					//·�������У��䶯�ɱ���Ϊ����ֵ

	//�ж�ʱ�䴰
	VRut tvR = vR;
	if (!UpDateTime(cus, pos, tvR)) return M;	//·�������У��䶯�ɱ���Ϊ����ֵ

	//����䶯�ɱ�	
	int pCus = vR.Ruts[pos - 1],						//��¼�����֮ǰ�Ŀͻ���
		fCus = vR.Ruts[pos];							//��¼�����֮��Ŀͻ���
	float oCost = Dis[pCus][fCus],								//��¼·��֮ǰ�ĳɱ�
		vCost = Dis[pCus][cus] + Dis[cus][fCus] - oCost;//��¼�䶯�ɱ�
	return vCost;
}

//����һ����·��
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

//���㵱ǰ��ĳɱ�
float sumCost(vector<VRut> &preSolution)
{
	float sCost = 0;
	for (int i = 0; i < (int)preSolution.size(); ++i)
		sCost += preSolution[i].Cost;
	return sCost;
}

//ѡ��ǰ���пͻ�������·��
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

//ѡ��ǰ���пͻ������ٵ�·��
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

//����·���ƻ����޸�
void SingleRdel(vector<VRut> &resSolution)
{
	//ѡ��ͻ������ٵ�·��
	int rIdx = rand() % resSolution.size();

	vector<int> cusSet;
	for (int i = 1; i < (int)resSolution[rIdx].Ruts.size() - 1; ++i)
		cusSet.push_back(resSolution[rIdx].Ruts[i]);
	////���½��ɱ�·������
	//for (int i = 0; i < (int)Tabu.size(); ++i)
	//{
	//	if (Tabu[i][0] == rIdx)
	//		Tabu.erase(Tabu.begin() + i);//ɾ����Ӧ��Tabu��¼
	//	else if (Tabu[i][0] > rIdx)
	//		--Tabu[i][0];				 //��������rIdx��·��������Ҫ���еݼ�
	//}
	resSolution.erase(resSolution.begin() + rIdx);


	for (int i = 0; i < (int)cusSet.size(); ++i)
		Insert(cusSet[i], resSolution);

	//�ж����Ž�ɱ�
	float currCost = sumCost(resSolution);
	if (currCost < GBestCost)
	{
		GBestCost = currCost;
		Solution = resSolution;
		cout << "Single Opt: " << currCost << endl;
	}
}

//ѡ�����ǰ��·���пͻ�����������2���Ŀͻ������������ܽڵ���������4��
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

//����ĳ��·���ĳɱ�
float CalRutCost(const vector<int> &rut)
{
	int rSize = (int)rut.size();
	float rCost = 0;
	for (int i = 1; i < rSize; ++i)
		rCost += Dis[rut[i-1]][rut[i]];
	return rCost;
}

//����ĳ��·����������
float CalRutWgt(const vector<int> &rut)
{
	float rWgt = 0;
	int rSize = (int)rut.size();
	for (int i = 1; i < rSize-1; ++i)
		rWgt += q[rut[i]];
	return rWgt;
}

//2-opt intra-route�Ż������ѡ��ĳ��·���е������ͻ�����л�����֮���жϻ���֮��ʱ�䴰Լ��
vector<VRut> IntraOpt2()
{
	vector<VRut> resSolution = Solution;

	vector<int> rutIdx = ChsRutM2();
	if (!rutIdx.size()) return vector<VRut>();//û�п��Խ���2 intra-route������·��

	int rIdx = rutIdx[rand() % rutIdx.size()];		//ѡ������ڲ�����·������
	vector<int> tSet;
	for (int i = 1; i <= (int)resSolution[rIdx].Ruts.size() - 2; ++i)
		tSet.push_back(i);

	random_device rd;
	mt19937 engine2(rd());
	shuffle(tSet.begin(), tSet.end(), engine2);

	int cIdx1 =min(tSet[0],tSet[1]), cIdx2 = max(tSet[0], tSet[1]);	//ѡ������ڻ����������ͻ��������
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
		
		//��ʱ�䴰������Ҫ����ѡ���Ŀͻ����½�����·�����������ڲ�����·�����и���
		VRut tRut = resSolution[rIdx];
		if ((int)tRut.Ruts.size() == 4)
			resSolution.erase(resSolution.begin() + rIdx);//��������·��ֻ�������ͻ��㣬��ֱ�ӽ�·��ɾ������
		else
		{
			tRut.Ruts.erase(tRut.Ruts.begin() + cIdx2);
			tRut.AtTimes.erase(tRut.AtTimes.begin() + cIdx2);
			if (rSize2 > rSize1)
			{//UpDateTime�и�����·�����в���Ҫ��cIdx1λ�õĿͻ���ɾȥ
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
		//�½�·��
		CrtNewRut(cus1, resSolution);
		CrtNewRut(cus2, resSolution);
		return resSolution;
		*/
		resSolution.push_back(VRut(1000*M));
		return resSolution;
	}
		
	tRut.Cost = CalRutCost(tRut.Ruts);		   //����·���ɱ�

	resSolution[rIdx] = tRut;
	return resSolution;
}

//ĳ������ĳ·��ɾ��һ����֮���������һ����
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

//2-opt inter-route�Ż������ѡ��ĳ����·���е������ͻ�����л�����֮���жϻ���֮���ʱ�䴰��������Լ��
vector<VRut> InterOpt2()
{
	vector<VRut> resSolution = Solution;
	vector<int> tSet;
	for (int i = 0; i < (int)resSolution.size(); ++i)
		tSet.push_back(i);

	random_device rd;
	mt19937 engine2(rd());
	shuffle(tSet.begin(), tSet.end(), engine2);

	int rIdx1 = tSet[0], rIdx2 = tSet[1];						  //ѡ������·��������

	int cIdx1 = rand() % (resSolution[rIdx1].Ruts.size() - 2) + 1,//ѡ�����ڽ����Ŀͻ��������
		cIdx2 = rand() % (resSolution[rIdx2].Ruts.size() - 2) + 1,
		cus1 = resSolution[rIdx1].Ruts[cIdx1],
		cus2 = resSolution[rIdx2].Ruts[cIdx2];

	VRut tRut1 = resSolution[rIdx1],
		tRut2 = resSolution[rIdx2];

	DelInst(resSolution, rIdx1, cIdx1, cus2);
	DelInst(resSolution, rIdx2, cIdx2, cus1);


	return resSolution;
}

//1-opt inter-route�Ż������ѡ��һ���ͻ��㣬�ڵ�ǰ�����е�·����Ѱ������λ�ý��в��룬�����ܲ��룬���½�һ��·��
vector<VRut> InterOpt1()
{
	vector<VRut> resSolution = Solution;

	//����ѡ��һ��·���е�����һ���ͻ���
	int cus = ChosOneCus(resSolution);
	//�����̰���޸�
	return Insert(cus, resSolution);
}

//ѡ�������������
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