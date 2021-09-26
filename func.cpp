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
vector<VRut> g_Solution;			//ȫ�����Žⷽ��

/******************************TS��ض���************************************/
vector<vector<int>> Tabu;		//���ý��ɱ�

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
				break;//�ܷ��س�վ���ʼ��������ϣ��������һ���ͻ�����������һ��·��
			}
		}
		g_Solution.push_back(vR);
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
		vR.Cost += Dis[j-1][0];
		return true;
	}
	else
	{//���ܷ��س�վ����ȥ��һ���ͻ��㣬֮���ٷ��س�վ
		vR.Ruts.pop_back();
		atTime = vR.AtTimes.back() + s[vR.Ruts.back()] + Dis[vR.Ruts.back()][0] / V;
		vR.AtTimes.push_back(atTime);
		vR.Ruts.push_back(0);
		vR.Cost += Dis[j-2][0];
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
	if(!UpDateTime(cus, pos, vR)) return false;
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
		
		if(!UpDateTime(vR.Ruts[i], i, vR))		//�ݹ����
			return false;					

		return true;
	}
}

//����ѡ��һ��·���е�����һ���ͻ���
int ChosOneCus(vector<VRut> &tempSol)
{
	int rIdx = rand() % (int)tempSol.size(),						//���ѡ��һ��·��
		cIdx = (rand() % ((int)tempSol[rIdx].Ruts.size() - 2)) + 1,//���ѡ��һ���ǳ�վ�Ŀͻ���
		objCus = tempSol[rIdx].Ruts[cIdx];

	//���½��ɱ�
	if (Tabu.size() == MAXTABU)
		Tabu.erase(Tabu.begin());
	Tabu.push_back({ rIdx, tempSol[rIdx].Ruts[cIdx] });			//����·���е�ĳѡ�еĿͻ���


	tempSol[rIdx].Ruts.erase(tempSol[rIdx].Ruts.begin() + cIdx);		//��ѡ�еĿͻ���ɾ��
	tempSol[rIdx].AtTimes.erase(tempSol[rIdx].AtTimes.begin() + cIdx);//����Ӧ�ĵ���ʱ��ɾ��
	if (tempSol[rIdx].Ruts.size() == 2)								//��·����û�пͻ����ˣ���·��Ҳɾ��
	{
		//���½��ɱ��е�·������
		for (int i = 0; i < (int)Tabu.size(); ++i)
		{
			if (Tabu[i][0] == rIdx)
				Tabu.erase(Tabu.begin() + i);//ɾ����Ӧ��Tabu��¼
			else if (Tabu[i][0] > rIdx)
				--Tabu[i][0];				 //��������rIdx��·��������Ҫ���еݼ�
		}
		tempSol.erase(tempSol.begin() + rIdx);

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

//���ͻ������·����
void InsertCusToRut(vector<VRut> &tSolution, int preRIdx, int preCusIdx, int cus, const float &preInstCost)
{
	tSolution[preRIdx].Ruts.insert(tSolution[preRIdx].Ruts.begin() + preCusIdx, cus);
	tSolution[preRIdx].Loads += q[cus];
	tSolution[preRIdx].Cost += preInstCost;
	UpDateTime(cus, preCusIdx, tSolution[preRIdx]);
}

//�����̰�������޸�
void Insert(int cus, vector<VRut> &tempSol)
{
	//�ڵ�ǰ�ⷽ�������ѡ��3/5��·������̰������Ա�
	int n = (int)tempSol.size();				//��ǰ�ⷽ����·����
	int rCnt = (int)floor(5.0*n/5.0);
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
		for (int j = 0; j < tempSol[i].Ruts.size() - 1; ++j)
		{//����ÿһ������λ��
			float insertCost = CalCost(cus, j + 1, tempSol[i]);
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
		//��鵱ǰ�����Ƿ��ڽ��ɱ���
		for (int i = 0; i < (int)Tabu.size(); ++i)
		{
			if (Tabu[i][0] == bestRIdx && Tabu[i][1] == cus)
			{
				//����Ƿ���ɲ������Բ������ڵ�ǰ���Ž�Ľ�
				vector<VRut> tSolution = tempSol;	//��ʱ��¼���Ž�
				tSolution[bestRIdx].Ruts.insert(tSolution[bestRIdx].Ruts.begin() + bestCIdx, cus);
				tSolution[bestRIdx].Loads += q[cus];
				tSolution[bestRIdx].Cost += bestCost;
				UpDateTime(cus, bestCIdx, tSolution[bestRIdx]);
				float currCost = SumCost(tempSol);	//���㵱ǰ��
				if (currCost < GBestCost)
				{//���ǣ��򽫽��ɲ����ͷţ�����¼���Ž�
					Tabu.erase(Tabu.begin() + i);
					tempSol = tSolution;
					return;
				}
				else
				{//�����ǣ���ѡ��һ�����ɱ�֮���·�������ͻ����������
					CrtNewRut(cus, tempSol);
					return;
				}
			}
		}
		//���������ڽ��ɱ���
		tempSol[bestRIdx].Ruts.insert(tempSol[bestRIdx].Ruts.begin() + bestCIdx, cus);
		tempSol[bestRIdx].Loads += q[cus];
		tempSol[bestRIdx].Cost += bestCost;
		UpDateTime(cus, bestCIdx, tempSol[bestRIdx]);
	}
	else//�½�һ��·��
		CrtNewRut(cus, tempSol);
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

//���㵱ǰ��ĳɱ�
float SumCost(vector<VRut> &tempSol)
{
	float sCost = 0;
	for (int i = 0; i < (int)tempSol.size(); ++i)
		sCost += tempSol[i].Cost;
	return sCost;
}

//ѡ��ǰ���пͻ�������·��
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

//ѡ��ǰ���пͻ������ٵ�·��
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

//����·���ƻ����޸�
void SingleRdel(vector<VRut> &tempSol)
{
	//ѡ��ͻ������ٵ�·��
	int rIdx = rand()% tempSol.size();

	vector<int> cusSet;
	for (int i = 1; i < (int)tempSol[rIdx].Ruts.size() - 1; ++i)
		cusSet.push_back(tempSol[rIdx].Ruts[i]);
	//���½��ɱ�·������
	for (int i = 0; i < (int)Tabu.size(); ++i)
	{
		if (Tabu[i][0] == rIdx)
			Tabu.erase(Tabu.begin() + i);//ɾ����Ӧ��Tabu��¼
		else if (Tabu[i][0] > rIdx)
			--Tabu[i][0];				 //��������rIdx��·��������Ҫ���еݼ�
	}
	tempSol.erase(tempSol.begin() + rIdx);

	
	for (int i = 0; i < (int)cusSet.size(); ++i)
		Insert(cusSet[i], tempSol);

	//�ж����Ž�ɱ�
	float currCost = SumCost(tempSol);
	if (currCost < GBestCost)
	{
		GBestCost = currCost;
		g_Solution = tempSol;
		cout << currCost << endl;
	}
}

