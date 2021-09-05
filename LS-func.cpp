#include "LS-header.h"
#include "header.h"

using namespace std;


/******************************TS��ض���************************************/
vector<vector<int>> Tabu;		//���ý��ɱ�
vector<VRut> LS_Solution;	
float LS_GBestCost = 0;						//��¼ȫ�����Ž�

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
int ChosOneCus()
{
	int rIdx = rand() % (int)LS_Solution.size(),						//���ѡ��һ��·��
		cIdx = (rand() % ((int)LS_Solution[rIdx].Ruts.size() - 2)) + 1,//���ѡ��һ���ǳ�վ�Ŀͻ���
		objCus = LS_Solution[rIdx].Ruts[cIdx];

	//���½��ɱ�
	if (Tabu.size() == MAXTABU)
		Tabu.erase(Tabu.begin());
	Tabu.push_back({ rIdx, LS_Solution[rIdx].Ruts[cIdx] });			//����·���е�ĳѡ�еĿͻ���


	LS_Solution[rIdx].Ruts.erase(LS_Solution[rIdx].Ruts.begin() + cIdx);		//��ѡ�еĿͻ���ɾ��
	LS_Solution[rIdx].AtTimes.erase(LS_Solution[rIdx].AtTimes.begin() + cIdx);//����Ӧ�ĵ���ʱ��ɾ��
	if (LS_Solution[rIdx].Ruts.size() == 2)								//��·����û�пͻ����ˣ���·��Ҳɾ��
	{
		//���½��ɱ��е�·������
		for (int i = 0; i < (int)Tabu.size(); ++i)
		{
			if (Tabu[i][0] == rIdx)
				Tabu.erase(Tabu.begin() + i);//ɾ����Ӧ��Tabu��¼
			else if (Tabu[i][0] > rIdx)
				--Tabu[i][0];				 //��������rIdx��·��������Ҫ���еݼ�
		}
		LS_Solution.erase(LS_Solution.begin() + rIdx);

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
void Insert(int cus)
{
	//�ڵ�ǰ�ⷽ�������ѡ��3/5��·������̰������Ա�
	int n = (int)LS_Solution.size();				//��ǰ�ⷽ����·����
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
		for (int j = 0; j < (int)LS_Solution[i].Ruts.size() - 1; ++j)
		{//����ÿһ������λ��
			float insertCost = CalCost(cus, j + 1, LS_Solution[i]);
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
				vector<VRut> tSolution = LS_Solution;	//��ʱ��¼���Ž�
				tSolution[bestRIdx].Ruts.insert(tSolution[bestRIdx].Ruts.begin() + bestCIdx, cus);
				tSolution[bestRIdx].Loads += q[cus];
				tSolution[bestRIdx].Cost += bestCost;
				UpDateTime(cus, bestCIdx, tSolution[bestRIdx]);
				float currCost = sumCost();	//���㵱ǰ��
				if (currCost < LS_GBestCost)
				{//���ǣ��򽫽��ɲ����ͷţ�����¼���Ž�
					Tabu.erase(Tabu.begin() + i);
					LS_Solution = tSolution;
					return;
				}
				else
				{//�����ǣ���ѡ��һ�����ɱ�֮���·�������ͻ����������
					CrtNewRut(cus);
					return;
				}
			}
		}
		//���������ڽ��ɱ���
		LS_Solution[bestRIdx].Ruts.insert(LS_Solution[bestRIdx].Ruts.begin() + bestCIdx, cus);
		LS_Solution[bestRIdx].Loads += q[cus];
		LS_Solution[bestRIdx].Cost += bestCost;
		UpDateTime(cus, bestCIdx, LS_Solution[bestRIdx]);
	}
	else//�½�һ��·��
		CrtNewRut(cus);
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
void CrtNewRut(int cus)
{
	VRut newVr;
	newVr.Ruts.push_back(cus);
	newVr.Ruts.push_back(0);
	newVr.AtTimes.push_back(e[cus]);
	newVr.AtTimes.push_back(e[cus]+s[cus]+Dis[cus][0]/V);
	newVr.Loads += q[cus];
	newVr.Cost = 2 * Dis[0][cus];
	LS_Solution.push_back(newVr);
}

//���㵱ǰ��ĳɱ�
float sumCost()
{
	float sCost = 0;
	for (int i = 0; i < (int)LS_Solution.size(); ++i)
		sCost += LS_Solution[i].Cost;
	return sCost;
}

//ѡ��ǰ���пͻ�������·��
int ChsMRut()
{
	int rutIdx = 0;
	int maxSize = 0;
	for (int i = 0; i < (int)LS_Solution.size(); ++i)
	{
		if ((int)LS_Solution.size() > maxSize)
		{
			maxSize = (int)LS_Solution.size();
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
	for (int i = 0; i < (int)LS_Solution.size(); ++i)
	{
		if ((int)LS_Solution.size() < minSize)
		{
			minSize = (int)LS_Solution.size();
			rutIdx = i;
		}
	}
	return rutIdx;
}

//����·���ƻ����޸�
void SingleRdel()
{
	//ѡ��ͻ������ٵ�·��
	int rIdx = rand()%LS_Solution.size();

	vector<int> cusSet;
	for (int i = 1; i < (int)LS_Solution[rIdx].Ruts.size() - 1; ++i)
		cusSet.push_back(LS_Solution[rIdx].Ruts[i]);
	//���½��ɱ�·������
	for (int i = 0; i < (int)Tabu.size(); ++i)
	{
		if (Tabu[i][0] == rIdx)
			Tabu.erase(Tabu.begin() + i);//ɾ����Ӧ��Tabu��¼
		else if (Tabu[i][0] > rIdx)
			--Tabu[i][0];				 //��������rIdx��·��������Ҫ���еݼ�
	}
	LS_Solution.erase(LS_Solution.begin() + rIdx);

	
	for (int i = 0; i < (int)cusSet.size(); ++i)
		Insert(cusSet[i]);

	//�ж����Ž�ɱ�
	float currCost = sumCost();
	if (currCost < LS_GBestCost)
	{
		LS_GBestCost = currCost;
		cout << currCost << endl;
	}
}

//LS�Ż�
void LS_Opt()
{
	LS_Solution = Solution;
	//TS�Ż�
	int gloTime = 500,			//����ȫ���Ż�����
		locTime = 100;			//����ֲ��Ż�����
	LS_GBestCost = sumCost();		//�����ʼ�⣬��ʼ�����Ž⣬������С��Ŀ��

	for (int i = 0; i < gloTime; ++i)
	{
		for (int j = 0; j < locTime; ++j)
		{
			//����ѡ��һ��·���е�����һ���ͻ���
			int cus = ChosOneCus();
			//�����̰���޸�
			Insert(cus);

			float currCost = sumCost();//������С��Ŀ��
			if (currCost < GBestCost)
			{
				GBestCost = currCost;
				Solution = LS_Solution;
				cout << "LS Find Better LS_Solution : " << currCost << endl;
			}
			if (currCost < LS_GBestCost)
			{
				LS_GBestCost = currCost;
			}
		}
		//����·���ƻ��޸�
		SingleRdel();
	}
	//OutPut();
}

void OutPut()
{
	outPf << "The Minimum Total Cost = " << sumCost() << endl;
	outPf << "Concert Schedule of Each Route is as Follow: " << endl;
	int i = 0;
	while (i < (int)LS_Solution.size())
	{
		outPf << "No." << i + 1 << ": ";
		//���·������
		int j = 0;
		for (; j < (int)LS_Solution[i].Ruts.size()-1; ++j)
			outPf << LS_Solution[i].Ruts[j] << " -> ";
		outPf << LS_Solution[i].Ruts[j] << ";" << endl;

		//���·������ʱ������
		j = 0;
		for (; j < (int)LS_Solution[i].AtTimes.size() - 1; ++j)
		{
			outPf << fixed << setprecision(2) << LS_Solution[i].AtTimes[j] << " -> ";
		}
		outPf << fixed << setprecision(2) << LS_Solution[i].AtTimes[j] << ";" << endl;

		++i;
	}
}

