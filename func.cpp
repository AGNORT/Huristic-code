#include "header.h"

using namespace std;

ofstream outPf;

/*********************************��������****************************************/
int cusNum;			//�ͻ�����Ŀ
int pointNum;		//���нڵ����Ŀ
int carNum;			//������Ŀ
float Q;			//�����������
float altra;		//����ʵ����
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
vector<VRut> Solution;			//һ���ⷽ��-һ������

/*******************************ACO��ض���*************************************/
int antNum;						//ÿ����Ⱥ�����ϵĸ���
vector<vector<VRut>> antColn;	//��Ⱥ
vector<float> antCost;			//��¼��Ⱥ��ÿ�����ϵĳɱ�
float alpha = 3;				//����ѡ��������alpha
float beta = 2;					//����ѡ��������beta
float rho = 0.01;				//��Ϣ����ʧ����rho
float bestE = 2;				//�ֲ�����·������Ϣ�ظ���Ȩ��
vector<vector<float>> phero;	//��Ϣ�ؾ���
vector<vector<float>> eta;		//����ʽ��Ϣ����

float GBestCost = 0;			//��¼ȫ�����Ž�

//��ȡ����
void ReadData()
{
	ifstream inPut("E:/�����ļ�/�о���/��ȷ�㷨/VRPTW_BCP/��ʦ������/r111-25.txt");
	outPf.open("r110-25.txt");
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
	inPut >> cusNum >> carNum >> Q >> altra >> C >> V >> M >> tau;
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

	antNum = cusNum;		//�趨��Ⱥ�����ϵ�����Ϊ�ͻ���ĸ���
	GBestCost = M;			//����ȫ�����Ž�
	inPut.close();
}

//���ĳ��·�����ĳ���ͻ���֮���Ƿ����
bool JgeVal(const VRut & preRut, int preNode)
{
	int bNode = preRut.Ruts.back();
	float atTime = preRut.AtTimes.back() + s[bNode] + Dis[bNode][preNode] / V;
	//���ʱ�䴰
	if (atTime > l[preNode])
		return false;
	//���������
	float weight = preRut.Loads + q[preNode];
	if (weight > Q)
		return false;
	return true;
}

//ÿһ��·�����һ���ͻ���֮�󣬸���·���������Ϣ
bool UpdateInfo(VRut & preRut, int preNode)
{
	int bNode = preRut.Ruts.back();
	float atTime = preRut.AtTimes.back() + s[bNode] + Dis[bNode][preNode] / V;
	float weight = preRut.Loads + q[preNode];
	//���ʱ�䴰 ������
	if (!JgeVal(preRut, preNode))
		return false;

	//��������ʱ�䴰������Ҫ�������·����Ϣ
	preRut.Ruts.push_back(preNode);
	preRut.AtTimes.push_back(max(atTime,e[preNode]));
	preRut.Loads = weight;
	preRut.Cost += Dis[bNode][preNode];

	return true;
}

//��ʼ����Ϣ�ؾ��������ʽ��Ϣ����
void InitPherom()
{
	phero.resize(pointNum);
	eta.resize(pointNum);
	for (int i = 0; i < pointNum; ++i)
	{
		phero[i].resize(pointNum, 0);
		eta[i].resize(pointNum, 0);
	}

	for (int i = 0; i < pointNum; ++i)
	{
		for (int j = 0; j < pointNum; ++j)
		{
			phero[i][j] = 1;
			if(Dis[i][j] != 0)
				eta[i][j] = 1 / Dis[i][j];
		}
	}
}

//��ʼ����Ⱥ
void InitAC()
{
	vector<int> cusSet;
	for (int i = 1; i <= cusNum; ++i)
		cusSet.push_back(i);
	random_shuffle(cusSet.begin(),cusSet.end());

	for (int i = 0; i < antNum; ++i)
	{
		vector<VRut> t(1);
		antColn.push_back(t);
		for (int j = 0; j < (int)antColn[i].size(); ++j)
			UpdateInfo(antColn[i][j], cusSet[i]);
	}
}

//ѡ��û�з��ʹ��Ŀͻ��㼯��
vector<int> FinduvCSet(const vector<VRut> & preAnt)
{
	//��¼�Ѿ����ʵĿͻ��㼯��
	vector<int> vCusSet;
	for (int i = 0; i < (int)preAnt.size(); ++i)
	{
		for (int j = 1; j < (int)preAnt[i].Ruts.size(); ++j)
			vCusSet.push_back(preAnt[i].Ruts[j]);
	}
	vector<int> uvCusSet;//��¼δ���ʵĿͻ��㼯��
	for (int i = 1; i <= cusNum; ++i)
	{
		if(find(vCusSet.begin(),vCusSet.end(), i) == vCusSet.end())
			uvCusSet.push_back(i);
	}
	return uvCusSet;
}

//��δ���ʵĿͻ��㼯�����ҿ�����ͻ��㼯��
vector<int> FindN(const VRut & preRut, const vector<int> &uvCusSet)
{
	vector<int> NSet;
	for (int i = 0; i < (int)uvCusSet.size(); ++i)
	{
		if (JgeVal(preRut, uvCusSet[i]))
			NSet.push_back(uvCusSet[i]);
	}
	return NSet;
}

//�������
float CalProb(int bNode, int pNode, const vector<int> &NSet)
{
	//�����ĸ
	float dSum = 0,		//��¼��ĸ��ֵ
		mSum = 0;		//��¼���ӵ�ֵ
	for (int i = 0; i < (int)NSet.size(); ++i)
		dSum += pow(phero[bNode][NSet[i]], alpha)*pow(eta[bNode][NSet[i]],beta);//�ۼӷ�ĸ

	mSum = pow(phero[bNode][pNode], alpha)*pow(eta[bNode][pNode], beta);		//�������
	
	if (mSum == 0 || dSum == 0) return 0;
	return mSum / dSum;
}

//ѡ��Ӧ����չ�Ŀͻ���
int ChsNode(int bNode, const vector<int> &NSet)
{
	float bestProb = 0;		//��¼���Ÿ���(max)
	int bestNode = -1;		//��¼���ſͻ���
	for (int i = 0; i < (int)NSet.size(); ++i)
	{
		float tP = CalProb(bNode, NSet[i], NSet);
		if (tP >= bestProb-1e-6)
		{
			bestProb = tP;
			bestNode = NSet[i];
		}

	}
	if (bestNode == -1)
	{//������
		cout << "probalisity calculation error��" << endl;
		float tP = CalProb(bNode, NSet.back(), NSet);

		getchar();
	}
	return bestNode;
}

//Ϊÿһ�����Ϲ����ⷽ��
void ConstructSol(vector<VRut> & preAnt)
{
	//ѡ�����ǰ����û�з��ʵĿͻ��㼯��
	vector<int> uvCusSet = FinduvCSet(preAnt);

	for (int i = 0; i < (int)preAnt.size(); ++i)
	{
		if (!uvCusSet.size())
		{
			if (preAnt.back().Ruts.back() != 0)
				UpdateInfo(preAnt.back(), 0);
			return;
		}

		while (true)
		{//�ظ�Ϊ��ǰ·������ͻ���
			int bNode = preAnt[i].Ruts.back();
			if (!uvCusSet.size())
			{
				UpdateInfo(preAnt[i], 0);
				break;
			}

			//Ѱ�ҵ�ǰ·�����һ���ͻ���Ŀ�����ͻ��㼯��
			vector<int> NSet = FindN(preAnt[i], uvCusSet);
			if (!NSet.size())
			{//����ǰ·�����ܵ�����������ͻ��㣬�򷵻س�վ,�������ѡ��һ��δ���ʵĿͻ����½�һ��·��
				UpdateInfo(preAnt[i], 0);
				preAnt.push_back(VRut());
				int randIdx = rand() % uvCusSet.size();
				UpdateInfo(preAnt.back(), uvCusSet[randIdx]);
				uvCusSet.erase(uvCusSet.begin() + randIdx);
				break;
			}

			//���ø���ѡ����ѡ��Ӧ����չ�Ŀͻ���
			int preCus = ChsNode(bNode, NSet);
			UpdateInfo(preAnt[i], preCus);


			uvCusSet.erase(find(uvCusSet.begin(), uvCusSet.end(),preCus));//ɾ���Ѿ����ʵĿͻ���
		}
	}

}

//����ÿһ�����ϵĳɱ�
float CalCost(vector<VRut> preAnt)
{
	float cost = 0;
	for (int i = 0; i < (int)preAnt.size(); ++i)
		cost += preAnt[i].Cost;
	return cost;
}

//������Ϣ����Ϣ
void UpdatePherom(int locBIdx)
{
	//�ȼ�����Ϣ������ЧӦ
	for (int i = 0; i < pointNum; ++i)
		for (int j = 0; j < pointNum; ++j)
			phero[i][j] = (1 - rho)*phero[i][j];
	
	//�ټ��㵱ǰAC��Ӱ��
	for (int i = 0; i < antNum; ++i)
	{
		int rNum = (int)antColn[i].size();
		if (i != locBIdx)
		{//��ͨ·����Ϣ�ظ���
			for (int j = 0; j < rNum; ++j)
			{
				for (int k = 0; k < (int)antColn[i][j].Ruts.size()-1; ++k)
					phero[k][k + 1] += 1/antCost[i];//100
			}
		}
		else
		{//�ֲ�����·����Ϣ�ظ���
			for (int j = 0; j < rNum; ++j)
			{
				for (int k = 0; k < (int)antColn[i][j].Ruts.size() - 1; ++k)
					phero[k][k + 1] += bestE / antCost[i];//100
			}
		}
	}
}

//�������Ž���Ϣ����Ϣ
void UpdateBestPherom()
{
	int rNum = (int)Solution.size();
	float bestCost = CalCost(Solution);
	for (int i = 0; i < rNum; ++i)
	{
		for (int k = 0; k < (int)Solution[i].Ruts.size() - 1; ++k)
			phero[k][k + 1] += bestE / bestCost;//100
	}
}

