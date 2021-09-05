#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <algorithm>
#include <time.h>
#include <random>
#include <iomanip>


#define MAXTABU 5		//���������ɲ���

extern std::ofstream outPf;

/*********************************��������****************************************/
extern int cusNum;			//�ͻ�����Ŀ
extern int pointNum;		//���нڵ����Ŀ
extern int carNum;			//������Ŀ
extern float Q;			//�����������
extern float altra;		//����ʵ����
extern float C;			//������ʻ�ı䶯�ɱ�
extern float V;			//������ʻ���ٶ�
extern float M;			//һ���㹻��ĳ���
extern float tau;			//�ͷ��ɱ�ϵ��

/**********************************���϶���***************************************/
extern std::vector<float> q;				//�ͻ����������
extern std::vector<float> s;				//�ͻ���ķ���ʱ��
extern std::vector<float> e;				//�ͻ������ʱ�䴰
extern std::vector<float> l;				//�ͻ������ʱ�䴰
extern std::vector<std::vector<float>> Dis;		//�ͻ���֮��ľ������

/*******************************ACO��ض���*************************************/
extern int antNum;						//ÿ����Ⱥ�����ϵĸ���
extern float alpha;						//����ѡ��������alpha
extern float beta;						//����ѡ��������beta
extern float rho;						//��Ϣ����ʧ����rho
extern float bestE;						//�ֲ�����·������Ϣ�ظ���Ȩ��
extern std::vector<float> antCost;		//��¼��Ⱥ��ÿ�����ϵĳɱ�
extern std::vector<std::vector<float>> phero;		//��Ϣ�ؾ���
extern std::vector<std::vector<float>> eta;			//����ʽ��Ϣ����

extern float GBestCost;						//��¼ȫ�����Ž�


/*���Ž���ض���*/
//һ������·��
class VRut
{
public:
	std::vector<int> Ruts;			//��ǰ����·��
	std::vector<float> AtTimes;		//��ǰ����·����Ӧ�ĳ�������ʱ��
	float Loads;					//������������
	float Cost;						//����·���ĳɱ�
	//float RTimes;					//ʣ��ȴ�ʱ�伯�� �����ǵȴ�ʱ��ɱ�
	VRut()
	{//��ʼ��·��
		Ruts.push_back(0);
		AtTimes.push_back(0);
		Loads = 0;
		Cost = 0;
	}
};
extern std::vector<VRut> Solution;
extern std::vector<std::vector<VRut>> antColn;	//��Ⱥ


/**********************************��������***************************************/
//��ȡ����
void ReadData();

//��ʼ����Ϣ�ؾ���
void InitPherom();

//��ʼ����Ⱥλ��
void InitAC();

//Ϊÿһ�����Ϲ����ⷽ��
void ConstructSol(std::vector<VRut> & preAnt);

//����ÿһ�����ϵĳɱ�
float CalCost(std::vector<VRut> preAnt);

//������Ϣ����Ϣ
void UpdatePherom(int locBIdx);

//�������Ž���Ϣ����Ϣ
void UpdateBestPherom();

//LS�Ż�
void LS_Opt();