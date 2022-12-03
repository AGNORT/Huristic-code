#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <algorithm>
#include <time.h>
#include <random>

extern std::ofstream outPf;

/*********************************��������****************************************/
extern int cusNum;			//�ͻ�����Ŀ
extern int pointNum;		//���нڵ����Ŀ
extern int carNum;			//������Ŀ
extern float Q;			//�����������
extern float alpha;		//����ʵ����
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

/******************************TS��ض���************************************/
//extern std::vector<std::vector<int>> Tabu;	//���ý��ɱ�<·���������ͻ�����>������ĳ���ͻ��㱻���뵽ĳ��·���У���ֹ�ظ�����

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
	VRut(float C)
	{
		Cost = C;
	}
};
extern std::vector<VRut> Solution;

/**********************************��������***************************************/
//��ȡ����
void ReadData();

//������ʼ��
void CrtInitSol();

//�жϳ����ܷ񷵻س�վ
bool BackToDpt(VRut &vR, int &j);

//��ĳһ���ͻ�����뵽ĳһ��·���е�ĳһ��λ�ã��ж�·��������
bool JgeRutValy(int cus, int pos, VRut & vR);

//��ĳһ���ͻ�����뵽ĳһ��·���е�ĳһ��λ�ã����²����֮��Ŀͻ���ĵ���ʱ�䣬���ж�·��������
bool UpDateTime(int cus, int pos, VRut & vR);

//����ѡ��һ��·���е�����һ���ͻ���
int ChosOneCus(std::vector<VRut> &resSolution);

//�����̰�������޸�
std::vector<VRut> Insert(int cus, std::vector<VRut> &resSolution);

//���㽫�ͻ�����뵽ĳ��·����ĳ��λ�õı䶯�ɱ�
float CalCost(int cus, int pos, VRut &vR);

//����һ����·��
void CrtNewRut(int cus, std::vector<VRut> &preSolution);

//���㵱ǰ��ĳɱ�
float sumCost(std::vector<VRut> &preSolution);

//����·���ƻ����޸�
void SingleRdel(std::vector<VRut> &resSolution);

//2-opt intra-route�Ż������ѡ��ĳ��·���е������ͻ�����л�����֮���жϻ���֮��ʱ�䴰Լ��
std::vector<VRut> IntraOpt2();

//1-opt inter-route�Ż������ѡ��һ���ͻ��㣬�ڵ�ǰ�����е�·����Ѱ������λ�ý��в��룬�����ܲ��룬���½�һ��·��
std::vector<VRut> InterOpt1();

//2-opt inter-route�Ż������ѡ��ĳ����·���е������ͻ�����л�����֮���жϻ���֮���ʱ�䴰��������Լ��
std::vector<VRut> InterOpt2();

//ѡ�������������
std::vector<VRut> ChsNbOpt(int i);