#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <algorithm>
#include <time.h>
#include <random>
#include <iomanip>
#include "header.h"


#define MAXTABU 5		//���������ɲ���

/******************************TS��ض���************************************/
extern std::vector<std::vector<int>> Tabu;	//���ý��ɱ�<·���������ͻ�����>������ĳ���ͻ��㱻���뵽ĳ��·���У���ֹ�ظ�����

extern std::vector<VRut> LS_Solution;
extern float LS_GBestCost;						//��¼ȫ�����Ž�


/**********************************��������***************************************/
//�жϳ����ܷ񷵻س�վ
bool BackToDpt(VRut &vR, int &j);

//��ĳһ���ͻ�����뵽ĳһ��·���е�ĳһ��λ�ã��ж�·��������
bool JgeRutValy(int cus, int pos, VRut & vR);

//��ĳһ���ͻ�����뵽ĳһ��·���е�ĳһ��λ�ã����²����֮��Ŀͻ���ĵ���ʱ�䣬���ж�·��������
bool UpDateTime(int cus, int pos, VRut & vR);

//����ѡ��һ��·���е�����һ���ͻ���
int ChosOneCus();

//�����̰�������޸�
void Insert(int cus);

//���㽫�ͻ�����뵽ĳ��·����ĳ��λ�õı䶯�ɱ�
float CalCost(int cus, int pos, VRut &vR);

//����һ����·��
void CrtNewRut(int cus);

//���㵱ǰ��ĳɱ�
float sumCost();

//����·���ƻ����޸�
void SingleRdel();


//������
void OutPut();

//����������ΪĿ��
int sumVehicle();