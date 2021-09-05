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


#define MAXTABU 5		//设置最大禁忌步长

/******************************TS相关定义************************************/
extern std::vector<std::vector<int>> Tabu;	//设置禁忌表<路径索引，客户点编号>不允许某个客户点被插入到某条路径中，防止重复搜索

extern std::vector<VRut> LS_Solution;
extern float LS_GBestCost;						//记录全局最优解


/**********************************函数声明***************************************/
//判断车辆能否返回场站
bool BackToDpt(VRut &vR, int &j);

//将某一个客户点插入到某一条路径中的某一个位置，判断路径可行性
bool JgeRutValy(int cus, int pos, VRut & vR);

//将某一个客户点插入到某一条路径中的某一个位置，更新插入点之后的客户点的到达时间，并判断路径可行性
bool UpDateTime(int cus, int pos, VRut & vR);

//任意选择一条路径中的任意一个客户点
int ChosOneCus();

//随机加贪婪插入修复
void Insert(int cus);

//计算将客户点插入到某条路径的某个位置的变动成本
float CalCost(int cus, int pos, VRut &vR);

//创建一条新路径
void CrtNewRut(int cus);

//计算当前解的成本
float sumCost();

//整条路径破坏和修复
void SingleRdel();


//结果输出
void OutPut();

//车辆数最少为目标
int sumVehicle();