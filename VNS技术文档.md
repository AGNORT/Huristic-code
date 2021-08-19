[toc]
#  1、简介-introduction
&emsp;&emsp;变邻域搜索（Variable Neighborhood Search）是一种求解组合优化和全局优化问题的元启发式算法，它的基本思想是：**在局部搜索(LS)算法的框架中系统地变化邻域结构的选择。**
&ensp;下面本文将介绍以下几部分内容，第二部分本文将介绍VNS的基本机制；第三部分将介绍VNS的相关拓展；第四部分将介绍VNS和其他启发式算法的混杂应用；第五部分

----------
将介绍VNS在求解VRPTW中的具体思路，最后将给出使用VNS求解VRPTW问题的代码。
#  2、VNS的基本机制
&emsp;&emsp;首先定义$N_k,(k= 1,...,k_{max})$表示事先定义好的邻域结构集合，定义$N_k(x)$表示第$k$个邻域结构中构造的解的集合。VNS基于三个简单的事实：
①某一个邻域结构的局部最优解不一定是另外一个邻域结构的局部最优解；
②对于所有邻域结构来说，全局最优解均是其局部最优解；
③对于许多问题情境，一个或者多个邻域的局部最优解应该彼此相近。
&emsp;&emsp;最后一个事实，虽然有点经验主义，但是暗示一个局部最优解通常会为全局最优解提供有效信息。
##  2.1 变邻域下降方法-Variable Neighborhood Descent
&ensp;在VND中，事先选定的邻域结构按照一个确定的顺序进行执行，其流程如下图所示：
![在这里插入图片描述](https://img-blog.csdnimg.cn/7c0c8d927c1f43e7b1566d4a4b6765ec.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MzE2MDc0NA==,size_16,color_FFFFFF,t_70#pic_center)
&emsp;&emsp;首先进行初始化，选择邻域结构$N_k,(k= 1,...,k_{max})$，构建一个初始解，选定终止准则；之后循环执行VND，直到达到终止准则。其中VND分为两步，第一步为重置计数变量$k=1$，第二步为顺序执行邻域结构，直到$k=k_{max}$。在每一次选定邻域结构进行执行的时候又可以分为两步(a)在第$k$个邻域结构生成的解方案中随机选择一个解方案$x'$-shaking；(b)若选择的解方案$x'$优于当前解，则使用$x'$替换当前解，同时重新从第一个邻域结构开始执行优化；否则选择下一个邻域结构进行解的优化。
&emsp;&emsp;除了上述顺序执行的方法，VND还可以进行一定的改进，使用嵌套策略来执行邻域结构。假设有3个邻域结构，$k_{max}=3$，则嵌套结构执行方法为在外层只使用第3个邻域结构，而对于第3个邻域结构生成的每一个解方案$x'$，顺序执行邻域结构1和2组成的VND。
&emsp;&emsp;缩减VNS(Reduced Variable Neighborhood Search)方法同上述VND方法类似，但在shaking之后，不是对$x'$执行后续的邻域操作，而是对当前最优解进行后续的邻域操作。
##  2.2基本变邻域搜索方法-Basic Variable Neighborhood Search
&emsp;&emsp;VNS综合了邻域选择的确定性和随机性因素，其流程如下图所示：
![在这里插入图片描述](https://img-blog.csdnimg.cn/c16cdc4e87604e319a1ee66df721f36d.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MzE2MDc0NA==,size_16,color_FFFFFF,t_70#pic_center)
&emsp;&emsp;基本VNS与VND的区别主要在于(2.b)中在执行完邻域操作随机选定一个解$x'$之后，VNS不是直接判断解$x'$的优劣，而是对其进行进一步的局部搜索(LS)探索，将LS得到的解$x''$与当前解进行判断，保留较优的解。
##  2.3 终止准则
&emsp;&emsp;终止准则的选择可以有但不限于以下三种方式：
①规定一个最大的CPU运算时间；
②规定一个总的最大的迭代次数；
③规定一个最大迭代次数$maxIte$，若在$maxIte$之内解没有发生优化，则终止算法。
#  3、VNS的拓展和混杂算法
##  3.1 VNS的拓展
&emsp;&emsp;基本的VNS是一种基于descent，first improvement method和randomization的算法。在基本VNS流程的(2.c)中，可以加入一定的概率机制，当解$x''$次于当前解时，以一定的几率接受退化的解，来实现一种descent-ascent 机制；在选择邻域结构时，也可以加入轮盘赌的机制，优先选择优化效果较好的邻域结构进行执行；在(2.a)中可以选出最优的解$x'$来进行后续的优化；还可以规定一个$k_{min}$和$k_{step}$，替换$k←1$为$k←k_{min}$，$k←k+1$为$k←k_{step}$。
##  3.2 变邻域分解搜索-Variable Neighborhood Decomposition Search
&emsp;&emsp;VNDS方法将基本VNS拓展称为一种两级VNS，VNDS的步骤如下所示：
![在这里插入图片描述](https://img-blog.csdnimg.cn/42c3b292d1a544d98603ee18c5a53cb5.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MzE2MDc0NA==,size_16,color_FFFFFF,t_70#pic_center)
&emsp;&emsp;VNDS相对于基本VNS的主要区别在于(2.b)中，VNDS不是直接对于解$x'$进行局部搜索LS操作，而是在(2.a)时选出一个解集合$y=x'/\ x$，在新的解空间$y$(全部解空间的子集)中进行局部寻优，记寻找到的最优解为$y'$，整个解空间中的最优解记为$x''$，之后再执行(2.c)更新最优解或者执行下一次邻域结构优化。
##  3.3 偏变邻域搜索-Skewed VNS
&emsp;&emsp;SVNS用来探索距离当前解较远的另外的求解空间，以求得到不同的解结构，寻找到更优的解，SVNS的流程如下图所示：
![在这里插入图片描述](https://img-blog.csdnimg.cn/96b5900178704ec3b84c1c7e505abaac.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MzE2MDc0NA==,size_16,color_FFFFFF,t_70#pic_center)
&emsp;&emsp;从上述流程可以看出，SVNS相对于基本VNS，主要将(2.c)分解成为了两部分，新的(2.c)用来判断是否进行更新最优解，新的(2.d)用来判断是否进行重新开始邻域搜索还是继续进行邻域搜索，其中利用了函数$\rho(x,x'')$来判断当前解$x$和局部最优解$x''$之间的距离，(`本人理解`：两个解之间的差异程度，如在VRPTW中，可以指两个路径方案用的车辆数，访问客户点的顺序差异程度等等)，当$\rho(x,x'')$的值较小的时候，参数$\alpha$取值应该较大，来使求解空间转移到差距更大的地方进行寻优。$\alpha\rho(x,x'')$的选取还可以采取学习机制或者自适应的机制进行更新。
##  3.4 并行变邻域搜索-Parallel VNS
&emsp;&emsp;在(2.b)时采用多线程的编程方式，对于某一个邻域产生的多个解方案来说，给每一个解方案开一个进程进行局部搜索LS，最终保留所有进程中求解结果最好的结果。
#  4、VNS和其他启发式算法的混合使用-hybrids
##  4.1 VNS 和 TS
&emsp;&emsp;有两种方式来进行VNS和TS的混合使用，第一种是在TS框架之中使用VNS，第二种是在VNS框架中使用TS。
&emsp;&emsp;在VNS框架中使用TS时，只需要将禁忌表机制加入到VNS搜索的过程即可，为所有的邻域搜索算子建立一个禁忌表，规定解禁步长和宽恕准则，可以有效避免backward搜索。
&emsp;&emsp;在TS框架中使用VNS时，只需要将TS中解的优化步骤中的LS优化变为VNS优化即可。
##  4.2 VNS 和 GRASP
&emsp;&emsp;GRASP是一种两阶段元启发式算法，在第一阶段使用贪婪随机的思想进行初始解的构建，在第二阶段使用LS或者枚举的思想进行解的优化。所以，一种顺其自然的方式来结合VNS和GRASP是在GRASP的第二阶段使用VNS来进行优化。
#  5、利用VNS框架求解VRPTW





 




