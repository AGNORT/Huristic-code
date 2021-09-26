[toc]

#  1、模拟退火算法介绍
$\qquad$模拟退火算法(Simulated Annealing)是一种用于求解离散优化和部分连续优化问题的有效的元启发式算法。SA算法的核心特点在于它通过接收更差的解来辅助跳出局部最优解，以求得到全局最优解。
$\qquad$SA算法起源于晶体降温的物理实验，将传热动力学的行为和离散优化问题最优解的寻优进行相互关联，最终将这种关联成功应用到了优化问题上。在SA求解的过程中，较优的解总是被接受，但是一部分次优的解也会被接受，接收次优解的概率取决于温度参数，而温度参数在迭代的过程中通常是一个非增(non-increasing)的参数。
$\qquad$模拟退火算法可以使用Makov链来证明最优收敛性，下面介绍精确SA算法的理论内容。

##  1.1 精确SA算法
###  1.1.1 Metropolis Acceptance Criterion
$\qquad$首先定义$\Omega$表示求解空间；定义$f$表示目标函数；定义$\omega^*$表示最优解，则$f(\omega)\geq f(\omega^*)，\omega \in \Omega$；定义邻域空间$N(\omega)，\omega \in \Omega$表示LS算法通过一步迭代可以找到的邻域解集合。SA算法基于Metropolis Acceptance Criterion来判断是否接受新的较差的邻域解$\omega$来替代当前解。Metropolis Acceptance Criterion的概率计算如下所示：$$P(Accep \ \omega'\ as\ next\ Solution)=\begin{cases}
          exp[-(f(\omega')-f(\omega))/t_k],  & \text{if $f(\omega')-f(\omega)>0$} \\
            1, & \text{if $f(\omega')-f(\omega)\leq0$} \\
        \end{cases}$$

 $\qquad$其中$t_k$表示和外循环次数$k$相关的温度参数，满足：$$t_k>0 \  \ \ \ \  \underset{k→+\infty}{lim}t_k=0$$

$\qquad$上式是基本SA算法应用的接收准则。

### 1.1.2 Boltzmann distribution   
$\qquad$进一步的搜索方向选择需要用到Boltzmann distribution等式，其描述了在温度$T$情况下，邻域解$\omega$出现的概率：$$P(System\ is\ in \ state\ \omega\ at\ temperature\ T)=\frac{exp(-f(\omega)/t_k)}{\sum_{\omega''\in\Omega}exp(-f(\omega'')/t_k)}$$

$\qquad$还需要用到选择邻域解$\omega'$的概率$g_k(\omega,\ \omega')$，其满足：

$$\underset{\omega'\in N(\omega)}{\sum} g_k(\omega,\ \omega')=1,\ forall\  \omega \in \Omega,\ k=1,2,...$$ 

$\qquad$则在某一次迭代时，选择某个邻域解$\omega$进行更新当前解的概率为：

$$P_k(\omega,\ \omega')=\begin{cases}
        g_k(\omega,\ \omega')exp(-\Delta_{\omega,\omega'}/t_k),\ \  if\ \omega \in N(\omega), \ \omega \neq \omega  \\
        0,\qquad \qquad\qquad\qquad\qquad\ \  if\ \omega \notin N(\omega), \ \omega \neq \omega  \\
        1-\underset{\omega''\neq\omega}{\underset{\omega''\in N(\omega)}{\sum}}p_k(\omega,\ \omega''),\quad \ \ \ \ \ if\ \omega=\omega
        \end{cases}$$

$\qquad$从上式获得每个邻域解的选择概率之后，选择概率最大的邻域解作为此次搜索的方向，继续执行搜索，这种精确SA算法可以通过Makov Chain证明其可以收敛到最优解的最优性。

##  1.2 启发式SA算法
$\qquad$上述精确SA算法最大的缺陷在于每一步计算所有邻域解会耗费大量的时间，导致其求解时间很长。所以在实际应用之中，通常将SA算法和其他启发式算法进行结合，借助SA的思想来跳出局部最优解，通过快速迭代寻优，最终得到一个较好的近似最优解。
$\qquad$因此启发式SA算法只需要用到上述Metropolis Acceptance Criterion的准则便已经足够，在每一次构建出一个新的邻域解之后，判断其是否优于当前解，若是则直接用其替换当前解；若不是，则计算一个接收次优解的概率$P$，之后再随机生成一个概率$P^r$，若$P^r\geq P$，则接受当前次优解，否则放弃当前次优解。

###  1.2.1 温度处理
$\qquad$在应用SA算法时，一个重要的考察项是温度相关的参数设置，包括初始温度，降温速率的选择和停止降温的标准。本人在应用SA时设置初始温度$t_0$的方式如下：

$$t_0=p^v*f(init)/ln(0.5)$$

$\qquad$其中$p^v$表示设置的降温速率参数，一般取值为0.005，这样降温速率即为0.99975；本人在实际应用中设置停止降温的准则为温度小于初始温度的$1/3$，之后会使用当前最优解$f(pre)$重新计算一个初始温度：

$$t_0=p^v*f(pre)/ln(0.5)$$ 

$\qquad$上述退火速率是一个静态的调整方式，但更加有效的方式可以使用自适应的方式，根据目标函数值等信息来动态调整退火速率。

#  2、结合SA的登山算法
$\qquad$登山算法为LS提供了一个通用的框架来求解棘手的离散优化问题。所有的登山算法都有相似的结构，只在下述两个部分稍有改动。第一部分为**登山随机变量**，它用来判断是接收一个解还是拒绝一个解；第二部分为**邻域结构**，这个跟问题的特征息息相关。下面使用SA作为**登山随机变量**，给出嵌入SA的登山算法伪代码。
##  2.1 Hill Climbing Pseudo Code
1：选择一个初始解 $\omega \in \Omega$，初始化初始温度$t_0=0.005*f(\omega)/ln(0.5)$
2：设置外循环最大迭代次数 $K$ 和内循环最大迭代次数数组$M_k,\ k=1,2,...,K$
3：设置外/内循环迭代计数变量$k=m=1$
4：Repeat while $k \leq K$
5：$\qquad$Repeat while $m \leq M_k$
6：$\qquad$生成一个邻域解$\omega' \in N(\omega)$
7：$\qquad$计算当前解和邻域解之间的差值$\Delta_{\omega, \omega'}=f(\omega')-f(\omega)$
8：$\qquad$If $\Delta_{\omega, \omega'} \leq0$ then $\omega ←\omega'$
9：$\qquad$If $\Delta_{\omega, \omega'} >0$ then $\omega ←\omega'$ 依概率 $exp(-\Delta_{\omega, \omega'}/t_k)$
10:$\qquad m←m+1$
11:$\qquad$Until $m=M_k$
12: 更新温度参数$t_k←t_k*0.99975$
13: If $t_k<\frac{1}{3}*t_0$ Then 重新设置$t_0=0.005*f(\omega)/ln(0.5)$
14: $k←k+1$
15: Until $k=K$ 或者其他终止准则满足结束算法
##  2.2 应用于VRPTW问题
$\qquad$在SA+HC算法应用到VRPTW问题时，邻域结构选择移除和插入一个客户点，LS算法细节可见<https://blog.csdn.net/weixin_43160744/article/details/119107248>之后套用上述算法框架即可。
