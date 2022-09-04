###  mini-cheetah 状态估计算法

#### 理论部分

MIT状态估计包括了姿态估计和位置速度估计，其中位置和速度估计应用了kalman滤波算法。

kalman滤波分为预测步和更新步，算法过程如下：
$$
\begin{align}
机器人的状态方程和观测方程： \\
状态方程：&x_{k+1}=A_kx_k+B_ku_k+Q_k \\
观测方程：&y_k = C_kx_{k+1}+R_k \\ 
\\ 
\\
kalman滤波：  \\
预测步：\\ &x^{-}_{k+1}=A_kx_k+B_ku_k \\
&P^{-}_{k+1}=A_kP_kA^T_{k}+Q_k \\

更新步：\\ &K_{k+1}=P^{-}_{k+1}C^T_{k+1}[C_{k+1}P^{-}_{k+1}C^T_{k+1}+R_k]^{-1} \\
&e_{k+1}=z_{k+1}-C_{k+1}x^{-}_{k+1} \\
&x_{k+1}=x^{-}_{k+1}+K_{k+1}e_{k+1} \\
&P_{k+1}=(I-K_{k+1}C_{k+1})P^{-}_{k+1}(I-K_{k+1}C_{k+1})^T+K_{k+1}R_{k+1}K_{k+1}^T
\end{align}
$$

* 其中  $x_k=[p^T \ v^T \ p_1^T \ p_2^T \ p_3^T \ p_4^T]^T$  ，$p$ 是在世界坐标系下机身的位置，$v$ 是在世界坐标系下机身的速度。 $p_i$ 是第 $i$ 条腿在世界坐标系下的位置。

#### 代码部分



#### 参考资料

* [State Estimation for Legged Robots -Consistent Fusion of Leg Kinematics and IMU]()

* [Cheetah-Software方案分析](https://blog.csdn.net/Kalenee/article/details/126440918)
* [MIT开源四足机器人状态估计算法](https://zhuanlan.zhihu.com/p/354034309)





