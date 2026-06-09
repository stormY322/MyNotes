
仅考虑旋转关节。

将n个关节角度表示为$\theta  = (\theta_{1},\theta_{2},\dots, \theta_{n})^{\mathrm{T}}$，k个末端执行器的位置表示为$(s_{1},s_{2},\dots,s_{k})^{\mathrm{T}}$，每个末端执行器的位置都是一个关节角度的函数，即$s_{i} = s_{i}(\theta)$

target positions被表示为$(t_{1},t_{2},\dots,t_{k})^{\mathrm{T}}$

逆运动学就是找一组$\theta$令$s_{i}(\theta) = t_{i}$。由于机器人的旋转通常使用正弦余弦等描述，寻找解的过程本质上是求解非线性方程组。

为了求解这个非线性方程组，可以用一阶导做线性近似。
$$
	f(\theta + \Delta\theta) = f(\theta) +  \frac{ \partial f }{ \partial \theta }|_{\theta} \Delta\theta + O(\Delta\theta^2)
$$
$$
	\dot{s} = J(\theta)\dot{\theta}
$$
也就出现了Jacobian。

在离散时间内，方程变为
$$
	\Delta s \approx J \Delta \theta
$$
所以目标就是在这个离散时间内，找到$\Delta \theta$，让$J\Delta \theta = t-s$

## Jacobian Transpose method

避免计算矩阵的逆，将迭代更新量定义为
$$
	\Delta \theta = \alpha J^{\mathrm{T}}e
$$
证明有效性
当$\Delta \theta = \alpha J^{\mathrm{T}} e$时，位置变化为$\Delta s = \alpha J J^{\mathrm{T}} e$
对任意矩阵$J$和误差向量$e$，求内积
$$
	\langle JJ^{\mathrm{T}}e,e\rangle = \langle J^{\mathrm{T}}e,J^{\mathrm{T}}e \rangle = \lVert J^{\mathrm{T}}e \rVert^2 \geq 0
$$
意味着对任意$J,e$，位移向量与误差向量的夹角小于九十度，也就是一定会缩减误差向量的模长。

其中，最优步长一般被设为
$$
	\alpha = \frac{\langle e,JJ^{\mathrm{T}}e \rangle}{\langle J J^{\mathrm{T}}e, J J^{\mathrm{T}}e \rangle}
$$

## Pseudoinverse method
伪逆法是寻求直接解$e = J\Delta \theta$的方法，直接用J的Moore-Penrose逆获得$\Delta \theta$
$$
	\Delta \theta = J^{\dagger}e
$$

伪逆法具有两个最优化性质：
	1. 在可到达情况下能给出最小模长的$\Delta \theta$
	2. 在不可到达情况下能给出令残差$\lVert J\Delta\theta-e \rVert$最小的解

同时，可以通过伪逆求解零空间，即对于任意向量$\phi$，$J(I-J^{\dagger}J)\phi = 0$，这个矩阵$I-J^{\dagger}J$能将任意关节速度向量投影到J中并不产生任何效果，即投影到零空间中。

因此，解可以为
$$
	\Delta \theta = J^{\dagger}e + (I-J^{\dagger}J)\phi
$$
与
$$
	\Delta\theta = J^{\dagger} e
$$
等价。

## Damped Least Squares method

与伪逆法相比，DLS不再只追求最小化$\lVert J\Delta\theta - e \rVert ^2$，而是也考虑目前构型的稳定性，即最小化
$$
	\lVert J\Delta\theta-e \rVert ^2 + \lambda^2 \lVert \Delta\theta \rVert^2
$$
从数学角度上来说
$$
\begin{align}
	E(\Delta \theta) &= 	\lVert J\Delta\theta-e \rVert ^2 + \lambda^2 \lVert \Delta\theta \rVert^2 \\
	&= \frac{1}{2}(J\Delta\theta-e)^{\mathrm{T}}(J\Delta\theta-e) + \frac{1}{2} \lambda^2\Delta\theta ^{\mathrm{T}}\Delta\theta \\
	&=\frac{1}{2}(\Delta\theta ^{\mathrm{T}}J^{\mathrm{T}}J\Delta\theta-\Delta\theta ^{\mathrm{T}}J^{\mathrm{T}}e-e^{\mathrm{T}}J\Delta\theta+e^{\mathrm{T}}e)+\frac{1}{2}\lambda^2\Delta\theta ^{\mathrm{T}}\Delta\theta \\ \\
	\frac{ \partial E }{ \partial \Delta\theta } & = J^{\mathrm{T}}J\Delta\theta-J^{\mathrm{T}}e+\lambda^2\Delta\theta = 0 \\ \\
	 \\
	\therefore (J^{\mathrm{T}}J+\lambda^2I)\Delta\theta  &= J^{\mathrm{T}}e \\
	\Delta\theta &= (J^{\mathrm{T}}J + \lambda^2I)^{-1}J^{\mathrm{T}}e
\end{align}
$$

但是在实际上使用时，这个计算量过大。

使用以下恒等式，对矩阵进行变换。证明方式为左乘$(J^{\mathrm{T}}J + \lambda^2I)$，右乘$(JJ^{\mathrm{T}} + \lambda^2I)$，展开结果一致。
$$
	(J^{\mathrm{T}}J + \lambda^2I)^{-1}J^{\mathrm{T}} = J^{\mathrm{T}}(JJ^{\mathrm{T}} + \lambda^2I)^{-1}
$$
所以
$$
	\Delta\theta = J^{\mathrm{T}}(JJ^{\mathrm{T}}+\lambda^2I)^{-1}e
$$

一种计算方式是将$\lVert J\Delta\theta-e \rVert ^2 + \lambda^2 \lVert \Delta\theta \rVert^2$转化为增广矩阵的二范数
$$
	\left\| \begin{pmatrix} 
	J \\
	\lambda I
	\end{pmatrix}\Delta\theta - 
	\begin{pmatrix}
	e \\
	0
	\end{pmatrix}
	\right\|
$$用标准最小二乘法的方程$(A^{\mathrm{T}}A)x = A^{\mathrm{T}}b$，得到
$$
	\begin{pmatrix}
		J \\
		\lambda I
	\end{pmatrix} ^{\mathrm{T}}
	\begin{pmatrix}
		J \\
		\lambda I
	\end{pmatrix} \Delta \theta = 
	\begin{pmatrix}
		J \\
		\lambda I
	\end{pmatrix} ^{\mathrm{T}} 
	\begin{pmatrix}
		e \\
		0
	\end{pmatrix}
$$
注意 
$$
	\begin{pmatrix}
		A \\
		B
	\end{pmatrix} A^{\mathrm{T}} = 
	\begin{pmatrix}
		A^{\mathrm{T}} & B^{\mathrm{T}}
	\end{pmatrix}
$$
$$
	\Delta\theta = (J^{\mathrm{T}}J + \lambda^2I)^{-1}J^{\mathrm{T}}e
$$
对于$\lambda$选择，有多种动态选择算法。

这种方法是受到岭回归的启发，具有相同的形式。


可以通过SVD的角度来分析伪逆和DLS之间的差别。

对Jacobian进行SVD分解得
$$
	J = U D V^{\mathrm{T}} = \Sigma^{m}_{i=1}\sigma_{i}u_{i}v_{i}^{\mathrm{T}}
$$
所以J的伪逆
$$
	J^{\dagger} = V D ^{\dagger}U^{\mathrm{T}} = \Sigma^{m}_{i=1}\sigma_{i}^{-1}v_{i}u_{i}^{\mathrm{T}}
$$
同样，对DLS用SVD分析，寻找$J^{\mathrm{T}}(JJ^{\mathrm{T}}+\lambda^2I)^{-1}$的表达，与$J^{\dagger}$比较
$$
	\begin{align}
		JJ^{\mathrm{T}} + \lambda^2I = (UDV^{\mathrm{T}})(VD^{\mathrm{T}}U^{\mathrm{T}}) + \lambda^2I &  = U(DD^{\mathrm{T}}+\lambda^2I)U^{\mathrm{T}} \\
	\end{align}
$$
其中，$DD^{\mathrm{T}}+\lambda^2I$是一个对角阵，对角线上的值为$\sigma_{i}^2 + \lambda^2$，所以这个矩阵的逆同样是对角阵，值为$\frac{1}{\sigma^2_{i}+\lambda^2}$。

所以
$$
	J^{\mathrm{T}}(JJ^{\mathrm{T}}+\lambda^2I)^{-1} = VD^{\mathrm{T}}(DD^{\mathrm{T}}+\lambda^2I)^{-1}U^{\mathrm{T}}=\Sigma_{i=1}^m \frac{\sigma_{i}}{\sigma_{i}^2+\lambda^2}v_{i}u_{i}^{\mathrm{T}}
$$

注意到，伪逆法系数为$\frac{1}{\sigma_{i}}$，DLS系数为$\frac{\sigma_{i}}{\sigma_{i}^2+\lambda^2}$

表明，当远离奇异点，即$\sigma$比较大时，两者相似。但当$\sigma$趋近于0，也就是接近奇异点时，伪逆法的系数趋于无穷，而DLS则趋近于0，避免$\Delta \theta$过大。

注意，Jacobian的SVD分解中，$\Sigma$矩阵中的特征值表明了对该方向上的传动比，$\sigma$越大表明这个方向上关节的转动越容易使末端执行器产生巨大的位移。

当接近奇异点时，$\sigma$趋近于0。
