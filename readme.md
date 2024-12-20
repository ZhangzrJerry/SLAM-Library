# SLAM 文献库

## Contents

- **[Filter-Based SLAM](#Filter-Based SLAM)**

  - *[Dynamic Bayesian Network](#Dynamic Bayesian Network)*

- **[Graph-Based SLAM](#Graph-Based SLAM)**

  - *[Factor Graph](#Factor Graph)*
    - [A Tutorial on Graph-Based SLAM](#A Tutorial on Graph-Based SLAM)
  - *[Observability and Consistency](#Observability and Consistency)*
    - [On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation](#On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation)
  - *[Initialization](#Initialization)*
    - [Robust initialization of monocular visual-inertial estimation on aerial robots](#Robust initialization of monocular visual-inertial estimation on aerial robots)
    - [Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling shutter cameras](#Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling shutter cameras)
  - *[Visual SLAM](#Visual SLAM)*
  - *[Lidar SLAM](#Lidar SLAM)*
  - *[Radar SLAM](#Radar SLAM)*
  - *[Multi-Sensor SLAM](#Multi-Sensor SLAM)*

- **Learning-Based SLAM**
  - Survey
  - Computer Vision
  
- **[SLAM for Swarm](#SLAM for Swarm)**
  - [EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments](#EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments)
  
---


## Filter-Based SLAM

### Dynamic Bayesian Network


## Graph-Based SLAM

### Factor Graph

#### Exactly Sparse Extended Information Filters for Feature-based SLAM

M. R. Walter, R. M. Eustice, and J. J. Leonard, “Exactly Sparse Extended Information Filters for Feature-based SLAM,” *The International Journal of Robotics Research*, vol. 26, no. 4, pp. 335–359, Apr. 2007, doi: [10.1177/0278364906075026](https://doi.org/10.1177/0278364906075026).

作者给出了在边际概率和条件概率中协方差矩阵和信息矩阵的表达的总结

<img src="./assets/image-20241216171952392.png" alt="image-20241216171952392" style="zoom:50%;" />

当需要删除一个变量时，直接丢弃变量和对应的测量值,会使系统损失一部分信息。更合适的做法是使用边际概率,将丢弃变量所携带的信息通过剩余变量的隐式表达来保留。在边缘化的操作中，稀疏的信息矩阵慢慢变得稠密，原本不相关的变量慢慢变得相关。

<img src="./assets/image-20241216143119541.png" alt="image-20241216143119541" style="zoom:33%;" />

---

#### A Tutorial on Graph-Based SLAM

G. Grisetti, R. Kummerle, C. Stachniss, and W. Burgard, “A Tutorial on Graph-Based SLAM,” *IEEE Intell. Transport. Syst. Mag.*, vol. 2, no. 4, pp. 31–43, 2010, doi: [10.1109/MITS.2010.939925](https://doi.org/10.1109/MITS.2010.939925).

SLAM问题被定义为：在给定初状态和一系列输入与观测的基础上，找到有最大似然概率的轨迹和地图。其中常见的地图表达包括多层表面地图、点云地图和二维栅格地图。
$$
p(\bold x_{1:t},\bold m|\bold z_{1:t},\bold u_{1:t},\bold x_0)
$$
在图优化SLAM中，待估计的变量被定义为节点，而待估计变量构成的空间约束以残差形式构成图的边。图优化SLAM由此被解耦为从原始传感器数据构成图（前端）和在约束中找到有最大似然概率的参数（后端）两个问题。本文假设前端很完美地找到观测的对应关系，待优化的图被很好地建立起来的基础上，讨论后端图优化的基本技术。
$$
\bold x^* = \arg\min_{\bold x}\sum_{(i,j)\in\mathcal C} \bold e_{ij}^T\Omega_{ij}\bold e_{ij}
$$
其中残差利用一阶雅可比近似
$$
\bold e_{ij}(\breve{\bold x}_i+\Delta\bold x_i,\breve{\bold x}_j+\Delta\bold x_j) = \bold e_{ij}(\breve{\bold x}+\Delta\bold x) \approx \bold e_{ij} + \bold J_{ij}\Delta\bold x
$$
总损失函数有
$$
\begin{aligned}
\bold F(\breve{\bold x}+\Delta\bold x) &= \sum_{(i,j)\in\mathcal C}\bold e_{ij}(\breve{\bold x}+\Delta\bold x)^T\Omega_{ij}\bold e_{ij}(\breve{\bold x}+\Delta\bold x) \\
&\approx \sum_{(i,j)\in\mathcal C}(\bold e_{ij} + \bold J_{ij}\Delta\bold x)^T\Omega_{ij}(\bold e_{ij} + \bold J_{ij}\Delta\bold x) \\
&= \sum_{(i,j)\in\mathcal C}\underbrace{\bold e_{ij}^T\Omega_{ij}\bold e_{ij}}_{c_{ij}} + 2\underbrace{\bold e_{ij}^T\Omega_{ij}\bold J_{ij}}_{\bold b_{ij}}\Delta\bold x + \Delta\bold x^T\underbrace{\bold J_{ij}^T\Omega_{ij}\bold J_{ij}}_{\bold H_{ij}}\Delta\bold x \\
&= c+2\bold b^T\Delta\bold x + \Delta\bold x^T\bold H\Delta\bold x
\end{aligned}
$$
当上式关于 $\Delta\bold x$ 的导数为 $\bf0$ 时
$$
\bold H\Delta\bold x^* = -\bold b
$$
进一步论文给出了一种定义在流形上的误差项
$$
\bold e_{ij}(\bold x) = (\bold z_{ij}^{-1}\oplus(\bold x_i^{-1}\oplus\bold x_j))_{[1:6]} = \breve{\bold e}_{ij}+\tilde{\bold J}_{ij}\Delta\tilde{\bold x}
$$

$$
\begin{aligned}
\tilde{\bold J}_{ij} &= \begin{bmatrix} \cdots & \frac{\partial\bold e_{ij}(\breve{\bold x}\boxplus\Delta\tilde{\bold x})}{\partial\Delta\tilde{\bold x}_i}\bigg|_{\Delta\tilde{\bold x}=0} & \cdots & \frac{\partial\bold e_{ij}(\breve{\bold x}\boxplus\Delta\tilde{\bold x})}{\partial\Delta\tilde{\bold x}_j}\bigg|_{\Delta\tilde{\bold x}=0} & \cdots \end{bmatrix} \\
&= \begin{bmatrix} \cdots & \frac{\partial\bold e_{ij}(\breve{\bold x})}{\partial\breve{\bold x}_i}\cdot\frac{\breve{\bold x}_i\boxplus\Delta\tilde{\bold x}}{\partial\Delta\tilde{\bold x}_i}\bigg|_{\Delta\tilde{\bold x}=0} & \cdots & \frac{\partial\bold e_{ij}(\breve{\bold x})}{\partial\breve{\bold x}_j}\cdot\frac{\breve{\bold x}_j\boxplus\Delta\tilde{\bold x}}{\partial\Delta\tilde{\bold x}_j}\bigg|_{\Delta\tilde{\bold x}=0}  & \cdots \end{bmatrix}
\end{aligned}
$$

其中增量算子 $\oplus,\boxplus$ 为
$$
\bold x_i\oplus\bold x_j = \begin{bmatrix}\bold q_i(\bold t_j) \\ \bold q_i\otimes\bold q_j\end{bmatrix}
$$

$$
\bold x_i\boxplus\Delta\tilde{\bold x}_i = \bold x_i\oplus\begin{bmatrix} \Delta\tilde{\bold t}_i \\ \Delta\tilde{\bold q}_i \\ \sqrt{1-||\Delta\tilde{\bold q}_i||^2} \end{bmatrix}
$$

---

#### G^2^o: A general framework for graph optimization

R. Kummerle, G. Grisetti, H. Strasdat, K. Konolige, and W. Burgard, “G^2^o: A general framework for graph optimization,” in _2011 IEEE International Conference on Robotics and Automation_, Shanghai, China: IEEE, May 2011, pp. 3607–3613. doi: [10.1109/ICRA.2011.5979949](https://doi.org/10.1109/ICRA.2011.5979949).

基本上沿续了 **A Tutorial on Graph-Based SLAM** 的思路，补充了一些工程上的实现和实验的结果：使用阻尼法 $(\bold H+\lambda\bold I)\Delta\bold x^*=-\bold b$ 求解高斯牛顿，利用信息矩阵自伴随的特性用舒尔补快速求解 $\Delta\bold x^*$

---

### Observability and Consistency

#### Observability and fisher information matrix in nonlinear regression

C. Jauffret, “Observability and fisher information matrix in nonlinear regression,” *IEEE Transactions on Aerospace and Electronic Systems*, vol. 43, no. 2, pp. 756–759, 2007, doi: [10.1109/TAES.2007.4285368](https://doi.org/10.1109/TAES.2007.4285368).

给出了三种可观性的定义

在 $\theta_0$ 处简单可观 **Simply Observable**
$$
\forall\theta\in\R^d, \theta\ne\theta_0\Rightarrow h(\theta)\ne h(\theta_0)
$$
全局可观 **(Simply) Observable**
$$
\forall\theta,\theta'\in\R^d, \theta\ne\theta'\Rightarrow h(\theta)\ne h(\theta')
$$
在 $\theta_0$ 处局部可观 **Locally Observable**
$$
\exist U_{\theta_0} \subset\R^d\ (\text{open subset containing }\theta_0) \\
\forall \theta\in U_{\theta_0},\ \theta\ne\theta_0\Rightarrow h(\theta)\ne h(\theta_0)
$$
那么对于测量模型 $X=h(\theta)+\epsilon$ 而言，如果噪声的支持集 $p_\epsilon=\R^n$ 并且它的信息矩阵在 $\theta_0$ 处非奇异，或等价地 $\nabla_\theta h(\theta_0)\nabla_\theta^Th(\theta_0)$ 非奇异，那么该无噪声系统在 $\theta_0$ 处局部可观

---

#### On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation

Z. Zhang, G. Gallego, and D. Scaramuzza, “On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation,” *IEEE Robot. Autom. Lett.*, vol. 3, no. 3, pp. 2710–2717, Jul. 2018, doi: [10.1109/LRA.2018.2833152](https://doi.org/10.1109/LRA.2018.2833152).

视觉惯性系统存在四自由度不可观（平移和绕重力方向的旋转），高斯牛顿法求解线性方程时，奇异矩阵H不满秩（存在四维的零空间），求解稳定性较差。这些额外的规范自由度 Gauge Freedom 必须被妥善处理。作者对比了三种常见的处理策略以及它们对应的效果

**Gauge Fixation** 选择将第一帧的摄像机位姿固定，在全局优化中不更新。这等价于在全局优化时将第一帧摄像机位姿对应的残差的雅可比矩阵设为零矩阵。

**Gauge Prior** 选择为第一帧位姿加上定义在流形上的正则化惩罚项 $||\bold r_0^P||_{\Sigma_0^P}^2=||(\bold p_0-\bold p_0^0,\Delta\phi_{0z})||_{\Sigma_0^P}^2$

**Free Gauge** 方法则不增加先验约束，而是在优化时使用阻尼牛顿法，使 $\epsilon\bold I+\bold H$ 正定

作者接下来讨论了 **Gauge Prior** 中权重矩阵的选择的策略，并对比了三种方法的效果的差异。在权重矩阵等于协方差构成的单位阵 $\bold\Sigma_0^P = \boldsymbol\sigma_0^2\bold I$ 时，$||\bold r_0^P||_{\Sigma_0^P}^2=w^P||\bold r_0^P||^2, w^P = \frac1{\sigma_0^2}$。当 $w^P\to\infty$ 时，**Gauge Prior** 变成 **Gauge Fixation**，当 $w^P=0$ 时，**Gauge Prior** 变成 **Gauge Free**

权重 $w$ 对均方误差、迭代次数、收敛时间的影响都不大，并且在超过某个阈值后，三者都趋于平稳 

<img src="./assets/image-20241216140001042.png" alt="image-20241216140001042" style="zoom:50%;" />

作者进一步实验讨论这种反常的收敛时间的增加。对于非常大的先验权重 $10^8$，该算法会降低重新投影误差，同时保持先验误差几乎等于零。相反，对于较小的先验权重（例如  $50\sim500$），优化算法会在前两次迭代中降低重新投影误差，但代价是增加先验误差。然后优化算法花费许多次迭代来微调先验误差，同时保持重新投影误差较小（沿轨道移动），因此计算时间会增加。

<img src="./assets/image-20241216140459907.png" alt="image-20241216140459907" style="zoom:50%;" />

总的来说，三种方法对精度的影响并不显著。在权重合适的情况下，**Gauge Prior** 方法与 **Gauge Fixation** 几乎有相同的表现，而 **Free Gauge** 方法由于需要更少的迭代次数来收敛，消耗的时间会较少

---

#### Consistency analysis for sliding-window visual odometry

T.-C. Dong-Si and A. I. Mourikis, “Consistency analysis for sliding-window visual odometry,” in *2012 IEEE International Conference on Robotics and Automation*, St Paul, MN, USA: IEEE, May 2012, pp. 5202–5209. doi: [10.1109/ICRA.2012.6225246](https://doi.org/10.1109/ICRA.2012.6225246).

在边际化过程中看似信息增加，实则可信度下降了，造成了可观性的退化。将FEJ方法应用在滑窗，避免了秩的增加和虚假信息的产生。但这导致线性化精确程度的失准，不过这种损失并不显著。

#### On the Importance of Uncertainty Representation in Active SLAM

M. L. Rodriguez-Arevalo, J. Neira, and J. A. Castellanos, “On the Importance of Uncertainty Representation in Active SLAM,” *IEEE Trans. Robot.*, vol. 34, no. 3, pp. 829–834, Jun. 2018, doi: [10.1109/TRO.2018.2808902](https://doi.org/10.1109/TRO.2018.2808902). ==TODO==

---

#### Symmetry and Uncertainty-Aware Object SLAM for 6DoF Object Pose Estimation

N. Merrill *et al.*, “Symmetry and Uncertainty-Aware Object SLAM for 6DoF Object Pose Estimation,” in *2022 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)*, New Orleans, LA, USA: IEEE, Jun. 2022, pp. 14881–14890. doi: [10.1109/CVPR52688.2022.01448](https://doi.org/10.1109/CVPR52688.2022.01448). ==TODO==

---

### Initialization

#### Inertial-Based Scale Estimation for Structure from Motion on Mobile Devices

J. Mustaniemi, J. Kannala, S. Särkkä, J. Matas, and J. Heikkilä, “Inertial-based scale estimation for structure from motion on mobile devices,” in *2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Vancouver, BC, Canada: IEEE Press, 2017, pp. 4394–4401. doi: [10.1109/IROS.2017.8206303](https://doi.org/10.1109/IROS.2017.8206303).

利用IMU原始信息而不是积分信息完成对齐工作，避免误差在预积分过程中被累计，开源了MATLAB代码。

通过图像计算相机外参数，利用B样条曲线插值得到角速度和线性加速度，再与IMU测量的原始角速度加速度构建联合损失函数。同时在频域上针对IMU噪声特性，引入高通滤波和低通滤波抑制噪声。

<img src="./assets/50f77ce956c07102e9c33189d21ad027.jpg" alt="50f77ce956c07102e9c33189d21ad027" style="zoom:20%;" />

---

#### Robust initialization of monocular visual-inertial estimation on aerial robots

T. Qin and S. Shen, “Robust initialization of monocular visual-inertial estimation on aerial robots,” in *2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Vancouver, BC: IEEE, Sep. 2017, pp. 4225–4232. doi: [10.1109/IROS.2017.8206284](https://doi.org/10.1109/IROS.2017.8206284).

利用松耦合的方式对齐IMU与视觉

<img src="./assets/image-20241208153454063.png" alt="image-20241208153454063" style="zoom:67%;" />

IMU测量模型有
$$
\begin{aligned}
\hat{\boldsymbol\omega} &= \boldsymbol\omega^b+\mathbf b^g+\mathbf n^g \\
\hat{\mathbf a} &= \mathbf R_{bw}(\mathbf a^w+\mathbf g^w)+\mathbf b^a+\mathbf n^a
\end{aligned} 
$$
预积分
$$
\begin{aligned}
\boldsymbol\alpha_{b_ib_j}&=\iint_{t \in [i,j]} \mathbf R_{b_ib_t} (\hat{\mathbf a}^{b_t}-\mathbf b^a) {\rm d} t^2\\
\boldsymbol\beta_{b_ib_j} &= \int_{t \in [i,j]} \mathbf R_{b_ib_t} (\hat{\mathbf a}^{b_t}-\mathbf b^a) {\rm d} t \\
\mathbf q_{b_ib_j}&= \int_{t \in [i,j]} \mathbf q_{b_ib_t} \otimes \begin{bmatrix} 0 \\ \frac12(\hat{\boldsymbol\omega}^{b_t}-\mathbf b^g) \end{bmatrix} {\rm d}t
\end{aligned}
$$
有一阶泰勒展开近似
$$
\begin{aligned}
\boldsymbol\alpha_{b_ib_j} &\approx \hat{\boldsymbol\alpha}_{b_ib_j} + \mathbf J_{b^a}^\alpha\delta\mathbf b^a + \mathbf J_{b^g}^\alpha\delta\mathbf b^g \\
\boldsymbol\beta_{b_ib_j} &\approx \hat{\boldsymbol\beta}_{b_ib_j} + \mathbf J_{b^a}^\beta\delta\mathbf b^a + \mathbf J_{b^g}^\beta\delta\mathbf b^g \\
\mathbf q_{b_kb_{k+1}} &\approx \hat{\mathbf q}_{b_kb_{k+1}}\otimes\begin{bmatrix}1\\\frac12\mathbf J_{b^g}^{\mathbf q}\delta\mathbf b^g\end{bmatrix}
\end{aligned}
$$
SfM给出视觉约束，$\bar ·$ 表示非米制单位
$$
\begin{aligned}
\mathbf q_{c_0b_k} &= \mathbf q_{c_0c_k} \otimes \mathbf q_{bc}^{-1} \\
s\bar{\mathbf p}_{c_0b_k} &= s\bar{\mathbf p}_{c_0c_k} - \mathbf R_{c_0b_k}\mathbf p_{bc}
\end{aligned}
$$

##### 估计外参数 $\bold q_{bc}$

对于相邻时刻 $k,k+1$ 的 IMU 旋转积分 $\mathbf q_{b_kb_{k+1}}$ 和视觉测量 $\mathbf q_{c_kc_{k+1}}$
$$
\mathbf q_{b_kc_{k+1}}=\mathbf q_{bc}\otimes\mathbf q_{c_kc_{k+1}}=\mathbf q_{b_kb_{k+1}}\otimes\mathbf q_{bc}
$$

$$
\left([\mathbf q_{c_kc_{k+1}}]_R - [\mathbf q_{b_kb_{k+1}}]_L\right)\mathbf q_{bc}=\mathbf0
$$

 $[·]_L, [·]_R$ 为四元数的左乘矩阵和右乘矩阵，$[·]_\times$ 为反对称矩阵
$$
\begin{aligned}\
[\mathbf q]_L &=q_\omega\mathbf I+\begin{bmatrix}0 & -\mathbf q_v^T \\ \mathbf q_v & [\mathbf q_v]_\times\end{bmatrix} \\
[\mathbf q]_R &= q_\omega\mathbf I+\begin{bmatrix}0 & -\mathbf q_v^T \\ \mathbf q_v & -[\mathbf q_v]_\times\end{bmatrix} \\
[\boldsymbol\omega]_\times &= \begin{bmatrix}0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0\end{bmatrix}
\end{aligned}
$$

将多个时刻的 IMU 预积分和视觉测量累计，即可得到关于 $\mathbf q_{bc}$ 的超定方程组
$$
\begin{bmatrix}
[\mathbf q_{c_0c_1}]_R-[\mathbf q_{b_0b_1}]_L \\
[\mathbf q_{c_1c_2}]_R-[\mathbf q_{b_1b_2}]_L \\
\vdots \\
[\mathbf q_{c_nc_{n-1}}]_R-[\mathbf q_{b_nb_{n-1}}] \\
\end{bmatrix} \mathbf q_{bc} = \mathbf 0
$$
利用奇异值分解可解得 $\mathbf q_{bc}$

##### 估计陀螺仪偏置

标定得到 $\mathbf q_{bc}$ 后，利用旋转约束，可估计处陀螺仪偏置
$$
\delta\mathbf b^g = \arg\min_{\delta\mathbf b^g}\sum_{k\in{\mathcal B}} \left|\left|\left\lfloor\mathbf q_{c_0b_{k+1}}^{-1}\otimes\mathbf q_{c_0b_k} \otimes\mathbf q_{b_kb_{k+1}} \right\rfloor_{\text{xyz}}\right|\right|^2
$$
$\mathcal B$ 为所有关键帧的集合，其中目标函数取最小值 $0$ 时
$$
\mathbf q_{c_0b_{k+1}}^{-1}\otimes\mathbf q_{c_0b_k} \otimes\mathbf q_{b_kb_{k+1}}=\begin{bmatrix}1\\\mathbf0\end{bmatrix}
$$
又由旋转预积分一阶泰勒近似
$$
\mathbf q_{b_kb_{k+1}} \approx \hat{\mathbf q}_{b_kb_{k+1}}\otimes\begin{bmatrix}1\\\frac12\mathbf J_{b^g}^{\mathbf q}\delta\mathbf b^g\end{bmatrix}
$$
联立得到
$$
\begin{bmatrix}1\\\frac12\mathbf J_{b^g}^{\mathbf q}\delta\mathbf b^g\end{bmatrix} = \hat{\mathbf q}_{b_kb_{k+1}}^{-1}\otimes\mathbf q_{c_0b_k}^{-1}\otimes\mathbf q_{c_0b_{k+1}}
$$
考虑虚部
$$
\mathbf J_{b^g}^{\mathbf q}\delta\mathbf b^g = 2\left\lfloor\hat{\mathbf q}_{b_kb_{k+1}}^{-1}\otimes\mathbf q_{c_0b_k}^{-1}\otimes\mathbf q_{c_0b_{k+1}}\right\rfloor_{\text{xyz}}
$$
进一步可以构建正定方程组，通过 Cholesky 分解求解 $\delta\mathbf b^g$
$$
(\mathbf J_{b^g}^{\mathbf q})^T\mathbf J_{b^g}^{\mathbf q}\delta\mathbf b^g = 2(\mathbf J_{b^g}^{\mathbf q})^T\left\lfloor\hat{\mathbf q}_{b_kb_{k+1}}^{-1}\otimes\mathbf q_{c_0b_k}^{-1}\otimes\mathbf q_{c_0b_{k+1}}\right\rfloor_{\text{xyz}}
$$
解得 $\delta\mathbf b^g$ 后，重新计算预积分项 $\hat{\boldsymbol\alpha}_{b_kb_k+1}, \hat{\boldsymbol\beta}_{b_kb_{k+1}},\hat{\mathbf q}_{b_kb_{k+1}}$

##### 初始化速度、重力和尺度因子

所有我们希望估计的变量包括
$$
\mathcal X_I=\begin{bmatrix}\mathbf v_0^{b_0} & \mathbf v_1^{b_1} & \cdots \mathbf v_n^{b_n} & \mathbf g^{c_0} & \mathbf s\end{bmatrix}^T
$$
由 $1.3$ 式和 $2.1$ 式，得到
$$
\begin{aligned}
& \boldsymbol\alpha_{b_kb_{k+1}} = \mathbf R_{b_kc_0} \left( s(\bar{\mathbf p}_{c_0b_{k+1}} - \bar{\mathbf p}_{c_0b_k}) + \frac12\mathbf g^{c_0}\Delta t_k^2 - \mathbf R_{c_0b_k}\mathbf v_{k}^{b_k} \Delta t_k \right) \\
& \boldsymbol\beta_{b_kb_{k+1}} = \mathbf R_{b_kc_0} (\mathbf R_{c_0b_{k+1}}\mathbf v_k^{b_k}+\mathbf g^{c_0}\Delta t_k - \mathbf R_{c_0b_k}\mathbf v_k^{b_k})
\end{aligned}
$$
整理方程得到
$$
\hat{\mathbf z}_{b_{k+1}}^{b_k} = \begin{bmatrix} \hat{\boldsymbol\alpha}_{b_kb_{k+1}} - \mathbf p_{bc} + \mathbf R_{b_kc_0}\mathbf R_{c_0b_{k+1}}\mathbf p_{bc} \\ \hat{\boldsymbol\beta}_{b_kb_{k+1}} \end{bmatrix} = \mathbf H^k\mathcal X_I^k+\mathbf n^k
$$
其中
$$
\begin{aligned}
\mathcal X_I^k &= \begin{bmatrix} \mathbf v^{b_k}_k & \mathbf v^{b_{k+1}}_{k+1} & \mathbf g^{c_0} & s \end{bmatrix}^T \\
\mathbf H_{b_{k+1}}^{b_k} &= \begin{bmatrix} -\mathbf I\Delta t_k & \mathbf0 & \frac12\mathbf R_{b_kc_0}\Delta t_k^2 & \mathbf R_{b_kc_0}(\bar{\mathbf p}_{c_0b_{k+1}} - \bar{\mathbf p}_{c_0b_k}) \\ -\mathbf I & \mathbf R_{b_kc_0}\mathbf R_{c_0b_{k+1}} & \mathbf R_{b_kc_0}\Delta t_k & 0 \end{bmatrix}
\end{aligned}
$$
进而可以转化为最小二乘问题求解
$$
\mathcal X_I=\arg\min_{\mathcal X_I}\sum_{k\in\mathcal B}||\hat{\mathbf z}_{b_{k+1}}^{b_k} - \mathbf H_{b_{k+1}}^{b_k}\mathcal X_I^k||^2
$$
同样可以通过 Chologky 分解求得

##### 重力向量优化

在重力模长已知的情况下，重力向量实际自由度为 $2$，可以利用球面坐标进行参数化
$$
\hat{\mathbf g}^{c_0}=||g||\bigg(\frac{\tilde{\mathbf g}^{c_0}}{||\tilde{\mathbf g}^{c_0}||}+w_1\mathbf b_1+w_2\mathbf b_2\bigg) 
$$
$\tilde{\mathbf g}^{c_0}$ 为 $2.15$ 中求得的重力向量，记 $\frac{\tilde{\mathbf g}^{c_0}}{||\tilde{\mathbf g}^{c_0}||}$ 为 $\hat{\bar{\mathbf g}}^{c_0}$

<img src="./assets/image-20241208193854261.png" alt="image-20241208193854261" style="zoom: 50%;" />

可以通过如下方式找到一组基底垂直于 $\hat{\bar{\mathbf g}}^{c_0}$
$$
\begin{aligned}
& \mathbf b_1 = \begin{cases}
\hat{\bar{\mathbf g}}^{c_0}\times[1,0,0]^T, &\hat{\bar{\mathbf g}}^{c_0}\ne[1,0,0]^T \\
\hat{\bar{\mathbf g}}^{c_0}\times[0,0,1]^T, &\text{otherwise}
\end{cases} \\
& \mathbf b_2 = \hat{\bar{\mathbf g}}^{c_0}\times \mathbf b_1
\end{aligned}
$$
将 $2.17$ 式代入 $2.15$ 式，得到
$$
\begin{aligned}
\mathcal X_I^k &= \begin{bmatrix} \mathbf v^{b_k}_k & \mathbf v^{b_{k+1}}_{k+1} & \mathbf w^{c_0} & s \end{bmatrix}^T \\
\mathbf H_{b_{k+1}}^{b_k} &= \begin{bmatrix} -\mathbf I\Delta t_k & \mathbf0 & \frac12\mathbf R_{b_kc_0}\begin{bmatrix}\mathbf b_1^T\\\mathbf b_2^T\end{bmatrix}\Delta t_k^2 & \mathbf R_{b_kc_0}(\bar{\mathbf p}_{c_0b_{k+1}} - \bar{\mathbf p}_{c_0b_k}) \\ -\mathbf I & \mathbf R_{b_kc_0}\mathbf R_{c_0b_{k+1}} & \mathbf R_{b_kc_0}\begin{bmatrix}\mathbf b_1^T\\\mathbf b_2^T\end{bmatrix}\Delta t_k & 0 \end{bmatrix}
\end{aligned}
$$
观测方程变为
$$
\hat{\mathbf z}_{b_{k+1}}^{b_k} = \begin{bmatrix} \hat{\boldsymbol\alpha}_{b_kb_{k+1}} - \mathbf p_{bc} + \mathbf R_{b_kc_0}\mathbf R_{c_0b_{k+1}}\mathbf p_{bc}-\frac12\mathbf R_{b_kc_0}\tilde{\mathbf g}^{c_0}\Delta t_k^2 \\ \hat{\boldsymbol\beta}_{b_kb_{k+1}} -\mathbf R_{b_kc_0}\tilde{\mathbf g}^{c_0}\Delta t_k \end{bmatrix}
$$
利用最小二乘对 $\mathcal X_I^k$ 进一步优化

##### 视觉惯性对齐

根据旋转的性质和李代数的指数映射，我们可以构建从 $c_0$ 系到 $w$ 系的旋转矩阵 $\mathbf R_{wc_0}$
$$
\mathbf R_{wc_0}=\exp\bigg[\arctan\left(\frac{||\hat{\mathbf g}^{c_0}\times\hat{\mathbf g}^{w}||}{\hat{\mathbf g}^{c_0}\cdot\hat{\mathbf g}^{w}}\right)\cdot\frac{\hat{\mathbf g}^{c_0}\times\hat{\mathbf g}^{w}}{||\hat{\mathbf g}^{c_0}\times\hat{\mathbf g}^{w}||}\bigg]
$$

接着为所有 $c_0$ 系为坐标系的向量左乘 $\mathbf R_{wc_0}$，同时将非米制的 $\bar{\mathbf p}$ 通过尺度因子 $s$ 恢复为 $\mathbf p$

##### 未估计的参数

作者通过实验指出二者加速度计偏置 $\mathbf b^a$ 和相机与IMU间的平移向量 $\mathbf p_{bc}$ 对系统精度影响极小，可以不在初始化中显式优化

<img src="./assets/image-20241216154218541.png" alt="image-20241216154218541" style="zoom:67%;" />

---

#### Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling shutter cameras

S. Lovegrove, A. Patron-Perez, and G. Sibley, “Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling shutter cameras,” in *Procedings of the British Machine Vision Conference 2013*, Bristol: British Machine Vision Association, 2013, p. 93.1-93.11. doi: [10.5244/C.27.93](https://doi.org/10.5244/C.27.93).

这篇论文主要工作是建立了卷帘相机在连续时域下的数学模型，初始化方法是用IMU对齐视觉。

相比离散时域，在连续时域上表示有助于融合高帧率的传感器和异步时间戳的设备。对卷帘相机的研究尚未成熟，现有的工作主要集中在如何消除卷帘相机的畸变，而后复用标准全局快门相机的SLAM模型，这种解耦合的处理方式增加了无法修正的偏差。

单目视觉系统存在7自由度不可观：6自由度姿态+尺度，一种方法通过回环检测和序列图松弛对尺度显式参数化，另一种方法是加入可以测量绝对尺度的设备。在本文中作者使用了惯性单元。相比过往建立在欧拉角上的数学表示，作者引入李群和李代数上的旋转表示避免了奇异点，同时能够更好地近似最小扭矩的轨迹。
$$
\bold T_{b,a} = \begin{bmatrix}
\bold R_{b,a} & \bold a_b \\
\bold 0^T & 1
\end{bmatrix},\ \bold T_{b,a}\in\mathbb{SE}3,\ \bold R_{b,a}\in\mathbb{SO}3
$$
作者希望轨迹的参数化方程是局部可控，二阶导连续，能近似最小力矩轨迹的。三次B样条曲线可以很好表示 $\R^3$ 上的轨迹，但在三维旋转上表现一般。因此作者选用了李代数上的累计基函数，这种函数最早被应用在计算机动画中的四元数插值中。

自由度为 $k-1$ 的B样条曲线的标准基函数表示为
$$
\begin{aligned}
\bold p(t) &= \sum_{i=0}^n\bold p_i B_{i,k}(t) \\
&= \bold p_0 B_{0,k}(t) + \bold p_1 B_{1,k}(t) + \cdots + \bold p_n B_{n,k}(t) \\
&= \bold p_0B_{0,k}(t) + \sum_{i=1}^{n-1}\bold p_iB_{i,k}(t) + \bold p_nB_{n,k}(t) \\
&= \bold p_0\tilde B_{0,k}(t) + \sum_{i=1}^n\bold p_i\sum_{j=i}^nB_{j,k}(t) - \sum_{i=0}^{n-1}\bold p_i\sum_{j=i+1}^nB_{j,k}(t) \\
&= \bold p_0\tilde B_{0,k}(t) + \sum_{i=1}^n(\bold p_i-\bold p_{i-1})\tilde B_{i,k}(t)
\end{aligned}
$$
其中 $\bold p_i\in\R^N$ 是在 $t_i,i\in[0,\dots,n]$ 时刻的控制点，$B_{i,k}(t)$ 是由德伯尔-考克斯递归公式 De Boor - Cox recursive formula 计算得到的基函数。进一步利用李群的指数映射和对数映射，将公式重写在李群域上
$$
\bold T_{w,s}(t) = \exp(\tilde B_{0,k}\log(\bold T_{w,0}))\prod_{i=1}\exp(\tilde B_{i,k}(t)\log(\bold T_{w,i-1}^{-1}\bold T_{w,i}))
$$
作者进一步在假设控制点时间间隔不变的情况下（大多数单目系统应该都能满足），对时间做了归一化处理
$$
u(t) = \frac{t-t_0}{\Delta t} - s_i
$$
并在参数 $k=4$ 下转化为矩阵表达
$$
\tilde{\bold B}(u) = \bold C\begin{bmatrix}1\\u\\u^2\\u^3\end{bmatrix},\dot{\tilde{\bold B}}(u) = \bold C\begin{bmatrix}0\\1\\2u\\3u^2\end{bmatrix},\ddot{\tilde{\bold B}}(u) = \bold C\begin{bmatrix}0\\0\\2\\6u\end{bmatrix},\bold C=\frac16\begin{bmatrix}6&0&0&0\\5&3&-3&1\\1&3&3&-2\\0&0&0&1\end{bmatrix}
$$

$$
\bold T_{w,s}(u) = \bold T_{w,i-1}\prod_{j=1}^3\exp(\tilde{\bold B}(u)_j\log(\bold T_{w,i+j-1}^{-1}\bold T_{w,i+j}))
$$

$\dot{\bold T}_{w,s},\ddot{\bold T}_{w,s}$ 则由链式法则和指数映射的一阶近似得到

给定第一次观测的逆深度 $\rho\in\R^+$, 对应点在两帧的坐标分别为 $\bold p_a,\bold p_b\in\R^2$, 其中 $\pi(\bold P)=\frac1{P_2}[P_0,P_1]^T$
$$
\bold p_b = \mathcal W(\bold p_a;\bold T_{b,a},\rho)=\pi\bigg(\begin{bmatrix}\bold K_b&\bold 0\end{bmatrix}\bold T_{b,a}\begin{bmatrix}\bold K_a^{-1}\begin{bmatrix}\bold p_a\\1\end{bmatrix}&\rho\end{bmatrix}\bigg)
$$
对于一般的视觉惯性系统给出损失函数

<img src="./assets/image-20241217134126216.png" alt="image-20241217134126216" style="zoom: 50%;" />

而对卷帘相机则将 $\bold p_b$ 用 $\bold p_b(t)$ 替代重新建模

<img src="./assets/image-20241217134242665.png" alt="image-20241217134242665" style="zoom:50%;" />

<img src="./assets/image-20241217134300099.png" alt="image-20241217134300099" style="zoom:50%;" />

这一系统表现出了良好的自校准的能力

---

### Visual SLAM

#### Performance evaluation of visual SLAM using several feature extractors

J. Klippenstein and H. Zhang, “Performance evaluation of visual SLAM using several feature extractors,” in *2009 IEEE/RSJ International Conference on Intelligent Robots and Systems*, St. Louis, MO, USA: IEEE, Oct. 2009, pp. 1574–1581. doi: [10.1109/IROS.2009.5354001](https://doi.org/10.1109/IROS.2009.5354001).

在平均归一化误差和累计不确定性上对 Harris, KLT 和 SIFT 进行测试
$$
\bar\epsilon_k = \frac1N\sum_{i=1}^N|| \bold r_k^{(i)} - \hat{\bold r}_k^{(i)} ||_{\Sigma_{r,k}^{-1}}^2
$$

$$
\overline{\text{AU}} = \frac1N\sum_{i=1}^N\sum_{k=1}^{N_s}\frac43\pi\sqrt{\det(\Sigma_{k,r})}
$$

实验表明，在大部分室内情况下三种特征提取器的累计不确定性没有显著差异，平均归一化误差也遵循相同的趋势，表现几乎相似。在累计不确定性上，SIFT的表现稍稍更优秀。但本文并没有涉及到特征匹配的相关比较。

<img src="./assets/image-20241216173936054-1734341979242-9.png" alt="image-20241216173936054" style="zoom: 50%;" />

<img src="./assets/image-20241216174010595.png" alt="image-20241216174010595" style="zoom:50%;" />

---

#### An evaluation of the RGB-D SLAM system

F. Endres, J. Hess, N. Engelhard, J. Sturm, D. Cremers, and W. Burgard, “An evaluation of the RGB-D SLAM system,” in *2012 IEEE International Conference on Robotics and Automation*, St Paul, MN, USA: IEEE, May 2012, pp. 1691–1696. doi: [10.1109/ICRA.2012.6225199](https://doi.org/10.1109/ICRA.2012.6225199).

SIFT效果好但计算消耗大，实验采用了基于GPU运算的检测器，ORB计算快并且可以应对视角变换，过少的SURF特征点会导致不准确和运动估计解算失败，过多的特征点让特征匹配变慢并且可能生成更多的假阳匹配，因此实验采用了可变的阈值来维持一定数量的SURF特征点。

三对特征点是刚体变换中所需要的最少的对应点，由此利用RANSAC拒绝错误匹配。然后将特征的位置通过深度信息转换到三维空间，但由于深度相机与彩色相机的快门缺乏同步并且视觉中突出的点往往处于物体便捷，导致三维特征位置往往会有错误的深度。

在这一工作中新捕获的帧会和过往的20帧（3帧最新的和17帧均匀分布的更早的帧）进行比较，如果能够匹配则插入到图中。如果不能和它们匹配，取决于系统一般有两种策略：容忍图碎片化，即存在无边连接的节点，等待后续的回环检测以连接。或在假设运动模型恒定的前提下，将其连接到图的前一个节点中，但这可能导致更高的误差。在这一评估工作中，采用了后者的策略。地图使用基于八叉树的体素地图OctoMap.

第一轮实验中作者在9个FR1数据集上进行测试，最好的测试效果在简单的xyz和rpy序列中取得，最差结果在穿越办公空间的长回合中取得。相比SIFT和SURF在帧之间重合较少、角速度线速度较快的情况下仍能较好的追踪轨迹，ORB特征相比不那么准确，也无法通过调整检测器的参数找到更多关键点来解决。

![c6dbb809c4ab1a6b694b60868d67c9fd](./assets/c6dbb809c4ab1a6b694b60868d67c9fd.jpg)

在实验的过程中作者还发现，当图中存在错误边时会使映射结果变差。作者希望在后续优化关键点匹配方案：如添加特征字典、修建从未被匹配的特征以及直接将关键点作为地标进行非线性优化。

---

#### A comparison of feature descriptors for visual SLAM

J. Hartmann, J. H. Klussendorff, and E. Maehle, “A comparison of feature descriptors for visual SLAM,” in *2013 European Conference on Mobile Robots*, Barcelona, Catalonia, Spain: IEEE, Sep. 2013, pp. 56–61. doi: [10.1109/ECMR.2013.6698820](https://doi.org/10.1109/ECMR.2013.6698820).

特征提取器希望找到特征点亮度、尺度、平移不变的描述，使系统能够完成运动结构恢复和回环检测的任务。

<img src="./assets/image-20241217134940615.png" alt="image-20241217134940615" style="zoom:33%;" />

作者在RGBD数据集和自己的数据集上做了实验，表明SIFT有最好的表现但最耗费计算资源，BRIEF在二进制描述子中有最好的表现，大多数情况下选择哪种描述子对精度事实上影响并不大。

---

#### Keyframe-Based Visual-Inertial SLAM using Nonlinear Optimization

S. Leutenegger, P. Furgale, V. Rabaud, M. Chli, K. Konolige, and R. Siegwart, “Keyframe-Based Visual-Inertial SLAM using Nonlinear Optimization,” in *Robotics: Science and Systems IX*, Robotics: Science and Systems Foundation, Jun. 2013. doi: [10.15607/RSS.2013.IX.037](https://doi.org/10.15607/RSS.2013.IX.037). ==TODO==

---

#### On-Manifold Preintegration for Real-Time Visual--Inertial Odometry 

C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, “On-Manifold Preintegration for Real-Time Visual--Inertial Odometry,” *IEEE Trans. Robot.*, vol. 33, no. 1, pp. 1–21, Feb. 2017, doi: [10.1109/TRO.2016.2597321](https://doi.org/10.1109/TRO.2016.2597321). ==TODO==

感觉这篇的数学表示很详细，结合之前李群李代数的笔记，小小重新推一遍

##### 特殊正交群 $SO(3)$

定义特殊正交群 SO(3)$
$$
SO(3) = \set{R\in\R^{3\times3}:R^TR=I,\det(R)=1}
$$
定义流形上的切空间 $\mathfrak {so}(3)$
$$
\boldsymbol\omega^\wedge = \begin{bmatrix}
\omega_1 \\ \omega_2 \\ \omega_3
\end{bmatrix}^\wedge = \begin{bmatrix}
0 & -\omega_3 & \omega_2 \\
\omega_3 & 0 & -\omega_1 \\
-\omega_2 & \omega_1 & 0 \\
\end{bmatrix} \in \mathfrak{so}(3) \tag{1}
$$
$\mathfrak {so}(3)$ 和 $\R^3$ 的转换由反对称算子给出
$$
S=\boldsymbol\omega^\wedge, S^\vee=\boldsymbol\omega
$$
由指数映射的泰勒展开和叉积的性质可以得到 $\exp:\mathfrak {so}(3)\to SO(3)$
$$
\begin{aligned}
R 
&= \exp(\phi^\wedge) \\ 
&= \sum_{n=0}^\infty\frac{(\phi^\wedge)^n}{n!} \\
&= I + \bigg(||\phi||-\frac{||\phi||^3}{3!}+\frac{||\phi||^5}{5!}+\cdots\bigg)\phi^\wedge+\bigg(\frac{||\phi||^2}{2!}-\frac{||\phi||^4}{4!}+\frac{||\phi||^6}{6!}-\cdots\bigg)(\phi^\wedge)^2 \\
&= I + \frac{\sin||\phi||}{||\phi||}\phi^\wedge+\frac{1-\cos||\phi||}{||\phi||^2}(\phi^\wedge)^2 \\
&\approx I + \phi^\wedge
\end{aligned} \tag{3}
$$
对数映射则由特殊正交群的迹和李群性质得到
$$
\begin{aligned}
{\rm tr}(R) &= {\rm tr}(I) + \frac{\sin||\phi||}{||\phi||}{\rm tr}(\phi^\wedge)+\frac{1-\cos||\phi||}{||\phi||^2}{\rm tr}[(\phi^\wedge)^2] \\
&= 3 + 0 + \frac{1-\cos||\phi||}{||\phi||^2}(-||\phi^\wedge||^2) \\
&= 3 + 0 + \frac{1-\cos||\phi||}{||\phi||^2}(-2||\phi||^2) \\
&= 3 + 2\cos||\phi|| - 2
\end{aligned}
$$

$$
||\phi|| = \arccos\bigg(\frac{{\rm tr}(R) - 1}2\bigg) + 2k\pi
$$

当 $\phi\ne0\Leftrightarrow R\ne I$ 时，构造对此部分和非对称部分
$$
R=\underbrace{\frac{\sin||\phi||}{||\phi||}\phi^\wedge}_{\frac12(R-R^T)}+\underbrace{I+\frac{1-\cos||\phi||}{||\phi||^2}(\phi^\wedge)^2}_{\frac12(R+R^T)}
$$
于是
$$
\begin{aligned}
\phi &= \log(R)^\vee \\
&= \bigg[\frac{||\phi||(R-R^T)}{2\sin||\phi||}\bigg]^\vee \\
&= \frac{||\phi||}{2\sin||\phi||}\begin{bmatrix}
r_{32} - r_{23} \\
r_{13} - r_{31} \\
r_{21} - r_{12}
\end{bmatrix}
\end{aligned} \tag{5}
$$
为简便下面将 $\text{Exp}$ 和 $\text{Log}$ 定义在 $\R^3$ 和 $SO(3)$ 之间

记三维空间中一任意向量 $\bf v$ 绕旋转轴 $\bf n=\begin{pmatrix}n_x\\n_y\\n_z\end{pmatrix}$ 旋转 $\theta$ 得到新向量 $\bf v_{rot}$，其中 $\bf n$ 为单位向量，$n_x^2+n_y^2+n_z^2=1$. 则 $\bf v$ 可以根据平行和正交于 $\bf n$ 分为两部分 $\bf v_\parallel$ 和 $\bf v_\perp$，满足 $\bf v=\bf v_\parallel+\bf v_\perp$

其中

$$
\begin{cases}
\mathbf v_\parallel = (\mathbf v\cdot\mathbf n)\mathbf n \\
\mathbf v_\perp = \mathbf v-(\mathbf v\cdot\mathbf n)\mathbf n = -\mathbf n\times(\mathbf n\times\mathbf v)
\end{cases}
$$

$$
\begin{aligned}
\mathbf v_{rot} &= \mathbf v_{\parallel rot} + \mathbf v_{\perp rot} \\
&= \mathbf v_\parallel + \cos\theta\mathbf v_\perp + \sin\theta\mathbf n\times\bf v_\perp \\
&= (\mathbf v\cdot\mathbf n)n + \cos\theta\mathbf v_\perp + \sin\theta\mathbf n\times\mathbf v \\
&= \cos\theta\mathbf v+(1-\cos\theta)(\mathbf n\cdot\mathbf v)\mathbf n + \sin\theta\mathbf n\times\mathbf v
\end{aligned}
$$

也就得到了罗德里格斯旋转公式 Rodrigues' Rotation Formula

将其重新写在流形上则有
$$
R = \cos(||\phi||)I+[1-\cos(||\phi||)]\frac{\phi\phi^T}{||\phi||^2} + \sin(||\phi||)\phi^\wedge
$$
由此得到$R$ 的左右扰动雅可比矩阵
$$
\begin{cases}
J_l(\phi) &= I + \frac{1-\cos(||\phi||)}{||\phi||^2}\phi^\wedge + \frac{||\phi||-\sin(||\phi||)}{||\phi||^3}(\phi^\wedge)^2 \\
J_r(\phi) &= I - \frac{1-\cos(||\phi||)}{||\phi||^2}\phi^\wedge + \frac{||\phi||-\sin(||\phi||)}{||\phi||^3}(\phi^\wedge)^2 \\
J_l^{-1}(\phi) &= I - \frac12\phi^\wedge + \left(\frac1{||\phi||^2} + \frac{1+\cos(||\phi||)}{2||\phi||\sin(||\phi||)}\right)(\phi^\wedge)^2 \\
J_r^{-1}(\phi) &= I + \frac12\phi^\wedge + \left(\frac1{||\phi||^2} + \frac{1+\cos(||\phi||)}{2||\phi||\sin(||\phi||)}\right)(\phi^\wedge)^2 \\
\end{cases} \tag{8}
$$
由于特殊正交群无法定义一般形式的增量，只能利用李代数的扰动模型
$$
\text{Exp}(\phi+\Delta\phi) = \text{Exp}(\bold J_l\Delta\phi)\text{Exp}(\phi) = \text{Exp}(\phi) = \text{Exp}(\bold J_r \Delta\phi) \tag{7}
$$
李群上有二元运算符李括号
$$
[A,B] = AB-BA
$$
由贝克-坎贝尔-豪斯多夫公式 Baker-Campbell-Hausdorff Formula 可得
$$
\begin{aligned}
&\log(\text{Exp}(\alpha)\text{Exp}(\beta))\\
=&\log(AB)\\=&\sum_{n=1}^\infty\frac{(-1)^{n-1}}n\sum_{r_i+s_i>0,i\in[1,n]}\frac{(\sum_{i=1}^n(r_i+s_i))^{-1}}{\Pi_{i=1}^n(r_i!s_i!)}[A^{r_1}B^{s_1}A^{r_2}B^{s_2}\cdots A^{r_n}B^{s_n}]\\
=&A+B+\frac12[A,B]+\frac1{12}[A,[A,B]]-\frac1{12}[B,[A,B]]+\cdots\\
\approx&
\begin{cases}
\beta+J_l(\beta)^{-1}\alpha,&\alpha\to0\\
\alpha+J_r(\alpha)^{-1}\beta,&\beta\to0\\
\end{cases}
\end{aligned}\tag{9}
$$
对任意向量 $v\in\R^3$, 由叉积及特殊正交群的性质可得
$$
(Rp)^\wedge v = (Rp) \times v = (Rp) \times (RR^{-1}v) = R[p\times (R^{-1}v)] = Rp^\wedge R^Tv
$$
$Rp^\wedge R^T=(Rp)^\wedge$ 对指数映射泰勒展开的每一项成立，因此有
$$
R\exp(\phi^\wedge)R^T = \exp((R\phi)^\wedge)
$$
等价地有
$$
R\text{Exp}(\phi)R^T=\text{Exp}(R\phi) \tag{10}
$$

##### 特殊正交群的不确定性描述

一种直觉地定义是在旋转矩阵上右乘一个微小扰动，这个扰动服从正态分布
$$
\tilde R=R\cdot\text{Exp}(\epsilon),\ \epsilon\in\mathcal N(0,\Sigma)\tag{12}
$$
对高斯分布有积分模型
$$
\int_{\R^3}p(\epsilon){\rm d}\epsilon = \int_{\R^3}\frac1{\sqrt{(2\pi)^3\det(\Sigma)}}\cdot\exp\bigg({-\frac12||\epsilon||^2_\Sigma}\bigg){\rm d}\epsilon=1 \tag{13}
$$
将 $\epsilon$ 用 $\text{Log}(R^{-1}\tilde R)$ 进行替换
$$
\int_{SO(3)}\frac1{\sqrt{(2\pi)^3\det(\Sigma)}}\cdot\exp\bigg({-\frac12\big|\big|\text{Log}(R^{-1}\tilde R)\big|\big|^2_\Sigma}\bigg)\left|\frac{{\rm d}\epsilon}{{\rm d}\tilde R}\right|{\rm d}\tilde R=1
$$
缩放因子由右乘模型给出
$$
\left|\frac{{\rm d}\epsilon}{{\rm d}\tilde R}\right| = \left|\frac1{J_r(\text{Log}(R^{-1}\tilde R))}\right|
$$
重新整理得到
$$
\int_{SO(3)}\frac1{\sqrt{(2\pi)^3\det(\Sigma)}}\cdot\left|\frac1{J_r(\text{Log}(R^{-1}\tilde R))}\right|\cdot\exp\bigg({-\frac12\big|\big|\text{Log}(R^{-1}\tilde R)\big|\big|^2_\Sigma}\bigg){\rm d}\tilde R=1\tag{14}
$$
由多元高斯分布模型可得
$$
p(\tilde R) =\frac1{\sqrt{(2\pi)^3\det(\Sigma)}}\cdot\left|\frac1{J_r(\text{Log}(R^{-1}\tilde R))}\right|\cdot\exp\bigg({-\frac12\big|\big|\text{Log}(R^{-1}\tilde R)\big|\big|^2_\Sigma}\bigg) \tag{15}
$$
对于微小扰动，利用常数项近似 $\frac1{\sqrt{(2\pi)^3\det(\Sigma)}}\cdot\left|\frac1{J_r(\text{Log}(R^{-1}\tilde R))}\right|$, 得到旋转矩阵的负对数似然函数
$$
\begin{aligned}
\mathcal L(R) &= \frac12\big|\big|\text{Log}(R^{-1}\tilde R)\big|\big|^2_\Sigma + c \\
&= \frac12\big|\big|\text{Log}(\tilde R^{-1}R)\big|\big|^2_\Sigma + c
\end{aligned} \tag{16}
$$

##### 流形上的高斯牛顿法

对于一般的高斯牛顿法有
$$
\mathbf x^* = \arg\min_{\mathbf x} f(\mathbf x) \Rightarrow \mathbf x^* = \mathbf x + \arg\min_{\Delta\mathbf x}f(\mathbf x+\Delta\mathbf x)
$$
对于流形则有
$$
x^*=\arg\min_{x\in\mathcal M} f(x) \Rightarrow x^* = \mathcal R_x\cdot\arg\min_{\delta x\in\R^n} f(\mathcal R_x(\delta x))\tag{18}
$$
在 $SO(3)$ 群中映射被定义为
$$
\mathcal R_R(\delta\phi) = R\text{Exp}(\delta\phi),\ \delta\phi\in\R^3\tag{20}
$$
在 $SE(3)$ 群中则为
$$
\mathcal R_T(\delta\phi,\delta\mathbf p)=\begin{bmatrix}R\text{Exp}(\delta\phi) & \mathbf p+R\delta\mathbf p\end{bmatrix},\ \begin{bmatrix}\delta\phi \\ \delta\mathbf p\end{bmatrix}\in\R^6\tag{21}
$$

---

#### VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator

T. Qin, P. Li, and S. Shen, “VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator,” *IEEE Trans. Robot.*, vol. 34, no. 4, pp. 1004–1020, Aug. 2018, doi: [10.1109/TRO.2018.2853729](https://doi.org/10.1109/TRO.2018.2853729). ==TODO==

---

J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers, “A benchmark for the evaluation of RGB-D SLAM systems,” in *2012 IEEE/RSJ International Conference on Intelligent Robots and Systems*, Vilamoura-Algarve, Portugal: IEEE, Oct. 2012, pp. 573–580. doi: [10.1109/IROS.2012.6385773](https://doi.org/10.1109/IROS.2012.6385773).

---

C. Forster, M. Pizzoli, and D. Scaramuzza, “SVO: Fast semi-direct monocular visual odometry,” in *2014 IEEE International Conference on Robotics and Automation (ICRA)*, Hong Kong, China: IEEE, May 2014, pp. 15–22. doi: [10.1109/ICRA.2014.6906584](https://doi.org/10.1109/ICRA.2014.6906584).

---

R. Mur-Artal, J. M. M. Montiel, and J. D. Tardos, “ORB-SLAM: A Versatile and Accurate Monocular SLAM System,” *IEEE Trans. Robot.*, vol. 31, no. 5, pp. 1147–1163, Oct. 2015, doi: [10.1109/TRO.2015.2463671](https://doi.org/10.1109/TRO.2015.2463671).

---

R. Mur-Artal and J. D. Tardos, “ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras,” *IEEE Trans. Robot.*, vol. 33, no. 5, pp. 1255–1262, Oct. 2017, doi: [10.1109/TRO.2017.2705103](https://doi.org/10.1109/TRO.2017.2705103).

---

J. Engel, V. Koltun, and D. Cremers, “Direct Sparse Odometry,” *IEEE Trans. Pattern Anal. Mach. Intell.*, vol. 40, no. 3, pp. 611–625, Mar. 2018, doi: [10.1109/TPAMI.2017.2658577](https://doi.org/10.1109/TPAMI.2017.2658577).

---

C. Campos, R. Elvira, J. J. G. Rodriguez, J. M. M. Montiel, and J. D. Tardos, “ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual–Inertial, and Multimap SLAM,” *IEEE Trans. Robot.*, vol. 37, no. 6, pp. 1874–1890, Dec. 2021, doi: [10.1109/TRO.2021.3075644](https://doi.org/10.1109/TRO.2021.3075644).

---

### Lidar SLAM

Y. Wang, Z. Sun, C.-Z. Xu, S. E. Sarma, J. Yang, and H. Kong, “LiDAR Iris for Loop-Closure Detection,” in *2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Las Vegas, NV, USA: IEEE, Oct. 2020, pp. 5769–5775. doi: [10.1109/IROS45743.2020.9341010](https://doi.org/10.1109/IROS45743.2020.9341010).

Z. Liu *et al.*, “Voxel-SLAM: A Complete, Accurate, and Versatile LiDAR-Inertial SLAM System,” Oct. 11, 2024, *arXiv*: arXiv:2410.08935. doi: [10.48550/arXiv.2410.08935](https://doi.org/10.48550/arXiv.2410.08935).

N. Wang, X. Chen, C. Shi, Z. Zheng, H. Yu, and H. Lu, “SGLC: Semantic Graph-Guided Coarse-Fine-Refine Full Loop Closing for LiDAR SLAM,” Nov. 10, 2024, *arXiv*: arXiv:2407.08106. doi: [10.48550/arXiv.2407.08106](https://doi.org/10.48550/arXiv.2407.08106).

W. Xu and F. Zhang, “FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter,” *IEEE Robot. Autom. Lett.*, vol. 6, no. 2, pp. 3317–3324, Apr. 2021, doi: [10.1109/LRA.2021.3064227](https://doi.org/10.1109/LRA.2021.3064227).

W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, “FAST-LIO2: Fast Direct LiDAR-Inertial Odometry,” *IEEE Trans. Robot.*, vol. 38, no. 4, pp. 2053–2073, Aug. 2022, doi: [10.1109/TRO.2022.3141876](https://doi.org/10.1109/TRO.2022.3141876).

J. Lin and F. Zhang, “Loam livox: A fast, robust, high-precision LiDAR odometry and mapping package for LiDARs of small FoV,” in *2020 IEEE International Conference on Robotics and Automation (ICRA)*, Paris, France: IEEE, May 2020, pp. 3126–3131. doi: [10.1109/ICRA40945.2020.9197440](https://doi.org/10.1109/ICRA40945.2020.9197440).

---

### Radar SLAM

[1]

L. He, X. Wang, and H. Zhang, “M2DP: A novel 3D point cloud descriptor and its application in loop closure detection,” in *2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Daejeon, South Korea: IEEE, Oct. 2016, pp. 231–237. doi: [10.1109/IROS.2016.7759060](https://doi.org/10.1109/IROS.2016.7759060).

---

[2]

Z. Hong, Y. Petillot, and S. Wang, “RadarSLAM: Radar based Large-Scale SLAM in All Weathers,” in *2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Las Vegas, NV, USA: IEEE, Oct. 2020, pp. 5164–5170. doi: [10.1109/IROS45743.2020.9341287](https://doi.org/10.1109/IROS45743.2020.9341287).

---

[3]

A. Kramer, C. Stahoviak, A. Santamaria-Navarro, A. Agha-mohammadi, and C. Heckman, “Radar-Inertial Ego-Velocity Estimation for Visually Degraded Environments,” in *2020 IEEE International Conference on Robotics and Automation (ICRA)*, Paris, France: IEEE, May 2020, pp. 5739–5746. doi: [10.1109/ICRA40945.2020.9196666](https://doi.org/10.1109/ICRA40945.2020.9196666).

---

[4]

C. X. Lu *et al.*, “milliEgo: single-chip mmWave radar aided egomotion estimation via deep sensor fusion,” in *Proceedings of the 18th Conference on Embedded Networked Sensor Systems*, Virtual Event Japan: ACM, Nov. 2020, pp. 109–122. doi: [10.1145/3384419.3430776](https://doi.org/10.1145/3384419.3430776).

---

[5]

X. Xu, H. Yin, Z. Chen, Y. Li, Y. Wang, and R. Xiong, “DiSCO: Differentiable Scan Context With Orientation,” *IEEE Robot. Autom. Lett.*, vol. 6, no. 2, pp. 2791–2798, Apr. 2021, doi: [10.1109/LRA.2021.3060741](https://doi.org/10.1109/LRA.2021.3060741).

---

[6]

N. Merrill *et al.*, “Symmetry and Uncertainty-Aware Object SLAM for 6DoF Object Pose Estimation,” in *2022 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)*, New Orleans, LA, USA: IEEE, Jun. 2022, pp. 14881–14890. doi: [10.1109/CVPR52688.2022.01448](https://doi.org/10.1109/CVPR52688.2022.01448).

---

[7]

K. Retan, F. Loshaj, and M. Heizmann, “Radar Odometry on SE(3) with Constant Acceleration Motion Prior and Polar Measurement Model,” Sep. 11, 2022, *arXiv*: arXiv:2209.05956. doi: [10.48550/arXiv.2209.05956](https://doi.org/10.48550/arXiv.2209.05956).

---

[8]

X. Li, H. Zhang, and W. Chen, “4D Radar-Based Pose Graph SLAM With Ego-Velocity Pre-Integration Factor,” *IEEE Robot. Autom. Lett.*, vol. 8, no. 8, pp. 5124–5131, Aug. 2023, doi: [10.1109/LRA.2023.3292574](https://doi.org/10.1109/LRA.2023.3292574).

---

[9]

H. Yin, S. Li, Y. Tao, J. Guo, and B. Huang, “Dynam-SLAM: An Accurate, Robust Stereo Visual-Inertial SLAM Method in Dynamic Environments,” *IEEE Trans. Robot.*, vol. 39, no. 1, pp. 289–308, Feb. 2023, doi: [10.1109/TRO.2022.3199087](https://doi.org/10.1109/TRO.2022.3199087).

---

[10]

J. Zhang *et al.*, “4DRadarSLAM: A 4D Imaging Radar SLAM System for Large-scale Environments based on Pose Graph Optimization,” in *2023 IEEE International Conference on Robotics and Automation (ICRA)*, London, United Kingdom: IEEE, May 2023, pp. 8333–8340. doi: [10.1109/ICRA48891.2023.10160670](https://doi.org/10.1109/ICRA48891.2023.10160670).

---

[11]

Q. Huang, Y. Liang, Z. Qiao, S. Shen, and H. Yin, “Less is More: Physical-Enhanced Radar-Inertial Odometry,” in *2024 IEEE International Conference on Robotics and Automation (ICRA)*, Yokohama, Japan: IEEE, May 2024, pp. 15966–15972. doi: [10.1109/ICRA57147.2024.10611471](https://doi.org/10.1109/ICRA57147.2024.10611471).

---

[12]

E. Sie, X. Wu, H. Guo, and D. Vasisht, “Radarize: Enhancing Radar SLAM with Generalizable Doppler-Based Odometry,” in *Proceedings of the 22nd Annual International Conference on Mobile Systems, Applications and Services*, Jun. 2024, pp. 331–344. doi: [10.1145/3643832.3661871](https://doi.org/10.1145/3643832.3661871).

---

[13]

D. Wang, S. May, and A. Nuechter, “RIV-SLAM: Radar-Inertial-Velocity optimization based graph SLAM,” in *2024 IEEE 20th International Conference on Automation Science and Engineering (CASE)*, Bari, Italy: IEEE, Aug. 2024, pp. 774–781. doi: [10.1109/CASE59546.2024.10711511](https://doi.org/10.1109/CASE59546.2024.10711511).

---

[14]

Y. Xu, Q. Huang, S. Shen, and H. Yin, “Modeling Point Uncertainty in Radar SLAM,” Feb. 25, 2024, *arXiv*: arXiv:2402.16082. doi: [10.48550/arXiv.2402.16082](https://doi.org/10.48550/arXiv.2402.16082).

---

### Multi-Sensor SLAM

#### R<sup>2</sup> LIVE: A Robust, Real-Time, LiDAR-Inertial-Visual Tightly-Coupled State Estimator and Mapping

J. Lin, C. Zheng, W. Xu, and F. Zhang, “R<sup>2</sup> LIVE: A Robust, Real-Time, LiDAR-Inertial-Visual Tightly-Coupled State Estimator and Mapping,” *IEEE Robot. Autom. Lett.*, vol. 6, no. 4, pp. 7469–7476, Oct. 2021, doi: [10.1109/LRA.2021.3095515](https://doi.org/10.1109/LRA.2021.3095515).

---

#### R<sup>3</sup> LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package

J. Lin and F. Zhang, “R<sup>3</sup> LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package,” in *2022 International Conference on Robotics and Automation (ICRA)*, Philadelphia, PA, USA: IEEE, May 2022, pp. 10672–10678. doi: [10.1109/ICRA46639.2022.9811935](https://doi.org/10.1109/ICRA46639.2022.9811935).

---

#### VINS-Multi: A Robust Asynchronous Multi-camera-IMU State Estimator

L. Wang, Y. Xu, and S. Shen, “VINS-Multi: A Robust Asynchronous Multi-camera-IMU State Estimator,” May 23, 2024, *arXiv*: arXiv:2405.14539. doi: [10.48550/arXiv.2405.14539](https://doi.org/10.48550/arXiv.2405.14539).

针对不同类型相机在异步时间戳上的融合问题，设计了一套平行视觉前端和关键帧选择器的算法。视觉前端平行地为每一个相机提取并追踪特征，加入一个联合筛选器负责挑选放入后端优化的特征和关键帧。特征按照最后一帧追踪到的特征点的数量占比来分配。关键帧筛选则负责将最后帧特征点占比最高的帧以及帧更新间隔最长的帧发送到后端进行优化。

<img src="./assets/image-20241216162903831.png" alt="image-20241216162903831" style="zoom: 33%;" />

---

## Learning-Based SLAM

### Survey

#### Deep Learning Techniques for Visual SLAM: A Survey

S. Mokssit, D. B. Licea, B. Guermah, and M. Ghogho, “Deep Learning Techniques for Visual SLAM: A Survey,” *IEEE Access*, vol. 11, pp. 20026–20050, 2023, doi: [10.1109/ACCESS.2023.3249661](https://doi.org/10.1109/ACCESS.2023.3249661).

作者首先回顾了基于图优化视觉SLAM的基本工作框架，由视觉前端负责提取和追踪特征点，将特征点和关键帧传入后端进行全局位姿图优化，同时回环检测模块进行重定位工作。这一工作框架在图优化SLAM的发展中证实了其可靠有效，但在面对复杂多变的环境和日益灵活的上游任务时则显得窘迫。

<img src="./assets/image-20241216234425636.png" alt="image-20241216234425636" style="zoom:33%;" />

因此基于学习的SLAM应运而生，作者接着针对不同类别的深度学习分别展开论述

<img src="./assets/image-20241217155100282.png" alt="image-20241217155100282" style="zoom:33%;" />

##### 模块化学习 Modular Learning

分别回顾了传统方法在深度估计、光流、视觉里程计、定位、回环检测的方法，并给出了在各个领域上的深度学习方法

###### 深度估计 Depth Estimation

多目系统上的深度估计常表示为
$$
E(D)=\sum_x C(x,d_x) +\sum_x\sum_{y\in N_x} E_s(d_x,d_y)
$$
$C(x,d_x)$ 左图像素 $x=(i,j)$ 和右图像素 $y=(i,j-d_x)$ 的相似度以及一致性或约束正则项 $E_s(·)$ 的和，这种方法在外界条件合适的情况下表现不错，但是缺点是迭代优化 $D$ 需要消耗大量计算资源。

在监督学习中最早利用CNN计算左右图像的相似性，然后将相似性加载到决策网络，在通过上文的公式进行迭代优化。但这种解耦的优化模式使系统对噪声敏感，会收到遮挡和周期性外部条件的影响。另一种方式是直接利用左右图像生成回归视差图，优点是不需要显式配对特征节省了计算成本。在单目相机上的早期尝试是利用CNNs对深度信息回归估计，后面引入了细化网络和不同权重策略。

而在自监督学习上

##### 联合学习 Joint Learning



##### 置信学习 Confidence Learning



##### 主动学习 Active Learning

==TODO==

### Computer Vision

#### Depth map prediction from a single image using a multi-scale deep network

D. Eigen, C. Puhrsch, and R. Fergus, “Depth map prediction from a single image using a multi-scale deep network,” in *Proceedings of the 27th International Conference on Neural Information Processing Systems - Volume 2*, in NIPS’14. Cambridge, MA, USA: MIT Press, 2014, pp. 2366–2374.

提出了一种尺度不变的损失函数
$$
\begin{aligned}
D(y,y^*) &= \frac1n\sum_{i=1}^n(\log y_i-\log y_i^* + \alpha(y,y^*))^2 \\
&= \frac1n\sum_{i=1}^n(\log y_i-\log y_i^* + \frac1n\sum_{j=1}^n(\log y_j^*-\log y_j))^2 \\
&= \frac1{n^2}\sum_{i,j}((\log y_i-\log y_j) - (\log y_i^*-\log y_j^*))^2 \\
&= \frac1n\sum_i (\log y_i-\log y_i^*)^2-\frac1{n^2}\sum_{i,j}(\log y_i-\log y_i^*)(\log y_j-\log y_j^*) \\
&= \frac1n\sum_i d_i^2 - \frac1{n^2}\bigg(\sum_i d_i\bigg)^2
\end{aligned}
$$
同时提出了粗细网络分步学习的网络结构：先利用粗网络生成分辨率较低的图像，再利用细网络对分辨率精细化处理。好处是粗网络输出的每个结果依赖于图像的全局信息，而细网络输出的每个结果则只依赖局部信息。

<img src="./assets/image-20241217162202716.png" alt="image-20241217162202716" style="zoom:44%;" />

---

#### Predicting Depth, Surface Normals and Semantic Labels with a Common Multi-scale Convolutional Architecture

D. Eigen and R. Fergus, “Predicting Depth, Surface Normals and Semantic Labels with a Common Multi-scale Convolutional Architecture,” in *2015 IEEE International Conference on Computer Vision (ICCV)*, Santiago, Chile: IEEE, Dec. 2015, pp. 2650–2658. doi: [10.1109/ICCV.2015.304](https://doi.org/10.1109/ICCV.2015.304).

在作者2014年工作的基础上增加了多种尺度的细化网络，使网络能够适应不同尺度的特征

<img src="./assets/image-20241217164519123.png" alt="image-20241217164519123" style="zoom:50%;" />

---

#### Deep Ordinal Regression Network for Monocular Depth Estimation

H. Fu, M. Gong, C. Wang, K. Batmanghelich, and D. Tao, “Deep Ordinal Regression Network for Monocular Depth Estimation,” in *2018 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)*, Los Alamitos, CA, USA: IEEE Computer Society, Jun. 2018, pp. 2002–2011. doi: [10.1109/CVPR.2018.00214](https://doi.org/10.1109/CVPR.2018.00214).

提出了一种 Spacing-Discretization 策略，让网络更在意近处物体的深度信息的准确性

<img src="./assets/image-20241217165049845.png" alt="image-20241217165049845" style="zoom:50%;" />
$$
\begin{aligned}
\text{UD:}\ & t_i = \alpha + (\beta-\alpha)\frac iK \\
\text{SID:}\ & t_i = e^{\log\alpha + \frac iK\log\frac\beta\alpha}
\end{aligned}
$$
<img src="./assets/image-20241217165527844.png" alt="image-20241217165527844" style="zoom:50%;" />

---

#### CAM-Convs: Camera-Aware Multi-Scale Convolutions for Single-View Depth

J. M. Facil, B. Ummenhofer, H. Zhou, L. Montesano, T. Brox, and J. Civera, “CAM-Convs: Camera-Aware Multi-Scale Convolutions for Single-View Depth,” in *2019 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)*, Long Beach, CA, USA: IEEE, Jun. 2019, pp. 11818–11827. doi: [10.1109/CVPR.2019.01210](https://doi.org/10.1109/CVPR.2019.01210).

针对不同相机模型，设计了不同卷积模型

<img src="./assets/image-20241217170022311.png" alt="image-20241217170022311" style="zoom:33%;" />

---

## SLAM for Swarm



远距离时需要全局一致性

#### Cooperative Transportation Using Small Quadrotors Using Monocular Vision and Inertial Sensing

G. Loianno and V. Kumar, “Cooperative Transportation Using Small Quadrotors Using Monocular Vision and Inertial Sensing,” *IEEE Robot. Autom. Lett.*, vol. 3, no. 2, pp. 680–687, Apr. 2018, doi: [10.1109/LRA.2017.2778018](https://doi.org/10.1109/LRA.2017.2778018).

---

[2]

D. Saldana, B. Gabrich, G. Li, M. Yim, and V. Kumar, “ModQuad: The Flying Modular Structure that Self-Assembles in Midair,” in *2018 IEEE International Conference on Robotics and Automation (ICRA)*, Brisbane, QLD: IEEE, May 2018, pp. 691–698. doi: [10.1109/ICRA.2018.8461014](https://doi.org/10.1109/ICRA.2018.8461014).



近距离时需要高精度相对定位

#### EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments

X. Zhou, J. Zhu, H. Zhou, C. Xu, and F. Gao, “EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments,” in *2021 IEEE International Conference on Robotics and Automation (ICRA)*, Xi’an, China: IEEE, May 2021, pp. 4101–4107. doi: [10.1109/ICRA48506.2021.9561902](https://doi.org/10.1109/ICRA48506.2021.9561902).

https://blog.csdn.net/qq_45858842/article/details/139306068

[2]

P. C. Lusk, X. Cai, S. Wadhwania, A. Paris, K. Fathian, and J. P. How, “A Distributed Pipeline for Scalable, Deconflicted Formation Flying,” *IEEE Robot. Autom. Lett.*, vol. 5, no. 4, pp. 5213–5220, Oct. 2020, doi: [10.1109/LRA.2020.3006823](https://doi.org/10.1109/LRA.2020.3006823).



omni swarm去中心化通讯

