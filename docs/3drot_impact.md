这是一个非常经典且深刻的跨维度 SLAM 问题。你描述的现象，本质上是**将三维非刚体投影（从传感器的视角看）强行输入给基于二维刚体变换假设的算法**所导致的数学冲突。

我们可以通过空间几何、变换矩阵和优化理论来对你提出的三个问题进行严格的数学分析。

---

### 1. 降维打击：从 6-DOF 到 3-DOF 的数学映射（对应问题 2）

在 FAST-LIO 中，机器人的位姿是在三维特殊欧几里得群 $SE(3)$ 中定义的。一个完整的位姿包含了旋转矩阵 $R$ 和平移向量 $\mathbf{t}$：

$$T_{3D} = \begin{bmatrix} R(\phi, \theta, \psi) & \mathbf{t} \\ \mathbf{0}^T & 1 \end{bmatrix} \in SE(3)$$

其中 $\phi$ 为 Roll，$\theta$ 为 Pitch，$\psi$ 为 Yaw，$\mathbf{t} = [x, y, z]^T$。

然而，`slam_toolbox` 的核心算法（基于 Karto 或 Cartographer 的变体）建立在二维流形 $SE(2)$ 上：

$$T_{2D} = \begin{bmatrix} \cos\psi & -\sin\psi & x \\ \sin\psi & \cos\psi & y \\ 0 & 0 & 1 \end{bmatrix} \in SE(2)$$

当你把 FAST-LIO 的 TF 喂给 `slam_toolbox` 时，发生了一个**非单射的投影过程**。数学上，这是一个从 6 维状态空间到 3 维状态空间的截断函数 $f: \mathbb{R}^6 \rightarrow \mathbb{R}^3$：

$$f(x, y, z, \phi, \theta, \psi) = (x, y, \psi)$$

**数学矛盾**：
这种粗暴的截断打破了三维空间中平移和旋转的耦合关系。当雷达在三维空间中发生 Pitch（$\theta$）变化并向前移动时，实际的 $x, y, z$ 位移是受 $\theta$ 调制的。但在 $SE(2)$ 视角下，系统认为 $\theta = 0$，这导致 2D 轮式里程计（或退化后的 TF）的积分路径与真实的 3D 运动轨迹发生不可逆的偏离。

---

### 2. 切片畸变的几何证明（对应问题 1）

我们可以用旋转矩阵来证明你提到的“1/cos(30°) ≈ 1.15m”的现象。
![alt text](image.png)

假设世界坐标系 $W$ 下有一堵垂直平整的墙，距离原点为 $d$。墙面上任意一点的坐标可表示为 $\mathbf{P}_W = [d, y_W, z_W]^T$。
设雷达坐标系为 $B$。当雷达发生俯仰角（Pitch, $\theta$）倾斜，且不发生平移时，世界坐标点到雷达坐标系的转换关系为 $\mathbf{P}_W = R_y(\theta) \mathbf{P}_B$：

$$\begin{bmatrix} d \\ y_W \\ z_W \end{bmatrix} = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix} \begin{bmatrix} x_B \\ y_B \\ z_B \end{bmatrix}$$

`pointcloud_to_laserscan` 的原理是截取雷达坐标系 $B$ 中 $Z$ 轴方向接近于 0 的点（即 $z_B \approx 0$ 的水平切片）。我们将 $z_B = 0$ 代入上面的矩阵乘法中的第一行方程：

$$d = x_B \cos\theta + 0 \cdot \sin\theta$$

$$x_B = \frac{d}{\cos\theta}$$

**数学结论**：
在生成的 2D LaserScan 中，雷达到墙面的测量距离 $x_B$ 变成了 $d / \cos\theta$。由于 $\cos\theta \leq 1$，这意味着**只要雷达发生倾斜，2D 扫描到的距离在数学上必定大于真实距离**。这是一种典型的非线性投影畸变。

---

### 3. 2D Scan Matching (ICP/相关性) 失效的数学本质（对应问题 3）

`slam_toolbox` 依赖于将当前的 2D Scan 与已有的 2D Map 进行匹配（通常基于非线性最小二乘法，类似 ICP 或相关性扫描匹配）。
其目标是寻找最优的 2D 变换 $\Delta T_{2D}$（包含 $\Delta x, \Delta y, \Delta \psi$），使得当前点云集合 $P$ 投影到地图 $M$ 中的误差 $E$ 最小：

$$E = \sum_{i} \left\Vert{} M(\Delta T_{2D} \cdot \mathbf{p}_i) - 1 \right\Vert{}^2$$

**为什么优化会崩溃？**
ICP 算法收敛的一个根本前提是**刚体变换假设**：点云之间的形状不能发生形变，只能发生平移和旋转。

但是，根据我们在问题 1 中的推导，当雷达发生 Pitch 或 Roll 变化时，2D Scan 点云 $P$ 受到的变换并不是 $SE(2)$ 群内的刚体变换，而是一个包含了**尺度拉伸（Scale）和剪切（Shear）的仿射变换（Affine Transformation）**。

例如，对于 Pitch $\theta$：

* $x$ 方向的拉伸因子为 $1/\cos\theta$
* 对于 Roll $\phi$，会导致切片截取到不同高度的物理表面，相当于引入了不可预测的噪声 $v$。

因此，实际输入给优化的点 $\mathbf{p}'_i$ 变成了：

$$\mathbf{p}'_i = \begin{bmatrix} 1/\cos\theta & 0 \\ 0 & 1/\cos\phi \end{bmatrix} \mathbf{p}_i + \text{Noise}(z)$$

由于 $SE(2)$ 优化器（只允许平移和 Yaw 旋转）无法拟合上式中的尺度拉伸矩阵，残差 $E$ 的雅可比矩阵（Jacobian）在迭代时会指向错误的方向，导致算法陷入局部最优，表象就是：**建图出现重影、墙面断裂、回环彻底失效**。

### 总结

数学不会骗人：**你无法用一个只能解构 $SE(2)$ 刚体变换的算法，去消化包含了 $SE(3)$ 姿态变化引发的二维投影仿射畸变的数据**。手持雷达的运动天然是 6-DOF 的，要解决这个问题，必须保留完整的 $z, \phi, \theta$ 信息，使用直接在 $SE(3)$ 空间工作的 3D SLAM 后端。