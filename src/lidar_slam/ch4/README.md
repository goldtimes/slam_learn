# IMU预积分
    传统的滤波算法中，处理imu数据是一帧一帧的处理，imu数据来了，递推，递推，递推。然后在雷达或者其他观测到来的时候，通过紧耦合/松耦合的方式来更新状态量
    预积分的方式，虽然也是一帧一帧的处理，但是它是将关键帧之间的所有imu数据一起处理，然后最后通过优化的方式来更新状态量。
## 预积分量
    我们在代码公式推导中定义的预积分量就是t_i时刻->t_j时刻的相对运动量(P,R,V);它是无法直接测量出来，于是我们需要通过公式的转换来推导.同样，因为测量的数据都是带噪声
    我们要推导出预积分过程中噪声是如何变换(协方差是如何更新的)
### 步骤
    1. 预积分的测量模型
    2. 预积分的噪声模型
    3. 预积分关于零偏的更新
    4. 预积分放到图优化中的Jacobian推导
    5. 优化
预积分公式
旋转的测量模型
$$\Delta \widetilde R_{ij} =\prod_{k=i}^{j-1}Exp((\widetilde \omega_k - b_{g,i})\Delta t)$$   ( 1.1)R预积分量可以通过imu的值计算出来
$$\Delta \widetilde R_{ij} = R_i^T * R_j * Exp(\delta \theta_{ij})$$   ( 1.2)
$$\Delta R_{ij} = \Delta \widetilde R_{ij} \prod_{k=i}^{j-1}Exp(-\Delta \widetilde R_{k+1,j}^T J_{r,k} \eta_{gd,k}\Delta t)$$    ( 1.3)
$$\Delta R_{ij} = \Delta \widetilde R_{ij} * Exp(\delta \theta _{ij}) $$     ( 1.4) 真实值 = 预积分量 * 噪声

速度的测量模型
$$\Delta \widetilde v_{ij} = R_i^T(v_j - v_i - g\Delta t_{ij}) - \delta v_{ij} $$ (2.1)
$$\Delta \widetilde v_{ij} = \sum_{k=i}^{j-1} \Delta \widetilde R_{ik} * (\widetilde a_k - b_{a,i}) * \Delta t$$  (2.2) V预积分量可以通过imu的测量值计算出来
$$\Delta v_{ij} = \Delta \widetilde v_{ij} - \delta v_{ij} $$ (1.12)   (2.3) 真实值 = 预积分量 - 噪声
位移的测量模型
$$\Delta \widetilde p_{ij} = R_i^T(p_j - p_i - v_i\Delta t_{ij} - 0.5*g\Delta t_{ij}^2) - \delta p_{ij} $$ (3.1)
$$\Delta \widetilde p_{ij} = \sum_{k=i}^{j-1} (\Delta \widetilde v_{ik} * \Delta t)  + 1/2 * \Delta \widetilde R_{ik} * (\widetilde a_k - b_{a,i}) * \Delta t^2$$ (3.2) p预积分量可以通过imu的测量值计算出来
$$\Delta p_{ij} = \Delta \widetilde p_{ij} - \delta p_{ij} $$ (3.3) 真实值 = 预积分量 - 噪声

根据(1.1),(2,1),(3.1)的方程
左侧为预积分量，可以通过当前状态和imu的测量值计算出来
右侧为i,j时刻的状态变量和噪声。于是可以写出残差公式，利用最小二乘法。
噪声模型
推导j-1时刻的噪声递推到j时刻的噪声
零偏更新
注意公式里面都是把零偏固定在i时刻，认为i~j时刻的零偏是不变的。
预积分的雅可比矩阵
预积分的总结:
从一个时刻的关键帧开始预积分，可以在任意时刻停止预积分。把预积分的观测量，噪声以及雅可比矩阵取出来用于约束两帧之间状态。
当imu数据到达后
1. 利用公式计算预积分的三个观测量 $$\Delta \widetilde R_{ij}, \Delta \widetilde v_{ij},\Delta \widetilde p_{ij}$$
2. 计算三个噪声量的协方差矩阵，作为优化的信息矩阵
3. 预积分相对于零偏的雅可比矩阵
将这些结果放到优化过程中用起来