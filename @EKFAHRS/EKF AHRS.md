# EKF AHRS

##IMU model

The sensor's specific force $\mathbf f^b$ is a combination of earth gravity field vector $\mathbf g_1^t$ and the sensor's acceleration $\boldsymbol{a}_{tb}^t$:
$$
\mathbf f^b = \mathbf R_t^b(\boldsymbol{a}_{tb}^t - \mathbf g^t)
$$
The raw measurements from accelerometer and gyroscope, namely $\boldsymbol{\bar a}$ and $\boldsymbol{\bar{\omega}}$:
$$
\boldsymbol{\bar a} = \mathbf f^b + \mathbf x_a + \mathbf v_a \\
\boldsymbol{\bar \omega} = \boldsymbol{\omega}_{ib}^b + \mathbf x_g + \mathbf v_g
$$
are affected by acceleration bias $\mathbf x_a$, gyroscope bias $\mathbf x_g$ and addictive noise $\mathbf v$. Addictive noise are assumed to be Gaussion white noise, $\mathbf v_a \sim \mathcal{N}(\mathbf 0,\sigma_{\mathbf v_a}^2)$, $\mathbf v_g \sim \mathcal{N}(\mathbf 0,\sigma_{\mathbf v_g}^2)$, $\mathbf x_a, \mathbf x_g$ are random walk, $ \dot {\mathbf x_a}=\boldsymbol \omega_a$, $ \dot {\mathbf x_g}=\boldsymbol \omega_g$, $\mathbf \omega_a \sim \mathcal{N}(\mathbf 0,\sigma_{\mathbf \omega_a}^2)$, $\mathbf \omega_g \sim \mathcal{N}(\mathbf 0,\sigma_{\mathbf \omega_g}^2)$.

## State

State:  $\mathbf x =[\mathbf q^T, \mathbf x_g^T, \mathbf x_a^T]^T$

where $\mathbf q$ is the unit quaternion representing the rotation (the attitude) from body frame to navigation frame, $\mathbf x_g$ the gyroscope bias, $\mathbf x_a$ the accelemeter bias.

Error State: $\delta \mathbf x=[\boldsymbol \rho^T, \delta \mathbf x_g^T, \delta \mathbf x_a^T]^T$

For error analysis, the transformation from the actual to the estimated (denoted by a hat, $\hat x$ means the estimate of $x$) navigation frame can be represented via a multiplicative small angle transformation $(\mathbf I−\mathbf P)$ where the skew-symmetric matrix $\mathbf P$ has the representation:
$$
\mathbf P=[\boldsymbol \rho ]^\times=
\begin{bmatrix}
0 & -\epsilon _z & \epsilon _y \\
\epsilon _z & 0 & -\epsilon _x \\
-\epsilon _y & \epsilon _x & 0
\end{bmatrix}
$$
With the above definitions, we have that:
$$
\hat {\mathbf R}_b^n = (\mathbf I - \mathbf P)\mathbf R_b^n
$$
The following relations are derived:
$$
\mathbf R_b^n=(\mathbf I + \mathbf P)\hat {\mathbf R}_b^n \label 1
$$

## Error dynamics

$$
\dot {{\mathbf R}_b^n}={\mathbf R}_b^n \boldsymbol \Omega_{nb}^b \label 2
$$

where $\boldsymbol \Omega_{nb}^b=[\boldsymbol \omega_{nb}^b ]^\times$.

The estimate  $\hat {\mathbf R}_b^n$ can be computed as:
$$
\dot {\hat {\mathbf R_b^n}}=\hat {\mathbf R_b^n}(\hat {\boldsymbol \Omega_{ib}^b}-\hat {\boldsymbol \Omega_{in}^b})
$$
where $\hat {\boldsymbol \Omega}_{ib}^b=[\hat {\boldsymbol \omega}_{ib}^b ]^\times$ with $\hat {\boldsymbol \omega}_{ib}^b=\bar {\boldsymbol \omega}-\hat {\mathbf x_g}$ calculated based on the gyro measurements.

The derivative of equation $(\ref 1)$:
$$
\dot {{\mathbf R}_b^n}=\dot {\mathbf P} \hat {\mathbf R_b^n}+(\mathbf I + \mathbf P)\dot {\hat {\mathbf R_b^n}}
$$
From equation $(\ref 1)$, $(\ref 2)$ we have:
$$
\dot {{\mathbf R}_b^n}=(\mathbf I + \mathbf P)\hat {\mathbf R}_b^n(\boldsymbol \Omega_{ib}^b-\boldsymbol \Omega_{in}^b)
$$
Using the definitions:
$$
\delta \boldsymbol \Omega=\boldsymbol \Omega - \hat {\boldsymbol \Omega }
$$


Then we have:
$$
\begin{align}
\dot {\mathbf P} \hat {\mathbf R_b^n}+(\mathbf I + \mathbf P)\dot {\hat {\mathbf R_b^n}}&=
(\mathbf I + \mathbf P)\hat {\mathbf R}_b^n(\hat {\boldsymbol \Omega_{ib}^b}-\hat {\boldsymbol \Omega_{in}^b}+\delta \boldsymbol \Omega_{ib}^b-\delta\boldsymbol \Omega_{in}^b) \\
&=(\mathbf I + \mathbf P)\dot {\hat {\mathbf R_b^n}}+(\mathbf I + \mathbf P)\hat {\mathbf R}_b^n(\delta \boldsymbol \Omega_{ib}^b-\delta\boldsymbol \Omega_{in}^b) \\
\dot {\mathbf P} \hat {\mathbf R_b^n} &= \hat {\mathbf R}_b^n(\delta \boldsymbol \Omega_{ib}^b-\delta\boldsymbol \Omega_{in}^b) \\
\dot {\mathbf P} &= \hat {\mathbf R}_b^n(\delta \boldsymbol \Omega_{ib}^b-\delta\boldsymbol \Omega_{in}^b) \hat {\mathbf R_n^b} 
\end {align}
$$
The term $ \mathbf P\hat {\mathbf R}_b^n(\delta \boldsymbol \Omega_{ib}^b-\delta\boldsymbol \Omega_{in}^b)$ is dropped as it is second order in the error quantities. 

And $[\mathbf R \boldsymbol \omega]^\times=\mathbf R[\boldsymbol \omega]^\times \mathbf R^T$ [1], we have:
$$
\dot {\boldsymbol \rho}=\hat {\mathbf R}_b^n(\delta \boldsymbol \omega_{ib}^b-\delta \boldsymbol \omega_{in}^b)
$$
We have:
$$
\begin{align}
\delta \boldsymbol \omega_{ib}^b&=\boldsymbol \omega_{ib}^b-\hat {\boldsymbol \omega_{ib}^b}\\
&=(\boldsymbol{\bar \omega}- \mathbf x_g - \mathbf v_g)-(\boldsymbol{\bar \omega}-\hat{\mathbf x_g}) \\
&=-\delta \mathbf x_g-\mathbf v_g
\end{align}
$$
For simplification, we ignore the $ \boldsymbol \omega_{in}^b$ whicih is the rotation rate of the navigation frame relative to the inertial frame represented in navigation frame. We can derive:
$$
\dot {\boldsymbol \rho}=-\hat {\mathbf R}_b^n \delta \mathbf x_g-\hat {\mathbf R}_b^n \mathbf v_g
$$
The dynamic model for the state vector is:
$$
\begin{bmatrix}
\dot {\boldsymbol \rho} \\
\delta \dot {\mathbf x_g} \\
\delta \dot {\mathbf x_a} 
\end{bmatrix}
=
\begin{bmatrix}
\mathbf 0_3 & -\hat {\mathbf R}_b^n & \mathbf 0_3 \\
\mathbf 0_3 &\mathbf 0_3 &\mathbf 0_3 \\
\mathbf 0_3 &\mathbf 0_3 &\mathbf 0_3
\end{bmatrix}
\begin{bmatrix}
 {\boldsymbol \rho} \\
\delta {\mathbf x_g} \\
\delta {\mathbf x_a} 
\end{bmatrix} + 
\begin{bmatrix}
 -\hat {\mathbf R}_b^n & \mathbf 0_3 & \mathbf 0_3 \\
\mathbf 0_3 &\mathbf I_3 &\mathbf 0_3 \\
\mathbf 0_3 &\mathbf 0_3 &\mathbf I_3
\end{bmatrix}
\begin{bmatrix}
 {\boldsymbol v_g} \\
\boldsymbol {\omega_g} \\
\boldsymbol {\omega_a}
\end{bmatrix}
$$






































[1] Zhao S. Time Derivative of Rotation Matrices: A Tutorial[J]. arXiv preprint arXiv:1609.06088, 2016.





















 