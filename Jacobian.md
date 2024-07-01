<!--
 * @Author: CTC 2801320287@qq.com
 * @Date: 2024-06-23 17:53:07
 * @LastEditors: CTC 2801320287@qq.com
 * @LastEditTime: 2024-06-30 21:20:16
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
-->
# Joint Jacobian of Serial Manipulators

Many books mentioned that, the columns of Jacobian matrixes at serial manipulators' joint, are associated with twists.

## 1. Spatial Velocity

The spatial velocity follows

$$
\begin{aligned}
    [V_{i}^{s}] = \dot{T}_{i}^{0} (\vec{\theta}) \left( T_{i}^{0} (\vec{\theta}) \right)^{-1},
\end{aligned}
$$

where

$$
\begin{aligned}
    \dot{T}_{i}^{0} (\vec{\theta}) &=&& \frac{d}{d t} \left( \prod_{k = 1}^{i} e^{[\hat{\xi}_{k}] \theta_{k}} T_{i}^{0} (\vec{0}) \right) \\
    &=&& \left[ [\hat{\xi}_{1}] \dot{\theta}_{1} e^{[\hat{\xi}_{1}] \theta_{1}} \cdots e^{[\hat{\xi}_{i}] \theta_{i}} \right] T_{i}^{0} (\vec{0}) + \left[ e^{[\hat{\xi}_{1}] \theta_{1}} [\hat{\xi}_{2}] \dot{\theta}_{2} e^{[\hat{\xi}_{2}] \theta_{2}} \cdots e^{[\hat{\xi}_{i}] \theta_{i}} \right] T_{i}^{0} (\vec{0}) \\
    &&& + \cdots + \left[ e^{[\hat{\xi}_{1}] \theta_{1}} \cdots e^{[\hat{\xi}_{i - 1}] \theta_{i - 1}} [\hat{\xi}_{i}] \dot{\theta}_{i} e^{[\hat{\xi}_{i}] \theta_{i}} \right] T_{i}^{0} (\vec{0}), \\
    \left( T_{i}^{0} (\vec{\theta}) \right)^{-1} &=&& \left( T_{i}^{0} (\vec{0}) \right)^{-1} e^{- [\hat{\xi}_{i}] \theta_{i}} \cdots e^{- [\hat{\xi}_{1}] \theta_{1}}.
\end{aligned}
$$

Therefore, $[V_{i}^{s}]$ follows

$$
\begin{aligned}
    [V_{i}^{s}] &=&& \dot{T}_{i}^{0} (\vec{\theta}) \left( T_{i}^{0} (\vec{\theta}) \right)^{-1} \\
    &=&& [\hat{\xi}_{1}] \dot{\theta}_{1} + e^{[\hat{\xi}_{1}] \theta_{1}} [\hat{\xi}_{2}] e^{[\hat{\xi}_{1}] \theta_{1}} \dot{\theta}_{2} + \cdots + \prod_{j = 1}^{k - 1} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) [\hat{\xi}_{k}] \prod_{j = 1}^{k - 1} \left( e^{-[\hat{\xi}_{j}] \theta_{j}} \right) \dot{\theta}_{k} \\
    &&& + \cdots + \prod_{j = 1}^{i - 1} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) [\hat{\xi}_{k}] \prod_{j = 1}^{i - 1} \left( e^{-[\hat{\xi}_{j}] \theta_{j}} \right) \dot{\theta}_{i},
\end{aligned}
$$

which yields

$$
\begin{aligned}
    V_{i}^{s} &=&& \hat{\xi}_{1} \dot{\theta}_{1} + \mathrm{Ad}_{V} \left( e^{[\hat{\xi}_{1}] \theta_{1}} \right) \hat{\xi}_{2} \dot{\theta}_{2} + \cdots + \mathrm{Ad}_{V} \left( \prod_{j = 1}^{k - 1} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) \right) \hat{\xi}_{k} \dot{\theta}_{k} \\
    &&& + \cdots + \mathrm{Ad}_{V} \left( \prod_{j = 1}^{i - 1} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) \right) \hat{\xi}_{i} \dot{\theta}_{i} \\
    &=&& \left[ \hat{\xi}_{1}, \cdots, \mathrm{Ad}_{V} \left( \prod_{j = 1}^{k - 1} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) \right) \hat{\xi}_{k}, \cdots,  \mathrm{Ad}_{V} \left( \prod_{j = 1}^{i - 1} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) \right) \hat{\xi}_{i} \right] \begin{pmatrix}
        \dot{\theta}_{1} \\ \vdots \\ \dot{\theta}_{k} \\ \vdots \\ \dot{\theta}_{i}
    \end{pmatrix}.
\end{aligned}
$$

The spatial Jacobian matrix of joint $i$ is

$$
J_{i}^{s} = \left[ \hat{\xi}_{1}, \cdots, \mathrm{Ad}_{V} \left( \prod_{j = 1}^{i - 1} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) \right) \hat{\xi}_{i}, \mathbf{0}_{n \times (n - i)} \right].
$$

For $k \in [1,i]$, the $k$th column of $J_{i}^{s}$ follows

$$
J_{i,k}^{s} = \mathrm{Ad}_{V} \left( \prod_{j = 1}^{k - 1} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) \right) \hat{\xi}_{k}.
$$

## 2. Body Velocity

The body velocity follows

$$
\begin{aligned}
    [V_{i}^{b}] = \left( T_{i}^{0} (\vec{\theta}) \right)^{-1} \dot{T}_{i}^{0} (\vec{\theta}),
\end{aligned}
$$

where

$$
\begin{aligned}
    \dot{T}_{i}^{0} (\vec{\theta}) &=&& \frac{d}{d t} \left( \prod_{k = 1}^{i} e^{[\hat{\xi}_{k}] \theta_{k}} T_{i}^{0} (\vec{0}) \right) \\
    &=&& \left[ [\hat{\xi}_{1}] \dot{\theta}_{1} e^{[\hat{\xi}_{1}] \theta_{1}} \cdots e^{[\hat{\xi}_{i}] \theta_{i}} \right] T_{i}^{0} (\vec{0}) + \left[ e^{[\hat{\xi}_{1}] \theta_{1}} [\hat{\xi}_{2}] \dot{\theta}_{2} e^{[\hat{\xi}_{2}] \theta_{2}} \cdots e^{[\hat{\xi}_{i}] \theta_{i}} \right] T_{i}^{0} (\vec{0}) \\
    &&& + \cdots + \left[ e^{[\hat{\xi}_{1}] \theta_{1}} \cdots e^{[\hat{\xi}_{i - 1}] \theta_{i - 1}} [\hat{\xi}_{i}] \dot{\theta}_{i} e^{[\hat{\xi}_{i}] \theta_{i}} \right] T_{i}^{0} (\vec{0}), \\
    \left( T_{i}^{0} (\vec{\theta}) \right)^{-1} &=&& \left( T_{i}^{0} (\vec{0}) \right)^{-1} e^{- [\hat{\xi}_{i}] \theta_{i}} \cdots e^{- [\hat{\xi}_{1}] \theta_{1}}.
\end{aligned}
$$

Therefore, $[V_{i}^{b}]$ follows

$$
\begin{aligned}
    [V_{i}^{b}] &=&& \left( T_{i}^{0} (\vec{\theta}) \right)^{-1} \dot{T}_{i}^{0} (\vec{\theta}) \\
    &=&& \left( T_{i}^{0} (\vec{0}) \right)^{-1} \left( e^{[\hat{\xi}_{1}] \theta_{1}} \cdots e^{[\hat{\xi}_{i}] \theta_{i}} \right)^{-1} [\hat{\xi}_{1}] \dot{\theta}_{1} \left( e^{[\hat{\xi}_{1}] \theta_{1}} \cdots e^{[\hat{\xi}_{i}] \theta_{i}} \right) T_{i}^{0} (\vec{0}) \\
    &&& + \left( T_{i}^{0} (\vec{0}) \right)^{-1} \left( e^{[\hat{\xi}_{2}] \theta_{2}} \cdots e^{[\hat{\xi}_{i}] \theta_{i}} \right)^{-1} [\hat{\xi}_{2}] \dot{\theta}_{2} \left( e^{[\hat{\xi}_{2}] \theta_{2}} \cdots e^{[\hat{\xi}_{i}] \theta_{i}} \right) T_{i}^{0} (\vec{0}) \\
    &&& + \cdots + \left( T_{i}^{0} (\vec{0}) \right)^{-1} \left( e^{[\hat{\xi}_{i}] \theta_{i}} \right)^{-1} [\hat{\xi}_{1}] \dot{\theta}_{1} \left( e^{[\hat{\xi}_{i}] \theta_{i}} \right) T_{i}^{0} (\vec{0}) \\
    &=&& \left( \prod_{j = 1}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right)^{-1} [\hat{\xi}_{1}] \left( \prod_{j = 1}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right) \dot{\theta}_{1} \\
    &&& + \left( \prod_{j = 2}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right) [\hat{\xi}_{2}] \left( \prod_{j = 2}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right) \dot{\theta}_{2} \\
    &&& + \cdots + \left( \left( e^{[\hat{\xi}_{i}] \theta_{i}} \right) T_{i}^{0} (\vec{0}) \right)^{-1} [\hat{\xi}_{i}] \left( \left( e^{[\hat{\xi}_{i}] \theta_{i}} \right) T_{i}^{0} (\vec{0}) \right) \dot{\theta}_{i},
\end{aligned}
$$

which yields

$$
\begin{aligned}
    V_{i}^{b} &=&& \mathrm{Ad}_{V} \left( \prod_{j = 1}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right) \hat{\xi}_{1} \dot{\theta}_{1} + \mathrm{Ad}_{V} \left( \prod_{j = 2}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right) \hat{\xi}_{2} \dot{\theta}_{2} \\
    &&& + \cdots + \mathrm{Ad}_{V} \left( \prod_{j = k}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right) \hat{\xi}_{k} \dot{\theta}_{k} + \cdots \\
    &&& + \mathrm{Ad}_{V} \left( e^{[\hat{\xi}_{i}] \theta_{i}} T_{i}^{0} (\vec{0}) \right) \hat{\xi}_{i} \dot{\theta}_{i} \\
    &=&& \left[ \mathrm{Ad}_{V} \left( \prod_{j = 1}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right) \hat{\xi}_{1}, \cdots, \mathrm{Ad}_{V} \left( e^{[\hat{\xi}_{i}] \theta_{i}} T_{i}^{0} (\vec{0}) \right) \hat{\xi}_{i} \right] \begin{pmatrix}
        \dot{\theta}_{1} \\ \vdots \\ \dot{\theta}_{i}
    \end{pmatrix}.
\end{aligned}
$$

The body Jacobian matrix of joint $i$ is

$$
J_{i}^{b} = \left[ \mathrm{Ad}_{V} \left( \prod_{j = 1}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right) \hat{\xi}_{1}, \cdots, \mathrm{Ad}_{V} \left( e^{[\hat{\xi}_{i}] \theta_{i}} T_{i}^{0} (\vec{0}) \right) \hat{\xi}_{i}, \mathbf{0}_{n \times (n - i)} \right].
$$

For $k \in [1,i]$, the $k$th column of $J_{i}^{b}$ follows

$$
J_{i,k}^{b} = \mathrm{Ad}_{V} \left( \prod_{j = k}^{i} \left( e^{[\hat{\xi}_{j}] \theta_{j}} \right) T_{i}^{0} (\vec{0}) \right) \hat{\xi}_{k}.
$$
