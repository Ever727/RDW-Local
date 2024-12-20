# 重定向行走大作业报告

> 周子恒 计 24, 张皓晨 计 24, 黄宸宇 计 23

我们实现了控制器 S2C、S2O、APF、SRL、ARC，并且编写了随机生成虚拟、物理行走环境的脚本，支持标准矩形和多边形图形边界与障碍物的生成。此外我们编写了随机生成行走路径的脚本，并且编写了多种路径的模拟脚本，并绘制了行走图像，进行了数据统计。

## 使用方法

- 后端启动

```cmd
python client_base.py
```

- 前端启动

```cmd
uvicorn main:app --host 0.0.0.0 --port 8000 # remember to open port8000 to public IP
```

- 环境随机生成

```cmd
python enviroment_generator.py
```

- 路径随机生成

```cmd
bash path_generator.sh
```

- 路径模拟与图像绘制

```cmd
bash test.sh # Linux/MacOS
.\test.bat # Windows
```

## 实现的功能

### S2C

S2C 的策略是让用户朝着物理环境中心走。用户会基于当前视角与当前位置到中心的角度，选择更小的旋转方向进行转向，因此只要用户转到了朝着物理环境中心的方向，就能保证用户朝着物理环境中心走。更进一步地，为了让转向角更小，我们设置了临时转向点。如果当前方向与目标方向大于 160 度，则一直选择临时转向点；否则，选择中心点进行转向。

![S2C 示意图](src/S2C.png)

图示为用户在虚拟环境中走直线，物理环境下用户的行走轨迹。我们取曲率增益为 7.5m，用户在经过中心前一直朝着物理环境中心走，经过之后用户会转向到临时转向点，最终会画圆回到中心。

### S2O

S2O 的核心思想是通过确定物理环境中的中心点，给定一个以中心点为圆心的轨道圆，半径为 5m。对于物理环境中的用户，如果用户处于圆轨道之外，则过当前位置对圆轨道做两条切线，切点为用户选择转向的点。然后按用户当前物理视角 `user.angle` 与转向点的角度差，选取较小者进行转向；对于轨道内的用户，则做一条经过圆心与用户当前位置的直线，然后以圆心为轴，顺、逆时针旋转 60 度，选择转向方向，同样选择转向角度较小者。用户每走一步，都需要更新转向点。

![S2O 示意图](src/S2O.png)

图示为用户在虚拟环境中走直线，物理环境下用户的行走轨迹。我们取曲率增益为 7.5m，因此轨迹半径为 7.5m 的包络圆。

### APF

### SRL

### ARC

ARC 的策略是考虑现实环境与虚拟环境的对齐程度来调整增益策略。具体而言，定义 $d(p,\theta)$ 表示位置 $p$ 在当前环境下 $\theta$ 方向距离最近障碍物、边界的距离，我们定义对齐程度为：

$$
\begin{align*}
\text{dist}(q_t^{phys}, q_t^{virt}) = & \; |d(p_{phys}, \theta_{phys}) - d(p_{virt}, \theta_{virt})| \\
& + |d(p_{phys}, \theta_{phys} + 90^\degree) - d(p_{virt}, \theta_{virt} + 90^\degree)| \\
& + |d(p_{phys}, \theta_{phys} - 90^\degree) - d(p_{virt}, \theta_{virt} - 90^\degree)|
\end{align*}
$$

当 $\text{dist}(q_t^{phys}, q_t^{virt})>0$ 时，应用重定向策略，否则认为虚拟与现实环境对齐。具体增益策略如下：

$$g_t = \text{clamp}(\frac{d(p_{phys}, \theta_{phys})}{d(p_{virt}, \theta_{virt})}, \text{minTransGain}, \text{maxTransGain})$$

平移增益如上，其中 $\text{minTransGain}$ 和 $\text{maxTransGain}$ 分别为 0.86 和 1.26。曲率增益如下：

$$
\begin{align*}
\text{misalignLeft} = d(p_{phys}, \theta_{phys} + 90^\degree) - d(p_{virt}, \theta_{virt} + 90^\degree), \\
\text{misalignRight} = d(p_{phys}, \theta_{phys} - 90^\degree) - d(p_{virt}, \theta_{virt} - 90^\degree). \\
\end{align*}
$$

如果 $\text{misalignLeft} > \text{misalignRight}$，则应用左曲率增益；否则应用右曲率增益。我们有：

$$
\begin{align*}
\text{scalingFactor} = \text{min}(1, \text{misalignLeft}), \\
g_c = \text{max}(\text{minCurvGain}, \frac{\text{minCurvGain}}{\text{scalingFactor}})
\end{align*}
$$

其中 $minCurvGain$ 为 7.5m。旋转增益如下：

$$
\begin{align*}
\text{curRotaAlignment} = \text{dist}(q_t^{phys}, q_t^{virt}), \\
\text{prevRotaAlignment} = \text{dist}(q_{t-1}^{phys}, q_{t-1}^{virt}). \\
\end{align*}
$$

$$
g_r =
\begin{cases}
\text{minRotaGain} & \text{if } \text{curRotaAlignment} < \text{prevRotaAlignment}, \\
\text{maxRotaGain} & \text{if } \text{curRotaAlignment} > \text{prevRotaAlignment}, \\
1 & \text{otherwise.}
\end{cases}
$$

其中 $\text{minRotaGain}$ 和 $\text{maxRotaGain}$ 分别为 0.67 和 1.24。

最后是 reset 策略，当距离障碍物只有 0.7m 时，选择 20 个固定方向，分别计算 $d(p_{phys}, \theta_{i})$，选择障碍物的反方向并且距离最远的方向进行重置。

![ARC 示意图1](src/ARC1.png)

图为 ARC 策略在虚拟环境中从下方中间位置走直线向着左上角前进，物理环境下用户从下方中间开始的行走轨迹。由于一直不能保持对齐，因此 ARC 策略会导致用户一直转向，重定向时也是朝着与墙垂直的方向。

![ARC 示意图2](src/ARC2.png)

图为虚拟环境与现实环境完全一致的情况，可见前段时间物理环境下用户走的也是直线。但由于细微角度偏差带来的问题，ARC 策略仍然会导致用户转向。

## 方法效果基准测试

我们实现了环境与路径生成和测试脚本，在多个环境下测试了不同策略的效果。通过对比不同策略的效果，我们可以发现：

- 在中间较为空旷时，S2O 与 S2C 策略效果很好，但只要物理环境中间有障碍物时，策略就会失效，因为策略没有考虑障碍物的影响，一旦中间有障碍物就会导致大量的 reset。

- S2O 一定程度上可以规避中心、边缘位置有障碍物的情况，但如果其旋转轨道上有障碍物，则会导致大量的 reset。

- S2C 策略在某一侧障碍物不集中，并且中心无障碍时效果较好，这时 S2C 会朝着固定轨道运动。但如果障碍物集中，则会导致大量的 reset。

- 大部分情况下 APF 策略效果不错，能够适应局部最优策略来计算斥力，但如果用户处于障碍物密集区，则会导致用户落入了局部最优区，无法探测到整体物理环境和整体最优区域。这时如果中间区域空旷， S2O 策略效果会更好。

- ARC 策略在环境复杂时效果会好一些，因为它也考虑到了局部最优策略，并且环境越复杂，其对齐程度越可能高，因此 ARC 策略的效果会更好。但如果环境比较简单，则 ARC 策略就会出问题。考虑虚拟与现实环境都十分空旷，此时其他策略都不会失效，但如果 ARC 的虚拟和现实位置不同步，则 ARC 会不断应用增益进行转向，并且也很难调整到对齐位置，于是会造成很多 reset。