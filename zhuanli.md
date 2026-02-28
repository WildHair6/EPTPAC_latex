# 发明名称
一种双重饱和受限下的航天器精确预设时间预设精度控制方法

# 摘要
本发明公开了一种双重饱和受限下的航天器精确预设时间预设精度控制方法，属于航天器姿态控制技术领域。该方法针对受反作用飞轮力矩饱和及角动量饱和双重约束的刚体航天器，提出了一种能够保证在用户指定的终端时刻精确到达目标状态且满足预设精度要求的控制框架。本发明采用双环控制架构：外环规划器通过求解带约束的最优控制问题，生成在终端时刻精确到达目标且满足双重饱和约束的可行参考轨迹；内环控制器采用非奇异预设时间预设精度控制律，驱动跟踪误差在终端时刻前进入预设精度范围。本发明还提供了系统的参数综合方法，确保在存在有界扰动和惯量不确定性的情况下闭环系统的可行性与稳定性。本发明解决了现有预设时间控制存在的保守性问题（过早收敛）及忽视角动量饱和导致的物理不可实现问题。

# 权利要求书
1. 一种双重饱和受限下的航天器精确预设时间预设精度控制方法，其特征在于，包括以下步骤：
步骤1：建立受反作用飞轮驱动的刚体航天器姿态运动学与动力学模型，并定义力矩饱和与角动量饱和的双重约束条件；
步骤2：根据任务给定的初始状态、目标状态、终端时刻 $T_f$ 及预留的安全裕度，构建并求解带双重饱和约束的最优控制问题（OCP），生成满足动力学约束及执行机构约束的参考轨迹（$\boldsymbol{\sigma}_d, \boldsymbol{\omega}_d$）；
步骤3：基于参考轨迹，定义姿态跟踪误差 $\boldsymbol{\sigma}_e$ 和角速度跟踪误差 $\boldsymbol{\omega}_e$，设计非奇异预设时间滑动模态面 $\boldsymbol{s}$，并构建包含前馈补偿与预设时间镇定项的内环控制力矩律；
步骤4：根据系统扰动界值、惯量不确定性及预设精度要求，进行参数综合，确定满足双重饱和约束的控制器参数与安全裕度，将计算得到的控制力矩施加于航天器反作用飞轮。

2. 如权利要求1所述的方法，其特征在于，步骤1中定义的双重饱和列约束为：
$|\tau_i(t)| \le \tau_{i,\max}$
$|h_{w,i}(t)| \le h_{w,i,\max}$
其中，$i=1,2,3$ 表示由于体坐标系三轴，$\tau_i$ 为控制力矩，$h_{w,i}$ 为飞轮角动量，$\tau_{i,\max}$ 和 $h_{w,i,\max}$ 分别为最大力矩幅值和最大角动量幅值。

3. 如权利要求1所述的方法，其特征在于，步骤2中的最优控制问题构建如下：
以控制力矩范数平方积分作为性能指标，以航天器运动学方程、动力学方程、飞轮角动量累积方程为等式约束，以收紧后的力矩幅值 $(1-\gamma_u)\tau_{i,\max}$ 和角动量幅值 $(1-\gamma_h)h_{w,i,\max}$ 为不等式约束，求解得到节点力矩并插值得到参考力矩 $\boldsymbol{\tau}_{\mathrm{ref}}(t)$ 及参考轨迹；其中 $\gamma_u, \gamma_h$ 为约束收紧裕度。

4. 如权利要求1所述的方法，其特征在于，步骤3中设计的滑动模态面 $\boldsymbol{s}$ 为：
$\boldsymbol{s} = \boldsymbol{\omega}_e + \boldsymbol{Q}(\boldsymbol{\sigma}_e)$
其中 $\boldsymbol{Q}(\boldsymbol{\sigma}_e) = c_1 k_1 \frac{\boldsymbol{\sigma}_e}{(\|\boldsymbol{\sigma}_e\| + \varepsilon_1)^\eta} + c_1 k_2 \frac{\boldsymbol{\sigma}_e}{(\|\boldsymbol{\sigma}_e\| + \varepsilon_1)^{-\eta}}$
$c_1, k_1, k_2$ 为与预设收敛时间 $T_{p1}$ 及设计参数 $\alpha_1, \eta$ 相关的系数，$\varepsilon_1$ 为姿态预设精度常数。

5. 如权利要求4所述的方法，其特征在于，步骤3中的内环控制力矩律设计为：
$\boldsymbol{\tau} = -\boldsymbol{f}(t) - \boldsymbol{J}_0\dot{\boldsymbol{Q}}(\boldsymbol{\sigma}_e) + \boldsymbol{\tau}_c$
其中 $\boldsymbol{f}(t)$ 为已知动力学非线性项，$\boldsymbol{\tau}_c$ 为预设时间镇定项：
$\boldsymbol{\tau}_c = -c_2 \left[ k_3 \frac{\boldsymbol{s}}{(\|\boldsymbol{s}\| + \varepsilon_2)^\eta} + k_4 \frac{\boldsymbol{s}}{(\|\boldsymbol{s}\| + \varepsilon_2)^{-\eta}} \right]$
$c_2, k_3, k_4$ 为与预设收敛时间 $T_{p2}$ 及设计参数 $\alpha_2$ 相关的系数，$\varepsilon_2$ 为滑模变量预设精度常数。

6. 如权利要求1所述的方法，其特征在于，步骤4中的参数综合需满足以下可行性条件以保证闭环系统不违反双重饱和约束：
对于力矩约束：
$\Delta_{f,i,\max} + \overline{(J_0 \dot Q)}_i + \bar\tau_c \le \gamma_u\tau_{i,\max}$
对于角动量约束：
$H_{\max} + (\mu_i + \nu_i) (\varepsilon_2 + \bar Q) + \nu_i \omega_{d,\max} \le \gamma_h h_{w,i,\max}$
其中各项上界由预设精度 $\varepsilon_1, \varepsilon_2$、扰动界值及参考轨迹界值确定。

# 说明书

## 技术领域
本发明涉及航天器控制技术领域，具体涉及一种在反作用飞轮力矩和角动量双重饱和约束下的刚体航天器精确预设时间预设精度姿态控制方法。

## 背景技术
在空间交会对接、编队重构等时效性关键（Time-critical）的任务中，航天器姿态需要在满足轨道几何、通信窗口或多星协同要求的精确终端时刻 $T_f$ 到达指令状态。这类任务对受限于反作用飞轮双重饱和（即力矩饱和与角动量饱和）的航天器提出了严峻挑战。

现有的有限时间控制（FTC）和固定时间控制（FxTC）虽然能保证有限时间收敛，但FTC的收敛时间依赖于初始状态，而FxTC仅能提供一个保守的收敛时间上界，难以作为显式设计参数与任务要求的终端时刻精确匹配。预设时间控制（PTC）虽然允许显式设定收敛时间，但现有方法存在两方面主要缺陷：
1. **保守性强**：现有方法为保证理论上的稳定性，往往使系统在预设时间之前过早收敛（例如设定500秒，实际250秒即到达），导致初始控制量过大，且到达目标后需要进行长时间的姿态保持，造成能源和动量资源的浪费，并在多星协同中引发布局冲突风险。
2. **缺乏双重饱和处理**：现有PTC研究大多通过时间缩放或高增益来实现快速收敛，极易导致执行机构饱和。虽然部分研究考虑了力矩饱和，但极少同时考虑角动量饱和。一旦飞轮角动量达到饱和，航天器将彻底丧失控制力矩输出能力，导致控制失效。此外，对于大角度机动，若不考虑物理限制随意设定预设时间 $T_f$，极易导致潜在的约束违背。

基于优化的轨迹规划方法可以处理约束，但作为开环策略对扰动敏感；而现有的闭环反馈方法难以严格保证预设时间收敛且不易处理复杂的双重状态约束。因此，迫切需要一种既能保证在预设终端时刻精确到达，又能严格满足双重饱和约束的控制方案。

## 发明内容
本发明的目的在于克服现有技术的不足，提出一种双重饱和受限下的航天器精确预设时间预设精度控制方法（EPTPAC）。该方法通过外环规划与内环跟踪相结合的双环架构，确保航天器在严格满足力矩与角动量约束的前提下，在用户指定的时刻精确收敛至目标状态，且误差满足预设精度。

本发明采用的技术方案如下：
一种双重饱和受限下的航天器精确预设时间预设精度控制方法，包括以下步骤：

**步骤一：建立系统模型与约束**
建立刚体航天器的姿态运动学与动力学模型。采用修正罗德里格参数（MRPs）$\boldsymbol{\sigma}$ 描述姿态，动力学方程包含标称惯量 $\boldsymbol{J}_0$、模型不确定性 $\Delta\boldsymbol{J}$、环境扰动 $\boldsymbol{d}$ 及反作用飞轮控制力矩 $\boldsymbol{\tau}$。
定义反作用飞轮的双重饱和约束：
1. 力矩约束：$|\tau_i(t)| \le \tau_{i,\max}$
2. 角动量约束：$|h_{w,i}(t)| \le h_{w,i,\max}$

**步骤二：外环约束感知轨迹规划**
根据任务要求的总时长 $T_f$，构建带约束的最优控制问题（OCP）。
目标函数：最小化控制力矩及力矩变化率的加权范数。
约束条件：
- 满足航天器运动学与动力学微分方程；
- 满足边界条件：$t=0$ 时为初始状态，$t=T_f$ 时为目标状态；
- 满足收紧的饱和约束：$|\tau_i(t)| \le (1-\gamma_u)\tau_{i,\max}$，$|h_{w,i}(t)| \le (1-\gamma_h)h_{w,i,\max}$。其中 $\gamma_u, \gamma_h \in (0,1)$ 为预留给内环控制器的安全裕度。
求解该OCP得到参考轨迹 $\boldsymbol{\sigma}_d(t)$ 和 $\boldsymbol{\omega}_d(t)$，该轨迹在物理上可行且在 $T_f$ 时刻精确到达目标。

**步骤三：内环非奇异预设时间跟踪控制**
定义姿态跟踪误差 $\boldsymbol{\sigma}_e$ 和角速度跟踪误差 $\boldsymbol{\omega}_e$。
设计非奇异预设时间滑动模态面 $\boldsymbol{s}$：
$$ \boldsymbol{s} = \boldsymbol{\omega}_e + k_1 \frac{\boldsymbol{\sigma}_e}{(\|\boldsymbol{\sigma}_e\| + \varepsilon_1)^\eta} + k_2 \frac{\boldsymbol{\sigma}_e}{(\|\boldsymbol{\sigma}_e\| + \varepsilon_1)^{-\eta}} $$
其中 $\varepsilon_1$ 为姿态预设精度，$0<\eta<1$。
设计控制力矩律：
$$ \boldsymbol{tau} = \boldsymbol{\tau}_{\mathrm{eq}} + \boldsymbol{\tau}_c $$
$$ \boldsymbol{\tau}_c = - c_2 \left[ k_3 \frac{\boldsymbol{s}}{(\|\boldsymbol{s}\| + \varepsilon_2)^\eta} + k_4 \frac{\boldsymbol{s}}{(\|\boldsymbol{s}\| + \varepsilon_2)^{-\eta}} \right] $$
其中 $\varepsilon_2$ 为滑模变量预设精度。该控制器包含正则化项，避免了传统PTC在误差趋零时的奇异问题，并保证系统状态在预设时间 $T_{p1}, T_{p2}$ （需满足 $T_{p1}+T_{p2} < T_f$）内收敛至预设精度邻域。

**步骤四：可行性参数综合**
基于两个定理（力矩可行性定理与角动量可行性定理）确定系统参数。根据已知的扰动上界 $T_{d,\max}$ 和角动量漂移上界 $H_{\max}$，校验所选的裕度 $\gamma_u, \gamma_h$ 和精度参数 $\varepsilon_1, \varepsilon_2$ 是否满足不等式条件。如果满足，则理论保证闭环系统在整个 $[0, T_f]$ 区间内不会触犯真实的物理约束。

本发明的有益效果在于：
1. **精确终端时间收敛**：通过“规划+跟踪”的架构，外环保证到达时间的可行性，内环保证跟踪的快速性，避免了传统PTC方法的过早收敛问题，实现了真正的“Just-in-time”到达，节省了姿态保持阶段的能耗。
2. **物理可实现性保证**：不仅考虑了常见的力矩饱和，还显式处理了角动量饱和这一致命约束。通过参数综合方法，从理论上排除因误差累积导致飞轮动量饱和失控的风险。
3. **预设精度可调**：引入正则化参数 $\varepsilon_1, \varepsilon_2$，消除了控制奇异性，并允许用户根据任务需求和传感器精度灵活设定最终的稳态误差范围。

## 附图说明
[此处需插入附图1：EPTPAC控制框架流程图]
图1展示了本发明提出的双环控制架构，包含外环轨迹规划器、内环预设时间控制器及参数综合模块的交互关系。

[此处需插入附图2：姿态跟踪误差收敛曲线]
图2展示了在案例仿真中，姿态跟踪误差在预设时间内收敛至预设精度范围，并在终端时刻保持在精度要求内的效果。

[此处需插入附图3：飞轮力矩与角动量变化曲线]
图3展示了在控制过程中，反作用飞轮的输出力矩和累积角动量均严格保持在设定的物理阈值之内。

## 具体实施方式
下面结合具体实施例对本发明做进一步详细说明。

**1. 系统建模**
航天器姿态运动学与动力学方程描述为：
$$ \dot{\boldsymbol{\sigma}} = \boldsymbol{G}(\boldsymbol{\sigma}) \boldsymbol{\omega} $$
$$ (\boldsymbol{J}_0 + \Delta\boldsymbol{J}) \dot{\boldsymbol{\omega}} = - \boldsymbol{\omega}^{\times} [(\boldsymbol{J}_0 + \Delta\boldsymbol{J}) \boldsymbol{\omega} + \boldsymbol{h}_w ] + \boldsymbol{d} + \boldsymbol{\tau} $$
$$ \boldsymbol{\tau} = -\dot{\boldsymbol{h}}_w $$
其中 $\boldsymbol{\sigma}$ 为MRP姿态参数，$\boldsymbol{\omega}$ 为角速度，$\boldsymbol{J}_0$ 为标称惯量，$\boldsymbol{h}_w$ 为飞轮角动量。
约束条件为 $|\tau_i| \le 0.2$ Nm， $|h_{w,i}| \le 4.0$ Nms。

**2. 外环规划**
设定任务终端时间 $T_f = 120$ s。选取安全裕度 $\gamma_u = 0.2$ (即预留20%力矩裕度)，$\gamma_h = 0.1$ (即预留10%动量裕度)。
采用GPOPS-II软件求解最优控制问题，得到参考轨迹 $\boldsymbol{\sigma}_d(t), \boldsymbol{\omega}_d(t)$ 及参考输入 $\boldsymbol{\tau}_{\mathrm{ref}}(t)$。该轨迹在 $t=120$s 时精确到达目标姿态 $\boldsymbol{\sigma} = [0,0,0]^T$。

**3. 内环控制器设计**
设定任务精度要求 $\varepsilon_1 = 10^{-5}$。
选取参数 $\eta = 0.2$。
内环时间分配：滑模面收敛时间 $T_{p2} = 15$ s，姿态误差收敛时间 $T_{p1} = 100$ s。满足 $15+100 < 120$。
设计滑模面：
$$ \boldsymbol{s} = \boldsymbol{\omega}_e + c_1 \left( k_1 \frac{\boldsymbol{\sigma}_e}{(\|\boldsymbol{\sigma}_e\| + \varepsilon_1)^\eta} + k_2 \frac{\boldsymbol{\sigma}_e}{(\|\boldsymbol{\sigma}_e\| + \varepsilon_1)^{-\eta}} \right) $$
设计控制律：
$$ \boldsymbol{\tau} = -\boldsymbol{f}(t) - \boldsymbol{J}_0\dot{\boldsymbol{Q}} - c_2 \left[ k_3 \frac{\boldsymbol{s}}{(\|\boldsymbol{s}\| + \varepsilon_2)^\eta} + k_4 \frac{\boldsymbol{s}}{(\|\boldsymbol{s}\| + \varepsilon_2)^{-\eta}} \right] $$
其中 $\varepsilon_2$ 和控制增益 $\alpha_1, \alpha_2$ 根据参数综合公式计算得到，以确保 $\tau_c$ output不超过预留的 $\gamma_u \tau_{\max}$ 裕度。

**4. 实施效果**
在存在惯量不确定性（5%偏差）和外部扰动（最大0.03 Nm）的情况下进行仿真。
结果表明：
1. 航天器实际姿态在 $t=120$s 时精确到达目标，且误差小于 $10^{-5}$，没有出现提前收敛这导致的长时间等待。
2. 控制过程中，实际输出力矩最大值未超过 0.2 Nm，飞轮角动量未超过 4.0 Nms，严格满足物理约束。
3. 即使在加入瞬时大扰动（0.06 Nm，超出设计边界）的情况下，系统仍能快速恢复并在终端时刻满足精度要求，验证了鲁棒性。

本实施例验证了EPTPAC框架在解决时效性任务中双重饱和约束问题的有效性。
