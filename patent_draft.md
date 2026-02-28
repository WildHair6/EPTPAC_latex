# 1. 发明名称
一种双重饱和受限下的航天器精确预设时间预设精度控制方法

# 2. 技术领域
本发明属于航天器姿态控制技术领域，具体涉及一种在反作用飞轮力矩幅值和角动量幅值双重饱和约束下的刚体航天器精确预设时间预设精度姿态跟踪控制方法。

# 3. 背景技术
在空间交会对接、编队重构等时效性关键（Time-critical）任务中，航天器姿态需要在满足轨道几何、通信窗口或多星协同要求的精确终端时刻 $T_f$ 到达指令状态。这类任务对受限于反作用飞轮双重饱和（即力矩饱和与角动量饱和）的航天器控制提出了严峻挑战。

现有技术存在以下问题：
1. **现有预设时间控制（PTC）方法的保守性**：现有方法为保证理论稳定性，往往导致系统在预设时间之前过早收敛（例如设定500秒，实际250秒即到达）。这种过早收敛导致初始控制量过大，且到达目标后需要进行长时间的姿态保持，造成能源和动量资源的浪费。
2. **缺乏对双重饱和的统一处理**：现有研究大多只考虑力矩饱和，均忽视角动量饱和。然而，一旦飞轮角动量达到饱和，航天器将彻底丧失输出控制力矩的能力，导致控制失效。此外，对于大角度机动，若不考虑物理限制随意设定预设时间，极易导致潜在的约束违背。
3. **开环规划缺乏鲁棒性**：虽然基于优化的轨迹规划可以处理约束，但本质上是开环的，难以抵抗模型不确定性和外部扰动，导致终端时刻精度下降。

# 4. 发明内容
为了解决上述技术问题，本发明提供了一种双重饱和受限下的航天器精确预设时间预设精度控制方法（EPTPAC）。该方法采用双环控制架构，外环规划器生成满足终端时刻和饱和约束的参考轨迹，内环控制器实现对参考轨迹的预设时间预设精度跟踪，并通过参数综合确保全过程满足物理约束。这样既克服了传统预设时间控制过早收敛的问题，又解决了双重饱和约束下的物理可实现性难题。

本发明采取的技术方案如下：
一种双重饱和受限下的航天器精确预设时间预设精度控制方法，其特征在于，包括以下步骤：

**步骤1：建立系统模型与双重饱和约束**
建立刚体航天器的姿态运动学与动力学模型：
$$
\dot{\boldsymbol{\sigma}} = \boldsymbol{G}(\boldsymbol{\sigma}) \boldsymbol{\omega}, \quad (\boldsymbol{J}_0 + \Delta\boldsymbol{J}) \dot{\boldsymbol{\omega}} = - \boldsymbol{\omega}^{\times} [(\boldsymbol{J}_0 + \Delta\boldsymbol{J}) \boldsymbol{\omega} + \boldsymbol{h}_w ] + \boldsymbol{d} + \boldsymbol{\tau}
$$
其中，$\boldsymbol{\sigma}$ 为修正罗德里格斯参数（MRP），$\boldsymbol{\omega}$ 为角速度，$\boldsymbol{J}_0$ 为标称惯量，$\Delta\boldsymbol{J}$ 为惯量不确定性，$\boldsymbol{h}_w$ 为飞轮角动量，$\boldsymbol{d}$ 为外部扰动，$\boldsymbol{\tau} = -\dot{\boldsymbol{h}}_w$ 为控制力矩。
受限于反作用飞轮的双重饱和约束条件定义为：
$$
|\tau_i(t)| \le \tau_{i,\max}, \quad |h_{w,i}(t)| \le h_{w,i,\max}, \quad i \in \{1,2,3\}
$$

**步骤2：设计外环约束可行规划器**
为确保在用户指定的终端时刻 $T_f$ 精确到达目标状态，构建约束最优控制问题（OCP），生成参考轨迹 $(\boldsymbol{\sigma}_d, \boldsymbol{\omega}_d)$：
$$
\min \quad \mathcal{J} = \int_0^{T_f} \left( \|\boldsymbol{\tau}_{\mathrm{ref}}(t)\|^2 + \lambda \|\dot{\boldsymbol{\tau}}_{\mathrm{ref}}(t)\|^2 \right) dt
$$
该规划需满足边界条件 $\boldsymbol{\sigma}(0)=\boldsymbol{\sigma}_0, \boldsymbol{\sigma}(T_f)=\boldsymbol{\sigma}_{\mathrm{target}}$，并在规划层引入缩紧约束：
$$
|\tau_{\mathrm{ref},i}(t)| \le (1-\gamma_u)\tau_{i,\max}, \quad |h_{w,i}(t)| \le (1-\gamma_h)h_{w,i,\max}
$$
其中 $\gamma_u, \gamma_h \in (0,1)$ 为留给内环控制和抗扰动的裕度。

**步骤3：设计内环非奇异预设时间预设精度控制器**
构建耦合姿态与角速度误差的非奇异滑模面 $\boldsymbol{s}$：
$$
\boldsymbol{s} = \boldsymbol{\omega}_e + c_1 \left( k_1 \frac{\boldsymbol{\sigma}_e}{(\|\boldsymbol{\sigma}_e\| + \varepsilon_1)^\eta} + k_2 \frac{\boldsymbol{\sigma}_e}{(\|\boldsymbol{\sigma}_e\| + \varepsilon_1)^{-\eta}} \right)
$$
基于该滑模面，设计包含前馈补偿项和预设时间镇定项的控制力矩 $\boldsymbol{\tau}$：
$$
\boldsymbol{\tau} = -\boldsymbol{f}(t) - \boldsymbol{J}_0\dot{\boldsymbol{Q}}(\boldsymbol{\sigma}_e) - c_2 \left[ k_3 \frac{\boldsymbol{s}}{(\|\boldsymbol{s}\| + \varepsilon_2)^\eta} + k_4 \frac{\boldsymbol{s}}{(\|\boldsymbol{s}\| + \varepsilon_2)^{-\eta}} \right]
$$
其中，$T_{p1}, T_{p2}$ 分别为滑模面建立和误差收敛的预设时间，$c_1, c_2, k_i$ 为与预设时间相关的增益参数，$\varepsilon_1, \varepsilon_2$ 为预设稳态精度，$\eta \in (0,1)$ 为调节指数。该控制器确保系统状态在 $T_f$ 之前进入并保持在精度区域 $\|\boldsymbol{\sigma}_e\| \le \varepsilon_1, \|\boldsymbol{s}\| \le \varepsilon_2$ 内。

**步骤4：闭环参数综合与可行性保障**
通过参数综合，协调外环规划裕度与内环控制器参数，确保闭环系统在扰动下满足双重饱和约束。
导出的力矩饱和充分条件为：
$$
\Delta_{f,i,\max} + \overline{(J_0 \dot Q)}_i + \bar\tau_c \le \gamma_u \tau_{i,\max}
$$
角动量饱和充分条件为：
$$
H_{\max} + (\mu_i + \nu_i) (\varepsilon_2 + \bar Q) + \nu_i \omega_{d,\max} \le \gamma_h h_{w,i,\max}
$$
这保证了物理可实现性，即 $|\tau_i(t)| \le \tau_{i,\max}$ 和 $|h_{w,i}(t)| \le h_{w,i,\max}$ 全程成立。

# 5. 发明效果
本发明相比现有技术具有以下有益效果：
1. **精确终端时刻收敛**：通过双环设计，确保航天器在用户指定的终端时刻 $T_f$ 精确到达目标状态，避免了传统方法中的过早收敛或收敛时间不确定的问题。
2. **双重饱和约束保障**：显式处理了反作用飞轮的力矩饱和与角动量饱和，通过外环规划与内环参数综合，从物理层面保证了控制任务的可实现性，避免了因角动量饱和导致的控制失效。
3. **预设精度控制**：能够根据任务需求显式指定姿态控制精度，控制器参数设计简便，且具有良好的抗扰动鲁棒性。

# 6. 附图及附图说明
**图1** 为本发明的双环控制系统框架示意图。
**图2** 为实施例中航天器姿态跟踪响应曲线，显示实际姿态精确跟踪参考轨迹。
**图3** 为实施例中姿态跟踪误差与角速度跟踪误差曲线，显示误差在预设时间内收敛至预设精度范围。
**图4** 为实施例中控制力矩与飞轮角动量响应曲线，显示控制量全程满足双重饱和约束。

# 7. 具体实施方式
以下结合具体仿真案例验证本发明的有效性。
**参数设置**：
设定刚体航天器标称转动惯量 $\boldsymbol{J}_0 = \mathrm{diag}([200, 250, 300]) \mathrm{kg\cdot m^2}$，惯量不确定性为 $5\%$。
执行机构约束：最大力矩 $\tau_{i,\max} = 0.2 \mathrm{N\cdot m}$，最大飞轮角动量 $h_{w,i,\max} = 4.0 \mathrm{kg\cdot m^2/s}$。
任务要求：初始姿态 $[0.2, 0.3, -0.3]^\top$，目标姿态 $[0, 0, 0]^\top$，终端时刻 $T_f = 120\mathrm{s}$。
设计参数：预设精度 $\varepsilon_1 = 10^{-5}$，预设时间参数 $T_{p1}=100\mathrm{s}, T_{p2}=15\mathrm{s}$。

**实施结果**：
1. **精确收敛**：仿真结果表明，航天器姿态在 $t=120\mathrm{s}$ 时刻精确到达目标零姿态，姿态跟踪误差 $\boldsymbol{\sigma}_e$ 收敛至 $10^{-5}$ 量级，且无过早收敛现象。
2. **约束满足**：在整个机动过程中，三轴控制力矩始终保持在 $\pm 0.2 \mathrm{N\cdot m}$ 范围内，飞轮角动量始终保持在 $\pm 4.0 \mathrm{kg\cdot m^2/s}$ 范围内，未发生饱和越界。
3. **鲁棒性**：在施加 $0.03 \mathrm{N\cdot m}$ 的外部持续扰动及 $0.06 \mathrm{N\cdot m}$ 的瞬时大干扰下，系统仍能快速恢复并保持预设精度，验证了方法的鲁棒性。
