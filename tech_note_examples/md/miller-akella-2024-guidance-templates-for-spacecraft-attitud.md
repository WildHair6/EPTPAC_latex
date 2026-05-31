## Engineering Notes 

## Guidance Templates for Spacecraft Attitude Maneuver Planning 

Andrew J. Miller[∗] and Maruthi R. Akella[†] University of Texas at Austin, Austin, Texas 78705 

https://doi.org/10.2514/1.G007862 

## I. Introduction 

A TTITUDE control is an essential action in autonomous debrisremoval, a space mission that will become feasible in the near future. Nishida et al. have outlined a debris removal satellite system that uses a robotic arm to capture debris [1]. Hakima et al. propose a debris removal mission using deorbital CubeSats [2]. When the removal satellite attaches to the debris object, the inertia of the spacecraft system changes due to the rigid attachment to the piece of debris. Pieces of debris often have poorly determined mass properties and uncertainty in their exact spatial geometry, causing large errors in their inertia estimates. Although the exact inertia properties of the debris are unknown, the availability of upper and lower bound estimates of their principal inertias is practically a reasonable proposition. Adaptive controllers have the ability to online estimate the – composite inertia of the spacecraft debris system, but such schemes introduce control complexity and additionally the need for satisfaction of certain restrictive conditions such as persistence of excitation – and uniform detectability [3 5]. 

Any residual angular momentum of the debris object will transfer to the spacecraft/debris composite system due to conservation laws and will need to be stabilized before undertaking any further mission maneuvers. Figure 1 depicts a spacecraft attaching to a piece of debris and how the spacecraft–debris inertia and angular velocity change upon attachment. To enable large-scale autonomous debris removal, attitude controllers that require only inertia estimates while preserving available torque bounds will play a central role. 

Attitude control laws are commonly designed with the goal of meeting closed-loop stability and steady-state performance specifi– cations [6 8]. On the other hand, the transient response of the closedloop system and torque requirements are often difficult to quantify when using nonlinear control design tools. As a result, unless heuristic safeguards are additionally implemented, violations of the maximum available torque may occur during the transient phase of the maneuver sequence. A control law that violates the maximum commandable torque, even for a brief moment in the transient phase, can potentially compromise the required stability margins. It is necessary to develop attitude controllers that do not violate the 

maximum available torque. In terms of prior literature, Boskovic et al. have proposed an adaptive attitude control that considers input saturation [9,10]. Globally stabilizing saturated attitude control robust to external disturbances has been achieved through both a hybrid adaptive control scheme and a smooth switch-like controller. [11,12] Akella et al. have proposed an attitude tracking controller subject to torque saturation without angular velocity measurements [13]. The proportional derivative (PD) controller can be bounded to arbitrarily small torque bounds by tuning the control gains and parameterizing the attitude with bounded parameters such as the quaternion or modified Rodriguez parameters (MRPs) [14]. All of the mentioned approaches can help constrain the input torque, but most of them offer no time guarantees for the stabilization maneuver and only assure that the attitude states will be stabilized asymptotically, i.e., as time goes to infinity. Almeida and Akella have proposed a finite-time controller that stabilizes a spacecraft in a user-specified amount of timewithout knowledge of the inertia matrix [6]. Zhu et al. formulated a finite-time attitude stabilization scheme using a sliding mode controller while considering inertia uncertainty [15]. However, both of these aforementioned control schemes have no guarantees for control commands staying within prescribed torque bounds. Guo et al. have proposed a saturated finite-time attitude controller using sliding mode techniques but offer no estimate for the time to complete the maneuver [16]. For autonomous maneuver scheduling, it is critical to have finite-time guarantees (i.e., estimates for how long the maneuver is expected to last) and guarantees that the controller never violates the maximum available torque. 

The primary contribution of this Note is a novel control scheme for finite-time attitude stabilization that only requires upper and lower bound estimates of principal inertias. A preliminary version of this work was previously reported by the authors in Ref. [17]. First, a PD controller is used to slow down the spacecraft until certain wellcharacterized conditions are satisfied. Control gains are chosen such that the controller satisfies the torque bounds for the duration of the maneuver. The spacecraft is slowed down by the PD controller until it reaches a state where the hand-off controller is guaranteed to operate within the torque bounds. The hand-off controller ensures that the torque signal remains continuous in between the PD and finite-time controllers and thereby eliminates the possibility for high-bandwidth torque actuation. Finally, the finite-time controller brings the spacecraft to rest at the desired orientation within a user-specified time duration. Each phase only requires upper and lower bound estimates of the principal inertias, provides a guaranteed amount of time to finish, and can be performed without exceeding the maximum commandable torque. 

The Note is organized as follows: Section II presents motivation for our method of doing attitude stabilization in finite time with unknown inertia and torque constraints. Section III presents the governing equations and stabilization goal. Sections IVand Voutline the torque bounds, time guarantees, and stability of each controller. Section VI outlines how the combination of the three controllers 

Fig. 1 Spacecraft attaching to piece of space debris. 

accomplishes stabilization. Section VII showcases the proposed methodology on a debris capture example mission, and Sec. VIII summarizes the contributions. 

## II. Motivation 

## A. Known Inertia Solutions are Sensitive to Inertia Perturbations 

In the case where the inertia of the system is perfectly known and assuming perfect state measurements, open-loop-based attitude stabilization solutions are available. Two solutions for known inertia stabilization are outlined: a time-optimal solution and the patched solution. The time-optimal solution is posed as a discretized optimal trajectory problem using a problem-based approach, where the cost function is time and input constraints are placed on the 2-norm of the control torque [18]. The time interval is discretized into N � 80 intervals. The state constraints from interval to interval follow the approximate equations of motion from a first-order Euler integration step. Simulations of time-optimal attitude stabilization are carried out for the cases summarized in Table 1. 

To investigate the optimal solution’s sensitivity to inertia perturbations, simulations were run using the optimal torque signal with a perturbed inertia matrix. 100 runs were completed for each percent inertia uncertainty and each case. The inertia matrices were perturbed by adding Gaussian noise, with the standard deviation proportional to the percent uncertainty, in such a way that guarantees that the perturbed matrix does not violate the properties of an inertia matrix. Figures 2 and 3 show boxplots of the final principal angle and angular velocity for each stabilization case. Evenin the 2% inertia uncertainty case, the spacecraft’s final angular displacement far exceeds pointing requirements for attitude maneuvers. Analysis of other cases of restto-rest maneuvers with various net maneuver angles (ranging from 10° to 150°) found that the average final angle and average final angular velocity were both proportional to the maneuver angle for each percent inertia uncertainty. The time-optimal solution to the attitude stabilization problem is sensitive to inertia uncertainty. 

Computing the time-optimal maneuver is computationally expensive, especially when done onboard the spacecraft. A computationally efficient method for known inertia stabilization is the patched solution [19]. This method relies on rotating the spacecraft about a constant axis; that way the state of the spacecraft is perfectly known at all times. A full analytic state history and torque profile are available, and the total time of flight is given. The patched solution is guaranteed to never violate the maximum commandable torque for the duration of the maneuver. Using the patched maneuver to stabilize case 2 in Table 1, results in a time of flight of 15.52 s, twice as long as the optimal solution. 

Just as the time-optimal solution is highly sensitive to inertia perturbations, the patched solution suffers the same issue. Even debris removal missions where the geometry and mass properties of the debris are known with high accuracy suffer from inertia uncertainty due to the complicated and imperfect attachment of the retrieval spacecraft to the debris object. The sensitivity of the known inertia solutions to inertia uncertainties renders them generally ineffective for reorientation missions. Closed-loop-based attitude controllers that are robust to inertia uncertainties are necessary for these classes of missions. 

Fig. 2 Case 1: inertia perturbation analysis. 

Fig. 3 Case 2: inertia perturbation analysis. 

## B. Inadequacy of Standard PD Controllers 

For closed-loop attitude maneuvers done with a PD controller, the PD control gains are carefully tuned/chosen such that the control law never violates the maximum prescribed torque. This is commonly done through extensive offline verification with Monte Carlo simulation. Qualitatively, the controller is most likely to violate the maximum torque budget at the beginning phase of the maneuver, where the attitude and rate errors are the largest. As the controller brings the spacecraft closer to rest, the norms of the state quantities decay under an exponentially decreasing envelope, correspondingly leading to reduced requirements on the magnitude of the torque commands. As the maneuver progresses, the PD controller takes a conservative approach by failing to make use of the full amount of available torque. This work proposes using a sequence of a PD controller followed by a finite-time (FT) controller that takes advantage of excess torque from the PD controller to stabilize a spacecraft faster than strictly using a PD controller. The time advantages of the PD � FT controller clearly manifest under certain conditions on attitude stabilization maneuvers. Qualitatively speaking, the larger the total maneuverangle ϕ, the finer the pointing requirement δ,or the larger the spacecraft, the greater the time savings with the PD + FT controller. For example, stabilizing a 3U CubeSat is done faster with the PD controller until the total maneuver angle exceeds ϕ � 120°, then the PD + FT controller stabilizes faster for all pointing requirements. For a larger, 100 kg spacecraft with a total maneuver angle ϕ � 30°, stabilization is faster with the PD + FT controller for pointing requirements finer than δ � 30 arcs. Any increase in total maneuver angle or decrease in pointing requirement only furthers the time advantage of the PD � FT controller. The implication is that spacecraft operators cannot rely on the PD controller for all maneuver angles and pointing requirements and are often better served by a 

Table 1 System parameters and initial conditions for time-optimal cases 

||Case 1: 2U + 1U|Case 1: 2U + 1U|debris|Case 2: 3U CubeSat|
|---|---|---|---|---|
|σ0|[0.076; 0.076;|0.076] (30 deg rotation)||[0.2391; 0.2391; 0.2391] (90 deg rotation)|
|ω0 �rad∕s�||[0; 0; 0]||[−0.05; 0.05;−0.05]|
|J�kg⋅m2�|0.0145|0|−0.0015|0.0416<br>0<br>0|
||0|0.022|0|0<br>0.0083<br>0|
||−0.0015|0|0.0142|0<br>0<br>0.0416|
|umax �N⋅m�||0.0032||0.0032|
|tf �s�||3.3163||7.4503|

control scheme that delivers faster convergence toward the attitude stabilization objective. 

## III. System Dynamics 

The rigid body rotating in space is described kinematically by the MRPs through 

where e^ is the unit principal rotation vector and Φ is the principal rotation angle [14]. The kinematics of the MRPs and the dynamics of the rigid body are given by 

where B�σ���1 − σ[⊤] σ�I3 � 2σ[×] � 2σσ[⊤] , the angular velocity vector ω is expressed in the body fixed frame, and z[×] is the skewsymmetric matrix associated with z ∈ R[3] , J ∈ R[3][×][3] , J � J[⊤] > 0 is the inertia matrix, and u is the input torque. The notation j ⋅ j is used to denote magnitude for scalar variables and Euclidean norm for vectors and matrices. Note that the product σ[⊤] B�σ� satisfies the property: 

where b�σ�� 1 � σ[⊤] σ ≥ 1 is a scalar that satisfies jσj ≤ 12[b][�][σ][�][.] Also, B�σ� satisfies jB�σ�j � b�σ�. This work assumes that perfect measurements are available to the user and no torque disturbances. We denote J[�] and J to be the largest and smallest principal inertias, respectively. We assume that upper and lower bounds on the principal inertias are available to the user: 

where Jm < J is the known lower bound for the minimum principal inertia, and JM > J[�] is the known upper bound for the maximum principal inertia. 

The control objective is to bring the spacecraft–debris from its initial orientation and angular rate σ0, ω0 to rest at the desired final orientation σ � 0, ω � 0 in a finite amount of time. Additionally, the torque profile must never exceed the maximum commandable torque juj ≤ umax, and this must be done all without knowledge of the full inertia matrix (only JM, Jm known). 

## IV. Proportional Derivative Controller 

The classic PD controller is used to slow down and bring the – spacecraft debris system closer to the final orientation. The authors of Ref. [20] show the uniform exponential convergence bound due to the PD controller by construction of a strictified Lyapunov function. This work outlines the results crucial for convergence of state variables and expands those results to provide explicit torque bounds. The PD control law is 

The lower bound of VPD is rewritten in the following form: 

which will later be used for time guarantees, where μm � min �c − JM∕2; cJm∕4kp�. The decay of VPD is 

where α is chosen to satisfy 0 < α < min �c∕4kp; kp∕�kv�1 � 2c∕JM��; 1�. Equation (10) shows exponential convergence of VPD, which is used to show the convergence of jσj and jωj. The largest value of VPD occurs at initial time: 

Combining Eqs. (9) and (10) results in an upper bound for the norms of the state variables: 

Solving Eq. (12) for time gives an amount of time that guarantees that the PD controller will achieve the desired state bounds jωPDj, jσPDj, respectively: 

Note that μmjσPDj[2] < V[�] PD and μmjωPDj[2] < V[�] PD, and therefore evaluating the logarithm always results in a negative number, making tPD > 0 always. The PD controller is used until the orientation and angular velocity are within the desired magnitudes jσj ≤ jσPDj, and jωj ≤ jωPDj, which is guaranteed to happen within tPD seconds. Both jσPDj and jωPDj are left to be determined and will be chosen such that the finite-time controller satisfies the torque bound. 

## A. Choosing Control Gains 

The control gains must be chosen so that the torque never exceeds the maximum commandable torque umax. Take the norm of the controller in Eq. (5): 

Using the upper bound of the state variables in Eq. (9) results in 

where kp > 0 and kv > 0 are the proportional and derivative gains, respectively. The authors of Ref. [20] show uniform exponential stability through the following Lyapunov-like function: 

where c > 0 is a sufficiently large finite constant that will be defined later. The lower bound for VPD is 

Suppose that we parameterize the control gains as kp � ω[2] n, kv � 2ωn to mimic a critically damped second-order linear system. For ωn sufficiently small, c � JM∕2 � ϵ, where ϵ > 0. With the chosen parameterization, evaluate as μm � ϵ. By mapping between the MRPs and its shadow set on the switching surface σ[⊤] σ � 1 allows the MRPsto be bounded by jσj ≤ 1. The new parameterization results in the Lyapunov function evaluating to 

Simplify the torque upper bound u�PD as 

where parameter c satisfies the following inequality in order to keep the Lyapunov-like function positive definite. 

Table 2 Maximum size of debris for different classes of spacecraft 

|Class of spacecraft|Maximum mass of debris, kg|
|---|---|
|4 kg CubeSat|80|
|15 kg CubeSat|225|
|100 kg spacecraft|400|
|440 kg spacecraft|470|

## u�PD ��ω[2] n � 2ωn� 

There is a minimum value ofNotice in Eq. (17) that u�PD is not monotonically decreasing withu�PD that cannot reach arbitrarily small ωn. valuesThe minimumof umax, valueregardless ofthat u�PDthecanparameterizationobtain dependsof controlon the gains.initial angular rate jω0j and the maximum principal inertia JM. For spacecraft capturing a piece of debris tumbling less than 2° per second, Table 2 summarizes the maximum size of debris that the following spacecraft can stabilize with the PD controller without violating the maximum commandable torque. 

The spacecraft in Table 2 can stabilize larger pieces of debris when the debris has a smaller angular rate. A piece of debris with an arbitrarily small angular rate can be stabilized by a spacecraft with an arbitrarily small torque. Choose jω0j � 2ϵ, and ωn � ϵ: 

Equation (18) shows that limϵ→0 �uPD�ϵ�� 0. The above parameterupper bound of torqueization proves that the user can always findu�PD is lower than the maximum commandable jω0j and ωn such that the torque umax. For debris with a large angular velocity, such that the PD controller is not guaranteed to operate without violating the maximum commandable torque, an initial detumble phase to decrease the system’s angular velocity is necessary for stabilization. 

In practice, the user chooses kp, kv that minimizes tPD. Parameterto findize the control gains as ωn such that u�PD kp ≤�uωmaxn, k. Thev � ω2ωn nthat evaluates the smallest and preform a grid search tPD is selected for implementation. Once the PD controller has reached the desired state bounds jσj ≤ jσPDj, jωj ≤ jωPDj, the hand-off controller is used to ensure continuity in the torque signal while transitioning to the finite-time phase. Details of the hand-off controller are found in Appendix A. 

## V. Finite-Time Controller 

The finite-time controller in Ref. [6] is an attitude controller that guarantees state convergence in a user-specified amount of time without knowledge of the inertia matrix. Any user-selected control gains ϕ1 > 0, ϕ2 > 0 and total time of flight tf guarantee finite-time convergence of state variables as well as bounded control input. The freedom to choose any values for control gains and time of flight comes with the cost of no guarantees for the upper bound of the control torque. This work builds upon Ref. [6] to give explicit upper bounds on the control torque and minimum time of flight required to achieve such bound. 

## A. Lyapunov Function 

The finite-time attitude controller is 

The chosen coefficients ensure positive definiteness of the Lyapunov function by design. An upper bound on the Lyapunov function can be found using the fact that 2ab ≤ a[2] � b[2] : 

Similarly, a lower bound of the Lyapunov function is again found using the fact that 2ab ≤ a[2] � b[2] : 

The proof that the finite-time controller stabilizes the system in finite time is presented in Appendix B. The pieces from the proof referenced in rest of the section are summarized below. The states are upper bounded by the Lyapunov function, 

and the conditions on the control gains: 

## B. Torque Upper Bound 

Recall the finite-time control law uFT � −b�σ��ϕ1μ[4] σ � ϕ2μ[2] ω�. An upper bound on the control law is found by first taking the norm of the control law: 

Replace the state variables with their upper bounds in terms of the Lyapunov function found in Eq. (22). Since μ[−][a] ≤ 1 ∀a > 0, the greatest upper bound of the torque occurs at initial time: 

The upper bound of the state variables �V[�] FT∕αm�[1][∕][2] is completely known in terms of a priori- known constants. While the exact states are not known a priori, upper bounds are known jσj ≤ jσHj and jωj ≤ jωHj: 

Combine Eq. (26) with the torque upper bound in Eq. (25): 

with μ�t�� tf∕�tf − t�, where tf is the user chosen time of flight [6]. The corresponding Lyapunov function is 

Equation (27) is a constructive upper bound for the control torque in terms of the a priori known state bounds, control gains, and upper and lower estimates of the principal inertias. Notice that the term JM∕Jm 

appears in the upper bound. The further the spacecraft is from being spherically symmetric (JM � Jm) or the worse the estimates of the principal inertias are, the larger the torque bound will be. If the spacecraft was performing a stabilization maneuver with zero initial states, in other words, already at the desired state, the torque required to perform the maneuver is zero as expected. For large state bounds jσHj, jωHj, such that the right-hand side of Eq. (27) exceeds the maximum commandable torque, there is no guarantee that the finite-time controller will remain within the maximum commandable torque. 

## C. Optimizing Control Gains 

contributionIn this section, control gainsto the upper bound ϕ1, ϕof2 are optimized to reduce theirtorque u�FT, which in turn decreases the total time of flight. As in Eq. (25), the sum of the control gains �ϕ1 � ϕ2� is directly proportional to the upper bound of torque. Choosing control gains is posed as an optimization problem. 

With time of flight as a decisionvariable, the optimization problem in Eq. (28) is a nonlinear program. Clearly, there is a minimum time of flight that allows all constraints in Eq. (28) to be satisfied. To find the minimum time of flight, assume that the control gains take the value of their lower bounds: 

Using the equality constraint, the minimum time of flight to satisfy the control gain constraints is 

By fixing the time of flight to some value satisfying Eq. (30), the optimization problem reduces to a linear program that can be easily solved. Simplify by substituting ϕ2 into ϕ1 through the equality constraint. Also, the strict inequalities are turned into nonstrict inequalities by adding and subtracting an arbitrarily small ϵ and rewriting the optimization problem as 

Equation (31) is a convex optimization problem with a linear objective function. Additionally, the feasibility set is closed. By Lemma 8.6 in Ref. [21], the global minimizer of Eq. (31) belongs to the boundary of the feasibility set. Equation (31) has a solution: 

with an optimal objective function value: 

The optimization problem in Eq. (31) approaches the optimization problem in Eq. (28) as epsilon goes to zero, ϵ → 0. The time of flight 

explicitly appears in the final value of the objective function. Increas-ing the time of flight to infinite causes the upper bound of torque u�FT to decrease monotonically but not to arbitrarily small values due to the first term in Eq. (33). 

Equation (34) summarizes the choice of control gains and time of flight. The user chooses a time of flight tf that satisfies the following inequality and a small epsilon ϵ. The optimal control gains are given in terms of the time of flight and upper and lower bounds of the principal inertias: 

Nowthat control gainsandtime offlight are set, the state bounds jσHj, jωHj to satisfy the torque budget are found. We seek to find by which guaranteefactor cslowu�FTthe≤initialumax. statesSubstitutingjσ0j, jωj0σjHmustj � cslowdecreasejσ0j andin orderjωHj �to cslowjω0j, respectively, into the finite-time torque bound in Eq. (27), and isolating for cslow results in a depressed cubic in the form of 

and has a known solution [22]: 

where Δ � q[2] ∕4 � p[3] ∕27. The state bounds jσHj, jωHj are the handoff controller upper bounds that ensure that the finite-time controller does not violate the maximum commandable torque. The nextsection describes how to choose all control gains and state bounds to ensure no torque violations. 

## VI. Full Torque Bounded Stabilization 

This section outlines how to choose the various parameters for the full stabilization maneuver. Algorithm 1 shows a pseudocode to guide the user how to choose parameters to satisfy all requirements and find the guaranteed time to complete stabilization. 

Parameters are chosen beginning with the finite-time phase. The time of flight in the finite-time phase tf is chosen to satisfy the inequality in Eq. (30) and then the control gains ϕ1, ϕ2 follow in Eq. (32). The PD control gains kp, kv are chosen to satisfy the torque constraint for the PD phase via Eq. (15). Next, the hand-off state bounds jσHj, jωHj are chosen to satisfy both the finite-time torque bounds via Eqs. (35) and (36) and the hand-off torque bounds via 

Algorithm 1: Choosing parameters 

ϕ1; ϕ2; tf ← Eq: �34� kp; kv s:t: u�PD ≤ umax ← Eq: �15� Choose smaller of bounds jσHj; jωHj s:t: u�FT ≤ umax ← Eqs: �35� and �36� jσHj; jωHj s:t: u�H ≤ umax ← Eq: �48� jσPDj; jωPDj; tH ← Eqs: �46� and �47� tPD ← Eq: �13� tG � tPD � tH � tf 

Eq. (48). Then, the PD state bounds jσPDj, jωPDj as well as the handoff time of flight tH are chosen to guarantee that the hand-off controller remains within its state bounds via Eqs. (46) and (48). Finally, the time-guaranteed time of flight for the PD phase, tPD, is computed via Eq. (13). The stabilization is guaranteed to finish within tG seconds, and it is donewithout violating the maximum commandable torque and without full knowledge of the inertia matrix. 

## VII. Simulation Results 

## A. Debris Stabilization 

The simulation of a spacecraft stabilizing a piece of space debris using the proposed stabilization technique is shown. The satellite– debris system begins with initial orientation σ0 ��0.0252; 0.0252; 0.0252� (10° of angular displacement), and the debris has an initial angular velocity of ωD ��−0.03; 0.03; −0.03� rad∕s�∼3°∕s�. The 200 kg spacecraft and 80 kg debris have inertia matrices of 

respectively, before attachment. Once attached, the satellite–debris system has an inertia matrix of 

The spacecraft has a maximum commandable torque of umax � 0.5 N ⋅ m. The debris stabilization mission has a pointing requirement of δ � 1 arcs. For calculating the controller quantities, the 20% inertia uncertainty in the principal inertias is assumed. The upper and lower estimates of the principal inertias are therefore JM � 266.1 kg ⋅ m[2] and Jm � 132.1 kg ⋅ m[2] . Algorithm 1 is used to find the controller quantities, which are summarized in Table 3. 

The final row in Table 3 shows the actual amount of time spent in each phase of the maneuver. That is, the time it takes to reach the desired state bounds. In the PD phase, the guaranteed time of flight tPD is roughly 60 times greater than the actual time of flight tend. The conservatism is expected due to the exponential structure of the PD Lyapunov function. The guaranteed time to reach the pointing requirement is tG � 18;638 s, and the actual time to reach the pointing requirement is ttotal � 350.0 s. Figure 4 shows the norm of the MRPs and angular velocity for the duration of the maneuver. The 

Table 3 Controller quantities 

|Proportional derivative|Hand-off|Finite time|
|---|---|---|
|kp �1.932||ϕ1 �19.53|
|kv �3.864||ϕ2 �54.37|
|jσPDj �2.024⋅10−3|jσHj �4.050⋅10−3||
|jωPDj �5.257⋅10−4|jσHj �1.052⋅10−3||
|tPD �1.848⋅104 s|tH �3.256 s|tf �151.0 s|
|tend �3.042⋅102 s||tend �45.76 s|

Fig. 4 Norm of MRPs and angular velocity. 

Fig. 5 Norm of control torque. 

hand-off phase is included as the end of the PD phase. The horizontal line on the left plot signals the pointing requirement of 1 arcs. The numerical integration continues past the moment of satisfying the pointing requirement to show the behavior of the finite-time controller as it approaches the final time. By extending the trajectory of the MRP in the PD phase, the maneuver would take more than three times longer if only a PD controller was used. The augmentation of the PD controller with the finite-time controller reaches the pointing requirement, and well below it, faster than the PD controller alone. Figure 5 shows the norm of the torque in both phases of the maneuver. The horizontal line in Fig. 5 represents the maximum commandable torque. At no point in the maneuver does the torque violate the maximum commandable torque and the torque signal remains continuous at all times. Note that the small spike in control torque at the end of the PD phase is due to the hand-off controller. The state bounds in the PD phase were carefully designed to guarantee that the abrupt increase in torque does to not violate the maximum commandable torque. The proposed method successfully stabilizes the piece of space debris within the guaranteed time of flight. 

## B. Examining Guaranteed Time Conservatism 

The stabilization method is guaranteed to finish within tG seconds. Having time guarantees to complete a maneuver is useful for maneuver scheduling, but over conservative time guarantees that are unrealistic are impractical for scheduling. The guaranteed time is a conservative overestimate due to the exponential structure of the PD Lyapunov function. In practice, the spacecraft reaches the pointing requirement well before the time guarantee. Simulations are carried out for two classes of spacecraft stabilizing debris to investigate the extent of the conservatism of the guaranteed time of flight. Figure 6 shows the average ratio of guaranteed time of flight to actual time ηt � tG∕tend for each initial maneuver angle. Time ratio ηt is averaged for 100 simulations for each initial maneuver angle. Each simulation has an initial debris angular velocity with magnitude jωDj � 0.05 �rad∕s� and a random direction and initial attitude with angles as in Fig. 6 and a random direction. Although not reported here, extensive simulations have been completed varying the pointing requirement to conclude that it has the little effect on ηt, changing it by less than 10% for pointing requirements δ ∈ �1; 50� arcs. In 

Fig. 6 Time ratio for 3U CubeSat with 40 kg debris (left) and a 100 kg microsat with 200 kg debris (right). 

general, the user can expect ηt ∈ �20; 150�, meaning that the maneuver will finish at least 20 times faster than guaranteed time tG. For the most part, the larger the maneuver angle, the larger ηt. Figure 6 shows that the more massive the spacecraft–debris system is, the larger ηt one can expect. The guaranteed time-of-flight conservatism must be considered when tG is used for scheduling. 

Due to the absence of negative definite jσj[2] terms in Eq. (A6), the term −min�kv; ϕ2�jωj[2] cannot contribute any net negative VH terms to the right-hand side of V[_] H and is dropped. Investigate relevant bounds for terms in V[_] H. 

## VIII. Conclusions 

This Note presents an attitude controller that stabilizes a spacecraft in a finite amount of time that only requires upper and lower estimates of the principal inertias. The combination of a PD controller into a finite-time controller is done while maintaining a continuous torque signal through the use of a hand-off controller. Each phase is well characterized by constructive bounds on the states and is guaranteed to never violate the maximum commandable torque. Time guarantees to complete each phase are precomputed in a computationally fast manner and only require the initial conditions of the maneuver and principal inertiaestimates. Themethod is particularly useful for space debris stabilization where the inertia of the system is not fully known and having time of flight guarantees is valuable. For situations where the system’s angular velocity is greater than 3 deg ∕s, the torque signal is not guaranteed to remain within the maximum commandable torque. The combination of a PD controller into a finite-time controller is strictly faster than a PD controller alone, and the time advantages of the proposed method increase as pointing requirements become finer. The provided time guarantees are overly conservative by at least a factor of 20 for all cases tested, but they are guaranteed none the less. 

## Appendix A: Hand-Off Controller 

Once the PD controller has reached the desired state bounds jσj ≤ jσPDj, jωj ≤ jωPDj, the hand-off controller is used to ensure continuity in the torque signal while transitioning to the finite-time phase. The hand-off control law is 

Replacing all state terms with their upper bounds in terms of VH results in an ordinary differential equation for VH: 

where α ��ϕ1 � 2kp�∕�4 ⋅ min�ϕ1; Jm∕2�� and β � 1∕�4 ⋅ min�ϕ1; Jm∕2�� and can be solved via separation of variables 

Notice that the storage function becomes singular VH�t� → ∞ as t → ln ��α � βVH�0��∕βVH�0��∕α. The bounds of the state variables in terms of the storage function are 

where V[�] H � VH�tH�. The state variables are guaranteed to remain bounded so long that the user chooses a hand-off time of flight tH that is less than the blowup time as in Eq. (A8). The smaller the hand-off time of flight, the smaller the upper bound of the state variables. The torque upper bound for the hand-off controller is 

where ϕ1, ϕ2 are the finite-time control gains. The controller is structured to reach the following initial and final conditions to ensure a continuous torque signal between the PD and finite-time phases as in Eq. (A2). 

Note that t � 0 and t � tH in Eq. (A2) signify the beginning and end of the hand-off phase, respectively, and not the timescale of the whole maneuver. The following storage function shows the boundedness of the state variables while using the hand-off controller: 

Arbitrary state bounds jσHj, jωHj can be achieved by prescribing small-enough values of jσPDj, jωPDj, tH. Achieving arbitrary state boundsmaximumguaranteescommandablethat thetorquehand-offu�H controller≤ umax. Theneverrequiredexceedsstatethe bounds jσPDj, jωPDj are also functions of the finite-time torque bound. 

## Appendix B: Finite-Time Proof 

The following is a proof that the controller in Eq. (19) stabilizes a spacecraft in finite time. Derivatives of the Lyapunov functions are as follows: 

which has the following upper and lower bounds: 

The derivative of the storage function is as follows: 

The sign indefinite σ[⊤] ω terms are maximized with the triangle inequality and separated using the fact 2ab ≤ a[2] � b[2] . 

Substitute the control law into V[_] 1; V[_] 2 and use b�σ�� 1 �jσj[2] : 

Substitute the control law into V[_] 3 and use b�σ�� 1 �jσj[2] and 2ab ≤ a[2] � b[2] : 

Group like terms of V[_] FT: 

The term −Jm∕2ϕ1μ[10] jσj[4] is always negative for positive values of the control gain and is removed from the right-hand side of Eq. (B4). Rewrite the V[_] FT as follows: 

with the following function definitions: 

The following equations must hold to ensure V[_] FT ≤ 0 for all time: 

The inequality in Eq. (B10) is redundant by the inequality in Eq. (B8), and it is removed. The right-hand side of Eqs. (B7) and (B8) is largest at initial time; therefore, the inequalities holding at the initial time guarantee that they will hold for the rest of time: 

The control gains ϕ1; ϕ2 and time of flight tf must be chosen to satisfy the conditions in Eq. (B11). Once satisfied, the following shows the boundedness of VFT. 

The solution to the differential equation in Eq. (B5) is bounded for t ∈ �0; tf�. Proof. Rewrite Eq. (B5) as 

where L � min�γ1�0�; γ2�0��. Both γ1�0� and γ2�0� areguaranteed to be positive by the conditions in Eq. (B11). Combining Eq. (B12) with Eq. (21) results in the ordinary differential equation: 

Therefore, by the comparison lemma [23], 

The Lyapunov function VFT is upper bounded by its initial value. Therefore, VFT ∈ L∞ for t ∈ �0; tf�. Rearranging Eq. (21) and substituting V[�] FT results in 

Equation (B15) is used to prove the convergence of jσj and jωj: 

Since limt→tf μ[−][a] → 0 ∀a > 0, Eq. (B16) shows that limt→tf jσj, jωj → 0. Equation (B16) not only shows the convergence of σ and ω but also provide an upper bound that is used to construct the upper bound of the control torque. Since both state variables are upper bounded by a constant, it must be true that jσj ∈ L∞ and jωj ∈ L∞, as well as b�σ�� 1 �jσj[2] ∈ L∞. Equation (B16) also shows that the terms that appear in the control laware also μ[4] jσj ∈ L∞, μ[2] jωj ∈ L∞. Since the product of two functions in L∞ are also in L∞, then the control law is also uFT � −b�σ��ϕ1μ[4] σ � ϕ2μ[2] ω� ∈ L∞. 

## Acknowledgment 

The authors from The University of Texas at Austin would like to thank Intuitive Machines, LLC, for supporting this research under Sponsored Research Agreement UTA2 1-000218 and the NASA Commercial Lunar Payload Services (CLPS) program. 

## References 

- [1] Nishida, S.-I., Kawamoto, S., Okawa, Y., Terui, F., and Kitamura, S., “Space Debris Removal System Using a Small Satellite,” Acta Astronautica, Vol. 65, Nos. 1–2, 2009, pp. 95–102. https://doi.org/10.1016/j.actaastro.2009.01.041 

- [2] Hakima, H., Bazzocchi, M. C., and Emami, M. R., “A Deorbiter CubeSat for Active Orbital Debris Removal,” Advances in Space Research, Vol. 61, No. 9, 2018, pp. 2377–2392. https://doi.org/10.1016/j.asr.2018.02.021 

- [3] Li, Z.-X., and Wang, B.-L., “Robust Attitude Tracking Control of Spacecraft in the Presence of Disturbances,” Journal of Guidance, Control, and Dynamics, Vol. 30, No. 4, 2007, pp. 1156–1159. https://doi.org/10.2514/1.26230 

- [4] Luo, W., Chu, Y.-C., and Ling, K.-V., “H-Infinity Inverse Optimal Attitude-Tracking Control of Rigid Spacecraft,” Journal of Guidance, Control, and Dynamics, Vol. 28, No. 3, 2005, pp. 481–494. https://doi.org/10.2514/1.6471 

- [5] Thakur, D., Srikant, S., and Akella, M. R., “Adaptive Attitude-Tracking Control of Spacecraft with Uncertain Time-Varying Inertia Parameters,” Journal of Guidance, Control, and Dynamics, Vol. 38, No. 1, 2015, pp. 41–52. https://doi.org/10.2514/1.G000457 

- [6] de Almeida, M. M., and Akella, M. R., “Time-Varying Feedback for Attitude Regulation in Prescribed Finite-Time,” 2019 AIAA/AAS Astrodynamics Specialist Conference, AAS/AIAA, Univelt, San Diego, CA, 2019, p. 3939. 

- [7] Krishnan, H., Reyhanoglu, M., and McClamroch, H., “Attitude Stabilization of a Rigid Spacecraft Using Two Control Torques: A Nonlinear Control Approach Based on the Spacecraft Attitude Dynamics,” Automatica, Vol. 30, No. 6, 1994, pp. 1023–1027. https://doi.org/10.1016/0005-1098(94)90196-1 

(B12) 

- [8] Crouch, P., “Spacecraft Attitude Control and Stabilization: Applications of Geometric Control Theory to Rigid Body Models,” IEEE Transactions on Automatic Control, Vol. 29, No. 4, 1984, pp. 321–331. https://doi.org/10.1109/TAC.1984.1103519 

- [9] Boskovic, J. D., Li, S.-M., and Mehra, R. K., “Globally Stable Adaptive Tracking Control Design for Spacecraft Under Input Saturation,” Proceedings of the 38th IEEE Conference on Decision and Control (Cat. No. 99CH36304), Vol. 2, Inst. of Electrical and Electronics Engineers, New York, 1999, pp. 1952–1957, https://doi.org/10.1109/CDC.1999.830922 

- [10] Boskovic, J., Li, S.-M., and Mehra, R., “Robust Stabilization of Spacecraft in the Presence of Control Input Saturation Using Sliding Mode Control,” Guidance, Navigation, and Control Conference and Exhibit, AIAA Paper 1999-4047, 1999. https://doi.org/10.2514/6.1999-4047 

- [11] Hu, J., Zhang, H., and Wang, Z., “Hybrid Adaptive Control of Spacecraft Attitude with Input Saturation and External Disturbance,” Journal of Guidance, Control, and Dynamics, Vol. 42, No. 3, 2019, pp. 642–649. https://doi.org/10.2514/1.G003090 

- [12] Wallsgrove, R. J., and Akella, M. R., “Globally Stabilizing Saturated Attitude Control in the Presence of Bounded Unknown Disturbances,” Journal of Guidance, Control, and Dynamics, Vol. 28, No. 5, 2005, pp. 957–963. 

https://doi.org/10.2514/1.9980 

- [13] Akella, M. R., Valdivia, A., and Kotamraju, G. R., “Velocity-Free Attitude Controllers Subject to Actuator Magnitude and Rate Saturations,” Journal of Guidance, Control, and Dynamics, Vol. 28, No. 4, 2005, pp. 659–666. 

https://doi.org/10.2514/1.8784 

- [14] Schaub, H., and Junkins, J. L., Analytical Mechanics of Space – 

- Systems, AIAA Educational Series, AIAA, Reston, VA, 2003, pp. 79 141, Chap. 3. 

https://doi.org/10.2514/5.9781600861550.0071.0126 

- [15] Zhu, Z., Xia, Y., and Fu, M., “Attitude Stabilization of Rigid Spacecraft with Finite-Time Convergence,” International Journal of Robust and Nonlinear Control, Vol. 21, No. 6, 2011, pp. 686–702. https://doi.org/10.1002/rnc.1624 

- [16] Guo, Y., Huang, B., Song, S.-M., Li, A.-J., and Wang, C.-Q., “Robust Saturated Finite-Time Attitude Control for Spacecraft Using Integral Sliding Mode,” Journal of Guidance, Control, and Dynamics, Vol. 42, No. 2, 2019, pp. 440–446. https://doi.org/10.2514/1.G003520 

- [17] Miller, A. J., and Akella, M. R., “Guidance Templates for Spacecraft Attitude Motion Planning,” 33rd AAS/AIAA Spaceflight Mechanics Meeting, American Astronautical Soc., Paper AAS 21-753, 2023. 

- [18] Vanderbei, R. J., “Case Studies in Trajectory Optimization: Trains, Planes, and Other Pastimes,” Optimization and Engineering, Vol. 2, June 2001, pp. 215–243. 

- [19] Miller, A. J., and Akella, M. R., “Reference Trajectories for Arbitrary Track-to-Track Attitude Maneuvers,” Proceedings of the AAS/AIAA Astrodynamics Specialist Conference, Vol. 177, Univelt, San Diego, CA, 2021, pp. 4043–4056. 

- [20] Arjun Ram, S., and Akella, M. R., “Uniform Exponential Stability Result for the Rigid-Body Attitude Tracking Control Problem,” Journal of Guidance, Control, and Dynamics, Vol. 43, No. 1, 2020, pp. 39–45. 

https://doi.org/10.2514/1.G004481 

- [21] Calafiore, G. C., and El Ghaoui, L., Optimization Models, Cambridge Univ. Press, Cambridge, England, U.K., 2014, pp. 264–265, https://doi.org/10.1017/cbo9781107279667 

- [22] Nickalls, R. W., “A New Approach to Solving the Cubic: Cardan’s Solution Revealed,” Mathematical Gazette, Vol. 77, No. 480, 1993, pp. 354–359. https://doi.org/10.2307/3619777 

- [23] Khalil, H. K., Control of Nonlinear Systems, 3rd ed., Prentic-Hall, – 

- Upper Saddle River, NJ, 2002, pp. 144 161, Chap. 4. 
