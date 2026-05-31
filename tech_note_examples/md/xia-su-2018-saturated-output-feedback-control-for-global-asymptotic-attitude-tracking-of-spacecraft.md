## Engineering Notes 

## Saturated Output Feedback Control for Global Asymptotic Attitude Tracking of Spacecraft 

Yuquan Xia[∗] and Yuxin Su[†] Xidian University, 710071 Xi’an, People’s Republic of China 

DOI: 10.2514/1.G003566 

## I. Introduction 

T HE attitude control problem of rigid spacecraft has gained muchattention during the last few decades. Several control methods for rigid spacecraft can be found in the literature. For example, Wen and Kreutz-Delgado [1] proposed several control laws for the attitude tracking of spacecraft using unit-quaternion representation. Based on the passive theory, Egeland and Godhavn [2] developed an adaptive control scheme for the attitude tracking control of spacecraft. Using a nonlinear filter, Lizarralde and Wen [3] presented a velocity-free controller, which exploited the inherent passivity of the system. Yoon and Agrawal [4] developed an adaptive tracking control law for Hamiltonian multi-input/multi-output systems and then applied it to attitude tracking of the spacecraft. Using unit-quaternion feedback and a backstepping technique, Kristiansen et al. [5] proposed a tracking controller to stabilize the attitude of a micro satellite. Various optimal control techniques and H-infinity controls were proposed in – – [6 8]. In [9 12], various sliding mode controls were developed for better robustness to uncertainty dynamics and external disturbances. 

Although the aforementioned works had so many advantages, they all relied on the assumption that the actuators could supply any requested torque for the control of spacecraft. As we all know, this is unrealistic. Taking the actuator constraints into account, several saturated controllers have been proposed. More specifically, in [13], considering the actuator saturation constraints, a general design methodology was proposed for a Lyapunov optimal saturated control law and applied to the motion control of rigid spacecraft. By using the variable structure control design method, Boškovic et al. [14] formulated two control algorithms for the robust stabilization of spacecraft subject to input saturation, uncertainty dynamics, and external disturbances. Considering the attitude tracking control problem, an adaptive method combined with a sliding mode technique were employed in [15]. In 2010, Ali et al. [16] presented a bounded control design method for spacecraft attitude tracking with a backstepping technique. 

It is worth mentioning that the aforementioned saturated control results assumed that full state measurements were available in the controller formulation. However, the angular velocity sensor costs too much and the measurements are sensitive to noise. So, it is necessary to develop an output feedback control without velocity 

measurements [17]. In particular, based on the attitude tracking controller proposed in [1], an angular velocity observer was constructed and an output feedback attitude tracking controller was developed by Schlanbusch et al. [17]. Akella et al. [18] presented a smooth angular velocity observer; then, based on the separation property, a proportional-derivative (PD)-type attitude controller combined with the proposed observer was employed to achieve the attitude tracking control. Tayebi [19] proposed a quaternion-based dynamic output feedback control for the attitude tracking problem of a rigidbodywithoutvelocitymeasurements.Bygeometrichomogeneity and the Lyapunov theory, Zou et al. [20] developed a nonlinear semiglobal observer-based finite-time stable output feedback tracking control. In [21], based on the Chebyshev neural networks, two robust adaptive output feedback controllers were constructed. The weakness of the aforementioned output feedback controls for attitude tracking of spacecraft was that the control design did not incorporate input constraints. Using a linear filter, an output feedback controller was proposed by Akella et al. [22]. By adoption of the property of the unitquaternion representation and the hyperbolic tangent function, the actuator magnitude and rate saturations were explicitly considered. Gui and Vukovich [23] developed a finite-time angular velocity observer; then, based on the saturated finite-time full-state feedback controller, a saturated finite-time output feedback controller was obtained. 

As pointed out in [24], dueto the topological obstruction, therewas no continuous control law that could achieve the attitude control objective globally without the unwinding phenomenon, and only almost “global” stability was obtained [18,19,22,23]. To solve this problem, hysteresis-based hybrid controllers were proposed and applied to the attitude control of spacecraft [25,26]. In particular, based ontheLyapunovfunctionandabacksteppingtechnique,Mayhewetal. [25] proposed two quaternion-based hybrid controls for the global stabilization of spacecraft. The result was extended in [26] to full-state feedback, output feedback, and velocity measurements with constant bias. In [27], a hysteretic hybrid proportional-integral-derivative (PID) plus spacecraft dynamics controller was presented that could ensure the global stability of the set consisting of the two equilibria. In the work of Lee [28], a smooth controller was first constructed for almost semiglobal exponential stability; then, a hybrid control technique was used to achieve global exponential stability. It was further extended to obtain robustness with respect to a fixed disturbance by including an integral term. To remove the velocity measurements, a hybrid observer-based PD plus spacecraft dynamics (PD�) output feedback tracking controller was developed by Schlanbusch et al. [29]. Using a homogeneous finite-time control technique and a hybrid control method, three global finite-time attitude tracking control laws were designed in [30]. 

This Note investigates the global attitude tracking control of rigid spacecraft subject to actuator constraints and attitude measurements only.AsaturatedhybridoutputfeedbackPD-plus-spacecraftdynamics (SHOPD�) controller is proposed. By using a linear filter driven by the attitude trackingerror,therequirementoftheangular velocityinthe control law formulation is removed. Based on the property of the attitude parameters, the actual control torques can be explicitly upper bounded a priori. Using the hybrid control method, the unwinding phenomenonisavoidedand theglobal asymptotic stabilityisobtained. Compared with the results in [17,29], the proposed control has the advantage that it has an explicit upper bound, and hence it can completely avoid actuator saturation by selecting the control gains a priori such that the degraded performance and unpredicted motion caused by the actuator saturation can be removed. Different from [17,29], the proposed control exploits a model-independent filterbased output feedback control for the attitude tracking of spacecraft; whereas in [17,29], the model-based observer was adopted. In 

addition, the proposed control removes the unwinding phenomenon occurred in [17]. In comparison with [30], the proposed SHOPD� control offers an easygoing filter-based output feedback control for spacecraft instead of designing a hybrid-based observer, as in [30]. Hence, the proposed control presents a simple solution for global asymptotic tracking of spacecraft with actuator constraints and attitude measurements only. Simulations are performed to illustrate the effectiveness and improved performance of the proposed method. 

This Note is organized as follows. In Sec. II, the preliminaries are given in terms of the unit-quaternion and the problem statement is formulated. Section III presents the controller design and global stability analysis. Simulations are presented in Sec. IV. Finally, the Note is closed with some conclusions in Sec. V. 

Throughout this Note, the following notations are used: λmax�A� denotes the maximum eigenvalue of a matrix A, and kxk indicates the Euclidean norm of vector x. The norm of matrix A is defined askAk �the pcorresponding����������������������λmax�A[T] A�. I3inducedrepresentsnorm,thewhichidentitycanmatrixbe writtenof threeas dimension, and sgn�⋅� denotes the standard signum function. 

The relative attitude tracking error kinematics and dynamics are given by [7] 

where ωe ∈ R[3] denotes the angular velocity tracking error, which is derived by ωe � ω − Cωd, with C ∈ R[3][×][3] as the relative rotation matrix between the body-fixed frame and the desired frame and is given as follows [7]: 

## II. Problem Statement 

In this Note, we use the unit-quaternion representation to describe the attitude orientation of the spacecraft due to its globally nonsingular characteristic. With the identity element �0; 0; 0; 1�[T] , a unit-quaternion is defined as q ∈ S[3] �fx ∈ R[4] : x[T] x � 1g, where S[3] denotes a three-dimensional sphere embedded in R[4] . As in [7], the kinematic equations are given by 

where q ��qv[T] ; q4�[T] ∈ S[3] , with qv ��q1; q2; q3�[T] ∈ R[3] denoting the unit-quaternion vector subjected to norm constraints qv[T] qv � q[2] 4[�][1][;][ω][ ∈][R][3][is][the][angular][velocity][of][the][body-fixed] frame relative to the inertia frame and expressed in the body-fixed frame; and a[×] denotes a skew-symmetric matrix corresponding to a vector a ∈ R[3] , which is given as 

The dynamic equation of the spacecraft is as follows [7]: 

where J ∈ R[3][×][3] is the constant symmetric positive-definite inertia matrix, and u ∈ R[3] denotes the actuator input. 

As in [7,9], the desired attitude motion is generated by 

where qd ��q[T] dv[; q][d][4][�][T][∈][S][3][,][with][q][dv][��][q][d][1][; q][d][2][; q][d][3][�][T][∈][R][3][as] the unit-quaternionvector denoting the desired attitude; and ωd ∈ R[3] represents the target angular velocity expressed in the desired frame. Note that the desired attitude satisfies kqdk � 1. 

The relative attitude tracking error is denoted by qe ��e[T] v ; e4�[T] ∈ S[3] with ev ��e1; e2; e3�[T] ∈ R[3] and kqek � 1, which is derived by unit-quaternion multiplication and can be expressed as [7,17] 

where q[−] d[1][��][−][q][T] dv[; q][d][4][�][T][denotes the inverse of][ q][d][and, for any two] unit-quaternion qa ��qav[T] ; qa4�[T] and qb ��q[T] bv[; q][b][4][�][T][,][the][unit-] quaternion multiplication ⊗ is defined as 

The proposed control in this Note is based on the following assumptions and properties: 

Assumption 1 [15]: The inertia matrix satisfies kJk ≤ JM, where JM is a known positive constant. 

Assumption 2 [15]: The desired angular velocity ωd and its first time derivative ω_ d are bounded, i.e., there exist two known constants c1, c2 > 0 such that kωdk ≤ c1 and kω _ dk ≤ c2. 

Property 1 [7]: For any a, b ∈ R[3] , the skew-symmetric matrices a[×] and b[×] have the following properties: a) a[×] b[×] � ba[T] − a[T] bI3, b) a[×] b � −b[×] a, and c) ka[×] k �kak. 

Property 2 [7]: The relative rotation matrix C satisfies kCk � 1 and C[_] � −ω[×] e C. 

Property 3 [9]: Define 

Then, the matrix E�qe� satisfies E�qe�[T] E�qe�� I3. As a consequence, premultiplying the equation q_e ��1∕2�E�qe�ωe by E�qe�[T] yields 

Property 4 [9]: The matrix e4I3 � e[×] v satisfies k�e4I3 � e[×] v �k � 1. Property 5 [7]: The matrix �Cωd�[×] J � J�Cωd�[×] is skew symmetric. 

We assume that each actuator has a known maximum torque ui;max satisfying 

The objective of this Note is to design a saturated nonlinear controller subject to attitude measurements only and the following actuator constraints 

such that 

is globally asymptotically stable and the unwinding phenomenon is avoided completely, where ui denotes the actual control torque of the ith actuator. 

## III. Control Design 

## A. Control Formulation 

To avoid the unwinding phenomenon, the hysteresis-based hybrid control method is adopted here. Following the framework of [26], a hybrid system is first denoted as 

After simplifying Eq. (20), we have 

where F: R[n] → R[n] denotes the flow map with the flow set D, G: R[n] → R[n] denotes the jump map with the jump set E, and x[�] is the state value immediately after a jump. 

Now, we propose the following hysteresis-based saturated hybrid output feedback PD� controller for global asymptotic attitude tracking control of spacecraft with attitude measurements only: 

where k1, k2 ∈ R denote the constant positive control gains, A, B ∈ R[3][×][3] are constant diagonal positive-define filter matrices, qc ∈ R[3] is an auxiliary variable vector, v ��v1; v2; v3�[T] ∈ R[3] denotes the filter variable, S�v� ∈ R[3] is a vector saturation function defined by 

and h ∈ H �f−1; 1g is an auxiliary variable satisfying h[�] � −h. The continuous set D and the jump set E are defined, respectively, as follows [26]: 

where O � S[3] × R[3] × H, and η ∈ �0; 1� denotes the hysteresis gap to be designed. 

Remark 1: The auxiliary variable h works like the signum function sgn�e4� [25,26]. According to theorem 2.1 in [25] and theorem 3.2 in [26], the controller [Eq. (12)] with h replaced by the signum function sgn�e4� is sensitive to the measurement noise; thus, the hysteresis width h is introduced for managing a tradeoff between robustness to measurement noise and a small amount of hysteresis-induced inefficiency. 

Based on the fact that kqek � 1, we have jeij ≤ 1, i � 1; 2; 3. Taking Property 4 and Assumptions 1 and 2 into account, the control torque given by Eq. (12) can be upper bounded by 

It is clear that the actuator constraints can be satisfied by selecting the control gains a priori: 

Substituting the controller [Eq. (12)] into Eq. (8) yields 

Note that, when x ∈ D, all the states are continuous and h remains unchanged such that h[_] � 0. When x ∈ E, the jump only occurs with the variable h, and the other system states are still continuous. Hence, the hybrid closed-loop system dynamics are 

## B. Stability Analysis 

The main result of this Note is given in Theorem 1. Theorem 1: Consideringthespacecraftsystem givenby Eqs.(7)and (8) subject to attitude measurements only and actuator constraints [Eq. (10)], the proposed SHOPD� controller defined by Eqs. (12) and (13) ensures global asymptotic stability of the set 

Proof: The proof is divided into two steps. First, we prove that W[�] is stable; then, by LaSalle’s invariance principle for the hybrid system, the global attractivity of W[�] is obtained; thus, the global stability of W[�] can be achieved. 

Step 1: Consider the following Lyapunov function candidate: 

where 

and bi denotes the ith diagonal element of matrix B. 

Define x ��hqe; ωe; v�[T] . Note that, based on the definitions of the inertia matrix and the function S�v�, V � 0 when 

and V > 0 for all 

Upon applying the fact that ω � ωe � Cωd, Eq. (19) can be rewritten as 

Thus, the proposed Lyapunov function candidate V is positive definite and radially unbounded with respect to ωe, hqe, and v. By virtue of the facts that e[T] v ev � e[2] 4[�][1][ and][ h][2][�][1][, Eq. (24) can] be simplified as 

Taking the first time derivative of Eq. (25), we have 

When x ∈ D, substituting the closed-loop system trajectories [Eq. (22)] into Eq. (26) yields 

In light of Property 1, we have ω[T] e ω[×] e � 0 and ω[×] e Cωd � −�Cωd�[×] ωe; thus, 

## IV. Simulations 

In this section, we present several comparisons to illustrate the effectiveness and improved performance of the proposed control. 

Comparisons with the quaternion-based output feedback PD� (OPD�) control of [17] are first performed. The control law of OPD� is 

where ki, i � 1; 2; 3; kp, kd, lp, and ld are positive design constants; eq ��e[T] v ; 1 − e4�[T] with �e[T] v ; e4�[T] � q[−] d[1][⊗][q][being][the][attitude] tracking error; eeq ��e[T] ev; 1 − ee4�[T] with �e[T] ev; ee4�[T] � q[−] s[1] ⊗ q being the attitude estimation error; Te � 12 � e 4I3e �[T] v e×v � 

and 

where we have used Property 5 and the first equation of Eq. (7). After simplifying the preceding equation, we have 

When x ∈ E, the change in V over jump is 

In light of Eq. (16), we have 

By using the definition of S�v�, we conclude that V is monotonically nonincreasing along a flow set of the closed-loop system and strictly decreasing over jumps. Applying theorem 7.6 from [31], we have the fact that set W[�] is stable. 

Step 2: Because fx ∈ E: V�x[�] � − V�x�� 0g is an empty set, in theorem 4.7 from [31], we can claim that the system states converge to the largest invariant set 

From V[_] � 0, we have v � 0 for all t ≥ 0, and then e_v � 0 from Eq. (13); thus, e_4 � 0. By Eq. (9), we have 

for all t ≥ 0. It implies that ω_ e � 0 for all t ≥ 0. Moreover, from Eq. (21), we have hev � 0. By virtue of h ∈ H �f−1; 1g ≠ 0, hev � 0 implies that ev � 0. In light of he4 ≥−η, we have hqe ��0; 0; 0; 1�[T] . Based on this fact, 

is globally attractive. 

In light of Steps 1 and 2, the claimed result shown in Theorem 1 directly follows. 

Also, qs ��qsv[T] ; qs4�[T] ∈ S[3] with qsv ∈ R[3] denotes the estimated attitude of spacecraft, R[b] i[��][R][i] b[�][T][with][ R][i] b[�][I][3][�][2][q][4][q][×][v][�][2][�][q][×][v][�][2] denotes the rotation matrix from the body-fixed frame to the inertia frame [26], z ∈ R[3] is the auxiliary variable, ω[b] i;e[∈][R][3][denotes][the] estimated angular velocity expressed in the body-fixed frame, and ω[i] i;d[∈][R][3][ is the desired angular velocityexpressed inthe inertiaframe.] The inertia matrix and initial conditions are the same as in [17], i.e., 

qd�0���0; 0; 0; 1�[T] , ω�0���0.1; 0.2; −0.3�[T] �rad∕s�, and z�0�� �0; 0; 0�[T] . The desired angular velocity is 

The sampling period is T � 1�ms�. With the desired angular velocity expressed in the inertia frame ω[i] i;d[,][the][ω][d][of][the][proposed] controller [Eq. (12)] is calculated as �R[i] d[�][T][ω][i] i;d[with][R][i] d[�][I][3][�] 2qd4q[×] dv[�][2][�][q][×] dv[�][2][[26].] 

The gains of the proposed SHOPD� controller, Eqs. (12) and (13) are selected by trial and error until a good attitude tracking is obtained. They are k1 � 10, k2 � 14, A � B � diag�3; 3; 3�, and η � 0.4. The auxiliary variable h is initialized as h � 1. The gains of the OPD� controllergivenby Eqs. (32) and(33) are selected to be the same as in [17]; and they are kp � 10, kd � 7, lp � 100, ld � 75, and k1 � k2 � k3 � 1. The attitude and angular velocity tracking errors are plotted in Figs. 1 and 2. The requested control torques are shown in Fig. 3. As we see, both controllers gain the control objective and the proposed SHOPD� controller obtains a faster transient and better performance over the controller proposed in [17]. Note that the favorable result of the proposed SHOPD� control is achieved without larger control torques. 

After that, comparisons taking into account the inertia matrix uncertainties and measurement noise are also conducted out. The inertia matrix uncertainties are set as ΔJ � diag�0.3; 0.3; 0.3��kg ⋅ m[2] �. Similar to [26], the measurement noise is introduced to the actual attitudeq� ��q �ofme�the�∕kqspacecraft, � me�k, wherei.e., e�the � e∕measuredkek, each valueelementofofqe isis 

drawn from a zero-mean Gaussian distribution with unit variance, and m is drawn from a uniform distribution on the interval [0, 0.01]. The initial conditions and the control gains of both controls are kept unchanged. The attitude and angular velocity tracking errors are illustrated in Figs. 4 and 5. Figure 6 shows the requested control torques. It is clear that the proposed SHOPD� controller also achieves a better performance over the OPD� controller of [17], even in the presence of inertia matrix uncertainties and measurement noise. 

Furthermore, comparison with the saturated hybrid output feedback finite-time PD� (SHOFT�) controller proposed in [30] is conducted out. The reasoning behind this comparison is that both controllers are explicitly upper bounded a priori and they have the ability to ensure global stability and actuator constraints are not breached. The SHOFT� controller in [30] was given as 

Fig. 3 Input torque comparison with [17]. 

Fig. 4 Attitude tracking error comparison with [17] with inertia uncertainty and measurement noise. 

u � −k1κ1�hqe;1 − α1� − k2κ1�h[~] ~q;1 − α1���Cωd�[×] JCωd � JC _ωd (34) 

Fig. 5 Velocity tracking error comparison with [17] with inertia uncertainty and measurement noise. 

where k1, k2, and k3 are positive design constants; α1 � 2α2 − 1; α2 ∈ �0.5; 1�; qed ��e[T] edv[; e][ed][4][�][T][∈][S][3][is][the][estimated][attitude] tracking error; q~ ��e~[T] v ; e~4�[T] ∈ S[3] is an auxiliary variable; R�q~� ∈ R[3][×][3] denotes the rotation matrix corresponding to the unit-quaternion q~; qe ��e[T] v ; e4�[T] ∈ S[3] ; ωd ∈ R[3] and C ∈ R[3][×][3] are defined the same as that in the proposed controller [Eq. (12)]; and κ1�q; α� is a vector function defined as 

The inertia matrix uncertainties and the measurement noise are the same as the previous comparison. The other simulation conditions are the same as those in [30]. The inertia matrix is J � diag�15; 20; 10��kg ⋅ m[2] �. The initial conditions are 

qd�0���0; 0; 0; 1�[T] , and ω�0���0.3; −0.4; 0�[T] �rad∕s�. The desired angular velocity is given as 

The external disturbance is 

The maximum torques are assumed to be umax ��5; 5; 5�[T] N ⋅ m. The sampling period is T � 1 �ms�. 

Based on the system parameter, the inertia matrix uncertainties, and the desired trajectory, the upper bounds defined by Assumptions��� 1 and 2 can be determined as follows:��� JM � 20.3, c1 � p3 × 10[−][2] , and c2 � p3 × 10[−][4] . Hence, according to Eq. (18), the constraint on the control gains is 

Thus, the gains of the proposed SHOPD� controller are selected as k1 � 1.5, k2 � 3.4, A � diag�1; 1; 1�, B � diag�4; 4; 4�, and η � 0.2. Also, h is initialized as h � 1. The gains of the SHOFT� controller are selected to be the same as those in [30]; and they are k1 � 1.2, k2 � 2.4, k3 � 1.1, α2 � 0.75, δ � 0.3, and h�0�� h[~] �0�� 1. The attitude tracking errors, angular velocity tracking errors, and the controller torques are illustrated in Figs. 7–9. It is clear that both controllers accomplish their desired objective within the allowable torques. Meanwhile, the proposed SHOPD� controller gains a faster transient over the SHOFT� controller. 

Fig. 7 Attitude tracking error comparison with [30] with inertia uncertainty and measurement noise. 

Fig. 8 Velocity tracking error comparison with [30] with inertia uncertainty and measurement noise. 

Fig. 9 Input torque comparison with [30] with inertia uncertainty and measurement noise. 

## V. Conclusions 

A saturated hybrid output feedback control is proposed for global asymptotic tracking of spacecraft subject to actuator constraints and attitude measurements only. By a linear filter driven by the attitude trackingerror,theneedofangularvelocityinthecontrollerformulation is removed. The proposed control has an explicit upper bound, and hence the degraded performance of the actuator or unpredictable motion of the spacecraft caused by the actuator saturation can be completelyremovedbyselecting the controlgainsa priori.Lyapunov’s direct method is employed to prove the global asymptotic stability. Advantages of the proposed control include the ability to ensure global asymptotic tracking without actuator saturation and velocity measurements. Simulations verify the effectiveness and the improved performance of the proposed controller. 

## Acknowledgments 

The authors would like to thank Associate Editor Maruthi Akella and the anonymous reviewers for their valuable comments. 

## References 

- [1] Wen, J. T. Y., and Kreutz-Delgado, K., “The Attitude Control Problem,” IEEE Transactions on Automatic Control, Vol. 36, No. 10, 1991, pp. 1148–1162. 

   - doi:10.1109/9.90228 

- [2] Egeland, O., and Godhavn, J. M., “Passivity-Based Adaptive Attitude Control of a Rigid Spacecraft,” IEEE Transactions on Automatic Control, Vol. 39, No. 4, 1994, pp. 842–846. doi:10.1109/9.286266 

- [3] Lizarralde, F., and Wen, J. T., “Attitude Control without Angular Velocity Measurement: A Passivity Approach,” IEEE Transactions on Automatic Control, Vol. 41, No. 3, 1996, pp. 468–472. doi:10.1109/9.486654 

- [4] Yoon, H., and Agrawal, B. N., “Adaptive Control of Uncertain Hamiltonian Multi-Input Multi-Output Systems: With Application to Spacecraft Control,” IEEE Transactions on Control Systems and Technology, Vol. 17, No. 4, 2009, pp. 900–906. doi:10.1109/TCST.2008.2011888 

- [5] Kristiansen, R., Nicklasson, P. J., and Gravdahl, J. T., “Satellite Attitude Control by Quaternion-Based Backstepping,” IEEE Transactions on Control Systems and Technology, Vol. 17, No. 1, 2009, pp. 227–232. doi:10.1109/TCST.2008.924576 

- [6] Krstic, M., and Tsiotras, P., “Inverse Optimal Stabilization of a Rigid Spacecraft,” IEEE Transactions on Automatic Control, Vol. 44, No. 5, 1999, pp. 1042–1049. doi:10.1109/9.763225 

- [7] Luo, W., Chu, Y., and Ling, K. V., “Inverse Optimal Adaptive Control for Attitude Tracking of Spacecraft,” IEEE Transactions on Automatic Control, Vol. 50, No. 11, 2005, pp. 1639–1654. doi:10.1109/TAC.2005.858694 

- [8] Kang, W., “Nonlinear H∞ Control and Its Application to Rigid Spacecraft,” IEEE Transactions on Automatic Control, Vol. 40, No. 7, 1995, pp. 1281–1285. doi:10.1109/9.400476 

- [9] Lo, S., and Chen, Y., “Smooth Sliding-Mode Control for Spacecraft Attitude Tracking Maneuvers,” Journal of Guidance, Control, and Dynamics, Vol. 18, No. 6, 1995, pp. 1345–1349. doi:10.2514/3.21551 

- [10] Jin, E., and Sun, Z., “Robust Controllers Design with Finite Time Convergence for Rigid Spacecraft Attitude Tracking Control,” Aerospace Science and Technology, Vol. 12, No. 4, 2008, pp. 324–330. doi:10.1016/j.ast.2007.08.001 

- [11] Li, S., Wang, Z., and Fei, S., “Comments on the Paper: Robust Controllers Design with Finite Time Convergence for Rigid Spacecraft Attitude Tracking Control,” Aerospace Science and Technology, Vol. 15, No. 3, 2011, pp. 193–195. doi:10.1016/j.ast.2010.11.005 

- [12] Zou, A., Kumar, K. D., Hou, Z., and Liu, X., “Finite-Time Attitude Tracking Control for Spacecraft Using Terminal Sliding Mode and Chebyshev Neural Network,” IEEE Transactions on Systems, Man, and Cybernetics—Part B: Cybernetics, Vol. 41, No. 4, 2011, pp. 950–963. doi:10.1109/TSMCB.2010.2101592 

- [13] Robinett, R. D., Parker, G. G., Schaub, H., and Junkins, J. L., “Lyapunov Optimal Saturated Control for Nonlinear Systems,” Journal of Guidance, Control, and Dynamics, Vol. 20, No. 6, 1997, pp. 1083–1088. doi:10.2514/2.4189 

- [14] Boškovic, J. D., Li, S., and Mehra, R. K., “Robust Adaptive Variable Structure Control of Spacecraft Under Control Input Saturation,” Journal of Guidance, Control, and Dynamics, Vol. 24, No. 1, 2001, pp. 14–22. 

   - doi:10.2514/2.4704 

- [15] Boskovic, J. D., Li, S., and Mehra, R. K., “Robust Tracking Control Design for Spacecraft Under Control Input Saturation,” Journal of Guidance, Control, and Dynamics, Vol. 27, No. 4, 2004, pp. 627–633. doi:10.2514/1.1059 

- [16] Ali, I., Radice, G., and Kim, J., “Backstepping Control Design with Actuator Torque Bound for Spacecraft Attitude Maneuver,” Journal of Guidance, Control, and Dynamics, Vol. 33, No. 1, 2010, pp. 254–259. doi:10.2514/1.45541 

- [17] Schlanbusch, R., Loria, A., Kristiansen, R., and Nicklasson, P. J., “PD� Based Output Feedback Attitude Control of Rigid Bodies,” IEEE TransactionsonAutomaticControl,Vol. 57,No. 8,2012,pp.2146–2152. doi:10.1109/TAC.2012.2183189 

- [18] Akella, M. R., Thakur, D., and Mazenc, F., “Partial Lyapunov Strictification: Smooth Angular Velocity Observers for Attitude Tracking Control,” Journal of Guidance, Control, and Dynamics, Vol. 38, No. 3, 2014, pp. 442–451. doi:10.2514/1.G000779 

- [19] Tayebi, A., “Unit Quaternion-Based Output Feedback for the Attitude Tracking Problem,” IEEE Transactions on Automatic Control, Vol. 53, No. 6, 2008, pp. 1516–1520. doi:10.1109/TAC.2008.927789 

- [20] Zou, A., “Finite-Time Output Feedback Attitude Tracking Control for Rigid Spacecraft,” IEEE Transactions on Control Systems and Technology, Vol. 22, No. 1, 2014, pp. 338–345. doi:10.1109/TCST.2013.2246836 

- [21] Zou, A., Kumar, K. D., and Hou, Z., “Quaternion-Based Adaptive Output Feedback Attitude Control of Spacecraft Using Chebyshev Neural Networks,” IEEE Transactions on Neural Networks, Vol. 21, No. 9, 2010, pp. 1457–1471. doi:10.1109/TNN.2010.2050333 

- [22] Akella, M. R., Valdivia, A., and Kotamraju, G. R., “Velocity-Free Attitude Controllers Subject to Actuator Magnitude and Rate Saturations,” Journal of Guidance, Control, and Dynamics, Vol. 28, No. 4, 2005, pp. 659–666. 

   - doi:10.2514/1.8784 

- [23] Gui, H., and Vukovich, G., “Finite-Time Angular Velocity Observers for Rigid-Body Attitude Tracking with Bounded Inputs,” International Journal of Robust and Nonlinear Control, Vol. 27, No. 1, 2017, pp. 15–38. 

doi:10.1002/rnc.v27.1 

- [24] Bhat, S. P., and Bernstein, D. S., “A Topological Obstruction to Continuous Global Stabilization of Rotational Motion and the Unwinding Phenomenon,” Systems and Control Letters, Vol. 39, No. 1, 2000, pp. 63–70. 

doi:10.1016/S0167-6911(99)00090-0 

- [25] Mayhew, C. G., Sanfelice, R. G., and Teel, A. R., “Robust Global AsymptoticAttitude Stabilization of a RigidBody by Quaternion-Based ” 

- Hybrid Feedback, Proceedings of the 48th IEEE Conference on IEEE, IEEE Publ., Piscataway, NJ, 2009, pp. 2522–2527. doi:10.1109/cdc.2009.5400431 

- [26] Mayhew, C. G., Sanfelice, R. G., and Teel, A. R., “Quaternion-Based Hybrid Control for Robust Global Attitude Tracking,” IEEE Transactions on Automatic Control, Vol. 56, No. 11, 2011, pp. 2555–2566. 

   - doi:10.1109/TAC.2011.2108490 

- [27] Su, J., and Cai, K., “Globally Stabilizing Proportional-IntegralDerivative Control Laws for Rigid-Body Attitude Tracking,” Journal of Guidance, Control, and Dynamics, Vol. 34, No. 4, 2011, pp. 1260–1264. doi:10.2514/1.52301 

- [28] Lee, T., “Global Exponential Attitude Tracking Controls on SO(3),” IEEE Transactions on Automatic Control, Vol. 60, No. 10, 2015, pp. 2837–2842. 

doi:10.1109/TAC.2015.2407452 

- [29] Schlanbusch, R., Grotli, E. I., Loria, A., and Nicklasson, P. J., “Hybrid Attitude Tracking of Rigid Bodies Without Angular Velocity Measurement,” Systems and Control Letters, Vol. 61, No. 4, 2012, pp. 595–601. 

   - doi:10.1016/j.sysconle.2012.01.008 

- [30] Gui, H., and Vukovich, G., “Global Finite-Time Attitude Tracking via QuaternionFeedback,” SystemsandControlLetters,Vol.97,Nov.2016, pp. 176–183. 

   - doi:10.1016/j.sysconle.2016.09.017 

- [31] Sanfelice, R. G., Goebel, R., and Teel, A. R., “Invariance Principles for Hybrid Systems with Connections to Detectability and Asymptotic Stability,” IEEE Transactions on Automatic Control, Vol. 52, No. 12, 2007, pp. 2282–2297. 

   - doi:10.1109/TAC.2007.910684 
