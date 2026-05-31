## Engineering Notes 

## Potential Field-Based Sliding Surface Design and Its Application in Spacecraft Constrained Reorientation 

Juntang Yang,[∗] Yisheng Duan,[†] Mohamed Khalil Ben-Larbi,[‡] and Enrico Stoll[§] Technical University of Braunschweig, 38108 Braunschweig, Germany 

https://doi.org/10.2514/1.G005026 

## I. Introduction 

D URINGstraints onattitudethe pointingmaneuvers,of payloadsthere areonboardusuallythespecificspacecraft,conincluding attitude forbidden zones and attitude mandatory zones. These attitude-constrained zones are usually defined as cones around specific unit vector directions in space. One typical example of attitude forbidden zones is that an optical-sensitive instrument (e.g., a telescope) is not allowed to point toward bright objects in space in order to avoid damage. One example of attitude mandatory zones is that the antenna of a spacecraft is required to keep pointing to the direction of the ground station for communication purposes during reorientation maneuvers. 

Two main frameworks have been proposed to address the challenging problem of spacecraft constrained reorientation. Framework I is attitude planning and tracking, within which attitude constraints are considered by attitude planning and attitude controllers are implemented to track the designed rotational trajectory. Different attitude planning methods have been developed, such as the constraint monitor algorithm [1], the geometric method [2,3], the randomized exploring method [4], the discretization method [5,6], the recursive planning method based on rotational-path decomposition [7], the attitude planning method using gradient-based optimization [8], and the search-based planning in the rotation space SO(3) [9]. Though widely studied, attitude planning methods can be computationally expensive for the real-time onboard implementation. Framework II is to consider attitude constraints in the controller design, in which the most explored method is to combine artificial potential field (APF) with existing control methods. These APF-based methods include APF-based proportional and derivative (PD) controllers [10–12], APF-based backstepping controllers [13,14], APF-based sliding mode controllers [15], and APF-based kinematic steering laws [16]. APF-based methods are suitable for real-time onboard implementation because they are analytical. The inherent drawback 

of APF-based methods is the existence of local minima at which attractive and repulsive torques are balanced. The attitude of the spacecraft may be trapped at local minima rather than converging to the global minimum (i.e., the desired attitude). Only a limited number of works (e.g., [11,16]) have considered the problem of local minima. 

The combination of sliding mode control (SMC) with APF was first proposed by Uktin et al. [17] and Guldner and Utkin [18] for collision avoidance of robots. The basic idea is to track the gradient of an APF using the SMC. Once the sliding mode is reached, the state of a robot will converge to the desired one and satisfy collision avoidance at the same time. The strategy of combining SMC with APF was explored by Shen et al. [15] for spacecraft reorientation with pointing constraints. However, compared with the strategy proposed by Uktin et al. [17] and Guldner and Utkin [18], the way of combining SMC with APF in [15] is different. To be specific, the sliding mode in [15] is the classical one for attitude maneuvers, in which the desired attitude can be achieved, whereas the attitude constraints cannot be guaranteed. Thus, the standard SMC method needs to be modified to consider attitude constraints. In [15], a classical Lyapunov function augmented with an attractive and a repulsive potential field was used to design the sliding mode controller. 

Based on these observations, a question arises: How can we consider attitude constraints in the sliding surface design for attitude maneuvers by extending the idea of Utkin et al. [17] and Guldner and Utkin [18]? The main purpose of this Note is to answer this question. The main contributions are as follows: 

1) By introducing APF into the design process of the sliding surface, this Note presents new results of sliding surfaces using different attitude parameterizations for spacecraft attitude maneuvers. The existing classical results of sliding surfaces in the literature are shown to be special cases of the obtained new results. 

2) By applying the proposed sliding surface design principle, this Note designs a new sliding surface for spacecraft reorientation with attitude constraints. The novelty of the proposed sliding surface is that, once the sliding mode is reached, the attitude of the spacecraft will converge to the desired one while satisfying attitude constraints automatically even under external disturbances. Moreover, the existence and the characteristics of critical points of the potential field in the sliding mode are analyzed. As attitude constraints are considered in the sliding surface design, a standard controller design method of SMC can be implemented for the constrained reorientation without any adjustment. 

This Note is organized as follows. In Sec. II, dynamic equations of spacecraft rotational motion and attitude constraints on spacecraft reorientation maneuvers are presented. In Sec. III, based on a potential field-based sliding surface design principle, new results of sliding surfaces for spacecraft attitude maneuvers using different attitude parameterizations are presented. In Sec. IV, the proposed design principle is applied to the sliding surface design for spacecraft constrained reorientation and a sliding mode controller is developed based on the proposed sliding surface. The stability of the closedloop system under the proposed controller is proven using the Lyapunov method. The existence and the characteristics of critical points of the potential field in the sliding mode are analyzed. Section V presents simulation results to validate the proposed controller for the constrained reorientation. Conclusions are drawn in Sec. VI. 

- braunschweig.de. 

- m.ben-larbi@tu-braunschweig.de. 

- tu-braunschweig.de. 

## II. Preliminaries 

This section presents kinematic and dynamic equations of spacecraft rotational motion using different attitude parameterizations. Attitude constraints are described and mathematically formulated using unit quaternions. 

## A. Rotational Kinematics Based on Different Parameterizations and Dynamics 

The kinematics using unit quaternions is given as [19] 

where q ��q0; q�� is a unit quaternion representing the attitude, with vector part;q0 ∈ R being ω ��its 0scalar; ω� � is the quaternion form of the angular velocitypart and q� ��q1; q2; q3�[T] ∈ R[3] being its ω� ��ω1; ω2; ω3�[T] ∈ R[3] ; and ⊗ is quaternion multiplication. Given two quaternions, a ��a0; a�� and b ��b0; b[�] �, quaternion multiplication is calculated as a ⊗ b ��a0b0 − a� ⋅ b; a[�] 0b[�] � b�0 �a � a� × b[�] �. The conjugate of quaternion is defined aspart of quaternion is defined as vec�a�� aa�[�] . ��a0; −a�. The vector 

The kinematics using Rodrigues parameters ρ� ∈ R[3] is as follows [19]: 

where �ρ ��ρ1; ρ2; ρ3�[T] is related to unit quaternion by �ρ � q�∕q0 [19] and the matrix T�ρ�� is defined as T�ρ�� ≜ �1∕2��I3×3 � ρ�ρ�[T] � ρ�[×] � with I3×3 ∈ R[3][×][3] being the identity matrix. ρ�[×] is the cross-product matrix, which is defined as 

The kinematics using modified Rodrigues parameters σ� ∈ R[3] is summarized as [19] 

where σ� ∈ R[3] is related to unit quaternion by σ� � q�∕�1 � q0� [19] and the matrix F�σ� � is defined as F�σ�� ≜ �1∕4���1 − σ�[T] σ� �I3×3� 2�σ[×] � 2�σσ�[T] � with σ�[×] being the cross-product matrix. 

The dynamic equation is given as [19] 

where I ∈ R[3][×][3] is the inertia matrix of the spacecraft, u� ∈ R[3] is the control torque, d[�] ∈ R[3] is the external disturbance torque, and ω�[×] is the cross-product matrix. 

## B. Attitude Constraints Based on Unit Quaternions 

Both attitude forbidden zones and attitude mandatory zones are considered in this Note. These attitude-constrained zones are usually defined as cones around specific unit vector directions in space. An attitude forbidden zone is a cone that the boresight vector of certain instrument (e.g., a telescope) of a spacecraft is required to avoid during its reorientation. An attitude mandatory zone is a cone, in whichthe boresight vector of certain instrument (e.g.,an antenna) of a spacecraft is required to stay. 

Figure 1 illustrates a sketch of a spacecraft equipped with a telescope and an antenna, and its related attitude-constrained zones. The red- and blue-shaded cones represent the attitude forbidden zone (indicated by the subscript F) related to the telescope and the attitude mandatory zone (indicated by the subscript M) related to the antenna, respectively; r�[B] F[and][r][�][B] M[,][both][expressed in][the][body][frame][as][indi-] cated by the superscript B, denote unit boresight vectors of the telescope and the antenna, respectively; n�[I] F[and][n][�][I] M[, both expressed] in the inertial frame as indicated by the superscript I, represent the unitdirectionvector of the bright object to avoid andthe unitdirection vector of the desired direction for the communication, respectively; θF and θM are half angles of cones representing the attitude forbidden zone and the attitude mandatory zone, respectively. 

The unit boresight vector of the telescope, r�[B] F[,][is][required][to] stay outside the red-shaded attitude forbidden zone characterized 

Fig. 1 Spacecraft sketch with attitude-constrained zones. 

by n�[I] F[and][θ] F[.][This][requirement][can][be][mathematically][formulated] as 

where r�[I] F[is the expression of][r][�][B] F[in the inertial frame, which can be] calculated based on the transformation [19] r[I] F[�][q][ ⊗][r][B] F[⊗][q][�][, with] rr�[B] F[I] F[, respectively.][��][0][;][ �][r][B] F[�][and][ r][I] F[��][0][;][ �][r][I] F[�][denoting quaternion forms of][r][�][B] F[and] 

Based on unit quaternions, the requirement of the attitude forbidden zone in Eq. (6) can be rewritten as [13] 

with 

where� m1 � r�[B] F[⋅][n][�][I] F[−][cos][ θ] F[,][m][�] 2[�][r][�][B] F[×][n][�][I] F[,][and][A][ �][r][�][B] F[�][n][�][I] F[�][T][�] n[I] F[�][r][�][B] F[�][T][−][�][r][�][B] F[⋅][n][�][I] F[�][cos][ θ] F[�][I] 3×3[.][Note][that][M] F[is][a][symmetric] matrix, i.e., MF � M[T] F[.] 

in the blue-shaded attitude mandatory zone characterized byEarth, the unit boresight vector of the antenna,To keep the continuous communication with the ground station onr�[B] M[, is required to stay] n�[I] M[and] θM. Similar to the formulation of the attitude forbidden zone, the requirement of the attitude mandatory zone can be written as [13] 

wherebeing replaced by MM has the same form asr�[B] M[,][n][�][I] M[, and][ θ] M M[, respectively.] F in Eq. (8), with r�[B] F[,][n][�][I] F[, and][ θ] F 

It should be mentioned that, because attitude constraints considered inthis Noteare assumed to be fixedinthe inertial frame, both MF and MM are constant matrices once parameters of attitude constraints are set. In addition, it has been shown by Lee and Mesbahi [13] that −1 < �1∕2�q[T] MFq < 0 and 0 < �1∕2�q[T] MMq < 1 hold for attitude forbidden zones and attitude mandatory zones, respectively. 

## III. Potential Field-Based Sliding Surface Design for Spacecraft Attitude Maneuvers 

This section explains the basic idea of the potential field-based sliding surface design and presents new results of sliding surfaces using different attitude parameterizations based on the proposed design methodology. 

## A. Methodology 

The potential field-based sliding surface design methodology in this Note is inspired by sliding surface design in [17,18] and the sliding surface design using transformation to reduced form in [20]. In the potential field-based sliding surface design methodology, a potential function Vp is first introduced. Vp is designed so that it only depends on the spacecraft attitude with the property that Vp ≥ 0 for all attitudes and Vp � 0 holds if and only if at the desired attitude. 

Note that the designed potential function can also be regarded as a Lyapunov function for the sliding mode phase. Then, the sliding surface design is approached in a backward manner. To be specific, the expression of V[_] p is investigated and some constrained motions are chosen to ensure V[_] p to be negative definite. These constrained motions are actually candidates of the sliding surface. 

## B. Sliding Surface Design: Results Based on Different Attitude Parameterizations 

The proposed design methodology is applied in spacecraft attitude tracking maneuvers. The application of the design principle to the sliding surface design based on unit quaternion is described in detail. 

qe Let��qthee0; qAPF�e�, whereVp beq�e a��functionqe1; qe2; qofe3the�[T] anderrorqe �unitq[�] dquaternion[⊗][q][,][with] qd denoting the desired attitude. Vp is designed to have the property that Vp ≥ 0 and Vp � 0 if and only if qe ��1, where 1 ≜ �1; 0[�] � denotes the identity quaternion. (Note that 1 and −1 denote the same attitude.) The design of the sliding surface is based on the following observation: Taking the time derivative of_ Vp and using the relative kinematic equation qe ��1∕2�qe ⊗ ωe [21] with ωe � ω − ωd (note that both ω and ωd are expressed in the body frame) yields 

where the property that a[T] �b ⊗ c�� c[T] �b[�] ⊗ a� is used [13] and the gradient of Vp is defined as 

phenomenon. (When the unwinding phenomenon occurs, the spacecraft will not rotate in the shortest path toward the desired attitude [11,22].) By noting that �1∕2�kqe − δ�qe0�1k[2] ��1∕2��qe− δ�qe0�1�[T] �qe − δ�qe0�1�� 1 − δ�qe0�qe[T] 1, the gradient of potential function is ∇Vp ��∂Vp∕∂qe�� −δ�qe0�1 and ∇[2] Vp � 03×3. Thus, the sliding surface in Eq. (11) can be simplified to 

Note that the sliding surface in Eq. (13) is the same as the classical sliding surface for the attitude tracking in [23]. 

Similarly, the sliding mode design based on Rodrigues parameters and modified Rodrigues parameters can be obtained (see AppendicesTable 1. NoteAthat,and inB).TableThese1, formainunitresultsquaternion,are summarizedω� e ≜ ω� − ω�ind, qe ≜ q[�] d[⊗][q][,][and][∇][V][p][≜∂][V][p][∕∂][q][e][;][for][Rodrigues][parameters,] ω� e ≜ ω� − T[−][1] �ρ��T�ρ� d�ω� d, ρ� e ≜ ρ� − ρ� d, and ∇Vp ≜∂Vp∕∂ρ� e; for modified Rodrigues parameters, ω� e ≜ ω� − F[−][1] �σ��F�σ� d�ω� d, σ� e ≜ σ� − σ� d, and ∇Vp ≜∂Vp∕∂σ� e. Note that the special forms in – Table 1 are the same as the results in [23 26], respectively. 

Sliding surfaces for attitude regulation based on the proposed design methodology can be obtained by modifying results in Table 1 as the attitude regulationis a special case of the attitude tracking. Note that, with different designs of the potential field, different sliding surfaces for attitude maneuvers can be obtained. Moreover, by exploring the similarity between quaternions and dual quaternions, the sliding surface based on dual quaternions for relative pose (i.e., relative position and relative attitude) control in [27] can be obtained using the proposed sliding surface design methodology. 

� Let ωe � −Kαvec�q[�] e ⊗∇Vp�, with K_α ∈ R[3][×][3] being� a diagonal positive matrix, and then it yields Vp � −�1∕2�ω[T] e K[−] α[1][ω][�] e[≤][0][.] When Vp decreases to zero, the desired attitude is reached. Based on these observations, the sliding surface is designed as 

where Kα ∈ R[3][×][3] is a diagonal positive matrix. 

The potential function can be chosen as the following quadratic function: 

where k ⋅ k denotes the norm of quaternions; qe0 is the scalar element of qe; and the term δ�qe0� with the property that δ�qe0�� 1 if qe0 ≥ 0 and δ�qe0�� −1 if qe0 < 0 is introduced to avoid the unwinding 

In this section, we present the application of the proposed sliding mode design principle in spacecraft reorientation with attitude constraints. Unit quaternion is used as the attitude parameterization. 

A. Sliding Surface for Spacecraft Constrained Reorientation As for the reorientation case, based on the sliding surface in Eq. (11), the sliding surface is designed as 

where α ∈ R and α > 0. 

The attitude constraints are considered in the design of the APF. Inspired by the structure of the potential field by Hu et al. [11], the following potential field is designed: 

Table 1 Summary of sliding surfaces 

|||Special form of sliding surface with|
|---|---|---|
|Attitude parameterization|General form of sliding surface|quadratic form ofVp|
|Unit quaternion|�s��ωe�Kαvec�q�e ⊗∇Vp�|�s��ωe�δ�qe0�Kα�qe<br>(withVp ��1∕2�kqe−δ�qe0�1k2)|
|Rodrigues parameters|�s1 ��ωe�T−1��ρ�Kα∇Vp|�s1 ��ωe�T−1��ρ�Kα�ρe<br>(withVp ��1∕2�k�ρek2 ��1∕2��ρTe�ρe)|
||�s2 ��ωe�c∇Vp|�s2 ��ωe�Λ 0�ρe<br>(withVp ��1∕2��ρTeΛ�ρe andΛ 0 �cΛ)|
|Modified Rodrigues parameters|�s��ωe�F−1��σ�Kα∇Vp|�s��ωe�F−1��σ�Kα�σe<br>(withVp ��1∕2�k�σek2 ��1∕2��σTe�σe)|

where qd is the desired attitude, which is a constant unit quaternion for the reorientation problem; the term δ�qe0� has the same definition as in Eq. (12); β1 > 0 and β2 > 0 are parameters to be set; and barrier potential fields fF�q� and fM�q� are defined as [13] 

and 

where k1 > 0 and k2 > 0. For convenience, define f � fF � fM. By noting that kq − δ�qe0�qdk[2] � 2 − 2δ�qe0�q[T] d[q][,][ ∇][V][p][can be calculated as] 

where �∂f∕∂q���∂fF∕∂q���∂fM∕∂q�. The calculation of ∂fF∕∂q and ∂fM∕∂q is detailed in Appendix C. 

## B. Analysis of Critical Points of Potential Field in Sliding Mode 

When the state reaches and stays on the sliding surface, i.e., � � � � s � ω � αvec�q[�] ⊗∇Vp�� 0[�] , one has V[_] p � −�1∕2α�ω[T] ω ≤ 0. For Vp � 0, the desired attitude is reached. However, it is possible � that ω � −αvec�q[�] ⊗∇Vp�� 0[�] when q ≠ δ�qe0�qd, which leads to the problem of local minima [11]. 

Figure 2 shows the possible time histories of Vp with three different cases of critical points of the potential field in the sliding mode. It should be mentioned that, in this Note, critical points of the potential field mean the attitudes at which V[_] p � 0 holds. When V[_] p � 0 occurs at the desired attitude, this corresponds to the global minimum (as shown in Fig. 2a). When V[_] p � 0 occurs at an attitude that is not the desired one, the attitude can be either a local minimum or a saddle point. When a critical point is a local minimum, the attitude of the 

spacecraft will be trapped atthis local minimum (as shown inFig. 2b). When a critical point is a saddle point, any small disturbances or measurement noises can drive the attitude away from this saddle point (as shown in Fig. 2c). Hu et al. [11] presented a detailed quantitative analysis of the problem of local minima. However, it is difficult to extend their method to the case with complex attitude constraints. In this Note, we present a qualitative analysis of critical points as follows. 

For vec�q[�] ⊗∇Vp�� 0[�] , there are two cases for the solution: one case is q[�] ⊗∇Vp � 0, with 0 ≜ �0; 0[�] �, and the other is q[�] ⊗∇Vp � λ1, where λ ∈ R and λ ≠ 0. Case 1: q[�] ⊗∇Vp � 0 The following proposition is presented for the analysis. 

Proposition 1: q[�] ⊗∇Vp � 0 yields ∇Vp � 0. Furthermore, based on Eq. (18), ∇Vp � 0 yields 1 − δ�qe0�q[T] d[q][ ≠][0][,][and][hence] q ≠ δ�qe0�qd. 

Proof: See Appendix D. 

When q[�] ⊗∇Vp � 0 holds, based on Proposition 1, Eq. (18) can be rewritten as 

As indicated by Proposition 1, the attitude cannot reach the desired one. Thus, the solution (or solutions) to Eq. (19) can either be a local minimum or a saddle point of the potential field in the sliding mode. As shown in Eq. (19), the solution is related to the configuration of the potential field since f, ∂f∕∂q, β1, and β2 are involved. But it is difficult to obtain an analytical solution to Eq. (19). Here, we present a qualitative analysis to show the effect of the ratio β1∕β2 on the locations of critical points. Based on Eqs. (C1) and (C2), and �∂f∕∂q���∂fF∕∂q���∂fM∕∂q�, when q is in the feasible set (i.e., the set of attitudes at which attitude constraints are satisfied), ∂f∕∂q is well defined. Consider an extreme case where we increase the ratio β1∕β2 (e.g., increasing β1 while fixing β2) to ∞. According to Eq. (19), thevalue of ∂f∕∂q will increase to ∞, which, according to Eqs. (C1) and (C2), is actually the consequence when q[T] M[j] Fi[q][ and/or] q[T] MMq approach 0. Thus, it means that the solution attitude q to the 

**a) Only with global minimum** 

**b) With local minimum** 

**c) With saddle point** 

Fig. 2 Illustration of Vp with different critical points. 

equation ∇Vp � 0 will approach boundaries of attitude-constrained zones when the ratio β1∕β2 increases to a large value. 

Case 2: q[�] ⊗∇Vp � λ1 (λ ∈ R and λ ≠ 0) In this case, q[�] ⊗∇Vp � λ1 yields ∇Vp � λq. Furthermore, k∇Vpk �jλj holds as q is a unit quaternion. Based on Eq. (18), q[�] ⊗ ∇Vp is calculated as 

It can be verified that q � δ�qe0�qd leads to q[�] ⊗∇Vp � −2δ�qe0��β1 � β2f�1, which means that the desired attitude satisfies the condition that q[�] ⊗∇Vp � λ1. Depending on the configuration of the attitude-constrained zones (namely, the terms fF and fM) and the weight parameters set (namely, parameters β1 and β2), it is possible that q[�] ⊗∇Vp � λ1 holds for certain q ≠ δ�qe0�qd. This also leads to local minima or saddle points of the potential field. By collecting terms, Eq. (20) can be rewritten as 

When the ratio β1∕β2 increases to a very large value, compared with the first term in the square bracket in Eq. (21), the second term vanishes, which yields the following: 

Thus, δ�qe0�qd will be the unique solution to q[�] ⊗∇Vp � λ1 when the ratio β1∕β2 increases to a very large value. 

From the previous analysis, it can be concluded that locations of local minima or saddle points of the potential field can be adjusted by changing the ratio β1∕β2, which benefits from the introduction of the term β1kq − δ�qe0�qdk[2] in the potential field. The analysis presented in this subsection can be used to guide the setting of parameters of the potential field. 

## C. Sliding Mode Controller Design for Spacecraft Constrained Reorientation 

In this subsection, the control law is designed to drive states of the spacecraft onto the sliding surface. 

The following assumptions are made for the controller design: Assumption a: Full states of the spacecraft are available. (i.e.,Assumptiondi < Ld withb: External di being components of external disturbancesdisturbances are unknown but boundedd�) and the upper bound���� ���� Ld is known. Assumption c: The inertia matrix is constant and known. Based on the potential field in Eq. (15) and the sliding surface in Eq. (14), an SMC law is proposed using the reaching law method [20] as 

whereand k areα isparametersthe parameterto beof setthe withslidingσ > Lsurfaced andin k >Eq. 0(14);; sgnboth�s���σ �sgn�s1�; sgn�s2�; sgn�s3��[T] with si�i � 1; 2; 3� being components of �s; and the term �d∕dt��q[�] ⊗∇Vp� iscalculated as (see Appendix E for the derivation) 

Theorem: Consider the model of rigid body rotation in Eqs. (1) and (5). Assume that, for the potential field Vp in Eq. (15), parameters are set so that there is no solution to ∇Vp � 0 for all feasible q and the ratio β1∕β2 is set as a suitably large positive value. Under Assumptions a, b, and c, the controller in Eq. (23) based on the sliding surface in Eq. (14) ensures that the states of the spacecraft asymptotically converge to fq ��qd; ω� � 0[�] g. 

Proof: First, we prove that the surface s� � 0[�] can be reached in finite time. 

Consider a candidate Lyapunov function V ��1∕2�s�[T] Is�. V is a valid candidate Lyapunov function since V ≥ 0 for all �s and V � 0 if and only if �s � 0[�] . There exist two positive constants λmin and λmax so that �1∕2�λmin �s[T] s� ≤ V ≤ �1∕2�λmaxs�[T] s�. Thus, s�[T] s� ≥ �2∕λmax�V. 

Taking the time derivative of V, and using the dynamic equation in Eq. (5) with the control law in Eq. (23), it yields 

� � � Note that �s[T] �d[�] − σsgn�s�� ≤−as[T] sgn�s� with a � minfσ − dig > 0. In addition, it holds that 

Thus, Eq. (25) yields 

It follows that 

According to the lemma for the finite-time stability introduced in Appendix F, the Lyapunov function V can reach 0 in finite time. Thus, the sliding variable s� can reach 0[�] in finite time. 

Next, we prove that the control objective can be achieved in the sliding mode. 

When the sliding mode is reached, i.e., s� � ω� � � αvec�q[�] ⊗∇Vp�� 0[�] , ω � −αvec�q[�] ⊗∇Vp� holds. Based on � � Eq. (10), it yields V[_] p � −�1∕2α�ω[T] ω ≤ 0. As there is no solution to ∇Vp � 0 and the ratio β1∕β2 is set as a suitably large positive value, according to the analysis of critical points in Sec. IV.B, the states will asymptotically converge to fq ��qd; ω� � 0[�] g. (Note that qd and −qd are the same attitude.) This completes the proof. □ To avoid the chattering problem resulting from the use of the sign function,[28,29]. Thethe signith functioncomponentis replacedof s� isby replacedthe saturationby satfunction�si; ϵ� (i � 1; 2; 3). The saturation function is defined as [19] 

where ϵ is a small positive constant. With the saturation function, the sliding variable can only converge to a neighborhood of the sliding surface [29]. 

Remark 1: The Lyapunov function V is bounded based on the analysis in the proof [as shown in Eq. (26)]. Thus, the sliding variables� is bounded. According to Eq. (14), the boundedness of s� implies that ∇Vp is bounded. Based on the expression of ∇Vp in Eq. (18), it can be concluded that both fF and fM are bounded, which means that during the maneuverboth attitude-constrained zones are not violated. 

Remark 2: As long as the attitude does not start from a local minimum of the potential field, the attitude of the spacecraft will 

Table 2 Settings of attitude-constrained zones 

|Constrained zone|Constrained direction|Angle, deg|
|---|---|---|
|F1|�0.2530;−0.3700;−0.8939�T|25|
|F2|�0;0.7071;0.7071�T|40|
|F3|�−0.8532;0.4361;−0.2861�T|30|
|F4|�0.9577;−0.2562;0.1311�T|20|
|M|�0.5871;0.7520;−0.2996�T|60|

Table 3 Controller parameters 

|Controller|Parameters|
|---|---|
|Proposed controller|k1 �1,k2 �1,β1 �5β2,β2 �0.01|
||α�2,σ �0.35,k�4,ϵ�0.0001|
|APF-based PD|α�20,k1 �1.4,k2 �1.4|

Remark 3: Compared with the sliding surface in [15], the proposed sliding surface in Eq. (14) has a novel property that, once the sliding mode is reached, the attitude of the spacecraft will converge to the desired one while satisfying attitude constraints automatically even under external disturbances. As attitude constraints are considered in the sliding surface design, the standard SMC method can be implemented without any adjustment. 

Remark 4: The proposed controller in Eq. (23) needs the information of the true inertia matrix of the spacecraft, which is difficult to obtain in practice. For implementation purposes, the true inertia matrix^ I in Eq. (23) can be replaced with the estimated inertia matrix^ Icandidate. Let ΔI �LyapunovI − I denotefunctionthe modelV ��uncertainty.1∕2�s�[T] Is�. WithAgain,modelconsideruncer-a tainties, taking the time derivative of V, it yields 

where 

not be trapped at an undesired attitude since the Lyapunov functiondecreases monotonically. Thus, as long as it holds that s� ≠ 0[�] at the beginning of the maneuver, the problem of trapping at a local minimum can be avoided in the reaching phase. (In the SMC, the reaching phase is the phase before the sliding mode phase.) 

Fig. 3 Two-dimensional projection of traces of boresight vectors of telescope and antenna. 

is the lumped disturbance that includes both external disturbances and the model uncertainty. When the terms introduced by the model uncertainty, i.e., 

are small, a properly large value of σ can deal with the lumped disturbance. However, when the terms introduced by the model uncertainty are large, the reaching phase or the sliding mode will be influenced. To consider the model uncertainty explicitly, adaptive control methods proposed in literature (e.g., [30]) can be applied. 

## V. Numerical Simulation 

In this section, we demonstrate the effectiveness of the proposed APF-based SMC for spacecraft constrained reorientation and compare the proposed controller with the APF-based PD controller by Lee and Mesbahi [13]. The spacecraft in the simulation is equipped with a telescope and an antenna. The telescope and the antenna are aligned along the positive Z axis and the positive X axis, respectively, both in the body frame. The true inertia matrix of the spacecraft in the simulation is 

Fig. 4 Time histories of �1∕2�q[T] MFq and �1∕2�q[T] MMq under proposed controller and APF-based PD controller. 

The estimated inertia matrix used in the proposed controller is set as^ I � diag�50; 45; 80� kg ⋅ m[2] . The spacecraft is required to retarget its telescope from the initial orientation to the desired orientation. The initial and desired attitudes are set as q�0���0.6026; −0.73; 0.2; 0.253�[T] and qd ��0.9975; 0.0499; 0.0499; 0�[T] , respectively. As a rest-to-rest reorientation maneuver is considered, the initial and 

desired angular velocities are ω� �0�� ω� d � 0[�] . For the simulation, external disturbances are set as 

In the simulation, we consider a scenario with four attitude forbidden zones and one attitude mandatory zone. Settings of attitudeconstrained zones are listed in Table 2. Parameters for the proposed controller and the APF-based PD controller are listed in Table 3. 

Fig. 5 Time histories of error quaternion, angular velocity, control torque, and energy cost under proposed controller and APF-based PD controller. 

**a) Sliding variable** 

**b) The value of potential field** 

Fig. 6 Time histories of sliding variable and value of potential field under proposed controller. 

Parameters for controllers are set so that maneuver settling times under both controllers are similar. 

Figure 3 presents 2D projections of traces of boresight vectors of the telescope and the antenna on the unit celestial sphere under both controllers. Red- and blue-shaded areas indicate attitude forbidden zones and the attitude mandatory zone, respectively. Red diamond andcross denote the start andthe end of the telescope boresight vector trace, respectively. Blue circle and plus denote the start and the end of the antenna boresight vector trace, respectively. Under both controllers, both telescope boresight vector traces stay outside all attitude forbidden zones and both antenna boresight vector traces stay within the attitude mandatory zone. Thus, attitude constraints are not violated during the reorientation. It is noted that endpoints of both telescope boresight vector traces are different. This is caused by external disturbances in the simulation. With external disturbances, under the APF-based PD controller, the telescope cannot be targeted toward the desired direction precisely. However, the proposed controller can retarget the telescope precisely even with external disturbances. 

The time histories of the values of �1∕2�q[T] MFq and �1∕2�q[T] MMq are depicted in Fig. 4. The values of �1∕2�q[T] MFq for four attitude forbidden zones are always negative (Fig. 4a) and the values of �1∕2�q[T] MMq for the attitude mandatory zone are always positive (Fig. 4b). This verifies that all attitude constraints are satisfied during the rotation under both the APF-based PD and the proposed controller. Note that the values of �1∕2�q[T] MFq for attitude forbidden zone F2 under both controllers are very close to 0 during 0–25 s, which is consistent with the observation that some parts of both telescope boresight vector traces are very close to F2 in Fig. 3. 

Figure 5 plots the time histories of the error quaternion, the angular velocity, the control torque, and the energy cost under the proposed and the APF-based PD controller. As shown in Fig. 5a, the error quaternion qe converges to �1; 0; 0; 0�[T] under both controllers, which means that the telescope is retargeted to the desired orientation. The settling times under both controllers are similar, about 70 s. However, the control accuracy with external disturbances under the proposed controller is much better than that under the APF-based PD controller (as shown in Figs. 5b and 5d). As shown in Figs. 5c and 5e, under both the proposed and the APFbased PD controller, both the angular velocity and the control torque show peaks in their components. These peaks are caused by the fact that the telescope boresight vector approaches attitude forbidden zone F2. To avoid violating attitude constraints, repulsive torques are generated by the potential field to push away the telescope. Figure 5f plots the energy cost of a total simulation time of 150 s under the proposed and the APF-based PD controller. The energy cost is defined as 

As shown in Fig. 5f, the energy cost under the proposed controller is 113.42 N[2] ⋅ m[2] ⋅ s, which is less than half of that under the APFbased PD controller (268.49 N[2] ⋅ m[2] ⋅ s). The big difference in energy cost can be explained as follows. Under the APF-based PD controller, at the beginning, the spacecraft has a faster response (see large values of components of the angular velocity at the beginning in Fig. 5c) than that under the proposed controller as larger torques are commanded by the APF-based PD controller (see Fig. 5e). Such a fast response leads to a large control effort for the APF-based PD controller to stop the telescope boresight vector from approaching attitude forbidden zone F2. In contrast, the initial response under the proposed controller is slow as small torques are commanded at the beginning. Thus, small control efforts are required to avoid the telescope boresight vector entering attitude forbidden zone F2. 

Figure 6 plots the time histories of the sliding variable and thevalue of the potential field. As shown in Fig. 6a, components of the sliding variable can converge to 0 in finite time. The small oscillations of components of the sliding variable during 40–50 s result from the model uncertainty as discussed in Remark 4. The value of the potential field decreases monotonically (see Fig. 6b), where the sharp drop of the value during about 40–50 s corresponds to peaks of components of the angular velocity. 

## VI. Conclusions 

This Note presents a potential field-based sliding surface design principle for spacecraft attitude maneuvers. Based on the proposed design principle, new results of sliding surfaces using different attitude parameterizations are obtained, which are shown to be general forms of existing classical sliding surfaces for attitude maneuvers in the literature. Furthermore, the proposed design principle is applied to the sliding surface design for spacecraft reorientation with attitude constraints (both attitude forbidden and mandatory zones). The novelty of the proposed sliding surface is that, once the sliding mode is reached, the attitude of the spacecraft will converge to the desired one while satisfying attitude constraints automatically. The existence and the characteristics of critical points of the potential field in the sliding mode are analyzed. As attitude constraints are considered in the sliding surface design,a standard controller design method of SMC is implemented for the constrained reorientation without any adjustment. The stability of the closed-loop system under the proposed APF-based sliding mode controller is proven using the Lyapunov method. Numerical simulations are presented to demonstrate the effectiveness and the performance of the proposed APFbased SMC for spacecraft constrained reorientation. 

## Appendix A: Sliding Surface Design Based on Rodrigues Parameters 

with the definitionDefine the potential fieldρ� e � ρ� − Vρ�pd as a function of. Vp is designed to have the propertyρ� e ��ρe1; ρe2; ρe3�[T] that Vp ≥ 0 and Vp � 0 if and only if ρ� e � 0[�] . Taking the time derivative of Vp and using the kinematics in Eq. (2) yields 

� � � � � F[−] If[1] �σ��Fwe�σ� d�ω�letd − FF[−][1] �σ�σ��ω�K −α∇FV�σpd),�ωwithd �K−αK∈α∇RV[3][×] p[3] being(i.e.,a diago-ω � nal positive matrix, it yields that V[_] p � −�∇Vp�[T] Kα∇Vp ≤ 0. When Vp decreases to zero, the desired attitude is reached. Define ω� e ≜ ω� − F[−][1] �σ��F�σ� d�ω� d. The sliding surface can be designed as 

where Kα ∈ R[3][×][3] is a diagonal positive matrix. 

where 

The following two kinds of analysis can be used for the sliding surface design. 

� � � � ω� �AnalysisT[−][1] �ρ��T1:�ρ� d�Ifω� d −weT[−] let[1] �ρ��TK�αρ∇�ωV −p),T�withρd�ωdK�α ∈−KR[3] α[×] ∇[3] Vbeingp (i.e.,a diagonal positive matrix, it yields that V[_] p � −�∇Vp�[T] Kα∇Vp ≤ 0. When Vp decreases to zero, the desired attitude is reached. Define ω� e ≜ ω� − T[−][1] �ρ��T�ρ� d�ω� d. The sliding surface is designed as 

where Kα ∈ R[3][×][3] is a diagonal positive matrix. 

�1∕When2�ρ�[T] e �ρethe, thepotentialslidingfunctionsurfaceis inchosenEq. as(A2)Vp is��simplified1∕2�kρ� ek[2] �as s�1 � ω� e � T[−][1] �ρ��Kαρ� e, which is the same as the sliding surface in [24]. 

Analysis 2: With the definition of ω� e, the time derivative of Vp in Eq. (A1) can be rewritten as 

� � Note that T�ρ� ≥ �1∕2�I3×3 [25]. If_ we let ωe � −c∇�Vp with c ∈ R and c > 0, it yields that Vp � −c�∇Vp�[T] T�ρ�∇Vp ≤ −�1∕2�c�∇Vp�[T] ∇Vp ≤ 0. When Vp decreases to zero, the desired attitude is reached. Based on this observation, the sliding surface is designed as 

where c is a positive scalar. 

When the potential function is chosen as Vp ��1∕2�ρ�[T] e Λρ� e, whereEq. (A4) is simplified as Λ ∈ R[3][×][3] is a diagonal positive matrix, the sliding surface ins�2 � ω� e � Λ[0] ρ� e, with Λ[0] � cΛ, which is the same as the sliding surface in [25]. 

�Eq. (B2) is simplified as1∕The2�σ�[T] epotential �σe. Then,functionone has �s �can∇ω�Ve �pbe�Fchosenσ�[−] e[1] .�σ�Thus,�Kasα �σetheV. Note that the slidingp sliding��1∕2surface�kσ� ek[2] �in surface in [26] is a special result of the general form in this Note. 

## Appendix C: Related Calculation for Potential Field in Eq. (15) 

The first partial derivatives ∂fF∕∂q and ∂fM∕∂q are calculated by 

and 

The second partial derivatives ∂[2] fF∕∂q[2] and ∂[2] fM∕∂q[2] are calculated by 

and 

The Hessian matrix ∇[2] Vp ��∂�∇VP�∕∂q� is as follows: 

where 

## Appendix B: Sliding Surface Design Based on Modified Rodrigues Parameters 

with the definitionDefine the potential fieldσ� e � σ� − Vσ�pd as a function of. Vp is designed to have the propertyσ� e ��σe1; σe2; σe3�[T] that Vp ≥ 0 and Vp � 0 if and only if σ� e � 0[�] . Taking the time derivative of Vp and using the kinematics in Eq. (4) yields 

## Appendix D: Proof of Proposition 1 

First, we prove that q[�] ⊗∇Vp � 0 yields ∇Vp � 0. If ∇Vp ≠ 0, then the identity q[�] ⊗∇Vp � 0 can be rewritten as k∇Vpk�q[�] ⊗ �∇Vp∕k∇Vpk��� 0. As ∇Vp∕k∇Vpk is a unit quaternion, q[�] ⊗ �∇Vp∕k∇Vpk� is also a unit quaternion, i.e., kq[�] ⊗ �∇Vp∕k∇Vpk�k� 1. Thus, k∇Vpk�q[�] ⊗ �∇Vp∕k∇Vpk��� 0 yields k∇Vpk � 0, which is contradictory to the assumption that ∇Vp ≠ 0. Thus, q[�] ⊗∇Vp � 0 yields ∇Vp � 0. 

where 

Next, we prove that, based on Eq. (18), ∇Vp � 0 yields 1 − δ�qe0�q[T] d[q][ ≠][0][, and hence][ q][ ≠][δ][�][q][e][0][�][q][d][.] In fact, if 1 − δ�qe0�q[T] d[q][ �][0][, Eq. (18) can be simplified as][ ∇][V][p][�] −2δ�qe0��β1 � β2f�qd. Since δ�qe0���1, β1 > 0, β2 > 0, f > 0, and qd is a unit quaternion, one has k∇Vpk � 2�β1 � β2f� > 0, which is contradictory to the conclusion ∇Vp � 0. Thus, based on Eq. (18), ∇Vp � 0 yields 1 − δ�qe0�q[T] d[q][ ≠][0][.][It][is][obvious][that] 1 − δ�qe0�q[T] d[q][ ≠][0][ yields][ q][ ≠][δ][�][q][e][0][�][q][d][. This completes the proof of] Proposition 1. 

## Appendix E: Derivation of Eq. (24) 

_ where q[�] � −�1∕2�ω ⊗ q[�] (which can be obtained by differentiating q[�] ⊗ q � 1) and 

are used. 

## Appendix F: Lemma for Finite-Time Stability 

Lemma [31]: A Lyapunov condition of the finite-time stability can be given as 

and the settling time can be estimated by 

where V�x0� is the initial value of the Lyapunov function V. 

## Acknowledgment 

J. Yang gratefully acknowledges the support from China Scholarship Council (201506290048). 

## References 

- [1] Singh, G., Macala, G., Wong, E., Rasmussen, R., Singh, G., Macala, G., Wong, E., and Rasmussen, R., “A Constraint Monitor Algorithm for the Cassini Spacecraft,” Guidance, Navigation, and Control Conference, AIAA Paper 1997-3526, 1997. https://doi.org/10.2514/6.1997-3526 

- [2] Hablani, H. B., “Attitude Commands Avoiding Bright Objects and Maintaining Communication with Ground Station,” Journal of Guidance, Control, and Dynamics, Vol. 22, No. 6, 1999, pp. 759–767. https://doi.org/10.2514/2.4469 

- [3] Biggs, J. D., and Colley, L., “Geometric Attitude Motion Planning for Spacecraft with Pointing and Actuator Constraints,” Journal of Guidance, Control, and Dynamics, Vol. 39, No. 7, 2016, pp. 1672–1677. https://doi.org/10.2514/1.G001514 

- [4] Frazzoli, E., Dahleh, M., Feron, E., and Kornfeld, R., “A Randomized Attitude Slew Planning Algorithm for Autonomous Spacecraft,” AIAA Guidance, Navigation, and Control Conference, AIAA Paper 20014155, 2001. 

- [5] Kjellberg, H. C., and Lightsey, E. G., “Discretized Constrained Attitude Pathfinding and Control for Satellites,” Journal of Guidance, Control, and Dynamics, Vol. 36, No. 5, 2013, pp. 1301–1309. https://doi.org/10.2514/1.60189 

- [6] Tanygin, S., “Fast Autonomous Three-Axis Constrained Attitude Pathfinding and Visualization for Boresight Alignment,” Journal of Guidance, Control, and Dynamics, Vol. 40, No. 2, 2017, pp. 358–370. 

   - https://doi.org/10.2514/1.G001801 

- [7] Xu, R., Wang, H., Xu, W., Cui, P., and Zhu, S., “Rotational-Path Decomposition Based Recursive Planning for Spacecraft Attitude Reorientation,” Acta Astronautica, Vol. 143, Feb. 2018, pp. 212–220. https://doi.org/10.1016/j.actaastro.2017.11.035 

- [8] Celani, F., and Lucarelli, D., “Spacecraft Attitude Motion Planning Using Gradient-Based Optimization,” Journal of Guidance, Control, and Dynamics, Vol. 43, No. 1, 2020, pp. 140–145. https://doi.org/10.2514/1.G004531 

- [9] Tan, X., Berkane, S., and Dimarogonas, D. V., “Constrained Attitude Maneuvers on SO(3): Rotation Space Sampling, Planning and LowLevel Control,” Automatica, Vol. 112, Feb. 2020, Paper 108659. https://doi.org/10.1016/j.automatica.2019.108659 

- [10] McInnes, C. R., “Large Angle Slew Maneuvers with Autonomous Sun Vector Avoidance,” Journal of Guidance, Control, and Dynamics, Vol. 17, No. 4, 1994, pp. 875–877. https://doi.org/10.2514/3.21283 

- [11] Hu, Q., Chi, B., and Akella, M. R., “Anti-Unwinding Attitude Control of Spacecraft with Forbidden Pointing Constraints,” Journal of Guidance, Control, and Dynamics, Vol. 32, No. 3, 2018, pp. 1–13. https://doi.org/10.2514/1.G003606 

- [12] Shen,Q., Yue, C.,andGoh, C. H., “Velocity-Free AttitudeReorientation of a Flexible Spacecraft with Attitude Constraints,” Journal of Guidance, Control, and Dynamics, Vol. 40, No. 5, 2017, pp. 1293– 1299. 

https://doi.org/10.2514/1.G002129 

- [13] Lee, U., and Mesbahi, M., “Feedback Control for Spacecraft Reorientation Under Attitude Constraints via Convex Potentials,” IEEE Transactions on Aerospace and Electronic Systems, Vol. 50, No. 4, 2014, pp. 2578–2592. https://doi.org/10.1109/TAES.2014.120240 

- [14] Cheng, Y., Ye, D., Sun, Z., and Zhang, S., “Spacecraft Reorientation Control in Presence of Attitude Constraint Considering Input Saturation and Stochastic Disturbance,” Acta Astronautica, Vol. 144, March 2018, pp. 61–68. https://doi.org/10.1016/j.actaastro.2017.12.002 

- [15] Shen, Q., Yue, C., Goh, C. H., Wu, B., and Wang, D., “Rigid-Body Attitude Stabilization with Attitude and Angular Rate Constraints,” Automatica, Vol. 90, April 2018, pp. 157–163. https://doi.org/10.1016/j.automatica.2017.12.029 

- [16] Diaz Ramos, M., and Schaub, H., “Kinematic Steering Law for Conically Constrained Torque-Limited Spacecraft Attitude Control,” Journal of Guidance, Control, and Dynamics, Vol. 41, No. 9, 2018, pp. 1990–2001. 

https://doi.org/10.2514/1.G002873 

- [17] Utkin, V. I., Drakunov, S. V., Hashimoto, H., and Harashima, F., “Robot Path Obstacle Avoidance Control via Sliding Mode Approach,” Proceedings IROS’91: IEEE/RSJ International Workshop on Intelligent Robots and Systems’ 91, Inst. of Electrical and Electronics Engineers, New York, 1991, pp. 1287–1290. https://doi.org/10.1109/IROS.1991.174678 

- [18] Guldner, J., and Utkin, V. I., “Sliding Mode Control for Gradient Tracking and Robot Navigation Using Artificial Potential Fields,” IEEE Transactions on Robotics and Automation, Vol. 11, No. 2, 1995, pp. 247–254. https://doi.org/10.1109/70.370505 

- [19] Markley, F. L., and Crassidis, J. L., Fundamentals of Spacecraft Attitude Determination and Control, Springer, New York, 2014, Chaps. 2, 3, 7. 

- [20] Gao, W., and Hung, J. C., “Variable Structure Control of Nonlinear Systems: A New Approach,” IEEE Transactions on Industrial Electronics, Vol. 40, No. 1, 1993, pp. 45–55. https://doi.org/10.1109/41.184820 

- [21] Xia, Y., Zhu, Z., Fu, M., and Wang, S., “Attitude Tracking of Rigid Spacecraft with Bounded Disturbances,” IEEE Transactions on Industrial Electronics, Vol. 58, No. 2, 2010, pp. 647–659. https://doi.org/10.1109/TIE.2010.2046611 

- [22] Mayhew, C. G., Sanfelice, R. G., and Teel, A. R., “On Quaternion-Based Attitude Control and the Unwinding Phenomenon,” American Control Conference (ACC), Inst. of Electrical and Electronics Engineers, New York, 2011, pp. 299–304. https://doi.org/10.1109/ACC.2011.5991127 

- [23] Crassidis, J. L., Vadali, S. R., and Markley, F. L., “Optimal VariableStructure Control Tracking of Spacecraft Maneuvers,” Journal of Guidance, Control, and Dynamics, Vol. 23, No. 3, 2000, pp. 564–566. https://doi.org/10.2514/2.4568 

- [24] Dwyer, T. A. W., III, and Sira-Ramirez, H., “Variable-Structure Control of Spacecraft Attitude Maneuvers,” Journal of Guidance, Control, and Dynamics, Vol. 11, No. 3, 1988, pp. 262–270. https://doi.org/10.2514/3.20303 

- [25] Chen, Y.-P., and Lo, S.-C., “Sliding-Mode Controller Design for Spacecraft Attitude Tracking Maneuvers,” IEEE Transactions on Aerospace and Electronic Systems, Vol. 29, No. 4, 1993, pp. 1328–1333. https://doi.org/10.1109/7.259536 

- [26] Crassidis, J. L., and Markley, F. L., “Sliding Mode Control Using Modified Rodrigues Parameters,” Journal of Guidance, Control, and Dynamics, Vol. 19, No. 6, 1996, pp. 1381–1383. https://doi.org/10.2514/3.21798 

- [27] Yang, J., and Stoll, E., “Adaptive Sliding Mode Control for Spacecraft Proximity Operations Based on Dual Quaternions,” Journal of Guidance, Control, and Dynamics, Vol. 42, No. 11, 2019, pp. 2356–2368. https://doi.org/10.2514/1.G004435 

- [28] Slotine, J.-J., and Sastry, S. S., “Tracking Control of Non-Linear Systems Using Sliding Surfaces, with Application to Robot Manipulators,” International Journal of Control, Vol. 38, No. 2, 1983, pp. 465–492. https://doi.org/10.1080/00207178308933088 

- [29] Slotine, J.-J., and Coetsee, J., “Adaptive Sliding Controller Synthesis for Non-Linear Systems,” International Journal of Control, Vol. 43, No. 6, 1986, pp. 1631–1651. https://doi.org/10.1080/00207178608933564 

- [30] Cristi, R., Burl, J., and Russo, N., “Adaptive Quaternion Feedback Regulation for Eigenaxis Rotations,” Journal of Guidance, Control, and Dynamics, Vol. 17, No. 6, 1994, pp. 1287–1291. https://doi.org/10.2514/3.21346 

- [31] Yu, S., Yu, X., Shirinzadeh, B., and Man, Z., “Continuous Finite-Time Control for Robotic Manipulators with Terminal Sliding Mode,” Automatica, Vol. 41, No. 11, 2005, pp. 1957–1964. https://doi.org/10.1016/j.automatica.2005.07.001 
