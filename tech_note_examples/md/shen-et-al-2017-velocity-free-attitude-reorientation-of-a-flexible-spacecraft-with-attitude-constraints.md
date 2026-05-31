## Engineering Notes 

## Velocity-Free Attitude Reorientation of a Flexible Spacecraft with Attitude Constraints 

Qiang Shen,[∗] Chengfei Yue,[†] and Cher Hiang Goh[‡] National University of Singapore, Singapore 119077, Republic of Singapore 

DOI: 10.2514/1.G002129 

## I. Introduction 

NE of the essential functions for various spacecraft is to point O an onboard instrument’s boresight along a prescribed inertial direction. In such a mission, the equipped sensitive payloads are required to be kept sufficiently far away from unwanted celestial objects or bright sources of energy. In view of this requirement, the capacity of an attitude controller to handle attitude constraints should be guaranteed. Otherwise, it will lead to damage of certain payloads and inferior control performance. For example, the infrared telescopes may be required to slew from one direction in space to another without direct exposure to the sun vector or other infrared bright regions [1]. Generally, this type of attitude maneuver can be regarded as a spacecraft reorientation problem in the presence of attitude-constrained zones and has attracted more and more attention in practical spacecraft missions. 

A satellite’s motion is governed by kinematic and dynamic equations, and the mathematical models are highly nonlinear and coupled. Extensive nonlinear control algorithms have been proposed for solving the spacecraft reorientation problem, such as proportional-derivative feedback control [2], sliding mode control [3–5], backstepping control [6,7], adaptive control [8], and inverse optimal control [9,10]. However, it should be noted that attitude constraints are not taken into account in the aforementioned literature. An attitude reorientation problem with consideration of attitude-constrained zones has been examined in only a few research works. Approaches to solve constrained attitude control problems can be generalized into two main categories: path planning methods and potential function methods. The path planning methods determine a feasible attitude trajectory before the reorientation maneuver according to the geometric relations with the exclusion zones. Consequently, a constraint-free attitude control law is developed to follow the designed attitude path. In [11], based on the analysis of the vectorial kinematics on sphere, attitude motion planning was considered in the presence of bright objects and a communication link with a ground station is maintained. In [12], assuming that there exists a constraint-free guidance loop, a 

randomized attitude slew planning algorithm was proposed to determine a time-parameterized sequence of “virtual attitude” that effectively steers the current attitude to the target attitude while avoiding constraint violations. In [13], the unit celestial sphere was discretized into a graph using an icosahedron-based pixelization subroutine, and the A[�] pathfinding approach was employed to find an admissible minimum path cost trajectory. In [14], a maneuver planning strategy was derived to accomplish the required single-axis pointing of an underactuated spacecraft in the presence of obstacles along the angular path and constraints on admissible rotation axes. 

Although the path-planning-based methods are able to handle certain classes of attitude constraints, these methods have a disadvantage in that they could not be extended to more complex scenarios, involving multiple celestial constrained zones, as often encountered in spacecraft missions [15]. Meanwhile, because path planning techniques are usually based on computationally demanding search methods, the computational tractability and closed-loop stability of the overall system may not be guaranteed using path-planning-based methods [16]. On the contrary, potential function methods formulate the attitude-constrained zones in the context of an artificial potential, which is further used for synthesizing the corresponding attitude control law to avoid unwanted celestial objects while achieving the desired attitude. Because this kind of approach is analytical, without the need of any change in the overall structure of the attitude control software or hardware, it is suitable for onboard computation andprovidesflexible autonomous operations. In [17], a Gaussian function was used as a potential function to describe the dynamic environment, and control torques were then chosen such that the satellite attitude converges to the desired final orientation without violating a list of user-defined pointing constraints. Because Euler angles are used to represent attitude in [17], the proposed control algorithm may suffer from singularity. Instead of using Euler angles, the unit quaternion not only prevents singularity but also reduces expensive computational load created by the Euler angle expression [18]. In [19], a repulsive potential function was used for a constrained slew maneuver, where the camera’s charge-coupled device chip is prevented from exposure to the sun directly. In [16], attractive and repulsive components of the potential function are designed in quaternion error vector space to guarantee target attitude convergence and constrained direction avoidance, respectively. In [15], a convex logarithmic barrier potential was formulated using the convex parameterization of attitude constraint sets in the unit-quaternion space, and two attitude control laws based on the backstepping technique were proposed for the constrained attitude control problem. 

In this Note, based on potential function methods, a velocity-free control law that can achieve rest-to-rest three-axis attitude reorientation with autonomous avoidance of the undesired celestial objects is presented for a flexible spacecraft. The spacecraft orientation inthe presence of constraintsare formulated interms of unitquaternion and isfurther parameterized toa convex set representation.Becausethe proposed quadratic potential function is proved to be strictly convex, it has the capability to not only handle multiple attitude-constrained zones but also guarantee convergence toward the desired attitude. Following this, an auxiliary unit-quaternion dynamic system introduced in [20] is employed to synthesize thevelocity-free attitude controller that achieves asymptotic stability toward the desired attitude. The advantage of the proposed velocity-free control scheme is the simple design and structure, which is of great interest for aerospace industry with real-time implementation when onboard computing power is limited. To the best of the authors’ knowledge, the result presented in this Note is the first attempt in literature to accomplish attitude reorientation for flexible spacecraft without 

(7) 

using explicit velocity feedback, while attitude constraints are also addressed. 

## II. Preliminaries 

In this Note, the unit-quaternion representation is used to describe the orientation of a spacecraft. A quaternion is defined as Q �� q1 q2 q3 q0 �[T] �� q[T] q0 �[T] ∈ Q, where the vector part q ∈ R[3] , the scalar part q0 ∈ R, and Q is the set of quaternion. The symbol “⊗” denotes the quaternion multiplication operator of two quaternions Qi �� q[T] i qi0 �[T] ∈ Q and Qj �� q[T] j qj0 �[T] ∈ Q, which are defined as follows: 

where J � J[T] denotes the positive-definite inertia matrix of the spacecraft and τ ∈ R[3] denotes the control torque about the body axes. Equation (7) describes the flexible dynamics under the hypothesis of small elastic deformations, where η ∈ R[N] is the modal coordinate vector, δ ∈ R[N][×][3] is the coupling matrix between the elastic and rigid dynamics, C � diag�Λ[2] 1[;][: : : ;][ Λ][2] N[�][and][K][ �] diag�2ζ1Λ1; : : : ; 2ζNΛN� denote the damping and stiffness matrices, N is the number of the elastic modal considered, and Λi and ζi (i � 1; : : : ; N) are natural frequencies and corresponding damping, respectively. 

For the controller design, the following variable is introduced: 

and has the quaternion QI �� 0 0 0 1 �[T] as the identity element. The matrix S�x� ∈ R[3][×][3] is a skew-symmetric matrix satisfying S�x�y � x × y for any vectors x, y ∈ R[3] , and “×” denotes vector cross product. The set of unit quaternion Qu is a subset of quaternion Q such that 

^ where the vector^ part q � n sin�ϕ∕2� and the scalar part q0 � cos�ϕ∕2�; n and ϕ refer to the Euler axis and the rotation angle about the Euler axis. The unit-quaternion conjugate or inverse is defined as Q[�] �� −q[T] q0 �[T] . 

A. Kinematics Equation 

The spacecraft kinematics in terms of the unit-quaternion can be given by 

where Q �� q1 q2 q3 q0 �[T] �� q[T] q0 �[T] ∈ Qu denotes the unit-quaternion describing the attitude orientation of the body frame B with respect to inertial frame I , ω ∈ R[3] is the inertial angular velocity vector of the spacecraft with respect to an inertial frame I and expressed in the body frame B, and ν: R[3] → R[4] is defined as the mapping ν�ω��� ω[T] 0 �[T] . 

Let Qd ∈ Qu denote the desired attitude. In this Note, the rest-torest attitude reorientation problem of rotating a rigid spacecraft from its current attitude Q to a desired attitude Qd is considered. The unitquaternion error Qe ∈ Qu is defined as Qe � Q[�] d[⊗][Q][ �] � q[T] e qe0 �[T] , which describes the discrepancy between the actual unit-quaternion Q and the desired unit-quaternion Qd. The kinematics represented by unit-quaternion error is described as [21] 

where ωe � ω − R�Qe�[T] ωd, R�Qe� is the unit-quaternion error Qe related rotation matrix [22] defined as R�Qe���q[2] e0[−][q][Te][q][e][�][I][3][�] 2qeq[T] e − 2qe0S�qe�, and ωd denotes the desired angular velocity. In this Note, because the rest-to-rest attitude reorientation problem is only considered, the desired angle velocity is ωd � 0, which yields ωe � ω. Therefore, the attitude error kinematics for rest-to-rest attitude reorientation maneuver in Eq. (4) can be rewritten as 

## B. Flexible Spacecraft Dynamics 

The dynamics for the attitude motion of a flexible spacecraft can be expressed by the following equations [8]: 

Taking time derivative of ψ leads to 

Define a new state variable ξ �� η[T] ψ[T] �[T] , then it follows from Eqs. (7) and (9) that 

Thus, the dynamics in Eq. (6) can be described as 

## C. Attitude Constraints Based on Unit Quaternion 

Suppose a half-cone angle strictly greater than θ should be maintained between the normalized boresight vector y of the spacecraft instrument and the normalized vector x pointing toward a certain celestial object, as shown in Fig. 1. This means that the cones with an apex angle of θ emanating from the bright objects vector should exclude the boresight vector of the sensitive onboard instruments during the reorientation maneuver. When the attitude of the spacecraft is determined by Q, the new boresight vector of the instrument in the inertial coordinates is 

where R�Q� is a rotation matrix given by R�Q���q[2] 0[−][q][T][q][�][I][3][�] 2qq[T] − 2q0S�q�. Then, the constraints can be expressedby thevector dot product 

Consequently, it follows from Eq. (13) that 

which can be further rewritten as 

Fig. 1 Demonstration of attitude constraint. 

� _ Jω _ � δ[T] η � −S�ω��Jω � δ[T] η�� τ (6) 

Suppose there are i constrained objectives associated with the jth onboard sensitive instrument in the spacecraft rotational space. Then, the spacecraft attitude Q ∈ Qu for which the boresight vector yj with respect to the ith celestial object should satisfy the following constraint 

where 

with 

Subsequently, to represent the possible attitude for the jth instrumentspecified as and the ith celestial object, a subset Qpji[of][Q][u][is] 

The angle θ[j] i[is the constraint angle about the direction of the][ i][th] object specified by xi for the jth instrument boresight vector yj. Without loss of generality, the domain of the angle θ[j] i[for all][ i][ and][ j][ is] restricted to be (0, π). 

For Q ∈ Qpji[, we have][ Q][T][M] i[j][Q][ −][cos][ θ][ <][ 0][, which can be further] written as Q[T] M[�][j] i[Q][ <][ 0][ with][M][�][j] i[�][M][j] i[−][cos][ θ][j] i[I] 4[.][For][θ][j] i[∈][�][0][;][ π][�][,] one has 

whereeigenvalueλmin�ofM[�][j] imatrix[�][and][λ] Mmax�[j] i[,][�][respectively.][M][�][j] i[�][denote][the][Then,][minimal][the][set][and][Q] p[j] i[maximal][can][be] equivalently represented as a convex set 

where M[~][j] i[is a positive-definite matrix. The proof of the boundedness] of Q[T] M[�][j] i[Q][ in Eq. (20) as well as its convex representation in Eq. (21)] can be established by applying Propositions 3 and 4 in [15]. 

## III. Velocity-Free Attitude Reorientation Controller Design 

A. Potential Function Design 

The potential function V�Q�: Qp → R, is defined as 

where the set Qp �fQ ∈ QujQ ∈ Qpji[g][(][i][ �][1][;][ 2][;][: : : ; n][and] j � 1; 2; : : : ; m) represents the possible attitudes of the spacecraft on which the boresight vector of the onboard instruments lie outside of the constrained attitude. The preceding potential function includes two terms multiplying together, that is, 

The first term kQd − Qk[2] represents an attractive potential field denoting the distance of the current attitude and the desired one, and the second term 

represents the specific celestial body constrained repulsive potential field. The parameter α is a design variable that is used to adjust the relative weighting between the attractive and repulsive potential. It is usually chosen to be a big value such that magnitude of control input is reasonable. 

- Lemma 1: The potential function in Eq. (22) has the following 

- properties: 

- 1) V�Qd�� 0. 

2) V�Q� > 0, for all Q ∈ Qp \ fQdg. 

3) ∇[2] V�Q� > 0 is positive definite for all Q ∈ Qp and Qd ∈ Qp. 

Proof: It is clear from the definition of the potential function in Eq. (22) that V�Qd�� 0 and V�Q� > 0 if Q ∈ Qp \ fQdg. The following analysis illustrates the derivation of the last property of the preceding lemma. 

Since Q[T] M[�][j] i[Q][ <][ 0][if][Q][ ∈][Q] p[,][it][is][equivalent][to][the][following] 

terms: 

where M[~][j] i[is assumed to be the form] 

andmeans thatμ is a positiveM[~][j] i[is a positive-definite matrix.] constant such that −λmax�M[�][j] i[��][μ][ >][ 0][,][which] 

Subsequently, if Q ∈ Qp, the potential function in Eq. (22) is equivalent to 

The gradient of V�Q� in Eq. (25) is given by 

Since kQd − Qk[2] � 2 − 2Q[T] d[Q][, it follows that] 

Then, it leads to 

Thus, the Hessian ∇[2] V�Q� is given by 

Based on the fact that kQd − Qk[2] � 2 − 2Q[T] d[Q][d][,][the][preceding] equation is further rewritten as 

Multiplying the last equation by Q[T] and Q from the left and the right, respectively, it leads to 

In view of Q[T] M[~][j] i[Q][ >][ 0][ and][ Q][T][Q] d[∈][�][−][1][;][ 1][�][, one has] 

where M[~][j] i[>][ 0][ and Eq. (23) is used.] From Eq. (30), it is clear that Q[T] ∇[2] VQ remains positive for all Q, Qd ∈ Qp. Therefore, the Hessian of V�Q� is positive definite, which means that the potential function in Eq. (22) is strictly convex. □ 

In summary, the preceding three properties show that the potential function V�Q� defined in Eq. (22) is smoothand strictly convexfor all Q ∈ Qp and Qd ∈ Qp, and it has a global minimum at Q � Qd. 

## B. Velocity-Free Controller Design 

Introducing an auxiliary unit quaternion 

withlater. We define the unit quaternionQ� �0��� q��0�[T] q�0�0��[T] ∈ QQu[~] , �andQ[�][�] Ω⊗ ∈QRe[3] ��willq~[T] beq~0given �[T] ∈ Qu describing the discrepancy between the unit-quaternion error Qe and the auxiliary unit-quaternion signal Q[�] . Then, we have 

which implies that 

The velocity-free attitude regulation controller is designed as 

where the operator Vec�⋅� denotes the vector part of �⋅�. Consider the following Lyapunov candidate: 

From ω[T] S�ω��J0ω � δ[T] ψ�� 0, it is obtained that 

where W is a symmetric positive-definite matrix satisfying the following Lyapunov equation: 

Because the matrix 

has eigenvalues with negative real parts, there always exists a symmetric positive-definite matric W such that Eq. (38) is verified [8]. Note that 

and substituting the control law (34) into Eq. (37) yields 

where matrix ϒ is given by 

By using the Schur complement lemma [23], and for the appropriate choice of matrix W, the matrix ϒ could be a positivedefinite matrix [24]. Then, it follows that 

If Ω � Γq ~ with Γ � Γ[T] > 0, one has 

Therefore, it is clear from Eq. (43) that Qe, Q[~] , ω, and V�Q� are bounded. Consequently, one can obtain that V[�] l is bounded. Hence, according to Barbalat’s Lemma, one can conclude that limt→∞ q~�t�� 0 and limt→∞q~0�t���1. Because Q[~] , Qe, Q, and Qd are bounded, it is clear that ∇V is bounded, which further leads to the boundedness of τ from control law (34). Because τ and Q[~] are �~ bounded, it is clear that Q is bounded, and consequently limt→∞ Q[_~] �t�� 0. From the definition of Q[_~] in Eq. (33), together limAgain, sincewitht→∞qfacts~0�t��� limthatt→1∞, q~it�limtis��teasy→0∞, one can obtain thatQ[_~] to�t��verify0, thatlimlimt→ lim∞t→q~∞t�→t���∞ωΩ − �0Ω, ��0, andand0. hence it is clear that limt→∞ω � 0, which in turn yields that τ � 0 from Eq. (6). Therefore, in view of control law (34) and 

where P is a positive-definite matrix. The time derivative of Vl is 

Table 1 Simulation parameters 

|Constrained zone|Constrained|Constrained|object|Angle, deg|
|---|---|---|---|---|
|CZ 1|�0.183|−0.983|−0.036�|30|
|CZ 2|�0|0.707|0.707�|25|
|CZ 3|�−0.853|0.436|−0.286�|25|
|CZ 4|�0.122|−0.140|−0.983�|20|

limt→∞ Q[_~] �t�� 0, we obtain that limt→∞∇V[�] �t�� 0 since Q ≠ 0. In addition, because the potential V�Q� is strictly convex, the following equivalence is ensured 

which consequently implies that limt→∞Q�t�� Qd. 

In summary, we have the following theorem. 

Theorem 1: The velocity-free controller (34), applied to the flexible spacecraft control systems expressed by Eqs. (3), (6), and (7) in the presence of attitude-constrained zones, guarantees that all closed-loop signals are bounded and that limt→∞ω � 0 and limt→∞Q�t�� Qd. 

## IV. Simulation Results 

To demonstrate the effectiveness and performance of the proposed controller, numerical simulation is performed to a flexible spacecraft in this section. It is assumed that the spacecraft carries a lightsensitive instrument with a fixedboresight inthe spacecraft bodyaxes aligned with the Z direction. The nominal main body inertia matrix of the spacecraft is 

and the coupling matrix between the elastic and rigid dynamics is given by [8] 

The natural frequencies are Λ1 � 0.7681, Λ2 � 1.1038, Λ3 � 1.8733, and Λ4 � 2.5496 rad∕s, and the corresponding damping ratios are ζ1 � 0.05607, ζ2 � 0.08620, ζ3 � 0.1283, and 

ζ4 � 0.2516. In the simulation, the spacecraft is retargeting its sensitive instrument (such as infrared telescopes or interferometers) while avoiding four celestial objects (such as sunlight or other bright objects) in the spacecraft reorientation configuration space. Four attitude-constrained zones are chosen without overlapping with each other. The details of the four attitude-constrained zones are given in Table 1, in which the normalized vectors pointing toward the corresponding celestial objects are expressed with respect to the inertial frame. Both initial and desired attitude are chosen such that they are out of the four attitude-constrained zones. The spacecraft is assumed to have the initial attitude Q�0�� � 0.329 0.659 −0.619 −0.2726 �[T] and initial angular velocity ω�0��� 0 0 0 �[T] rad∕s. The controller gains in control law (34) are chosen as k1 � 0.3Jp, k2 � 0.05Jp, and k3 � 0.005Jp, where Jp � diag�� 350 280 190 �� is a diagonal matrix in which the nonzero values are identical to the diagonal values of J0. Note that each of the controller gains in the simulation is selected as a multiplication of a constant and a diagonal matrix Jp containing diagonal elements of the inertia matrix. Although the control gains are not scale constants as defined in the original controller (34), the overall stability can still be guaranteed. The benefit of the gain modification is that it is easier to select proper gains to get a satisfactory control performance. For the auxiliary unit-quaternion defined in Eq. (31), the variable Ω � Γq ~ with Γ �� 1.5 1.5 1.5 �[T] . The variable α in the potential function is chosen as α � 50. 

To have a better illustration of the proposed method, the desired attitude that the flexible spacecraft is rotating to is Qd � � 0.38 −0.5 −0.5 −0.5963 �[T] , which is quite near to the second attitude-constrained zone. The target attitude is in a position at 34.58 deg from the center of the nearest attitude-constrained zone (i.e., CZ 2), which corresponds to the fact that the minimal angle between desired orientation and the boundary of the nearest forbidden cone is only 9.58 deg. As shown in Fig. 2, the pointing direction of the instrument generated by the proposed controller tries to reach the desired pointing direction from the initial pointing direction, but the second attitude-constrained cone lies in its way and prevents it from passing directly. To keep out of this attitudeconstrained cone, a large control toque in the opposite direction is produced, which keeps the pointing direction away from the constrained cone. The proposed controller generates a proper control action such that the third attitude-constrained cone is avoided and finally drives the instrument to reach the desired pointing direction. The corresponding simulation results for the quaternion error, angular velocity, control torque, and modal displacements are shown in Fig. 3, from which it is clear that acceptable control performance is achieved. In addition, two sharp increases of the control torque are observed in Fig. 3c because a large control torque is required to keep 

**a) Trajectory of sensitive instrument pointing direction in 3-D** 

Fig. 2 Case II: Trajectory of sensitive instrument pointing direction in three and two dimensions (3-D and 2-D) under the proposed control law (34). 

**c) Control torque d) Modal displacements** 

Fig. 3 Case II: Time responses of spacecraft attitude Qe, angular velocity ω, control torque τ, and modal displacements η under the proposed control law (34). 

Fig. 4 Case II: Trajectory of sensitive instrument pointing direction in 3-D and 2-D using controller in [20]. 

out of the attitude-constrained zones. A comparison with the technique discussed in [20] is also considered, where a velocity-free controller is developed without coping with attitude constraints. Figure 4 reports the simulation results using the controller in [20]. It is clear that the pointing trajectory goes into the second attitudeconstrained cone, which may cause damage to the sensitive onboard instruments. 

## V. Conclusions 

This Note focuses on the development of velocity-free attitude control laws for a rest-to-rest maneuver of flexible spacecraft under attitude constraints. The constrained spacecraft orientations are parameterized as a convex set using an intrinsic property of the attitude representation via unit quaternion. A new quadratic potential function is proposed to avoid the unwanted celestial objects by placing a large potential around the constrained directions. Based on such a potential function, a velocity-free attitude control law is developed to ensure the asymptotic stability of the closed-loop system by using auxiliary unit-quaternion dynamics. The performance of the proposed constrained attitude control algorithm has been discussed through numerical studies. In future work, angular velocity constraints should be investigated to reduce the maximal slew rate of flexible spacecraft. 

## References 

- [1] McInnes, C. R., “Large Angle Slew Maneuvers with Autonomous Sun Vector Avoidance,” Journal of Guidance, Control, and Dynamics, Vol. 17, No. 4, 1994, pp. 875–877. doi:10.2514/3.21283 

- [2] Wie, B., and Barba, P. M., “Quaternion Feedback for Spacecraft Large Angle Maneuvers,” Journal of Guidance, Control, and Dynamics, Vol. 8, No. 3, 1985, pp. 360–365. doi:10.2514/3.19988 

- [3] Boskovic, J. D., Li, S. M., and Mehra, R. K., “Robust Adaptive Variable Structure Control of Spacecraft Under Control Input Saturation,” Journal of Guidance, Control, and Dynamics, Vol. 24, No. 1, 2001, pp. 14–22. 

doi:10.2514/2.4704 

- [4] Shen, Q., Wang, D. W., Zhu, S. Q., and Poh, E. K., “Integral-Type Sliding Mode Fault-Tolerant Control for Attitude Stabilization of Spacecraft,” IEEE Transactions on Control Systems Technology, Vol. 23, No. 3, 2015, pp. 1131–1138. doi:10.1109/TCST.2014.2354260 

- [5] Shen, Q., Wang, D. W., Zhu, S. Q., and Poh, E. K., “Finite-Time FaultTolerant Attitude Stabilization with Actuator Saturation,” IEEE Transactions on Aerospace and Electronic Systems, Vol. 51, No. 3, 2015, pp. 2390–2405. doi:10.1109/TAES.2015.130725 

- [6] Kim, K., and Kim, Y., “Robust Backstepping Control for Slew Maneuver Using Nonlinear Tracking Function,” IEEE Transactions on Control Systems Technology, Vol. 11, No. 6, 2003, pp. 822–829. doi:10.1109/TCST.2003.815608 

- [7] Kristiansen, R., Nicklasson, P. J., and Gravdahl, J. T., “Satellite Attitude Control by Quaternion-Based Backstepping,” IEEE Transactions on Control Systems Technology, Vol. 17, No. 1, 2009, pp. 227–232. doi:10.1109/TCST.2008.924576 

- [8] Di Gennaro, S., “Output Stabilization of FlexibleSpacecraft with Active Vibration Suppression,” IEEE Transactions on Aerospace and Electronic Systems, Vol. 39, No. 3, 2003, pp. 747–759. doi:10.1109/TAES.2003.1238733 

- [9] Krstic, M., and Tsiotras, P., “Inverse Optimal Stabilization of a Rigid Spacecraft,” IEEE Transactions on Automatic Control, Vol. 44, No. 5, 1999, pp. 1042–1049. doi:10.1109/9.763225 

- [10] Xin,M.,andPan,H., “IndirectRobustControlof Spacecraftvia Optimal Control Solution,” IEEE Transactions on Aerospace and Electronic Systems, Vol. 48, No. 2, 2012, pp. 1798–1809. doi:10.1109/TAES.2012.6178102 

- [11] Hablani, H. B., “Attitude Commands Avoiding Bright Objects and Maintaining Communication with Ground Station,” Journal of Guidance, Control, and Dynamics, Vol. 22, No. 6, 1999, pp. 759–767. doi:10.2514/2.4469 

- [12] Frazzoli, E., Dahleh, M. A., Feron, E., and Kornfeld, R. P., “A Randomized Attitude Slew Planning Algorithm for Autonomous Spacecraft,” AIAA Guidance, Navigation, and Control Conference and Exhibit, AIAA, Reston, VA, 2001, pp. 1–8. doi:10.2514/6.2001-4155 

- [13] Kjellberg, H. C., and Lightsey, E. G., “Discretized Constrained Attitude Pathfinding and Control for Satellites,” Journal of Guidance, Control, and Dynamics, Vol. 36, No. 5, 2013, pp. 1301–1309. doi:10.2514/1.60189 

- [14] De Angelis, E. L., Giulietti, F., and Avanzini, G., “Single-Axis Pointing of Underactuated Spacecraft in the Presence of Path Constraints,” Journal of Guidance, Control, and Dynamics, Vol. 38, No. 1, 2015, pp. 143–147. 

doi:10.2514/1.G000121 

- [15] Lee, U., and Mesbahi, M., “Feedback Control for Spacecraft Reorientation Under Attitude Constraints Via Convex Potentials,” IEEE Transactions on Aerospace and Electronic Systems, Vol. 50, No. 4, 2014, pp. 2578–2592. 

doi:10.1109/TAES.2014.120240 

- [16] Avanzini, G., Radice, G., and Ali, I., “Potential Approach for Constrained Autonomous Manoeuvres of a Spacecraft Equipped with a Cluster of Control Moment Gyroscopes,” Journal of Aerospace Engineering, Vol. 223, No. 3, 2009, pp. 285–296. doi:10.1243/09544100JAERO375 

- [17] McInnes, C. R., “Non-Linear Control for Large Angle Attitude Slew Maneuver,” Third ESA Symposium on Spacecraft Guidance, Navigation, and Control, European Space Research and Technology Center, Noordwijk, The Netherlands, 1996, pp. 543–548. doi:1997ESASP.381.543M 

- [18] Hamilton, W. R., “On Quaternions; or on a New System of Imaginaries in Algebra,” Philosophical Magazine, Series 3, Vol. 25 1844, No. 163, 2009, pp. 10–13. 

doi:10.1080/14786444408644923 

- [19] Rafal, W., and Piotr, K., “Slew Maneuver Control for Spacecraft Equipped with Star Camera and Reaction Wheels,” Control Engineering Practice, Vol. 13, No. 3, 2005, pp. 349–356. doi:10.1016/j.conengprac.2003.12.006 

- [20] Tayebi, A., “Unit Quaternion-Based Output Feedback for the Attitude Tracking Problem,” IEEE Transactions on Automatic Control, Vol. 53, No. 6, 2008, pp. 1516–1520. doi:10.1109/TAC.2008.927789 

- [21] Shuster, M. D., “A Survey of Attitude Representation,” Journal of Astronautical Sciences, Vol. 41, No. 4, 1993, pp. 439–517. doi:1993JAnSc.41.439S 

- [22] Sidi, M. J., Spacecraft Dynamics and Control, Cambridge Univ. Press, Cambridge, England, U.K., 1997, pp. 88–111. 

- [23] Ben-Tal, A., El Ghaoui, L., and Nemirovksi, A., Robust Optimization, Princeton Univ. Press, Princeton, NJ, 2009, p. 163. 

- [24] Xiao, B., Hu, Q. L., and Zhang, Y. M., “Fault-Tolerant Attitude Control for Flexible Spacecraft Without Angular Velocity Magnitude Measurement,” Journal of Guidance, Control, and Dynamics, Vol. 34, No. 5, 2011, pp. 1556–1561. doi:10.2514/1.51148 
