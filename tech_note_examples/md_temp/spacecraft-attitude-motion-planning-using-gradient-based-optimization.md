JOURNAL OF GUIDANCE, CONTROL, AND DYNAMICS Vol. 43, No. 1, January 2020 

**==> picture [46 x 46] intentionally omitted <==**

## Engineering Notes 

## Spacecraft Attitude Motion Planning Using Gradient-Based Optimization 

Fabio Celani[∗] 

Sapienza University of Rome, 00138 Rome, Italy 

and Dennis Lucarelli[†] American University, Washington, D.C. 20016 

DOI: 10.2514/1.G004531 

space is the Lie group SO(3), its solution can be obtained by adapting a recent method for control synthesis on Lie groups, which known as “gradient ascent in function space” [11]. This method, originally used for optimal control of quantum mechanical systems, first expresses the controls as superpositions over a set of basis functions and exploits a functional derivative on the set of basis function coefficients to construct a gradient-based optimization problem. As in Ref. [11], the basis functions are chosen as the discrete analog of the prolate spheroidal wave functions, as developed by Slepian and Pollak in Refs. [12,13]. These functions, used widely in signal processing and spectral analysis, are optimally concentrated in time and bandwidth; and controls constructed from them are smoothly varying and can be parameterized to vanish at their endpoints, ensuring a restto-rest maneuver. In the second step, known as motion planning, the physical control torque is simply determined by the use of inverse attitude dynamics. As in Ref. [9], a time scaling is introduced to reduce the torque amplitude to within the allowed limits. 

## I. Introduction 

A TTITUDE motion planning is necessary in mission scenarios inwhich a spacecraft must perform large angle maneuvers with the additional requirement that sensitive instruments must not point to bright celestial objects. These so-called “keepout cones” define constraints that must be satisfied along the instrument trajectory. This research presents a control synthesis method for constructing an appropriate control torque that achieves the desired rest-to-rest maneuver and ensures that the keepout cones are avoided. 

In this work, spacecraft attitude is globally represented on the three-dimensional special orthogonal group SO(3). Performing motion planning on SO(3) carries benefits over representations such – as Euler angles [1,2] and quaternions [3 6]. Euler angles are defined only locally and exhibit kinematic singularities that can limit the width of the maneuvers. Quaternions are free of singularities and are often used in spacecraft attitude motion planning, but they present ambiguity in representing attitude because the unit sphere in R[4] double covers SO(3). Thus, because boundary conditions for the spacecraft attitude do not have a unique representation in quaternions, a quaternion-based motion planning may exhibit the unwinding behavior [7]. The proposed method presents the following advantages as compared to other approaches for spacecraft motion planning on SO(3). It is simpler because it does not require either randomization or discretization tools as in Ref. [8] or visual inspection to perform path planning as in Ref. [9]. Moreover, it naturally handles limits on the control torque amplitude, unlike Ref. [10]. From a practical point of view, the control torque resulting from the proposed approach is continuously differentiable and vanishes at its endpoints. Thus, it is easier to implement on real spacecraft than time-optimal control torques that may not vanish at the endpoints and are sometimes discontinuous during the maneuver [3,5,6]. 

The proposed method follows a two-step procedure as described in Ref. [9]. In the first, path planning is performed to determine an appropriate time behavior for the angular velocity so that the spacecraft is reoriented to the desired attitude while avoiding exclusion cones. Because, in the path-planning problem, the state 

Received 17 April 2019; revision received 27 August 2019; accepted for publication 29 August 2019; published online 30 September 2019. Copyright © 2019 by Fabio Celani and Dennis Lucarelli. Published by the American Institute of Aeronautics and Astronautics, Inc., with permission. All requests for copying and permission to reprint should be submitted to CCC at www. copyright.com; employ the eISSN 1533-3884 to initiateyour request. See also AIAA Rights and Permissions www.aiaa.org/randp. 

*Assistant Professor, School of Aerospace Engineering, Via Salaria 851. 

## II. Problem Statement 

In the spacecraft attitude motion planning problem on SO(3), the initial attitude Ri ∈ SO�3� and the desired final attitude Rf ∈ SO�3� are given, and both the initial angular velocity and the desired final angular velocity must be zero (rest-to-rest maneuver). The attitude R�t� ∈ SO�3� is subject to the attitude kinematics R[_] �t�� R�t�Ω�t�, where 

**==> picture [190 x 9] intentionally omitted <==**

in which ω1, ω2, and ω3 are the components of the spacecraft angular velocity along the body axes; and matrices A1, A2, and A3 form the basis for the Lie algebra corresponding to SO(3) (see Ref. [9]). The relation between ω �� ω1 ω2 ω3 �[T] and the control torque resolved in body frame T is given by the attitude dynamics J _ω � ω × Jω � T, in which J is the spacecraft inertia matrix. Disturbance torques are neglected. Denote with T[�] the maximum amplitude of Tj j � 1; 2; 3 dueto actuator constraints. Thespacecraft is equipped with an onboard sensor for which the pointing direction in body coordinates is given by unit vector r. There are C undesired pointing directions for the sensor that are specified in inertial coordinates by unit vectors wi, i � 1; : : : ; C. For example, r can be the pointing direction in body coordinates of an onboard optical sensor, and wi is the inertial direction of a bright celestial object. It is required that the boresight of the sensor avoids inertial direction wi with a minimum offset angle of 0 < θi < 90 deg. Thus, the following constraints are introduced: 

**==> picture [189 x 10] intentionally omitted <==**

Given the initial conditions R�0�� Ri, ω�0�� 0, the objective is to determine a torque input T�t� defined over a finite interval � 0 tf � that fulfills the amplitude constraint jTj�t�j ≤ T[�] and is such that the corresponding attitude R�t� and angular velocity ω�t� satisfy the final conditions R�tf�� Rf, ω�tf�� 0, and the pointing constraints [Eq. (2)]. 

## III. Path Planning Using Gradient-Based Optimization 

Path planning determines an appropriate time behavior for the angular velocity so that the spacecraft is reoriented to the desired attitude while avoiding exclusion cones. Thus, in this first phase, only attitude kinematics are considered, and the angular velocity ω �� ω1 ω2 ω3 �[T] is treated as the control input. As in Ref. [9], a 

> †Research Associate Professor, Department of Physics, 4400 Massachusetts Ave. NW. 

140 

141 

J. GUIDANCE, VOL. 43, NO. 1: 

ENGINEERING NOTES 

normalized time of τ � t∕tf is adopted. Clearly, 0 ≤ τ ≤ 1. From a numerical point of view, this is equivalent to setting the final time tf � 1 so that τ � t. In this Note, path planning is performed by first expressing the angular velocity as follows: 

**==> picture [187 x 25] intentionally omitted <==**

for a set of M basis functions vk�τ�, k � 1; : : : ; M. The basis functions must fulfill the following endpoint conditions vk�0�� vk�1�� 0 so that ω�0�� ω�1�� 0 as required. If the basis functions fulfill the following additional endpoint conditions 

**==> picture [81 x 20] intentionally omitted <==**

constraints in Eqs. (5) and (6) with respect to the basis function weights αjk must be provided. First, note that αjk affect the constraints through R�τl� only. Define 

**==> picture [113 x 22] intentionally omitted <==**

Then, the following holds: 

**==> picture [241 x 69] intentionally omitted <==**

then ω also satisfies 

**==> picture [79 x 20] intentionally omitted <==**

Consequently, as will be made clear in the next section, the control torque possesses the following property: T�0�� T�1�� 0. 

The path-planning problem is formulated as an optimization problem in which the decisionvariables are theweights αjk in Eq. (3). The objective function to be minimized is chosen as the following continuously differentiable function: 

**==> picture [144 x 26] intentionally omitted <==**

to reduce, to a certain extent, the amplitude of the angular velocity ω�τ� [see Eq. (3)]. This choice reduces the spacecraft maneuvering time tf, as will be described in the following section. Achieving the desired final attitude is enforced through the following equality constraint: 

**==> picture [155 x 12] intentionally omitted <==**

where tr denotes the trace of a matrix. In addition, exclusion cones are avoided by adding the inequality constraints [Eq. (2)]. The optimization problem is solved numerically by adopting the following approach. Discretize the interval [0 1] into N equal segments defining Δτ � 1∕N and τl ��l − 1�Δτ, l � 1; : : : ; N � 1. Inequality constraints are enforced only at the discrete times τl obtaining 

**==> picture [233 x 22] intentionally omitted <==**

Because a variation in a weight αjk affects the variable ωj�τ� at all times [and, in particular, affects ωj�τq�], we have the following: 

**==> picture [240 x 37] intentionally omitted <==**

The partial derivative of the infinitesimal rotation P�τq� can be computed as follows. Let λi i � 1; 2; 3 be the eigenvalues of Ω�τq� [see Eq. (1)] and ui ∈ R[3] i � 1; 2; 3 be the corresponding eigenvectors with a unit norm. Because Ω�τq� is of dimension three, it possesses a zero eigenvalue and two imaginary conjugate eigenvalues. Moreover, the 3 × 3 complex matrix U ≜ � u1 u2 u3 � is a unitary matrix because it satisfies U[†] U � UU[†] � I3×3, where U[†] denotes the conjugate transpose of U and I3×3 is the threedimensional identity matrix. The following result can then be stated. Proposition 1: It holds that 

**==> picture [65 x 23] intentionally omitted <==**

where E is a 3 × 3 complex matrix defined by 

**==> picture [196 x 26] intentionally omitted <==**

Proof: The proof can be obtained by adapting a result presented in section 3 of Ref. [14]. The details can be found in Ref. [15]. □ 

Thus, analytical expressions for the gradients of constraints (5) and – (6) based on approximation (7) can be obtained by using Eqs. (8 10). 

Moreover, R�τl� is computed through the following approximation. Consider the infinitesimal rotations 

## IV. Motion Planning 

**==> picture [207 x 10] intentionally omitted <==**

The solution to the attitude kinematics is approximated by the following product: 

**==> picture [222 x 41] intentionally omitted <==**

This approximate solution represents the continuous exact solution of the attitude kinematics when ω�τ� is approximated through a zero-order-hold operation. The approximation in Eq. (7) is also employed to compute R�1� appearing in the equality constraint in Eq. (5). To solve the optimization problem using steepest descent methods, the analytical expressions for the gradients of the 

The output of the path-planning phase is the time behavior ω[�] �τ�, 0 ≤ τ ≤ 1, which fulfills ω[�] �0�� ω[�] �1�� 0 and, possibly, 

**==> picture [87 x 19] intentionally omitted <==**

and it possesses the following property: let R[�] �τ� be the corresponding time behavior for the attitude with initial condition R[�] �0�� Ri. Clearly, R[�] �τ� fulfills the pointing constraints in Eq. (2), as well as the final condition R[�] �1�� Rf if the number of time segments N is large enough so that the approximation in Eq. (7) is sufficiently accurate. As in Ref. [9], the required torque can be obtained from the attitude dynamics as 

**==> picture [222 x 20] intentionally omitted <==**

142 

J. GUIDANCE, VOL. 43, NO. 1: ENGINEERING NOTES 

The control torque T[�] �τ� may not fulfill the amplitude constraints jT[�] j[�][τ][�j][ ≤][T][�][ 0][ ≤][τ][≤][1][. In that case, as in Ref. [9], the time scaling of] t � tfτ is performed in which tf > 1 is chosen to reduce the speed at which the path in SO(3) is traversed, and it is consequently chosen to have the torque amplitude within the prescribed limits. Specifically, consider R�t�� R[�] �t∕tf� 0 ≤ t ≤ tf. It is immediately seen that the angular velocity in the scaled time t is givenby ω�t�� 1∕tfω[�] �t∕tf�, 0 ≤ t ≤ tf. Moreover, it is easy to obtain that the input torque is equal to 

**==> picture [181 x 24] intentionally omitted <==**

Thus, tf must be selected larger than one so that the torque T�t� satisfies the amplitude constraint jTj�t�j ≤ T[�] 0 ≤ t ≤ tf. On the other hand, if the amplitude of T[�] �τ� is below its limits, then tf can be reduced to a value less than one. It appears from Eq. (11) that lowering the amplitude of ω[�] leads to a reduction of the amplitude of T[�] . Consequently, using Eqs. (11) and (12) we obtain that the smaller the amplitude of ω[�] the smaller the time tf that ensures the fulllment of the amplitude constraint on T�t�. Because adopting the cost function in Eq. (4) is approximately equivalent to minimizing the amplitude of ω[�] , such a cost function leads to an approximate minimization of the maneuvering time tf. 

## V. Case Studies 

We demonstrate the proposed approach through two case studies. The first case study has been formulated in Refs. [6,16], and it is inspired by the following maneuver performed by the Swift spacecraft. The satellite must perform a fast reorientation maneuver to point two telescopes at a desired gamma-ray burst. The spacecraft is equipped with a burst alert telescope that first senses the gammaray burst, and the satellite then must perform a slew to allow the x ray and UV/optical instruments to capture the afterglow of the event, which quickly fades. To prevent damage to the telescopes, their common axis must not enter three keepout cones with central axes pointing to the sun, the Earth, and the moon and appropriate offset angles. As an example, assume that the spacecraft has to perform a rotation of 3∕4π rad about its z axis. Thus, setting the initial attitude as Ri � I3×3, the desired final attitude is equal to Rf � exp�3∕4πA3�. The telescope axis has coordinates of r �� 1 0 0 �[T] in the body frame. The three keepout cones are specified as follows: 1) sun cone with inertial direction of w1 �� 0.5 0.866 0 �[T] and minimum offset angle of θ1 � 47 deg; 2) Earth cone with inertial direction of w2 �� 0 0 −1 �[T] and minimum offset angle of θ2 � 33 deg; and 3) moon cone with inertial direction of w3 � � 0.1795 0.3109 0.9333 �[T] and minimum offset angle of θ3 � 23 deg. 

There isa gap of 10 deg between the sun andEarth conesbut no gap between the sun and moon cones (see Fig. 1). Path planning is performed by solving the optimization problem formulated in Sec. III through the numerical approach previously discussed. Consequently, it is enough to select samples vk�τl� instead of the whole continuoustime basis functions vk�τ� because only those samples appear in the equality and inequality constraints. In this example, vk�τl� are chosen as the so-called Slepian sequences [13]. These sequences are parameterized by their length N and the half-bandwidth parameter W ∈ � 0 0.5 �. In this case study, the first M � 4 Slepian sequences with N � 500 and W � 0.015 have been considered. The values of M, N, and W have been determined by trial and error. The corresponding time behaviors are reported in Fig. 2 and show that vk�0� ≃ 0, �dvk∕dτ��0� ≃ 0, vk�1� ≃ 0, and �dvk∕dτ��1� ≃ 0. As a result, by Eq. (3), the following hold: ω�0� ≃ 0, �dω∕dτ��0� ≃ 0, ω�1� ≃ 0, �dω∕dτ��1� ≃ 0, ensuring that a rest-to-rest maneuver is obtained. To determine for the values of the weights αjk j � 1; : : : ; 3 k � 1; : : : ; 4, the optimization problem is solved numerically using a local gradient-based interior point method [17] implemented in scientific Python [18]. Note that the cost function in Eq. (4) naturally leads to setting αjk � 0 as the initial guess for the interior point algorithm. The outcome of the path-planning step is given by the time 

**==> picture [201 x 207] intentionally omitted <==**

Fig. 1 Case study 1: path of sensitivedirection on the unit sphere (yellow curve), exclusioncones,initial sensitivedirection(red arrow), and desired final sensitive direction (green arrow). 

**==> picture [219 x 174] intentionally omitted <==**

Fig. 2 Samples of basis functions vk�τl�, k � 1; : : : ; 4, and l � 1; : : : ; 500. 

samples of the angular velocity in normalized time of ω[�] �τl�, l � 1; : : : ; 500. The CPU time for solving the optimization problem using a PC with an Intel Core i7 at 1.7 GHz and 4 GB of RAM is 20 min. Samples of the torque expressed in normalized time T[�] �τl� can be obtained by using Eq. (11) through the adoption of a finite difference approximation for computing dω[�] ∕dτ. As in Ref. [6], consider an isoinertial spacecraft so that J � J0I3×3. By Eq. (12), samples of the physical control torque are obtained as follows: 

**==> picture [145 x 23] intentionally omitted <==**

where tf is the physical final time; and tl � tfτl. Time tf is chosen so that 

**==> picture [208 x 12] intentionally omitted <==**

Introduce time unitLet T[�][�] ∕J�������������0 � max TUj;l � jT[�] jp[�][τ] �����������Jl0[�] ∕[∕] T[J][�] 0; then, it is easy to see that thevalue[j][. It turns out that][T][�][�][∕][J] 0[�][40.53][.] � tf � TUqT[�] ∕J0 � 6.26 TUs guarantees that Eq. (13) is fulfilled, as confirmed by Fig. 3. The continuous-time control torque T�t� is 

143 

J. GUIDANCE, VOL. 43, NO. 1: ENGINEERING NOTES 

**==> picture [231 x 180] intentionally omitted <==**

Fig. 3 Case study 1: samples T�tl�, l � 1; : : : ; 500. 

**==> picture [231 x 188] intentionally omitted <==**

Fig. 5 Case study 1: pointing constraints. 

**==> picture [231 x 181] intentionally omitted <==**

Fig. 4 Case study 1: principal angle between Rf and R�t�. 

endpoints (see Fig. 3), making it is easier to implement on real spacecraft. 

The method is applied to a second case study, which was originally presented in Ref. [5]. Consider a satellite for Earth observation in low Earth orbit. A typical maneuver for the satellite to perform Earth observation is switching between pointing to one side of the ground track to pointing to its opposite side. Such a maneuver corresponds to a roll rotation. Assume that the required roll rotation is 60 deg wide. Thus, by setting the initial attitude as Ri � I3×3, the desired final attitude is equal to Rf � exp�π∕3A1�. The satellite is equippedwith a star tracker that must avoid sun and moon directions with prescribed offset angles during the maneuver. The pointing direction of the star tracker sensor is expressed in body coordinates by the following unit vector: r �� 0 −0.62 −0.79 �[T] . The two keepout cones are specified as follows: 1) sun cone with inertial direction of w1 � � −0.58 −0.08 −0.81 �[T] and a minimum offset angle of θ1 � 40 deg; and 2) moon cone with inertial direction of w2 � � 0.40 −0.13 −0.90 �[T] and a minimum offset angle of θ3 � 17 deg. 

The satellite inertia matrix is given by 

then simply obtained from the samples T�tl� through a zero-orderhold operation. Figure 3 shows that, from a practical point of view, T�t� can be considered continuously differentiable and vanishing at its endpoints. Both properties are consequences of our choice for the basis functions vk�τl� (see Fig. 2). Compared to time-optimal control torques that may not vanish at their endpoints and are sometimes discontinuous during the maneuver [3,5,6], the smooth torques obtained with our method facilitate implementation on real spacecraft. To validate the effectiveness of the obtained input torque, the attitude kinematics and dynamics with the appropriate initial conditions R�0�� I3×3 ω�0�� 0 are integrated numerically using a method that preserves the orthonormality of R, and the following results are obtained. The evolution of the principal angle between Rf and R�t� is displayed in Fig. 4. The obtained final attitude R�tf� satisfies 3 − tr� R[T] f R�tf��� 7.73 × 10[−][10] and kω�tf�k � 9.00 × 10[−][7] . The time behaviors of ci�t�� r[T] R�t�[T] wi − cos θi, i � 1; 2; 3 are shown in Fig. 5, confirming that the three pointing constraints are fulfilled. The path of the sensitive direction is displayed in Fig. 1. By inspection, the proposed method leads to a solution that apparently minimizes the length of the path. A different approach to solve the same attitude motion planning problem was presented in Ref. [6]. This approach uses quaternions for attitude representation, the maneuvering time is chosen as objective function, and a particle swarm is employed to find the time-optimal solution. Using such a method, a maneuvering time of about 3.5 TUs is achieved. The method proposed here does not explicitly minimize time and performs the maneuver in 6.26 TUs, which is substantially longer. However, our approach leads to a control torque that vanishes at its 

**==> picture [139 x 10] intentionally omitted <==**

The maximum torque along each body axis is equal to Tmax � 0.25 N∕m. 

The path-planning step is performed with the same approach as in case study 1. Specifically, samples vk�τl� are chosen again as the socalled Slepian sequences [13], and the first M � 4 Slepian sequences with N � 500 and W � 0.015 have been considered (see Fig. 2). The optimization problem is solved numerically using the same local gradient-based interior point method considered in case study 1. The CPU time for solving the optimization problem using the same PC as in the first case study is 47 min. Next, the motion planning step is performed by obtaining the maneuvering time of tf � 473 s, and samples of the control torque are reported in Fig. 6. The plots in the figure show that, as in case study 1, from a practical point of view, the control torque can be considered continuously differentiable and vanishing at its endpoints. To validate the effectiveness of the obtained input torque, the attitude kinematics and dynamics with the appropriate initial conditions R�0�� I3×3 and ω�0�� 0 are integrated numerically using a method that preserves the orthonormality of R, and the following results are obtained. The evolution of the principal angle between Rf and R�t� is displayed in Fig. 7. The obtained final attitude R�tf� satisfies 3− tr� R[T] f R�tf��� 8.06 × 10[−][10] and kω�tf�k � 6.12 × 10[−][8] . The time behaviors of ci�t�� r[T] R�t�[T] wi − cos θi, i � 1; 2, are shown in Fig. 8, confirming that the two pointing constraints are fulfilled. The path of the sensitive direction is displayed in Fig. 9, which shows that the proposed method leads to a solution that apparently minimizes the 

144 

J. GUIDANCE, VOL. 43, NO. 1: ENGINEERING NOTES 

**==> picture [231 x 180] intentionally omitted <==**

Fig. 6 Case study 2: samples T�tl�, l � 1; : : : ; 500. 

**==> picture [235 x 182] intentionally omitted <==**

Fig. 7 Case study 2: principal angle between Rf and R�t�. 

**==> picture [229 x 180] intentionally omitted <==**

Fig. 8 Case study 2: pointing constraints. 

length of the path. In Ref. [5], the same attitude motion planning problem is solved using a quaternion representation of the attitude and time-optimal approach based on particle swarm optimization. The resulting maneuver time was reported to be about 230 s, which was significantly shorter than the one obtained here. However, the corresponding control torques (figure 9 of Ref. [5]) do not vanish at the endpoints and, in one instance, exhibit discontinuities. 

**==> picture [213 x 209] intentionally omitted <==**

Fig. 9 Case study 2: path of sensitive direction on the unit sphere (green curve), exclusioncones,initial sensitivedirection(red arrow), and desired final sensitive direction (green arrow). 

## VI. Conclusions 

The spacecraft attitude motion planning approach presented in this work provides a systematic method for performing rest-to-rest maneuvers, taking into account multiple pointing constraints. It possesses the feature of representing the attitude on the special orthogonal group SO(3), thus avoiding singularities and ambiguities associated with other attitude representations. Compared to other methods that are based on the same attitude representation, the proposed method is simpler and more systematic. Moreover, it can provide a control torque that vanishes at its endpoints, which is simpler to implement than control torques determined through a time-optimal approach. 

## Acknowledgments 

D. Lucarelli gratefully acknowledges financial support from the Visiting Professor Program 2018 at Sapienza University of Rome. A previous version of this Note was presented at the 29th AAS/AIAA Space Flight Mechanics Meeting in Kaanapali, Hawaii, 13–17 January 2019. 

## References 

- [1] Hablani, H., “Attitude Commands Avoiding Bright Objects and Maintaining Communication with Ground Station,” Journal of Guidance, Control, and Dynamics, Vol. 22, No. 6, 1999, pp. 759–767. doi:10.2514/2.4469 

- [2] McInnes, C., “Large Angle Slew Maneuvers with Autonomous Sun Vector Avoidance,” Journal of Guidance, Control, and Dynamics, Vol. 17, No. 4, 1994, pp. 875–877. doi:10.2514/3.21283 

- [3] Boyarko, G., Romano, M., and Yakimenko, O., “Time-Optimal Reorientation of a Spacecraft Using an Inverse Dynamics Optimization Method,” Journal of Guidance, Control, and Dynamics, Vol. 34, No. 4, 2011, pp. 1197–1208. doi:10.2514/1.49449 

- [4] Kjellberg, H., and Lightsey, E., “Discretized Constrained Attitude Pathfinding and Control for Satellites,” Journal of Guidance, Control, and Dynamics, Vol. 36, No. 5, 2013, pp. 1301–1309. doi:10.2514/1.60189 

- [5] Spiller, D., Ansalone, L., and Curti, F., “Particle Swarm Optimization for Time-Optimal Spacecraft Reorientation with Keep-Out Cones,” Journal of Guidance, Control, and Dynamics, Vol. 39, No. 2, 2016, pp. 312–325. doi:10.2514/1.G001228 

- [6] Melton, R., “Differential Evolution/Particle Swarm Optimizer for Constrained Slew Maneuvers,” Acta Astronautica, Vol. 148, July 2018, 

145 

J. GUIDANCE, VOL. 43, NO. 1: ENGINEERING NOTES 

pp. 246–259. doi:10.1016/j.actaastro.2018.04.045 

- [7] Chaturvedi, N. A., Sanyal, A. K., and McClamroch, N. H., “Rigid-Body Attitude Control,” IEEE Control Systems Magazine, Vol. 31, No. 3, 2011, pp. 30–51. doi:10.1109/MCS.2011.940459 

- [8] Frazzoli, E., Dahleh, M., Feron, E., and Kornfeld, R., “A Randomized Attitude Slew Planning Algorithm for Autonomous Spacecraft,” AIAA Guidance, Navigation, and Control Conference and Exhibit, AIAA Paper 2001-4155, 2001. 

- [9] Biggs, J., and Colley, L., “Geometric Attitude Motion Planning for Spacecraft with Pointing and Actuator Constraints,” Journal of Guidance, Control, and Dynamics, Vol. 39, No. 7, 2016, pp. 1672– 1677. 

doi:10.2514/1.G001514 

- [10] Lee, T., “Geometric Tracking Control of the Attitude Dynamics of a Rigid Body on SO(3),” Proceedings of the 2011 American Control Conference, IEEE Publ., Piscataway, NJ, 2011, pp. 1200–1205. 

- [11] Lucarelli, D., “Quantum Optimal Control via Gradient Ascent in Function Space and the Time-Bandwidth Quantum Speed Limit,” Physical Review A: General Physics, Vol. 97, No. 6, 2018, Paper 062346. 

doi:10.1103/PhysRevA.97.062346 

- [12] Slepian, D., and Pollak, H. O., “Prolate Spheroidal Wave Functions, Fourier Analysis and Uncertainty—I,” Bell System Technical Journal, 

Vol. 40, No. 1, 1961, pp. 43–63. 

doi:10.1002/bltj.1961.40.issue-1 

- [13] Slepian, D., “Prolate Spheroidal Wave Functions, Fourier Analysis, and Uncertainty—V: The Discrete Case,” Bell System Technical Journal, Vol. 57, No. 5, 1978, pp. 1371–1430. doi:10.1002/bltj.1978.57.issue-5 

- [14] Machnes, S., Sander, U., Glaser, S., De Fouquières, P., Gruslys, A., Schirmer, S., and Schulte-Herbrüggen, T., “Comparing, Optimizing, and Benchmarking Quantum-Control Algorithms in a Unifying Programming Framework,” Physical Review A: General Physics, Vol. 84, No. 2, 2011, Paper 022305. doi:10.1103/PhysRevA.84.022305 

- [15] Celani, F., and Lucarelli, D., “Spacecraft Attitude Motion Planning on SO(3)UsingGradient-BasedOptimization,” AdvancesintheAstronautical Sciences, Univelt, Escondido, CA, Vol. 168, 2019, pp. 1397–1409. 

- [16] Pontani, M., and Melton, R., “Heuristic Optimization of Satellite Reorientation Maneuvers,” AIAA/AAS Astrodynamics Specialist Conference, AIAA Paper 2016-5581, 2016. 

- [17] Byrd, R. H., Hribar, M. E., and Nocedal, J., “An Interior Point Algorithm for Large-Scale Nonlinear Programming,” SIAM Journal on Optimization, Vol. 9, No. 4, 1999, pp. 877–900. doi:10.1137/S1052623497325107 

- [18] Oliphant, T. E., “Python for Scientific Computing,” Computing in Science Engineering, Vol. 9, No. 3, 2007, pp. 10–20. doi:10.1109/MCSE.2007.58 

