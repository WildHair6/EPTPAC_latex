JOURNAL OF GUIDANCE, CONTROL, AND DYNAMICS Vol. 49, No. 6, June 2026 

**==> picture [46 x 46] intentionally omitted <==**

## Technical Notes 

## Velocity-Free Saturated PredefinedTime Attitude Coordination Control for Spacecraft Formation Flying 

Chao Xiao,[∗] Yong Guo,[†] Lihao Wang,[∗] Aijun Li,[‡] and Changqing Wang[‡] Northwestern Polytechnical University, 710129 Xi’an, People’s Republic of China 

https://doi.org/10.2514/1.G009674 

## I. Introduction 

I Nspacecraft formation flying (SFF) technology in the space field,RECENT decades, with the continuous in-depth study of its application in on-orbit servicing [1], earth observation [2], asteroid exploration [3], and other aspects has attracted extensive attention. The core concept of SFF is to deploy a cluster of miniature spacecraft to imitate the function of a single large spacecraft, in which the attitude coordination control is a crucial component [4]. Consequently, numerous scholars have a particular interest in attitude coordination control for SFF and put forward a series of control schemes such as asymptotic stable control [5], finite-time control [6], and fixed-time control [7]. Notice that all the aforementioned attitude control studies are premised on the completion of position coordination control for SFF, such as that presented in [8]. In practical aerospace engineering applications, the settling time of formation spacecraft is an important performance index. However, asymptotic stability control yields a settling time upper bound that tends to infinity; finite-time control features a settling time upper bound dependent on initial system states; and fixed-time control involves a settling time upper bound formulated with numerous parameters, which complicates its calculation and limits its engineering applicability. 

Aiming at attaining the exact upper bound on settling time, scholars have presented multiple predefined-time control schemes for spacecraft, such as the prescribed performance function method [9] and Lyapunov function method [10]. Among them, the former drives the system state to converge in predefined time via adopting a prescribed performance function that converges within predefined time, which inevitably increases the complexity [11]. The latter achieves predefined-time convergence by introducing a Lyapunov function into controller design but is prone to singularity, which means the control input or sliding mode surface tends to infinity as the system state approaches zero [12]. In particular, both of them can generate an explicit expression of the upper bound on settling time. Moreover, compared with other methods mentioned above, the predefined-time control can provide an upper bound on settling time regulated by only one parameter, which greatly facilitates engineering applications. 

Received 19 October 2025; accepted for publication 29 January 2026; published online 27 February 2026. Copyright © 2026 by the American Institute of Aeronautics and Astronautics, Inc. All rights reserved. All requests for copying and permission to reprint should be submitted to CCC at www.copyright.com; employ the eISSN 1533-3884 to initiate your request. See also AIAA Rights and Permissions https://aiaa.org/publications/ publish-with-aiaa/rights-and-permissions/. 

- *Ph.D. Graduate Student, School of Automation, Shaanxi Province. 

†Associate Professor, School of Automation, Shaanxi Province; guoyong@ nwpu.edu.cn (Corresponding Author). 

> ‡Professor, School of Automation, Shaanxi Province. 

In engineering practice, the actuator has the input saturation constraint due to its inherent physical limitation, but it has not been considered in the above literatures, which will impact the dynamic performance and even cause system instability [13]. With the objective to cope with the actuator saturation, scholars have developed a series of saturated control schemes for SFF. For instance, an improved saturated super-twisting algorithm is constructed by means of the saturation function, and then a finite-time saturated control method is presented for SFF via combining the hyperbolic tangent function [14]. Additionally, a fixed-time saturated control method is advised for SFF by combining the saturation function with a saturated nonsingular sliding mode manifold based on the inverse tangent function [15]. Unfortunately, the aforementioned methods cannot address the constraints of actuator saturation and predefined-time convergence simultaneously. In view of this, scholars have suggested several predefined-time saturated control schemes to deal with the actuator saturation for spacecraft by integrating auxiliary system and saturation function [16,17]. 

In general, since formation spacecraft are typically miniature satellites with low cost, small size, and light weight, they cannot carry too many sensors, such as the angular velocity sensor [18]. Hence, the design of attitude coordination control strategy without angular velocity feedback is the focus of investigation for SFF, which has not been introduced into the control schemes of the literatures mentioned. In view of this, on the basis of individual attitude quaternion, a finite-time observer method for SFF is developed to estimate the attitude angular velocity [19]. Besides, a fixedtime velocity-free attitude coordination control method for SFF is presented via utilizing the filter system to simulate the angular velocity in controller [20]. Notice that the aforesaid velocity-free control schemes for SFF consisting of finite-time and fixed-time methods cannot satisfy the constraint of predefined-time convergence. Afterwards, a practical predefined-time extended-state observer for SFF is constructed by means of switching time-varying function to handle the constraints of angular velocity-free and predefined-time stable concurrently, which renders the observer errors to converge into a small region within the predefined time [21]. 

In summary, it is a significant issue to devise a distributed nonsingular predefined-time attitude coordination control strategy for SFF when considering the constraints of actuator saturation, external disturbance, and without angular velocity measurement. Inspired by the aforementioned literatures, this Note employs the predefined-time control, observer method, super-twisting algorithm, nonsingular sliding mode, and auxiliary system to address this issue. The main achievements are as follows: 

1) Combined with the predefined-time control and super-twisting algorithm, a novel angular velocity observer is constructed to drive observer errors to converge to zero in predefined time. 

2) Based on the constructed observer, a novel nonsingular predefined-time sliding mode (NSPTSM) surface for SFF without angular velocity measurement is proposed to make the attitude error converge into a small region near zero in predefined time. 

3) By employing the constructed observer, NSPTSM surface, auxiliary system, and saturation function, a novel distributed saturated NSPTSM attitude coordination controller without angular velocity feedback for SFF is designed to enable the NSPTSM surface to converge into a small region near zero in predefined time. 

## II. Preliminaries 

## A. Notations 

R[n] expresses the set of n × 1 real number vectors. R[n][×][m] represents the set of n × m real number matrixes. R� refers to the set of positive real numbers. For z ��z1; z2; z3�[T] , 

1830 

1831 

J. GUIDANCE, VOL. 49, NO. 6: 

TECHNICAL NOTES 

z[×] ��0; −z3; z2; z3; 0; −z1; −z2; z1; 0�. In denotes the n × 1 all-ones vector. I3×3 indicates the 3 × 3 identity matrix. λmin and λmax denote the maximum and minimum eigenvalues. k ⋅ k expresses the 2- norm of a vector or matrix. diagf⋅g indicates the block-diagonal matrix. The symbol ⊗ denotes the Kronecker product. sign�⋅� refers to the standard sign function. For arbitrary c ∈ R[n] , sgn[α] �c���jc1j[α] sign�c1�; jc2j[α] sign�c2�;: : : ; jcnj[α] sign�cn��[T] , where α is a constant. 

## B. Algebraic Graph Theory 

Consider SFF with the undirected communication topology. Let N �f1; 2;: : : ; ng, V � N × N express the sets of nodes and edges, respectively, and then the information interaction between spacecraft is defined as G �fN ; Vg, where the node refers to spacecraft and the edge represents communication link between spacecraft. Let �i; j� denote the information transmission from spacecraft i to j and A ��aij� ∈ R[n][×][n] express the weighted adjacency matrix of G. Note that aij � aji > 0 if �i; j� ∈ V, aij � 0 if otherwise. Additionally, the Laplacian matrix of G is represented as L � diag nj�1[a][1][j][;] nj�1[a][2][j][;: : : ;] nj�1[a][nj] − A. 

## C. Relevant Lemmas 

Consider the system as below: 

**==> picture [217 x 12] intentionally omitted <==**

in which f∶R[n] × R[m] × R� → R[n] refers to the nonlinear function, d indicates the disturbance, and t denotes the time variable. 

Lemma 1 [22]: If for any initial condition ζ0 of the system (1), there exists a radially unbounded Lyapunov function V�ζ� that satisfies V_�ζ� ≤− μTs ~~p~~ π γ1γ2 γ1V[1][−] μ2 � γ2V[1][�] μ2 , where 0 < μ < 1, γ1; γ2 > 0, Ts > 0 is the predefined time parameter, then the system (1) is predefined-time stable and the settling-time is Ts. 

Lemma 2 [22]: If for any initial condition ζ0 of the system (1), there exists a radially unbounded Lyapunov function V�ζ� that satisfies V[_] �ζ� ≤− μTs ~~p~~ π γ1γ2 γ1V[1][−] μ2 � γ2V[1] ~~[�]~~ μ2 � η, where 0 < μ < 1, γ1; γ2 > 0, Ts > 0 is the predefined time parameter, 0 < η < ∞, then the system (1) is practically predefined-time stable while satisfying 2 2 limt→Tps ζjV�ζ� ≤ min ημπγT1s�p1−γ1ϑγ�2 2−μ; ημπγT2s�p1−γ1ϑγ�2 2�μ , in which 0 < ϑ < 1 and the settling-time Tps � Ts∕pϑ. Lemma 3 [23]: For χi ≥ 0 �i � 1; 2;: : : ; n�, the inequality (2) holds 

**==> picture [141 x 37] intentionally omitted <==**

**==> picture [11 x 8] intentionally omitted <==**

Lemma 4 [24]: According to the Young’s inequality. For arbitrary p1; p2 ∈ R, σ1; σ2 > 0, it follows that 

**==> picture [227 x 19] intentionally omitted <==**

where i � 1; 2;: : : ; n denotes the ith spacecraft. In addition, ωi ∈ R[3] , di ∈ R[3] , Ji ∈ R[3][×][3] represent the angular velocity, external disturbance, and inertial matrix. The control torque with actuator saturation τi � sat�τic� ∈ R[3] is presented as 

**==> picture [200 x 24] intentionally omitted <==**

in which m � 1; 2; 3 expresses the mth component, τM > 0 denotes the saturation value of control torque, τic refers to the control input to be devised. And matrix Z�qi� is as Z�qi�� 14[��][1][−] q[T] i[q] i[�][I] 3×3[�][2][q][×] i[�][2][q] i[q][T] i[�][.] 

Following this, the attitude error for the ith spacecraft is as qie � �1−q[T] d[q] 1�[d][�] q[q][T] i[i][−][q][�][i][q][1][T] d[−][q][q][d] i[T][�][q][i][2][�][q][q][T] d[d][q][�][i][2][q][×] i[q][d] and its angular velocity error is as ωie � ωi − R�qie�ωd, where qd, ωd denote the desired attitude and its angular velocity, the rotation matrix is represented as R�qie�� R�qi��R�qd��[T] , and R�qi�� I3×3 − �41��1−qq[T] i[T] i[q][q][i][�][i][2][�][q][×] i[�] �1�8qq[T] i[×] i[q][i][�][2][q][×] i[.] Define Z�qie�� Zi, R�qie�� Ri, and then the attitude error model is deduced as below: 

**==> picture [191 x 11] intentionally omitted <==**

in which, Mi � Z[−] i[T][J] i[Z][−] i[1][,] Ci � −Z[−] i[T][��][J] i[Z][−] i[1][q][_] ie[�][×][Z][−] i[1][−] JiZ[_][−] i[1][��][Z][−] i[T][�][J] i[�][R] i[ω] d[�][×][��][R] i[ω] d[�][×][J] i[−][�][J] i[R] i[ω] d[�][×][�][Z][−] i[1][,] Ni � Z[−] i[T][��][R] i[ω] d[�][×][J] i[R] i[ω] d[�][J] i[R] i[ω][_] d[�][.] One further has 

**==> picture [187 x 23] intentionally omitted <==**

Consider the attitude error model (6) for SFF, and then several properties are met: 

Property 1 [25]: Mi is a symmetric positive definite matrix, and satisfying 

**==> picture [202 x 10] intentionally omitted <==**

Property 2 [26]: For Ci, there exists a positive number ρ1 such that 

**==> picture [149 x 9] intentionally omitted <==**

## B. Control Objective 

This Note is focused on designing a distributed nonsingular predefined-time attitude coordination control strategy for the attitude error model (6) of SFF to accomplish that all spacecraft attitude tracks the desired attitude in predefined time Tp, while considering the constraints of actuator saturation, external disturbance, and without angular velocity measurement. 

## IV. Main Results 

At first, three assumptions are provided as 

Assumption 1: The disturbance di is bounded and kdik ≤ dM, where dM > 0 indicates a known positive constant. 

## III. System Model and Control Objective 

## A. System Model 

Define the spacecraft attitude of the body-fixed reference frame with respect to the inertial reference frame as q ��q1; q2; q3�[T] , and then the attitude model of the ith spacecraft are as follows based on the modified Rodrigues parameters [14]: 

**==> picture [169 x 24] intentionally omitted <==**

so Assumptionis its time-derivative2: The desiredω_ d. angular velocity ωd is bounded, and 

Assumption 3 [26]: Since the control torque considered in this Note is bounded, the states of the spacecraft system are always bounded. 

## A. Angular Velocity Observer 

_ ^ Define xi1 ^� qie, xi2 �^ qie, observer errors oie1 � xi1 − xi1, and^ oie2 � xi2 − xi2, where xi1 denotes the estimation of xi1 and xi2 denotes the estimation of xi2. Combined with the predefined-time 

1832 

J. GUIDANCE, VOL. 49, NO. 6: TECHNICAL NOTES 

control and super-twisting algorithm, the angular velocity observer for the ith spacecraft is constructed as 

**==> picture [213 x 32] intentionally omitted <==**

in which 0 < μ < 1, ϕ � π∕μTs1pγ1γ2, ε1; ε2; ε3; γ1; γ2 > 0, Ts1 > 0 refers to the predefined time parameter. 

theobserverRemark 1:observeris that In contrast to most existing angular velocity observers,(10)thereis devoidis no needof CCii��xxii11;; xx^ii22��xx^ii22. −TheCi�xvirtuei1; x^i2�ofx^i2theto assume bounded. 

Subsequently, the following theorem is presented. 

Theorem 1: Considering the spacecraft attitude error model (6) satisfying Assumptions 1–3 under the actuator saturation and external disturbances, when the attitude angular velocity xi2 is unavailable, if the observer (10) is adopted, the observer errors oie1; oie2 will converge to zero in the predefined time Ts1. 

Proof: From (6) and (10), one obtains 

**==> picture [220 x 30] intentionally omitted <==**

where g�xi1; xi2; di�� M[−] i[1][�][Z][−] i[T][d] i[−][C] i[�][x] i1[;][ x] i2[�][x] i2[�][and][g][ ��][g] 1[;] g2; g3�[T] . 

In accordance to the Property 1, (7) and Assumption 1, one attains kM[−] i[1][Z][−] i[T][d] i[k][ ≤][4][d] M[λ][−] min[1][�][M][i][�][≤][D][1][,][where][D][1][is][a][positive][con-] stant. From the Property 2 and Assumption 3, one obtains that kM[−] i[1][C] i[x] i2[k][ ≤][ρ] 1[k][x] i2[k][2][λ][−] min[1][�][M][i][�][≤][D][2][,][where][ρ][1][and][D][2][are][pos-] itive constants. Defining D � D1 � D2, there exists a bounded number δi;m�m � 1; 2; 3� such that 0 < jδi;mj ≤ D and gm � δi;m sign�oie1;m�. 

Thus, (11) can be rewritten as 

**==> picture [216 x 30] intentionally omitted <==**

**==> picture [16 x 8] intentionally omitted <==**

Define the state vectors as ξi1;m � sgn[1][−] μ2�oie1;m� oie2;m T, ξi2;m � sgn[1] ~~[�]~~ μ2�oie1;m� oie2;m T. Then, based on (11), the derivatives of ξi1;m is given as ξ[_] i1;m � ϕO1U1ξi1;m, where O1 � μ joie1;mj[−][μ] � ~~2~~ 2�joie1;mj � θ1�[λ][1][μ] . Only if joie1;mj � 1, θ1 > 0 is an arbitrary positive constant; otherwise, θ1 � 0. λ1 is defined as follows 

**==> picture [156 x 24] intentionally omitted <==**

μ μ −1 And defining φ1 �joie1;m ~~j~~ 2 1 � ~~2~~ 2joie1;mj[μ] �joie1;mj � θ1�[λ][1][μ] , matrix U1 ∈ R[2][×][2] can be expressed as 

**==> picture [208 x 49] intentionally omitted <==**

_ Similarly, the derivatives of ξi2;m is obtained as ξi2;m � μ ϕO2U2ξi2;m, where O2 �joie1;mj[−][μ] � ~~2~~ 2�joie1;mj � θ1�[λ][2][μ] , λ2 is similar to λ1, 

**==> picture [220 x 49] intentionally omitted <==**

3μ μ −1 in which φ2 �joie1;mj 2 1 � ~~2~~ 2joie1;mj[μ] �joie1;mj � θ1�[λ][2][μ] . The Lyapunov candidate function Vi0;m is constructed as Vi0;m � Vi1;m � Vi2;m � ξ[T] i1;m[P][1][ξ][i][1][;m][�][ξ][T] i2;m[P][2][ξ][i][2][;m][, where][ V][i][1][;m][�] ξ[T] i1;m[P][1][ξ][i][1][;m][,][V][i][2][;m][�][ξ][T] i2;m[P][2][ξ][i][2][;m][,][P][1][;][ P][2][∈][R][2][×][2][are][positive][defi-] nite symmetric matrixes. Note that U1, U2 are Hurwitz if ε3 > D∕ϕ. Therefore, one can obtain that U[T] 1[P][1][�][P][1][U][1][≤−][Q][1][,] U[T] 2[P][2][�][P][2][U][2][≤−][Q][2][, where][ Q][1][,][ Q][2][are positive definite matrixes.] −μ2 μ2 □ −joMeanwhile, notice thatie1;mj[−][μ] ≤ kξi2;mk[−][μ] −≤−joieV1−i;m2μ2;mj[−][λ][μ] minμ2 ≤[�] k[P] ξ[2] i[�] 1[.] ;mkThere[−][μ] ≤−Vexisti1;m[λ] minλ1[�] ,[P][1] λ[�] 2[,] such that −�joie1;mj � θ1�[λ][1][μ] ≤−kξi1;mk[μ] ≤−Viμ21;m[λ] −maxμ2[�][P] 1[�][and] −�joie1;mj � θ1�[λ][2][μ] ≤−kξi2;mk[μ] ≤−Viμ22;m[λ] −maxμ2[�][P] 2[�][.] Moreover, γ1 and γ2 need to satisfy that γ1 ≤ μ μ min λmin�Q1�λmin2[�][P][1][�][λ] max[−][1][�][P] 1[�][;][ λ] min[�][Q] 2[�][λ] min2[�][P][1][�][λ] max[−][1][�][P] 1[�] , γ2 ≤ min λmin�Q1�λ−max1−μ2[�][P] 1[�][;][ λ] min[�][Q] 2[�][λ] −max1 ~~−~~ μ2[�][P] 1[�] . Consequently, based on Lemma 3, one has 

**==> picture [241 x 84] intentionally omitted <==**

Then, it follows that ξi1;m and ξi2;m will converge to zero in the predefined time Ts1 via utilizing the Lemma 1, implying that the observer errors oie1 and oie2 converge to zero in Ts1. Therefore, Theorem 1 has been proved. 

## B. Distributed Saturated NSPTSM Attitude Controller Without Angular Velocity Feedback 

Let qij � qie − qje denote the relative attitude error between the ith and jth �j � 1; 2;: : : ; n� spacecraft, lij indicate the element of L, bi > 0 express the element of diagonal matrix B. Thereby, the lumped attitude errors could be defined as ei1 � bixi1� nj�1[l][ij][x][j][1][,][e][i][2][�][b][i][x][i][2][�] nj�1[l][ij][x][j][2][.] Define e1 ��e11; e21;: : : ; en1�[T] , e2 ��e12; e22;: : : ; en2�[T] , x1 � �x11; x21;: : : ; xn1�[T] , x2 ��x12; x22;: : : ; xn2�[T] , H1 ��B � L� ⊗ I3×3, in which H1 is a symmetric positive definite matrix and H[−] 1[1] � H2. Therefore, the lumped attitude errors are reconstructed as e1 � H1x1, e2 � H1x2. 

For the spacecraft attitude error model (6) with angular velocity observer (10), the distributed NSPTSM surface is constructed as 

**==> picture [227 x 58] intentionally omitted <==**

where γ3; γ4; γs > 0, Vi3 � 0.5x[T] i1[x][i][1][denotes][the][Lyapunov][candi-] xdate^2 Define��function,x^12; x^22e^i;: : : ;2T�s2 >bx^i 0n ^x2i�2refers[T] �, s ��to thenj�1s1[l] ;[ij] s[x][^] predefined time2[j] ;: : : ;[2][,] se^n2�[T] ��, ande^12;parameter. ^ethen22;: : : ;(17) ^en2�[T] is, rewritten as 

1833 

J. GUIDANCE, VOL. 49, NO. 6: TECHNICAL NOTES 

**==> picture [148 x 10] intentionally omitted <==**

μ in which Λ1 � diag 2μTs2π ~~p~~ γsγ4 γ3 � γ4V132 I3×3;:::; 2μTs2π ~~p~~ γsγ4 γ3� μ γ4Vn2 3 I3×3 . Define τ ��τ1; τ2;: : : ; τn�[T] , τc ��τ1c; τ2c;: : : ; τnc�[T] , Z � diagfZ1; Z2;: : : ; Zng, C � diagfC1; C2;: : : ; Cng, M1 � diagfM1; M2;: : : ; Mng, N ��N1; N2;: : : ; Nn�[T] , d ��d1; d2;: : : ; dn�[T] . On the basis of (4), (5) and (18), a distributed saturated NSPTSM attitude controller without angular velocity feedback is devised as 

**==> picture [141 x 9] intentionally omitted <==**

**==> picture [233 x 28] intentionally omitted <==**

**==> picture [201 x 21] intentionally omitted <==**

where γ5; γ6; k1; k2 > 0, V4 � 0.5s[T] H2s indicates the Lyapunov candidate function, Ts3 > 0 refers to the predefined time parameter. Remark 2: Compared with other controllers that utilize 2μπTs V ~~[−]~~ μ2 � V μ2 directly in Λ1, implying that Λ[_] 1x1 occurs the singularity as x1 → 0, the controller (19–21) devised in this Note is nonsingular. Furthermore, since the switching function is not employed to cope with the singularity of Λ[_] 1x1, this inevitably reduces the structural complexity of the NSPTSM surface. 

Remark 3: Note that the controller (19–21) is antisaturation and its saturation upper bound depends on the parameter τM, which could be adjusted by the engineer. 

Define Δτc � τc − sat�τc�, Λ3 � 2μTs3π ~~p~~ γ5γ6 γ5V−5 μ2[�] ~~[2]~~ μ2γ6V5μ2 , and then an auxiliary system Ψ is devised as 

**==> picture [239 x 56] intentionally omitted <==**

where σa; ka; εa > 0, V5 � 0.5Ψ[T] Ψ represents the Lyapunov candidate function. 

Remark 4: The parameter σa in the auxiliary system Ψ serves to prevent the occurrence of singularity when kΨk � 0. Therefore, for better predefined-time control performance, the value of σa should be set as small as possible. 

## C. Stability Analysis 

Theorem 2: Considering spacecraft attitude error model (6) with angular velocity observer (10) for SFF satisfying Assumptions 1–3 under the actuator saturation and external disturbances, if the sliding mode surface and the distributed saturated NSPTSM attitude controller are designed as (18–22), the NSPTSM surface s and auxiliary system Ψ will converge to a small region Δ1 in predefined time Tp1 � Ts1 � Ts3.[[_]] 

_ ^ Λ_ 1Proof:e1 � Λ1Accordinge2. to (18), the derivative of s is as s � e[[_]] 2 � 

1) When kΨk ≥ σa. a) Consider T < Ts1. Based on Property 2, Assumption 3 and (16), one obtains that kΛ1oe2k ≤ D3 and kM[−][1] Cx^2k ≤ ρ2kx^2k[2] λ[−] min[1][�][M][�][≤][D][4][, where][ D][3][;][ ρ][2][; D][4][are positive] constants. Let D5 � D3 � D4 and β1 � ε3π∕μTs1pγ1γ2 � D5. According to (21), Δτc � τc − sat�τc�, and ksk ≤ p2λ−min12[�][H][2][�][V] 412[,] the derivative of V4 � 0.5s[T] H2s is obtained as 

**==> picture [241 x 88] intentionally omitted <==**

Based on (22), the derivative of V5 � 0.5Ψ[T] Ψ is given as 

**==> picture [196 x 37] intentionally omitted <==**

Define the Lyapunov candidate function as V6 � V4 � V5 � ni�1 3m�1[V][i][0][;m][.][Then,][combining][(23)][and][(24)][and] Lemma 3, the derivative of V6 can be derived as 

**==> picture [241 x 74] intentionally omitted <==**

The proof that follows is in two parts: Step i) When V4 ≥ 1, it follows that V412[≤][V] 14−μ2. Thus, one deduces that 

**==> picture [241 x 45] intentionally omitted <==**

If γ5 > μTs3p2γ5γ6β1∕πλmin12[�][H][2][�][,][one][further][derives] V_ 6 ≤ 0, implying that the NSPTSM surface s, auxiliary system Ψ, and observer errors oie1, oie2 are bounded. 

Step ii) When V4 < 1, it can be concluded that V[_] 6 ≤ p2β1λ−min12[�][H][2][�][.] Then, as T < Ts1, one further obtains that V6 is bounded. Obviously, when V4 < 1, the sliding mode surface s, auxiliary system Ψ, and observer errors oie1, oie2 are bounded. 

Therefore, for T < Ts1, even if the observer errors oie1 and oie2 will not converge to zero, s and Ψ cannot diverge to infinity.^ b). Consider T ≥ Ts1. As T ≥ Ts1, one obtains that x2 � x2 via using Theorem 1. Then, the derivative of s is deduced as _ _ s � e2 � Λ[_] 1e1 � Λ1e2. 

If k1 > dM, according to (21) and Δτc � τc − sat�τc�, the derivative of V4 is obtained as 

**==> picture [241 x 51] intentionally omitted <==**

Definecombiningthe Lyapunov(24), (27),candidateand Lemmafunction3, oneas Vobtains7 � V4 �thatV5V._ 7Then,≤− μTs3 ~~p~~ π γ5γ6 γ5V17−μ2 � γ6V17 ~~�~~ μ2 . On basis of Lemma 1, it follows that the NSPTSM surface s and auxiliary system Ψ converge to zero in predefined time Tp1 � Ts1 � Ts3. Notice that kΨk ≥ σa; therefore, 

1834 

J. GUIDANCE, VOL. 49, NO. 6: TECHNICAL NOTES 

the NSPTSM surface s and auxiliary system Ψ will converge to a small region Δ1 in predefined time Tp1 � Ts1 � Ts3, where Δ1 � f�s; Ψ�jksk ≤ σs; kΨk ≤ σag and σs is a small positive constant. 

Ψ_ �2) kWhenaΨ have the same sign, so the norm0 < kΨk < σa. In this case, Ψ kΨandk will keep increasingits time-derivative until it returns to case 1). kΨk ≥ σa. 

3) When kΨk � 0. In this case, Ψ[_] � εaI3n and εa > 0, so the norm kΨk will return to case 1). kΨk ≥ σa or case 2). 0 < kΨk < σa. Therefore, the NSPTSM surface s and auxiliary system Ψ will converge to the small region Δ1 in predefined time. Now, Theorem 2 is proven. □ 

Theorem 3: Considering spacecraft attitude error model (6) with angular velocity observer (10) for SFF and the distributed NSPTSM surface (18), if ksk ≤ σs is arrived, the attitude error x1 of SFF will converge into a small region Δ2 in predefined time Tp � Tp1 � Ts2∕pϑ1. 

^ Proof: Based on (18), one obtains that H2s � x2 � Λ1x1. 

1) Consider T < Ts1. In view of the proof of Theorem 2, when T < Ts1, the NSPTSM surface s is bounded, so is the state error x1. ^ 2) Consider T ≥ Ts1. When T ≥ Ts1, it can be attained that x2 � x2 by means of Theorem 1. One further has xi2 � μ − 2μTs2π ~~p~~ γsγ4 γ3 � γ4Vi23 xi1 ��H2s�i. Then, based on ksk ≤_ σs, the derivatives of Vi3 � 0.5x[T] i1[x][i][1][can][be][derived][as] Vi3 ≤ − μTs2 ~~p~~ π γsγ4 γ3Vi3 � γ4V1i3 ~~�~~ μ2 � p2σshVi3 12, where σsh � σskH2k. □ On the basis of Lemma 4, if p1 � 1, p2 � Vi3, σ1 � 12 ~~[,]~~[σ][2][�] 12 ~~[,]~~ it follows that Vi3 12 ≤ 12[V][i][3][ �] 12 ~~[.]~~[Subsequently,][one][further][has] V_ i3 ≤− μTs2 ~~p~~ π γsγ4 γ3 − σshμTs22pπ 2γsγ4 Vi3 � γ4V1i3 ~~�~~ μ2 � p22[σ][sh][.][If] γ3 > σshμTs22pπ 2γsγ4 and γs ≤ 2−2μ γ3 − σshμTs22pπ 2γsγ4 , it can be 

obtained that V[_] i3 ≤− μTs2 ~~p~~ π γsγ4 2−2μ[γ][s][V][i][3][ �][γ][4][V] 1i3 ~~�~~ μ2 � p22[σ][sh][.] On the basis of Lemma 4, if p1 � 1, p2 � Vi3, σ1 � μ2[,] σ2 � 1 − μ2 ~~[,]~~[it][follows][that][−][V][i][3][≤−] 2−2μ[V][i][31][−] μ2 � 2−μμ[.][Thus,][one] further obtains V[_] i3 ≤− μTs2 ~~p~~ π γsγ4 γsVi3[1][−] μ2 � γ4V1i3 ~~�~~ μ2 � η1, where η1 � 2Tπs2 γγ4s � p22[σ][sh][.][From][Lemma][2,][one][acquires][that][the][atti-] tude error x1 of SFF will converge the neighborhood of zero Δ2 in predefined time Tp � Tp1 � Ts2∕pϑ1. Defining 0 < ϑ1 < 1, one further has 

**==> picture [206 x 50] intentionally omitted <==**

Now, Theorem 3 has been proved. 

Remark 5: Compared with other control strategies, the upper bound on settling time of the proposed predefined-time control strategy in this Note is determined by only the parameter Tp, which can be adjusted by engineers. 

Remark 6: There exists a tradeoff between the magnitude of the applied control torque and the predefined time Tp, which means that the smaller the applied control torque, the longer the predefined time Tp is requested. 

## V. Simulations 

Considering the attitude error model (6) of four spacecraft under the constraints of actuator saturation, external disturbance, and without angular velocity measurement, Fig. 1 displays the undirected communication topology graph for SFF, in which B � diag�1; 1; 1; 1�, L ��1; −1; 0; 0; −1; 2; −1; 0; 0; −1; 2; −1; 0; 0; −1; 1�. 

Set the initial states of four spacecraft for SFF as 

**==> picture [87 x 79] intentionally omitted <==**

Fig. 1 The communication topology graph. 

qo1 ��0.6; −0.1; 0.3�[T] , qo2 ��−0.2; 0.1; −0.7�[T] , qo3 ��0.7; 0.3; −0.5�[T] , qo4 ��0.6; −0.4; −0.5�[T] , ωo1 � 10[−][3] �2; −1; −3�[T] rad∕s, ωo2 � 10[−][3] �1; −3; −1�[T] rad∕s, ωo3 � 10[−][3] �−1; 2; −1�[T] rad∕s, ωo4 � 10[−][3] �2; −1; −1�[T] rad∕s. 

Moreover, set the inertia matrices as [14] 

J1 ��20; 2; 0.9; 2; 17; 0.5; 0.9; 0.5; 15� kg ⋅ m[2] , J2 ��22; 1; 0.9; 1; 19; 0.5; 0.9; 0.5; 15� kg ⋅ m[2] , J3 ��18; 1; 1.5; 1; 12; 0.5; 1.5; 0.5; 17� kg ⋅ m[2] , J4 ��18; 1; 1; 1; 20; 0.5; 1; 0.5; 15� kg ⋅ m[2] . 

Set the desired attitude as qd � 0.4�cos�0.01t�; − sin�0.01t�; − cos�0.01t��[T] . Then, Set the external disturbances as 

d1 � 10[−][2] �7 sin�0.5t�; 4 cos�0.4t�; 5 sin�0.7t��[T] N ⋅ m, d2 � 10[−][2] �5 cos�0.5t�; 6 sin�0.4t�; 4 cos�0.7t��[T] N ⋅ m, d3 � 10[−][2] �6 sin�0.5t�; 5 cos�0.4t�; 4 sin�0.7t��[T] N ⋅ m, d4 � 10[−][2] �7 cos�0.5t�; 3 sin�0.4t�; 5 cos�0.7t��[T] N ⋅ m. 

Set the other parameters as μ � 0.5, Ts1 � 1, Ts2 � 10, Ts3 � 15, γ1 � 1, γ2 � 1, γ3 � 1, γ4 � 1, γ5 � 1, γ6 � 1, γs � 1, k1 � 0.2, k2 � 0.1, ε1 � 1, ε2 � 1, ε3 � 1, τM � 1.5, σa � 10[−][6] , ka � 10[−][5] , εa � 10[−][5] , the initial value of auxiliary system Ψi;m � 0.1�i � 1; 2; 3; 4; m � 1; 2; 3�, and the initial value of oie1;m, oie2;m�i � 1; 2; 3; 4; m � 1; 2; 3� are set as zero. Furthermore, to further highlight the effectiveness of the devised attitude coordination control strategy for SFF, the attitude coordination controller in Ref. [27] is used for comparison simulation. With the goal of fairness, the comparative controller runs in the same simulation setup as this Note. Meanwhile, the boundary layer method is employed to cope with the chattering [27]. Set the simulation duration – as 60s. Figures 2 9 depict the simulation results. 

Based on Figs. 2 and 4, one obtains that the attitude errors, angular velocity errors with the presented controller converge within 20s, implying that the attitude of SFF tracks the desired attitude in predefined time Tp. According to Figs. 3 and 5, the comparative controller achieves convergence within 30s and its control precision 

**==> picture [240 x 193] intentionally omitted <==**

Fig. 2 Curves of attitude errors with the presented controller. 

1835 

J. GUIDANCE, VOL. 49, NO. 6: TECHNICAL NOTES 

**==> picture [240 x 200] intentionally omitted <==**

Fig. 3 Curves of attitude errors with the comparative controller. 

**==> picture [237 x 195] intentionally omitted <==**

Fig. 4 Curves of angular velocity errors with the presented controller. 

**==> picture [240 x 195] intentionally omitted <==**

Fig. 5 Curves of angular velocity errors with the comparative controller. 

**==> picture [240 x 196] intentionally omitted <==**

Fig. 6 Curves of control torques with the presented controller. 

**==> picture [240 x 196] intentionally omitted <==**

Fig. 7 Curves of control torques with the comparative controller. 

**==> picture [240 x 196] intentionally omitted <==**

Fig. 8 Curves of observer errors oie1 with the presented controller. 

1836 

J. GUIDANCE, VOL. 49, NO. 6: 

TECHNICAL NOTES 

**==> picture [240 x 200] intentionally omitted <==**

Fig. 9 Curves of observer errors oie2 with the presented controller. 

jqie;mj ≤ 5 × 10[−][3] . Additionally, from Figs. 6 and 7, it follows that the control torques are saturated and without chattering. From Figs. 8 and 9, not only the observer errors can converge in predefined time Ts1, but also the control precision of observer errors is quite high. Then, one further obtains that the devised attitude coordination control strategy for SFF possesses higher control precision and better dynamic performance under the same simulation setup. 

## VI. Conclusions 

This Note investigates the velocity-free distributed NSPTSM attitude coordination control for SFF subject to the constraints of actuator saturation, external disturbance, and without angular velocity measurement. Firstly, combined with the predefined-time control and super-twisting algorithm, an angular velocity observer is constructed to realize precise angular velocity estimate. Secondly, a NSPTSM surface is proposed by means of the constructed observer, which renders the attitude error to converge into the neighborhood of zero in predefined time. Thirdly, a distributed saturated NSPTSM attitude coordination controller without angular velocity feedback is presented for SFF via employing the constructed observer, NSPTSM surface, auxiliary system, and saturation function, followed by the stability analysis. Lastly, simulations illustrate the effectiveness of presented control strategy. Future work intends to extend the proposed control strategy to the attitude-orbit coupled coordination model for SFF, and additionally incorporate the distinction between active and passive control phases as well as parameter optimization algorithms into the framework. 

## Acknowledgments 

This work was supported by the National Natural Science Foundation of China (No. 62273277) and Key Research and Development Program of Shaanxi (Grant No. 2023-GHZD-32). 

## References 

- [1] Hays, C. W., Henderson, T., Miller, K., Phillips, S., and Soderlund, A., “Angles-Only Cooperative Local Catalog Maintenance of CloseProximity Satellite Systems,” Journal of Guidance Control and Dynamics, Vol. 47, No. 12, 2024, pp. 2573–2586. https://doi.org/10.2514/1.G008280 

- [2] Alzubairi, A., Tameem, A., and Kada, B., “Spacecraft Formation Flying Orbital Control for Earth Observation Mission,” Scientific African, Vol. 26, 2024, Paper e02391. https://doi.org/10.1016/j.sciaf.2024.e02391 

- [3] Sun, R., Ahn, C. K., Liu, D., Wang, W., and Zhang, C., “Near-Asteroid Spacecraft Formation Control with Prescribed-Performance: A Dynamic Event-Triggered Reinforcement Learning Control Approach,” 

Aerospace Science and Technology, Vol. 161, 2025, Paper 110138. 

   - https://doi.org/10.1016/j.ast.2025.110138 

- [4] Santos, W. G., Mason, P., Stoneking, E. T., and Sarli, B. V., “Reconfigurable Guidance Strategy for Compensating Actuator Faults in Spacecraft Formation Flying,” Journal of Guidance Control and Dynamics, Vol. 48, No. 2, 2025, pp. 282–296. https://doi.org/10.2514/1.G008087 

- [5] Sun, R., Wang, J. H., Zhang, D. X., and Shao, X. W., “Neural-Network-Based Sliding-Mode Adaptive Control for Spacecraft Formation Using Aerodynamic Forces,” Journal of Guidance Control and Dynamics, Vol. 41, No. 3, 2018, pp. 754–760. https://doi.org/10.2514/1.G003063 

- [6] Zhou, J. K., Hu, Q. L., and Friswell, M. I., “Decentralized Finite Time Attitude Synchronization Control of Satellite Formation Flying,” Journal of Guidance Control and Dynamics, Vol. 36, No. 1, 2013, pp. 185–195. https://doi.org/10.2514/1.56740 

- [7] Cui, B., Zhang, L. J., Xia, Y. Q., and Zhang, J. H., “Continuous Distributed Fixed-Time Attitude Controller Design for Multiple Spacecraft Systems with a Directed Graph,” IEEE Transactions on Circuits and Systems Ii-Express Briefs, Vol. 69, No. 11, 2022, pp. 4478–4482. https://doi.org/10.1109/TCSII.2022.3179315 

- [8] Zhuang, M. L., Tan, L. G., Li, K. H., and Song, S. M., “Fixed-Time Formation Control for Spacecraft with Prescribed Performance Guarantee Under Input Saturation,” Aerospace Science and Technology, Vol. 119, 2021, Paper 107176. https://doi.org/10.1016/j.ast.2021.107176 

- [9] Hu, L. M., Wang, Z., Chen, C. R., and Yue, H., “Adaptive Approximate Predefined-Time Guaranteed Performance Control of Uncertain Spacecraft,” Mathematics, Vol. 13, No. 5, 2025, Paper 832. https://doi.org/10.3390/math13050832 

- [10] Zhang, Z. C., Bao, W. M., Hou, Q. M., Ju, Y. H., and Gao, Y. B., “Disturbance Estimation and Predefined-Time Control Approach to Formation of Multi-Spacecraft Systems,” Sensors, Vol. 24, No. 17, 2024, Paper 5671. https://doi.org/10.3390/s24175671 

- [11] Wang, Z., Ma, J., and Hu, L. M., “Predefined-Time Guaranteed Performance Attitude Takeover Control for Non-Cooperative Spacecraft with Uncertainties,” International Journal of Robust and Nonlinear Control, Vol. 33, No. 13, 2023, pp. 7488–7509. https://doi.org/10.1002/rnc.6761 

- [12] Wang, J., Yang, M., Wang, D. H., and Wang, T., “Distributed Average Consensus Attitude Synchronization of Multi Rigid Spacecrafts with Predefined Time Event-Triggered Sliding Mode Control,” Aerospace Science and Technology, Vol. 159, No. 2025, 2025, Paper 109975. https://doi.org/10.1016/j.ast.2025.109975 

- [13] Shahbazi, B., Malekzadeh, M., and Koofigar, H. R., “Robust Constrained Attitude Control of Spacecraft Formation Flying in the Presence of Disturbances,” IEEE Transactions on Aerospace and Electronic Systems, Vol. 53, No. 5, 2017, pp. 2534–2543. https://doi.org/10.1109/TAES.2017.2704160 

- [14] Xiao, C., Guo, Y., Xie, C. Q., Li, A. J., and Wang, C. Q., “Adaptive Super-Twisting Sliding Mode Attitude Coordination Control for Spacecraft Formation Flying with Actuator Saturation,” Advances in Space Research, Vol. 72, No. 10, 2023, pp. 4244–4255. https://doi.org/10.1016/j.asr.2023.08.049 

- [15] Zhuang, M. L., and Song, S. M., “Fixed-Time Coordinated Attitude Tracking Control for Spacecraft Formation Flying Considering Input Amplitude Constraint,” International Journal of Control Automation and Systems, Vol. 20, No. 7, 2022, pp. 2129–2147. https://doi.org/10.1007/s12555-021-0366-8 

- [16] Shi, X. N., Zhou, D., Chen, X. W., and Zhou, Z. G., “Actor-CriticBased Predefined-Time Control for Spacecraft Attitude Formation System with Guaranteeing Prescribed Performance on SO(3),” Aerospace Science and Technology, Vol. 117, 2021, Paper 106898. https://doi.org/10.1016/j.ast.2021.106898 

- [17] Sun, Y., Lyu, Y., Guo, Y. N., Gong, Y. M., and Zhu, H., “Adaptive Predefined-Time Control for Liquid-Filled Flexible Spacecraft Attitude Stabilization During Orbit Maneuver,” Advances in Space Research, Vol. 71, No. 6, 2023, pp. 2733–2744. https://doi.org/10.1016/j.asr.2022.11.052 

- [18] Zhu, Z. H., Gao, Z., and Guo, Y., “Distributed Global Velocity-Free Attitude Coordination Control for Multiple Spacecraft without Unwinding,” International Journal of Control Automation and Systems, Vol. 20, No. 2, 2022, pp. 411–420. https://doi.org/10.1007/s12555-020-0574-7 

- [19] Hu, Q. L., Zhang, J., and Zhang, Y. M., “Velocity-Free Attitude Coordinated Tracking Control for Spacecraft Formation Flying,” ISA Transactions, Vol. 73, 2018, pp. 54–65. https://doi.org/10.1016/j.isatra.2017.12.019 

1837 

J. GUIDANCE, VOL. 49, NO. 6: TECHNICAL NOTES 

- [20] Gao, Z., Zhu, Z. H., and Guo, Y., “Distributed Velocity-Free Attitude Coordination Control with Torque Constraint,” Journal of Circuits, Systems and Computers, Vol. 30, No. 11, 2021, Paper 2150200. https://doi.org/10.1142/S0218126621502005 

- [21] Zou, A. M., Tang, Y. L., Yu, X. R., and Jiao, D. X., “Velocity-Free Attitude Coordination Control of Multiple Rigid Spacecraft with Practical Predefined-Time Convergence,” International Journal of Robust and Nonlinear Control, Vol. 34, No. 18, 2024, pp. 12,109–12,128. https://doi.org/10.1002/rnc.7610 

- [22] Wu, C. H., Yan, J. G., Shen, J. H., Wu, X. W., and Xiao, B., “Predefined-Time Attitude Stabilization of Receiver Aircraft in Aerial Refueling,” IEEE Transactions on Circuits and Systems II—Express Briefs, Vol. 68, No. 10, 2021, pp. 3321–3325. https://doi.org/10.1109/TCSII.2021.3067695 

- [23] Xie, S. Z., and Chen, Q., “Adaptive Nonsingular Predefined-Time Control for Attitude Stabilization of Rigid Spacecrafts,” IEEE Transactions on Circuits and Systems II—Express Briefs, Vol. 69, No. 1, 2022, pp. 189–193. https://doi.org/10.1109/TCSII.2021.3078708 

- [24] Guo, Y., Xiao, C., Zhang, D. W., Li, A. J., and Wang, C. Q., “FullOrder Sliding Mode Control for the Attitude of the Spacecraft Under 

Input Saturation,” Journal of Aerospace Engineering, Vol. 36, No. 5, 2023, Paper 04023049. 

https://doi.org/10.1061/JAEEEZ.ASENG-4598 

- [25] Yang, H., You, X., Xia, Y., and Li, H., “Adaptive Control for Attitude Synchronisation of Spacecraft Formation via Extended State Observer,” IET Control Theory & Applications, Vol. 8, No. 18, 2014, pp. 2171–2185. https://doi.org/10.1049/iet-cta.2013.0988 

- [26] Guo, Y., Huang, B., Guo, J. H., Li, A. J., and Wang, C. Q., “VelocityFree Sliding Mode Control for Spacecraft with Input Saturation,” Acta Astronautica, Vol. 154, 2019, pp. 1–8. https://doi.org/10.1016/j.actaastro.2018.10.045 

- [27] Huang, B., Li, A. J., Guo, Y., and Wang, C. Q., “Rotation Matrix Based Finite-Time Attitude Synchronization Control for Spacecraft with External Disturbances,” ISA Transactions, Vol. 85, 2019, pp. 141–150. 

https://doi.org/10.1016/j.isatra.2018.10.027 

S. Chung Associate Editor 

