## Engineering Notes 

## Kinematic Path Planning for Underactuated Spacecraft Reorientation with Forbidden Pointing Constraints 

Chao Duan[∗] and Qinglei Hu[†] Beihang University, 100191 Beijing, People’s Republic of China Zheng H. Zhu[‡] 

York University, Toronto, Ontario M3J 1P3, Canada 

and 

Xiaodong Shao[§] and Huai-Ning Wu[¶] Beihang University, 100191 Beijing, People’s Republic of China 

https://doi.org/10.2514/1.G006968 

## I. Introduction 

W ITH increasing concerns about flight safety, constrained atti-tude planning techniques have drawn significant attention. In spacecraft reorientation applications, these safety requirements can manifest themselves as attitude-pointing constraints. On the other hand, the spacecraft may become underactuated due to actuator failures during their long-life operations in space, which would lead to constrained attitude planning being more challenging. 

Recently, a new geometrical planning method (namely, nonnominal Euler axis/angle rotation) has arisen. In Ref. [1], the nonnominal Euler axis/angle was proposed to describe the spacecraft feasible attitude when one of the three independent control torques is not available. Then, a one-step admissible nonnominal rotation scheme was given such that a single body-fixed axis could be accurately pointed toward an arbitrary axis in space [2]. However, concerning the three-axis attitude, there was always an alignment error for the one-step nonnominal Euler rotation. Avanzini and Giulietti introduced a new two-step nonnominal rotation scheme to achieve an arbitrary exact three-axis attitude maneuver [3], and they extended the approach to accommodate multistep maneuvering [4]. In Ref. [5], a feedback control scheme was addressed such that a single-axis attitude could be maneuvered exactly to the desired exact pointing with disturbance suppression. However, the potential attitude constraints for the spacecraft were not considered in these works. In this 

direction, only very few results are available in the literature nowadays. For instance, de Angelis et al. [6] employed the aforementioned nonnominal Euler rotation to perform a single-axis reorientation in the presence of one conical forbidden region. However, it assumed that the unactuated axis was constant in the inertial space, which is applicative to some special underactuated spacecraft, such as the spacecraft equipped with magnetic torque. Duan et al. [7] further expanded the scheme to the general cases in which the unactuated axis was fixed with the spacecraft body. Nevertheless, the three-axis constrained reorientation was still not achieved by the offered methods of Refs. [6,7]. 

Based on the aforementioned concerns, the challenging problem of three-axis attitude path planning for underactuated spacecraft with one conical attitude-forbidden pointing constraint is addressed. First, the feasible two-step maneuvering scheme is developed to reach any three-axis orientation for the unconstrained spacecraft, and an optimal angular traveling path is obtained from these feasible ones. Different from Refs. [3,4], the assumption of a zero value for the second element of the nominal Euler axis is relaxed. Subsequently, the boundary solutions of the unconstrained planning scheme for the forbidden pointing region are provided. A feasible domain for the unconstrained attitude planning scheme is proposed in relation to the forbidden pointing zone, which manifests as parameter intervals bounded by the boundary solutions. Moreover, the optimal planning scheme yielding the attitude constraint is proposed. 

## II. Preliminaries 

Considering two arbitrary reference frames represented by FP and FQ, the coordinate system’s rotation from FP to FQ can be expressed by the rotation matrix RQP ∈ SO�3�, where SO�3� is the special orthogonal group, and I denotes the identity matrix. The rotation from FP to FQ is achieved by only one pair of Euler axis/angle ( ^ge∕ϕe) that is referred to as the nominal Euler axis/angle. However, the coordinate transformation cannot be achieved by the nominal Euler rotation sometimes; for example, the control inputs cannot be aligned along the Euler axis due to the actuator faults of the spacecraft. Therefore, it is necessary to introduce the “nonnominal” Euler rotation [1]. For instance, a two-step nonnominal rotation from FP to FQ is given by 

^ ^ ^ where RGP � R�g1; ϕ1� and RQG � R�g2; ϕ2�,with gi∕ϕi (i � 1; 2) being the corresponding nonnominal Euler axis/angle. This shows that a one-step nominal Euler rotation is substituted with a two-step nonnominal Euler rotation. 

## III. Problem Formulation 

Throughout the Note, the spacecraft body frame is denoted by FB ≡ fO; b[^] x; b[^] y; b[^] zg, where O is located in the center of mass of the spacecraft; b[^] i (i � x; y) aligns along the one direction of the two available angular momenta; and b[^] z � b[^] represents the unactuated direction of the spacecraft, as shown in Fig. 1. Let FI and FT denote two reference frames, which coincide with the initial and desired positions of the coordinate axes of FB, respectively. The rotation ^ ^ matrix from FI to FT is expressed by RTI � R�e; θ�, where e � �e1; e2; e3�[T] and θ are the nominal Euler axis and angle, respectively. Moreover, the sensitive axis that coincides with the boresight of the positions ofsensitive payloadσ^ are represented byis denoted byσ^σ0^ and∈ Rτ^[3] , respectively. In addition,. The initial and desired 

Fig. 1 Illustration of reference frames and pointing constraints. 

and 

^ where α1 ∈ �−π; π�, g[can] 2[i] and ϕ[can] 2[i] (i � 1; 2) are two candidate ^ tangent function, andsolutions for g2 ande^ϕ∕θ2, is the nominal Euler axis/angle.tan[−][1] �⋅; ⋅� is the four-quadrant inverse 

The derivation here is similar to that in Ref. [4], with the exception of e2 ≠ 0. 

the forbidden region is represented by a cone with the center axis ofl^ ∈ R[3] and the semiaperture of λ ∈ �0; π∕2�, as shown in Fig. 1. Next, for the underactuated spacecraft maneuvered by two internal torques, we assume the following: 

Assumption 1: The total angular momentum of the spacecraft is zero, which provides the small-time local controllability for the system [8]. Note that the applicability of the methods proposed in this Note is limited when the angular momentum becomes significant, and it cannot be neglected in practical scenarios. 

Problem 1 (unconstrained reorientation): Considering any initial orientation FI and target orientation FT in inertial space, the spacecraft denoted by body frame FB is required to slew from FI to FT: 

whereσrespectively. Moreover, the sensitive axis^ 0 toward ϕ ∈τ^Rsuch that and ϕα ∈ R denote the instant and total angular travel,σ^ needs to maneuver from 

the sensitive axisProblem 2 (constrainedσ^ is further required byreorientation): Considering Problem 1, 

## IV. Main Results 

In this section, the feasible and minimum traveling angular solutions to Problems 1 and 2 are presented. 

## A. Planning Schemes for Problem 1 

In Ref. [4], it is demonstrated that any three-axis rest-to-rest reorientation can be accomplished by a two-step process of the nonnominal Euler rotation. Building upon the findings of Ref. [4], the feasible solutions to Problem 1 are presented as follows: 

^ ^ Proposition 1: Let RGI � RGI�g1; ϕ1� and RTG � RTG�g2; ϕ2� denote two nonnominal Euler rotations. Then, Problem 1 can be achieved by the successive^ RGI and RTG, and the nonnominal Euler axes/angles gi∕ϕi (i � 1; 2) are given as 

In the following, the relation between α1 and α2 ^is investigated,^ where α2 ∈ �−π; π� forms the coordinate of g2 with g2 � �cos α2; sin α2; 0�[T] . Comparing it with Eq. (7), we get tan�α2�� r22∕r21, where r2i; i � 1; 2; 3 is the component of r2. Substituting Eq. (6) into it, the expression of α1 in terms of α2 can be obtained as 

α1 � h[−][1] �α2� 

In the following, the minimum angular traveling solution for Problem 1 is investigated. First, the cost function for representing the total angular traveling length is defined as 

where ϕi ∈ �0; 2π� (i � 1; 2) is given in Eqs. (6) and (8). One can write Jϕ � f�α1� in a compact form because both ϕ1 and ϕ2 are functions of α1. Then, the minimum angular traveling solution for Problem 1 is presented. 

Proposition 2: The four candidate optimal solutions for cost function (14) are given by 

where 

and α[can] 1;o[i][∈][�][−][π][;][ π][�][(][i][ �][1][;][ 2][;][ 3][;][ 4][). Then, the global optimal solution] for the cost function is given by 

Derivation: Taking the derivative of f along α1 yields 

where 

dϕ1 2e3�e2 sin α1 � e1 cos α1� � (18) dα1 c 

(19) 

and a � e2 cos θ∕2 � e1e3 sin θ∕2, b � −e1 cos θ∕2 � e2e3 sin θ∕2, and c ��e2 cos α1 − e1 sin α1�[2] � e[2] 3[.] Letting df∕dα1 � 0, it follows that 

Then, using the root formula of the linear quadratic equation, four candidate solutions in α1 ∈ �−π; π� can be obtained as in Eq. (15). Furthermore, the optimal one among these candidate solutions should be Eq. (16). □ 

Furthermore, α[can] 1;hi[1][;can][2] ; i � 1; 2 is obtained directly from using Eq. (13). 

Recalling that the planning solutions to Problem 2 should be the subset of the solutions to Problem 1, the feasible solutions for Problem 2 are given as the following proposition: 

Proposition 4: For Problem 2, considering that the two successive nonnominal Euler rotations are given by Eqs. (5–8) and the eight candidate solutions of the boundary paths are given by Eqs. (22) and (23), then the attitude paths exist in the feasible region denoted by Z that is given as 

   - α1 ∈ �−π; a1� ∪ �a2; a3� ∪ �a4; a5� ∪ �a6; a7� ∪ �a8; π�; con1 or con13 

   - α1 ∈ �a1; a2� ∪ �a5; a6�; con2 or con14 

- B. Planning Schemes for Problem 2 

Considering Proposition 1, the forbidden pointing constraint [Eq. (4)] can be further translated into 

^ where f1 �jγ1 − μ1j − λ, f2 �jγ2 − μ2j − λ, μ1 � cos[−][1] �σ[T] 0I[g][^][1][�][,] γ1 � cos[−][1] �l[^][T] I[g][^] 1[�][,][μ] 2[�][cos][−][1][�][τ][^][T] T[g][^] 2[�][,][γ] 2[�][cos][−][1][�][l][^][T] T[g][^] 2[�][,][τ][^] T[�][σ][^] 0I[,] and l[^] T � RTIl[^] I. 

Subsequently, the boundary planning paths are investigated in terms of one attitude-forbidden pointing constraint. 

Proposition 3: For Problem 2, either of the two paths of σ^ in the firstboundary path. Then, the path ofor second step of the nonminimalσ^ is tangent torotation Λis referred in the first step ofto as the the rotation at α1 � αcan1;ri j[,][and][the][path][of][σ][^][is][tangent][to][Λ][in][the] second step of the rotation at α1 � αcan1;hij[(][i; j][ �][1][;][ 2][), where] 

and 

where 

k � 0;�1, A1 ���lI1 − σ1 cosλ�∕sinλ, B1 ���lI2 − σ2 cos λ�∕sinλ, Cσ[2] 11[�] �[σ] A[2] 2[2] 1[−][−][2][B][,][2] 1[�] l^I[σ] ��[2] 1[−] lI[σ] 1[2] 2; l[,][ D] I2; l[1] I[�] 3�[T][2] ;[�][A][1] A[B] 2[1] ���[�][σ][1][σ] lT[2] 1[�] −[,][ E] σ[1] 1[�] cos[A] λ[2] 1 �[�] ∕ sin[B][2] 1 λ[�] , B2 ���lT2 − σ2 cos λ�∕ sin λ, C2 � A[2] 2[−][B][2] 2[�][σ][2] 1[−][σ][2] 2[,] D^2 � 2�A2B2 � σ1σ2�, E2 � A[2] 2[�][B][2] 2[�][σ][2] 1[�][σ][2] 2[−][2][,] and lT � �lT1; lT2; lT3�[T] � RTIl[^] I. For Proposition 3, note that α[can] 1;ri[1][;can][2] and α[can] 2;ri[1][;can][2] ; i � 1; 2, can be obtained by extending the results for the boundary solutions (see equation 16 in Ref. [6]) defined in α1 ∈ �−π∕2; π∕2� to α1 ∈ �−π; π�. 

   - α1 ∈ �a3; a4� ∪ �a7; a8�; con3 or con15 

   - α1 ∈ �a2; a3� ∪ �a6; a7�; con5 or con17 

- α1 ∈ �a1; a2� ∪ �a5; a6�; con6 or con18 

- Z � α1 ∈ �a3; a4� ∪ �a7; a8�; con7 or con19 

   - α1 ∈ �−π; a1� ∪ �a4; a5� ∪ �a8; π�; con8 or con20 α1 ∈ �a1; a2� ∪ �a3; a4� ∪ �a5; a6� ∪ �a7; a8�; con9 or con21 α1 ∈ �−π; a1� ∪ �a4; a5� ∪ �a8; π�; con10 or con22 α1 ∈ �a2; a3� ∪ �a6; a7�; con11 or con23 

   - ∅; con4 or con12 or con16 or con24 

where 

where sort�x� represents the sorted arguments of any vector x in ascending order, and coni (i � 1; : : : ; 24) can be seen in the Appendix. 

Derivation: First, noting that f1 (f2) repeats at intervals of π, the solutions of Eq. (21) can be considered in a1 ∈ �0; π�. Then, the possible tendencies of fi (i � 1; 2) are depicted in Fig. 2. For instance, in Figs. 2a–2d, one group of the curves of fi (i � 1; 2) are presented, in which a5; a6 ∈ A1 and a7; a8 ∈ A2; and the solution intervals with respect to both f1 > 0 and f2 > 0 can be described by the sign of the function values on these roots. Specifically, in the case of fa[f] 6[2][�][1][ and][ fa] 7[f][1][�][1][ (see Fig. 2a), the interval is] 

as highlighted by the green lines; similarly, for fa[f] 6[2][�][1][ and][ fa] 7[f][1][�] −1 (see Fig. 2b), one can obtain that Z �fα1jα1 ∈ �a1; a2� ∪ �a5; a6�g. For fa[f] 6[2][�][−][1][and][fa][f] 7[1][�][1][(see][Fig.][2c),][one][has] Z �fα1jα1 ∈ �a3; a4� ∪ �a7; a8�g. For fa[f] 6[2][�][−][1][and][fa][f] 7[1][�][−][1] (see Fig. 2b), Z � ∅. Collecting those results gets 

Repeating similar progress for the cases for Figs. 2e–2l, the complete solution is obtained for Eq. (21). □ 

Fig. 2 Rootsfor f 1 � 0 and f 2 � 0 in �0;π�, where the concerned root for f 1 � 0 is markedby the blue squareand the onefor f 2 � 0 ismarkedby thered triangle. 

Proposition 5: If the guidance strategy is given by Proposition 1 and the definition field for the strategy is constrained by Z, then the minimum angular traveling solution for Problem 2 is given by 

where ai;z (i � 1; 2; : : : ) is the boundary value of Z. 

The derivation of Proposition 5 is a direct result, and it is omitted here. 

## V. Numerical Simulations 

In this section, a series of numerical examples is conducted to demonstrate the effectiveness of the proposed planning schemes. The initial reference frame is assumed to coincide with the body frame. The unactuated axis in the body frame is assumed as b[^] B ��0; 0; 1�[T] , ^ and the sensitive axis is σB ��0.843; 0.2; −0.5�[T] . The forbidden zone is fixedin the initial reference framewith the forbidden direction of l[^] I ��0.686; 0.7; 0.2�[T] and a semiaperture of λ � 0.524 rad. 

The attitude planning schemes presented in Propositions 4 and 5 for Problem 2 are verified, which also includes the validation of 

Fig. 4 Candidateoptimal solutions with respect tofeasibleregions, where the minimum solution for Jϕ is marked by a green circle. Also, the maximum and two medium solutions for Jϕ are marked by red squares. 

Propositions 1–3. The nominal Euler axis is assumed to be ^ e ��0.3; −0.2; 0.933�[T] , and the nominal Euler angle is selected as θ � 1.745 rad. 

Fig. 3 Boundary paths in 3 deg space: a) external-tangent path for first step, b) dual case of Fig. 3a, c) internal-tangent path for first step, d) dual case of Fig. 3c, e) external-tangent path for second step, f) dual case of Fig. 3e, g) internal-tangent path for second step, and h) dual case of Fig. 3g. 

Table 1 Candidate optimal solutions’ parameters 

|Solutions|α1, rad|^g1|ϕ1, rad|^g2|ϕ2, rad|Jϕ, rad|
|---|---|---|---|---|---|---|
|1|−1.695|�−0.124; −0.992; 0�T|2.476|�0.868; 0.496; 0�T|2.476|4.952|
|2|−0.179|�0.984; −0.178; 0�T|3.447|�−0.541; 0.841; 0�T|2.835|6.282|
|3|1.447|�0.124; 0.992; 0�T|3.807|�−0.868; −0.496; 0�T|3.808|7.615|
|4|2.963|�−0.984; 0.178; 0�T|2.836|�0.541; −0.841; 0�T|3.448|6.284|

Fig. 5 Global optimal path in three-dimensional space. 

Fig.The3, partialwhere theboundaryattitudesolutionspaths arisingfor Problemfrom g^[can] 22[2] areandpresentedϕ[can] 2[2] (seein Proposition 1) are omitted for brevity. It shows that the desired orientation can be achieved without an alignment error; meanwhile, the paths of the concerned sensitive axis are externally or internally tangent to the forbidden zone in either the first step or the second step of maneuvering. It is noteworthy that in Figs. 3c, 3d, 3g, and 3h, ^σ still enters the forbidden zone, which implies that not all the boundary solutions are feasible solutions, as well as that the feasible regions are needed to sift out the practical feasible ones. 

Then, Proposition 5 for the optimal planning scheme is validated. In Fig. 4, the total rotation angle for the two-step rotation scheme as well as the four candidate optimal solutions are presented, which show that Proposition 5 provides a practical shortest path. Moreover, the detailed information about the four candidate solutions is given in Table 1, and optimal planning path consistent with the attitude forbidden pointing constraint in three-dimensional space is depicted in Fig. 5. 

## VI. Conclusions 

A new path planning strategy based on the nonnominal Euler parameter is proposed for the constrained attitude retargeting of an underactuated spacecraft. The excellent features of the designed planning strategy hinge on the fact that the optimal path planning is accomplished for the three-axis attitude reorientation under one missing degree of freedom in control and one conical attitudeforbidden pointing constraint. Moreover, the proposed scheme can be implemented quickly, benefitting from its analytical nature. 

## Appendix: The Categorization Conditions for Proposition 4 

The conditions for Proposition 4 are given as 

con1∶a5; a6 ∈ A1; a7; a8 ∈ A2; fa[f] 6[2][�][1][; fa] 7[f][1][�][1] con2∶a5; a6 ∈ A1; a7; a8 ∈ A2; fa[f] 6[2][�][1][; fa] 7[f][1][�][−][1] con3∶a5; a6 ∈ A1; a7; a8 ∈ A2; fa[f] 6[2][�][−][1][; fa] 7[f][1][�][1] con4∶a5; a6 ∈ A1; a7; a8 ∈ A2; fa[f] 6[2][�][−][1][; fa] 7[f][1][�][−][1] 

con5∶a6 ∈ A2; a7 ∈ A1; a8 ∈ A2; fa[f] 6[1][�][1][; fa] 7[f][2][�][1] con6∶a6 ∈ A2; a7 ∈ A1; a8 ∈ A2; fa[f] 6[1][�][1][; fa] 7[f][2][�][−][1] con7∶a6 ∈ A2; a7 ∈ A1; a8 ∈ A2; fa[f] 6[1][�][−][1][; fa] 7[f][2][�][1] con8∶a6 ∈ A2; a7 ∈ A1; a8 ∈ A2; fa[f] 6[1][�][−][1][; fa] 7[f][2][�][−][1] con9∶a6 ∈ A2; a7 ∈ A2; a8 ∈ A1; fa[f] 5[2][�][1][; fa][f] 6[1][�][1] con10∶a6 ∈ A2; a7 ∈ A2; a8 ∈ A1; fa[f] 5[2][�][1][; fa][f] 6[1][�][−][1] con11∶a6 ∈ A2; a7 ∈ A2; a8 ∈ A1; fa[f] 5[2][�][−][1][; fa] 6[f][1][�][1] con12∶a6 ∈ A2; a7 ∈ A2; a8 ∈ A1; fa[f] 5[2][�][−][1][; fa][f] 6[1][�][−][1] con13∶a5; a6 ∈ A2; a7; a8 ∈ A1; fa[f] 6[1][�][1][; fa] 7[f][2][�][1] con14∶a5; a6 ∈ A2; a7; a8 ∈ A1; fa[f] 6[1][�][1][; fa] 7[f][2][�][−][1] con15∶a5; a6 ∈ A2; a7; a8 ∈ A1; fa[f] 6[1][�][−][1][; fa] 7[f][2][�][1] con16∶a5; a6 ∈ A2; a7; a8 ∈ A1; fa[f] 6[1][�][−][1][; fa] 7[f][2][�][−][1] con17∶a5 ∈ A2; a6 ∈ A1; a7 ∈ A2; a8 ∈ A1; fa[f] 6[2][�][1][; fa] 7[f][1][�][1] con18∶a5 ∈ A2; a6 ∈ A1; a7 ∈ A2; a8 ∈ A1; fa[f] 6[2][�][1][; fa] 7[f][1][�][−][1] con19∶a5 ∈ A2; a6 ∈ A1; a7 ∈ A2; a8 ∈ A1; fa[f] 6[2][�][−][1][; fa] 7[f][1][�][1] con20∶a5 ∈ A2; a6 ∈ A1; a7 ∈ A2; a8 ∈ A1; fa[f] 6[2][�][−][1][; fa] 7[f][1][�][−][1] con21∶a5 ∈ A2; a6 ∈ A1; a7 ∈ A1; a8 ∈ A2; fa[f] 5[1][�][1][; fa][f] 6[2][�][1] con22∶a5 ∈ A2; a6 ∈ A1; a7 ∈ A1; a8 ∈ A2; fa[f] 5[1][�][1][; fa] 6[f][2][�][−][1] con23∶a5 ∈ A2; a6 ∈ A1; a7 ∈ A1; a8 ∈ A2; fa[f] 5[1][�][−][1][; fa][f] 6[2][�][1] con24∶a5 ∈ A2; a6 ∈ A1; a7 ∈ A1; a8 ∈ A2; fa[f] 5[1][�][−][1][; fa] 6[f][2][�][−][1] where A1 ��α[can] 1;r1[1][;][ α] 1[can] ;r1[2][;][ α] 1[can] ;r2[1][;][ α] 1[can] ;r2[2][�][T] and A2 ��α[can] 1;h1[1][;][ α] 1[can] ;h1[2][;] α[can] 1;h2[1][;][ α] 1[can] ;h2[2][�][T][;][ fa] fi j[�][1][ if][ f] j[�][a] i[�][≥][0][; and][ fa] fi j[�][−][1][ otherwise.] 

## Acknowledgments 

This work was supported in part by the National Natural Science Foundation of China under grants 62227812 and 61960206011. It was also supported in part by the Zhejiang Provincial Natural Science Foundation under grant LD22E050004. 

## References 

- [1] Giulietti, F., and Tortora, P., “Optimal Rotation Angle About a Nonnominal Euler Axis,” Journal of Guidance, Control, and Dynamics, Vol. 30, No. 5, 2007, pp. 1561–1563. https://doi.org/10.2514/1.31547 

- [2] Avanzini, G., and Giulietti, F., “Constrained Slews for Single-Axis Pointing,” Journal of Guidance, Control, and Dynamics, Vol. 31, No. 6, 2008, pp. 1814–1817. https://doi.org/10.2514/1.38291 

- [3] Avanzini, G., and Giulietti, F., “Kinematic Planning of Slew Manoeuvres After Actuator Failure for Low-Cost Satellites,” Journal of Loss Prevention in the Process Industries, Vol. 22, No. 5, 2009, pp. 649–656. https://doi.org/10.1016/j.jlp.2009.04.008 

- [4] Avanzini, G., Berardo, L., Giulietti, F., and Minisci, E. A., “Optimal Rotation Sequences in Presence of Constraints on Admissible Rotation Axes,” Journal of Guidance, Control, and Dynamics, Vol. 34, No. 2, 2011, pp. 554–563. https://doi.org/10.2514/1.49805 

- [5] Zavoli, A., De Matteis, G., Giulietti, F., and Avanzini, G., “Single-Axis Pointing of an Underactuated Spacecraft Equipped with Two Reaction Wheels,” Journal of Guidance, Control, and Dynamics, Vol. 40, No. 6, 2017, pp. 1465–1471. https://doi.org/10.2514/1.G002182 

- [6] de Angelis, E. L., Giulietti, F., and Avanzini, G., “Single-Axis Pointing of Underactuated Spacecraft in the Presence of Path Constraints,” Journal of Guidance, Control, and Dynamics, Vol. 38, No. 1, 2014, pp. 143–147. https://doi.org/10.2514/1.G000121 

- [7] Duan, C., Hu, Q. L., Zhang, Y. M., and Wu, H. N., “Constrained SingleAxis Path Planning of Underactuated Spacecraft,” Aerospace Science and Technology, Vol. 107, Dec. 2020, Paper 106345. https://doi.org/10.1016/j.ast.2020.106345 

- [8] Krishnan, H., McClamroch, N. H., and Reyhanoglu, M., “Attitude Stabilization of a Rigid Spacecraft Using Two Momentum Wheel Actuators,” Journal of Guidance, Control, and Dynamics, Vol. 18, No. 2, 1995, pp. 256–263. 

https://doi.org/10.2514/3.21378 
