# **Nonlinear Model Predictive Detumbling of Small Satellites with a Single-axis Magnetorquer** 

Kota Kondo[∗] 

_Kyushu University, Fukuoka-shi, Fukuoka, 819-0395, Japan_ 

Ilya Kolmanovsky[†] 

_University of Michigan, Ann Arbor, Michigan 48109_ 

Yasuhiro Yoshimura[‡] 

_Kyushu University, Fukuoka-shi, Fukuoka, 819-0395, Japan_ 

Mai Bando[§] 

_Kyushu University, Fukuoka-shi, Fukuoka, 819-0395, Japan_ 

Shuji Nagasaki[¶] 

_Kyushu University, Fukuoka-shi, Fukuoka, 819-0395, Japan_ 

Toshiya Hanada[‖] 

_Kyushu University, Fukuoka-shi, Fukuoka, 819-0395, Japan_ 

## **Nomenclature** 

- _𝑩_ , _𝑩_ **0** = Earth’s magnetic field vector in body and orbital frame, respectively 

_𝐻_ 

   - = Hamiltonian 

- _𝑖_ = inclination 

- _𝐽_ cost = cost function 

- _𝑱_ = moment of inertia 

- _𝒎_ = magnetic dipole moment 

- _𝑚_ max = maximum control input of magnetic torquer 

- _𝑚 𝑥_ = magnetic moment of the single magnetic torquer along the _𝑥_ -axis 

- _𝑁_ = discretized step number on prediction horizon 

- _𝑸, 𝑅_ 1 _, 𝑅_ 2 = weight matrices 

- _𝑸_ **t** = terminal cost 

- _𝑟_ = distance between the satellite and the center of the Earth 

- _𝑻_ = control torque vector 

- _𝑇𝑠_ = prediction horizon 

- _𝑼_ = optimal input matrix 

- _𝑉_ = Lyapunov function 

- _𝑣_ = dummy input 

- _𝜆_ = Lagrange multiplier 

- _𝝎_ = angular velocity vector 

- _𝜔𝑒_ = argument of perigee 

- _𝜇_ = Lagrange multiplier for equality constraint 

- _𝜃_ = true anomaly 

Subscripts 

- _𝑖_ = _𝑖_ -th time step on prediction horizon 

- ∗ = conditions on prediction horizon 

## **I. Introduction** 

arious actuators are used in spacecraft to achieve attitude stabilization, including thrusters, momentum wheels, Vand control moment gyros. [1–3]. Small satellites, however, have stringent size, weight, and cost constraints, which makes many actuator choices prohibitive. Consequently, magnetic torquers have commonly been applied to spacecraft to attenuate angular rates [4, 5]. Approaches for dealing with under-actuation due to magnetic control torques dependency on the magnetic field and required high magnetic flux densities have been previously considered in [6, 7]. 

Generally speaking, control of a satellite that becomes under-actuated as a result of on-board failures has been a recurrent theme in the literature, see e.g., [8, 9] and references therein. Methods for controlling spacecraft with fewer actuators than degrees of freedom are increasingly in demand due to the increased number of small satellite launches [10]. 

Magnetic torquers have been extensively investigated for momentum management of spacecraft with momentum wheels [11] and for nutation damping of spin satellites [12], momentum-biased [13], and dual-spin satellites [14]. Nonetheless, severely under-actuated small spacecraft that carry only a single-axis magnetic torquer have not been previously treated. 

This note considers the detumbling of a small spacecraft using only a single-axis magnetic torquer. Even with a three-axis magnetic torquer, the spacecraft is under-actuated, while, in the case of only a single axis magnetic torquer, the problem is considerably more demanding. Our note examines the feasibility of spacecraft attitude control with a single-axis magnetic torquer and possible control methods that can be used. 

Our specific contributions are as follows. We demonstrate, through analysis and simulations, that the conventional B-dot algorithm for spacecraft detumbling fails with a single-axis magnetic torquer. Also, there has not been any previous analysis of a satellite’s controllability and stabilizability with a single magnetic actuator. We discuss these properties; this discussion motivates consideration of more advanced control approaches such as Nonlinear Model Predictive Control (NMPC) [15, 16]. Closed-loop simulation results with NMPC are reported, which illustrate the potential of NMPC to perform spacecraft detumbling with a single-axis magnetic torquer. These developments, which show the improved capability to detumble the spacecraft with NMPC as compared to the classical B-dot law in the case of a single magnetic torquer, contribute to advancements in small satellite technology. 

## **II. Spacecraft Rotational Dynamics** 

The body-fixed frame of a rigid spacecraft is assumed to be located at the center of mass and to be aligned with the principal axes of inertia. The evolution of body frame components of spacecraft angular velocity vector is described by the classical Euler’s equations [17]: 

A single-axis magnetic torquer interacts with the Earth’s local magnetic field and generates control torque according to [18]: 

Given a single-axis magnetic torquer with the coil along the _𝑥_ -axis, the control torque with the single-axis magnetorquer is written as _𝑻_ = [0 _,_ − _𝐵𝑧𝑚 𝑥, 𝐵𝑦𝑚 𝑥_ ] _[𝑇]_ . Thus, no torque is generated about the _𝑥_ -axis, which makes angular rate stabilization challenging. By aggregating the above equations, we obtain 

where _𝑚 𝑥_ is the control input. 

## **III. Detumbling Control Law** 

## **A. B-dot Algorithm** 

The conventional approach to detumbling small satellites with magnetic torquers exploits the B-dot algorithm [19]. The B-dot algorithm’s principle is to add damping through control moments, which leads to a reduction in spacecraft angular velocities. This section analyzes closed-loop stability with the B-dot law in the case of a single-axis magnetic 

torquer. Considering the Euler’s equations for spacecraft dynamics given is Eq. (1), define a Lyapunov function [19] as 

This Lyapunov function is positive everywhere except when _𝝎_ = 0 ( i.e., the equilibrium point). With the use of Eq. (1), its time derivative along trajectories of the system is found as 

The conventional B-dot feedback law in [20, 21] generates each axis magnetic dipole moment as follows. For instance, for the _𝑥_ -axis, 

where _𝑚_ max is the maximum magnitude of the magnetic dipole moment. 

The time derivative of Earth’s magnetic field vector _𝑩_ with respect to an inertial frame is given by 

where the left superscript _𝐼_ on _𝑩_ indicates ”with respect to an inertial frame.” Assuming sufficiently large angular velocity, _𝝎_ , so that _[𝐼] 𝑩_ **[�]** is small enough in magnitude compared to _𝑩_ **[�]** , where the latter is the derivative of Earth’s magnetic field vector with respect to a body fixed frame, Eq. (9) can be approximated as [4, 21]. 

Following [22], which treated the case of three single-axis magnetic torquers, suppose we proceed with closed-loop stabilizability analysis by computing the time derivative of _𝑉_ along closed-loop system trajectories. In the case of a 

single-axis magnetic actuation, we obtain 

It is clear that, although _𝑉_[�] ( _𝝎_ ) ≤ 0, the expression for _𝑉_[�] does not depend on _𝜔𝑥_ . Furthermore, since _𝑉_ ( _𝑡_ ) = _𝑉_ ( _𝝎_ ( _𝑡_ )) is a non-increasing function of _𝑡_ , _𝝎_ is bounded. Moreover _𝑉_[�] is a continuous function of _𝑩_ and _𝝎_ , which are bounded. Hence, _𝑉_[�] ( _𝑡_ ) is uniformly continuous in time [23]. Therefore, by Barbalat’s lemma, we conclude lim _𝑡_ →∞ _𝑉_[�] ( _𝑡_ ) = 0, which indicates that in the limit as _𝑡_ →∞ either _𝜔𝑦 𝐵𝑧_ = _𝜔𝑧 𝐵𝑦_ = 0 or _𝜔𝑦_ = _𝜔𝑧_ = 0. In the either case, since _𝜔𝑥_ can be arbitrary, B-dot algorithm appears to be incapable of detumbling a satellite with a single-axis magnetic actuation. This is confirmed by our subsequent numerical simulations. 

## **B. Controllability Analysis** 

This section discusses the controllability properties of the satellite angular velocity dynamics with the single-axis magnetic actuation. Note that both three and two-axis magnetically actuated satellites have been shown to be controllable [24, 25]. The single-axis magnetic actuation case is more challenging since there is only a single control input; this case has not previously been addressed in the literature. 

For the case of single-axis magnetic actuation, local weak controllability in the sense of [26, 27] can be demonstrated. The weak local controllability is necessary for local controllability. It implies that the set of reachable states at a given time from a given state starting at another given time instant contains an open neighborhood of the state space. Clearly, weak local controllability is necessary but not sufficient for local controllability. 

Note that equations of motion (3) can be written as 

The system is time-varying as _𝐵𝑦_ and _𝐵𝑧_ depend on the spacecraft position in orbit and hence on time. The necessary and sufficient conditions for local weak controllability of a time-varying nonlinear system can be obtained by extending � the state vector with the time, _𝑡_ , as an extra state with the dynamics _𝑡_ = 1, and analyzing the resulting autonomous system. This leads to conditions such as Theorem 4 in [28] which we adopt here. 

Define the operators ⟨ _𝜉, 𝜂_ ⟩ and [ _𝜉, 𝜂_ ] for two time-varying vector fields _𝜉_ an _𝜂_ as 

Note that [ _𝜉, 𝜂_ ] is the conventional Lie Bracket. The controllability distribution △ can now be defined for our time-varying nonlinear system based on Algorithm 1. 

**Algorithm 1:** Controllability distribution for time-variant nonlinear systems 

Set △0 = span{ _𝑓_ 1} and _𝑘_ = 0; 

## **end** 

⊕ sums up the span of two vector fields. 

The necessary and sufficient conditions for local weak controllability in [28] lead to the following result: Algorithm 1 converges in at most 2 steps for the system (15) and △2 is nonsingular and has rank 3 at a given _𝝎_ 0 and _𝑡_ 0 if and only if the system is locally weakly controllable from ( _𝝎_ 0, _𝑡_ 0). 

By examining the form of △2 (see calculations in the Appendix), we observe that _𝐽𝑦_ ≠ _𝐽𝑧_ (unequal moments of inertia about the two principal axes which are orthogonal to the axis along which the magnetic actuator is aligned) is a necessary condition for weak local controllability (and hence for local controllability). This is also apparent from (15) as the angular velocity component about _𝑥_ -axis becomes decoupled from the rest of the dynamics if _𝐽𝑦_ = _𝐽𝑧_ . 

Furthermore, the numerical evaluation of △2 along the orbits used in our NMPC simulations with IGRF model (36) for _𝐵𝑦_ and _𝐵𝑧_ confirms that the rank of △2 is equal to 3 and hence the system is locally weakly controllable. 

We note that we cannot establish a stronger property of (small-time) local controllability of the satellite with a single-axis magnetic actuation with the above analysis. While it appears to hold in our numerical simulations, such property is much harder to demonstrate and is left to future research. Nevertheless, conditions for local weak controllability are necessary for local controllability and hence are useful. 

## **C. Stabilizability Analysis** 

Existing approaches [17, 24, 29, 30] to stabilizing the spacecraft with magnetic actuation typically rely on the application of the theory of averaging [31]. Following this route in the case of single-axis magnetic actuation encounters technical difficulties as we now illustrate. 

Consider the equations of motion given by Eq. (15) and suppose a control law of the form, 

has been specified, where _𝜖>_ 0 is a small parameter. Let 

then 

is in the form to which the theory of averaging [31] can be applied. Letting 

and assuming that the limits exist, the averaged version of Eq. (20) has the form, 

By the straightforward application of Brockett’s necessary condition [32], the averaged system given by Eq. (21) is not smoothly or even continuously stabilizable, i.e., a stabilizing time-invariant control law given by _𝑢 𝑦_ ( _𝒉_[¯] ), _𝑢𝑧_ ( _𝒉_[¯] ), if exists, would have to be discontinuous. This conclusion is consistent with the interpretation of the averaged system dynamics as similar to a rigid body controlled by two control torques; such a system is known to be not smoothly or even continuously stabilizable by time-invariant feedback laws. Consequently, _𝑚_ ¯ _𝑥_ ( _**[𝝎]** 𝜖[, 𝑡]_[)][would have to be discontinuous as a] function of _𝝎_ . Unfortunately, the theory of averaging [31, 33] assumes smoothness (twice continuous differentiability) in [31] of the right hand side of ordinary differential equations being averaged. 

Hence, there is a complication in using the classical averaging theory to develop conventional control laws for the case of single-axis magnetic actuation. On the other hand, NMPC, if suitably formulated, is able to stabilize systems that are not smoothly or even continuously stabilizable, including underactuated spacecraft [34]. 

## **D. NMPC Formulation** 

This section formulates an NMPC approach to detumbling the satellite with the nonlinear dynamics represented by Eq.(3) based on the following receding horizon optimal control problem, 

subject to _𝑱𝝎_ **�** + _𝝎_ × _𝑱𝝎_ = _𝑻_ 

where _𝑡_ is the current time instant, _𝑇𝑠_ is the prediction horizon, _𝑸_ , _𝑸_ **t** , _𝑅_ 1, and _𝑅_ 2 are positive-definite weight matrices, and _𝑸_ **t** is terminal cost. The auxiliary input, _𝑣_ , is introduced following [35] to enforce the control constraints by recasting them as equality constraints in Eq. (22). The negative sign preceding _𝑅_ 2 in the cost function being minimized promotes keeping _𝑣_ positive and control constraints strictly satisfied. This receding horizon optimal control problem is chosen as it is synergistic with the continuation/generalized minimal residual method (C/GMRES) method [36]. Reference [37] provides a comparison of different strategies to handle inequality constraints in such a setting. 

## **E. NMPC with C/GMRES Algorithm** 

The C/GMRES method [36] is applied to design NMPC based on Eq.(22). As C/GMRES method has small computational footprint, its use is advantageous in small satellites with limited computational and electric power. Following C/GMRES method, the problem is first discretized as follows: 

where _𝑓_ ( _𝝎, 𝒖_ ) is the right hand side of equations of motion in Eq.(22), _𝐶_ ( _𝝎, 𝒖_ ) is the equality constraint in Eq.(22), _𝐿_ ( _𝝎, 𝒖_ ) = 2[1][(] _[𝝎][𝑇][𝑸𝝎]_[+] _[ 𝑅]_[1] _[𝑚]_[2] _𝑥_[) −] _[𝑅]_[2] _[𝑣,]_[ and][ Δ] _[𝜏]_[=] _[ 𝑇][𝑠]_[/] _[𝑁]_[.][Setting the initial state of the discretized problem to the current] 

angular velocity vector as _𝝎_[∗] 0[(] _[𝑡]_[)][=] _[ 𝝎]_[(] _[𝑡]_[)][, a sequence of control inputs][ {] _[𝒖]_[∗] _𝑖_[(] _[𝑡]_[)}] _𝑖[𝑁]_ =0[−][1] is found at each time instant _𝑡_ ; then the control given to the system is based on the first element of this sequence and is defined as _𝒖_ ( _𝑡_ ) = _𝒖_[∗] 0[(] _[𝑡]_[)][.] 

The solution of the discretized problem is based on introducing the Hamiltonian, _𝐻_ , as 

where _𝝀_ is the vector of co-states and _𝝁_ is the Lagrange multiplier associated with the equality constraint. The first-order necessary conditions for optimality dictate [38] that { _𝒖_[∗] _𝑖_[(] _[𝑡]_[)}] _𝑖[𝑁]_ =0[−][1][,][{] _[𝝁]_[∗] _𝑖_[(] _[𝑡]_[)}] _𝑖[𝑁]_ =0[−][1][,][{] _[𝝀]_[∗] _𝑖_[(] _[𝑡]_[)}] _𝑖[𝑁]_ =0[−][1][,][satisfy][the] following conditions: 

To determine { _𝒖_[∗] _𝑖_[(] _[𝑡]_[)}] _𝑖[𝑁]_ =0[−][1] and { _𝝁_[∗] _𝑖_[(] _[𝑡]_[)}] _𝑖[𝑁]_ =0[−][1][, which satisfy Eqs.(23–25) and (28–30), we define a vector of the inputs] and multipliers in Eq. (31) as 

This vector has to satisfy the equation, 

where _𝐻_ = 12[(] _[𝝎][𝑇][𝑸𝝎]_[+] _[ 𝑅]_[1] _[𝑚]_[2] _𝑥_[) −] _[𝑅]_[2] _[𝑣]_[+] _[ 𝜆][𝑥]_[{] _𝐽_[1] _𝑥_[(] _[𝐽][𝑦]_[−] _[𝐽][𝑧]_[)] _[𝜔][𝑦][𝜔][𝑧]_[} +] _[ 𝜆][𝑦]_[{] _𝐽_[1] _𝑦_[(] _[𝐽][𝑧]_[−] _[𝐽][𝑥]_[)] _[𝜔][𝑧][𝜔][𝑥]_[−] _[𝐵][𝑧][𝑚][𝑥]_[}] (33) + _𝜆𝑧_ { _𝐽_[1] _𝑧_[(] _[𝐽][𝑥]_[−] _[𝐽][𝑦]_[)] _[𝜔][𝑥][𝜔][𝑦]_[+] _[ 𝐵][𝑦][𝑚][𝑥]_[} +] _[ 𝜇]_[{] _[𝑚]_[2] _𝑥_[+] _[ 𝑣]_[2][ −] _[𝑚]_ max[2][}] 

In C/GMRES [36, 39], Eq. (32), which has to haold at each time instant, _𝑡_ , is replaced by a stabilized version, 

and then by 

where arguments are omitted. 

Finally, _𝑈_[�] can be determined from Eq. (35) with C/GMRES resulting in a form of a predictor-corrector strategy. 

## **IV. Simulation Results** 

This section presents simulation results of both the B-dot algorithm and NMPC for a spacecraft in a sun-synchronous orbit with orbital elements given in Table 1. 

## **A. Orbital Elements** 

**Table 1 Six elements of Aeolus (sun synchronous orbit)** 

|Semi-major axis|6691.6 [km]|
|---|---|
|Eccentricity|0.00046440|
|Inclination|96.700[deg]|
|Right Ascension of Ascending Node|100.90 [deg]|
|Argument of perigee|119.70 [deg]|
|Mean anomaly|240.49 [deg]|

## **B. Earth’s Magnetic Field Model** 

We employ International Geomagnetic Reference Field (IGRF) [40] as an Earth’s magnetic model in our simulations. However, to reduce the onboard computational complexity, NMPC uses a simpler magnetic dipole model described in Eq. (36) [41]. 

where _𝜂_ = _𝜃_ + _𝜔𝑒_ , _𝐷 𝑚_ = − _[𝑀] 𝑟_[3] _[𝑒]_[,] _[ 𝑀][𝑒]_[=][ 8] _[.]_[1][ ×][ 10][25][ [gauss][ ·][ cm][3][],] _[ 𝑟]_[is a distance between the satellite and the center of the] Earth, _𝜃_ is true anomaly, and _𝜔𝑒_ is argument of perigee. Figure 1 shows that there is a slight discrepancy between the two models on the sun-synchronous orbit, which, as we will see from the simulation results, will not preclude NMPC controller from achieving detumbling. 

**Fig. 1 Magnetic field in dipole and IGRF on the sun-synchronous orbit.** 

## **C. Comparison of B-dot and NMPC on Asymmetric Satellite** 

This section demonstrates NMPC’s advantages over the B-dot algorithm, using a general satellite model whose moments of inertia are given in Table 2. The maximum magnetic moment is set to 1.0 [A · m[2] ]. 

**Table 2 Moment of inertia of the asymmetric satellite** 

|Moments of inertia|_𝐽𝑥_|_𝐽𝑦_|_𝐽𝑧_|
|---|---|---|---|
|Value [kg·m2]|0.020|0.030|0.040|

## _1. The B-dot algorithm_ 

As shown in Eq.8, the B-dot law finds control inputs depending the Earth’s magnetic field. Following the common practice, to avoid the B-dot law generating unnecessary control inputs, its implementation is based on 

## _2. NMPC_ 

We here demonstrate NMPC’s capability of detumbling the spacecraft. Simulations are conducted with the NMPC parameters listed in Table 3, which were determined by trial and error. 

**Table 3 NMPC properties for the asymmetric satellite** 

||_𝑇𝑠_||_𝑄_|||||_𝑄_t||||_𝑅_1|_𝑅_2|_𝑁_|Δ_𝜏_|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|10|[sec]|diag([104,|102,|5|×|10])|diag([104,|102,|5|×|10])|10−1|10−1|10|1.0 [sec]|

where _𝑇𝑠_ is prediction horizon, _𝑄_ , _𝑄_ t _𝑅_ 1, and _𝑅_ 2 are weight matrices, _𝑁_ is discretized step number along prediction horizon, and Δ _𝜏_ = _𝑇𝑠_ / _𝑁_ . 

The initial conditions for the four study cases for which simulation results are reported below are chosen randomly with angular velocity components between -3.0 and 3.0 [deg/s]. The initial angular velocities and the results are all given in Table 4. These four simulations are representatives of a larger number of simulation case studies that we have performed. 

**Table 4 Initial conditions and results** 

|||_𝜔𝑥_[deg/s]|_𝜔𝑦_[deg/s]|_𝜔𝑧_[deg/s]|B-dot detumbled?|NMPC detumbled?|
|---|---|---|---|---|---|---|
|Case|1|2.429286|2.878490|-0.366780|No|Yes|
|Case|2|-1.576299|-0.246907|2.778531|No|Yes|
|Case|3|0.047150|-2.486905|-1.425107|Yes|Yes|
|Case|4|-0.626909|-0.795380|2.927892|No|No|

Figure 2 to 5 present the simulation results from the four case studies. When the magnitude of all the angular velocities are less than 0.10 [deg/s], the simulations are set to be terminated in both B-dot and NMPC simulations. The simulations are run for the maximum time of 150 [min]. 

**Fig. 2 Case 1: time history of angular velocities.** 

As can be seen in Fig. 2, although _𝜔𝑦_ and _𝜔𝑧_ are well attenuated, the B-dot algorithm is not able to sufficiently reduce _𝜔𝑥_ . NMPC, in contrast, is able to achieve detumbling. 

**Fig. 3 Case 2: time history of angular velocities.** 

Figure 3 also reports an example where the B-dot law does not achieve all axes detumbling, but the NMPC algorithm does. Note that in the beginning of the trajectory, NMPC increases _𝜔𝑧_ to be able to reduce _𝜔𝑥_ , which is a challenging variable to control. 

**Fig. 4 Case 3: time history of angular velocities.** 

Figure 4 is a case where both the B-dot method and the NMPC approach achieve detumbling. This requires about 

150 [min] for the B-dot algorithm, while NMPC is able to achieve this within a much shorter time period. 

Finally, Fig. 5 showcases a plot where neither the B-dot nor NMPC are able to detumble the spacecraft within the allocated time of 150 [min]. In the case of the B-dot algorithm, _𝜔𝑥_ persists at a nonzero value. NMPC, on the other hand, is able to gradually attenuate all angular velocity components but is not able to fully detumble the spacecraft within the given time. 

**Fig. 5 Case 4: time history of angular velocities.** 

**Fig. 6 Control input comparison.** 

Figs. 6 shows the time histories of control inputs in all the four cases. As can be seen, the NMPC controller requires much smaller control inputs. This can be advantageous in terms of reduced energy consumption and reduced electrical 

disturbance. 

## **V. Conclusions** 

The satellite’s detumbling with only on a single-axis magnetic actuator is a challenging problem, in particular, requiring a different approach to stabilization than for three-axis and two-axis magnetic actuation systems. The necessary conditions for the controllability of spacecraft angular velocities involve: (1) spacecraft not having equal moments of inertia about the two principal axes that are orthogonal to the axis along which the magnetic actuator is aligned and (2) satisfying the specific rank controllability conditions derived in the note. The latter have been shown to hold numerically for the spacecraft and the orbit considered in the simulations. The classical B-dot law appears to be incapable of eliminating spacecraft rotational motion in the simulations, which has also been predicted from the theoretical analysis. The Nonlinear Model Predictive Control (NMPC) strategy based on the continuation/generalized minimal residual (C/GMRES) method has been shown to achieve detumbling within the allocated time through simulations in most cases. The possibility of detumbling spacecraft with only the single-axis magnetic coil opens the possibility for small spacecraft missions with stringent cost and packaging constraints. 

## **Appendix** 

Below are the controllability distributions in Algorithm 1: 

where _𝑔_ 1 is the second term in Eq. (41). 

where _𝑔_ 2 = ⟨ _𝑔_ 1 _, 𝑓_ 0⟩ and _𝑔_ 3 = [ _𝑔_ 1 _, 𝑓_ 1]. The explicit calculations of _𝑔_ 2 and _𝑔_ 3 give: 

## **References** 

- [1] Jin, J., Park, B., Park, Y., and Tahk, M.-J., “Attitude control of a satellite with redundant thrusters,” _Aerospace Science and Technology_ , Vol. 10, No. 7, 2006, pp. 644 – 651. https://doi.org/https://doi.org/10.1016/j.ast.2006.04.005, URL http://www.sciencedirect.com/science/article/pii/S1270963806000514. 

- [2] Zhang Fan, Shang Hua, Mu Chundi, and Lu Yuchang, “An optimal attitude control of small satellite with momentum wheel and magnetic torquerods,” _Proceedings of the 4th World Congress on Intelligent Control and Automation (Cat. No.02EX527)_ , Vol. 2, 2002, pp. 1395–1398 vol.2. https://doi.org/10.1109/WCICA.2002.1020810. 

- [3] Wie, B., “Singularity Escape/Avoidance Steering Logic for Control Moment Gyro Systems,” _Journal of Guidance, Control, and Dynamics_ , Vol. 28, No. 5, 2005, pp. 948–956. https://doi.org/10.2514/1.10136, URL https://doi.org/10.2514/1.10136. 

- [4] Avanzini, G., and Giulietti, F., “Magnetic Detumbling of a Rigid Spacecraft,” _Journal of Guidance, Control, and Dynamics_ , Vol. 35, No. 4, 2012, pp. 1326–1334. https://doi.org/10.2514/1.53074, URL https://doi.org/10.2514/1.53074. 

- [5] Lovera, M., “Magnetic satellite detumbling: The b-dot algorithm revisited,” _2015 American Control Conference (ACC)_ , 2015, pp. 1867–1872. https://doi.org/10.1109/ACC.2015.7171005. 

- [6] Wood, M., Chen, W., and Fertin, D., “Model predictive control of low earth orbiting spacecraft with magneto-torquers,” _2006 IEEE Conference on Computer Aided Control System Design, 2006 IEEE International Conference on Control Applications, 2006 IEEE International Symposium on Intelligent Control_ , 2006, pp. 2908–2913. https://doi.org/10.1109/CACSD-CCAISIC.2006.4777100. 

- [7] Huang, X., and Yan, Y., “Fully Actuated Spacecraft Attitude Control via the Hybrid Magnetocoulombic and Magnetic Torques,” _Journal of Guidance, Control, and Dynamics_ , Vol. 40, No. 12, 2017, pp. 3358–3360. https://doi.org/10.2514/1.G002925, URL https://doi.org/10.2514/1.G002925. 

- [8] Horri, N. M., and Palmer, P., “Practical Implementation of Attitude-Control Algorithms for an Underactuated Satellite,” _Journal of Guidance, Control, and Dynamics_ , Vol. 35, No. 1, 2012, pp. 40–45. https://doi.org/10.2514/1.54075, URL https://doi.org/10.2514/1.54075. 

- [9] Han, C., and Pechev, A. N., “Underactuated Satellite Attitude Control with Two Parallel CMGs,” _2007 IEEE International Conference on Control and Automation_ , 2007, pp. 666–670. https://doi.org/10.1109/ICCA.2007.4376438. 

- [10] Maheshwarappa, M. R., and Bridges, C. P., “Software defined radios for small satellites,” _2014 NASA/ESA Conference on Adaptive Hardware and Systems (AHS)_ , 2014, pp. 172–179. https://doi.org/10.1109/AHS.2014.6880174. 

- [11] Trégouët, J., Arzelier, D., Peaucelle, D., Pittet, C., and Zaccarian, L., “Reaction Wheels Desaturation Using Magnetorquers and Static Input Allocation,” _IEEE Transactions on Control Systems Technology_ , Vol. 23, No. 2, 2015, pp. 525–539. https://doi.org/10.1109/TCST.2014.2326037. 

- [12] Stickler, A. C., and Alfriend, K., “Elementary Magnetic Attitude Control System,” _Journal of Spacecraft and Rockets_ , Vol. 13, No. 5, 1976, pp. 282–287. https://doi.org/10.2514/3.57089, URL https://doi.org/10.2514/3.57089. 

- [13] Pittelkau, M. E., “Optimal periodic control for spacecraft pointing and attitude determination,” _Journal of Guidance, Control, and Dynamics_ , Vol. 16, No. 6, 1993, pp. 1078–1084. https://doi.org/10.2514/3.21130, URL https://doi.org/10.2514/3.21130. 

- [14] Ruiter, A., “Magnetic Control of Dual-Spin and Bias-Momentum Spacecraft,” _Journal of Guidance, Control, and Dynamics_ , Vol. 35, 2012, pp. 1158–1168. https://doi.org/10.2514/1.55869. 

- [15] Henson, M. A., “Nonlinear model predictive control: current status and future directions,” _Computers & Chemical Engineering_ , Vol. 23, No. 2, 1998, pp. 187 – 202. https://doi.org/https://doi.org/10.1016/S0098-1354(98)00260-9, URL http://www. sciencedirect.com/science/article/pii/S0098135498002609. 

- [16] Findeisen, R., Imsland, L., Allgower, F., and Foss, B. A., “State and Output Feedback Nonlinear Model Predictive Control: An Overview,” _European Journal of Control_ , Vol. 9, No. 2, 2003, pp. 190 – 206. https://doi.org/https://doi.org/10.3166/ejc.9.190-206, URL http://www.sciencedirect.com/science/article/pii/S0947358003702751. 

- [17] Silani, E., and Lovera, M., “Magnetic spacecraft attitude control: a survey and some new results,” _Control Engineering Practice_ , Vol. 13, No. 3, 2005, pp. 357 – 371. https://doi.org/https://doi.org/10.1016/j.conengprac.2003.12.017, URL http://www.sciencedirect.com/science/article/pii/S0967066103002922, aerospace IFAC 2002. 

- [18] Psiaki, M. L., “Magnetic Torquer Attitude Control via Asymptotic Periodic Linear Quadratic Regulation,” _Journal of Guidance, Control, and Dynamics_ , Vol. 24, No. 2, 2001, pp. 386–394. https://doi.org/10.2514/2.4723, URL https://doi.org/10.2514/2.4723. 

- [19] Xia, X., Guo, C., and Xie, G., “Investigation on magnetic-based attitude de-tumbling algorithm,” _Aerospace Science and Technology_ , Vol. 84, 2019, pp. 1106 – 1115. https://doi.org/https://doi.org/10.1016/j.ast.2018.11.035, URL http: //www.sciencedirect.com/science/article/pii/S1270963818314640. 

- [20] Ovchinnikov, M. Y., Roldugin, D., Tkachev, S., and Penkov, V., “B-dot algorithm steady-state motion performance,” _Acta Astronautica_ , Vol. 146, 2018, pp. 66 – 72. https://doi.org/https://doi.org/10.1016/j.actaastro.2018.02.019, URL http: //www.sciencedirect.com/science/article/pii/S0094576517317332. 

- [21] Fonod, R., and Gill, E., “Magnetic Detumbling of Fast-tumbling Picosatellites,” _Proceedings of 69th International Astronautical Congress_ , International Astronautical Federation, IAF, France, 2018, pp. 1–11. URL https://www.iac2018.org/, 69th International Astronautical Congress, IAC 2018 ; Conference date: 01-10-2018 Through 05-10-2018. 

- [22] Reyhanoglu, M., and Drakunov, S., “Attitude stabilization of small satellites using only magnetic actuation,” _2008 34th Annual Conference of IEEE Industrial Electronics_ , 2008, pp. 103–107. https://doi.org/10.1109/IECON.2008.4757936. 

- [23] Wisniewski, R., and Blanke, M., “Three-Axis Satellite Attitude Control Based on Magnetic Torquing,” _IFAC Proceedings Volumes_ , Vol. 29, No. 1, 1996, pp. 7618 – 7623. https://doi.org/https://doi.org/10.1016/S1474-6670(17)58915-6, URL http://www.sciencedirect.com/science/article/pii/S1474667017589156, 13th World Congress of IFAC, 1996, San Francisco USA, 30 June - 5 July. 

- [24] Giri, D. K., and Sinha, M., “Magnetocoulombic Attitude Control of Earth-Pointing Satellites,” _Journal of Guidance, Control, and Dynamics_ , Vol. 37, No. 6, 2014, pp. 1946–1960. https://doi.org/10.2514/1.G000030, URL https://doi.org/10.2514/1.G000030. 

- [25] Giri, D. K., and Sinha, M., “Finite-time continuous sliding mode magneto-coulombic satellite attitude control,” _IEEE Transactions on Aerospace and Electronic Systems_ , Vol. 52, No. 5, 2016, pp. 2397–2412. https://doi.org/10.1109/TAES.2016.140503. 

- [26] Hermann, R., and Krener, A., “Nonlinear controllability and observability,” _IEEE Transactions on Automatic Control_ , Vol. 22, No. 5, 1977, pp. 728–740. https://doi.org/10.1109/TAC.1977.1101601. 

- [27] Cheng, D., Hu, X., and Shen, T., _Analysis and design of nonlinear control systems_ , Springer, 2010. 

- [28] Martinelli, A., “Rank Conditions for Observability and Controllability for Time-varying Nonlinear Systems,” , 2020. URL https://arxiv.org/abs/2003.09721, last accessed 17 January 2021. 

- [29] Lovera, M., and Astolfi, A., “Spacecraft attitude control using magnetic actuators,” _Automatica_ , Vol. 40, No. 8, 2004, pp. 1405 – 1414. https://doi.org/https://doi.org/10.1016/j.automatica.2004.02.022, URL http://www.sciencedirect.com/science/article/pii/ S0005109804000767. 

- [30] Lovera, M., and Astolfi, A., “Global Magnetic Attitude Control of Inertially Pointing Spacecraft,” _Journal of Guidance, Control, and Dynamics_ , Vol. 28, No. 5, 2005, pp. 1065–1072. https://doi.org/10.2514/1.11844, URL https://doi.org/10.2514/1.11844. 

- [31] Khalil, H. K., _Nonlinear systems; 3rd ed._ , Prentice-Hall, Upper Saddle River, NJ, 2002. URL https://cds.cern.ch/record/1173048, the book can be consulted by contacting: PH-AID: Wallet, Lionel. 

- [32] Brockett, R. W., “Asymptotic Stability and Feedback Stabilization,” _Differential Geometric Control Theory_ , 1983, pp. 181–191. 

- [33] Crouch, P., “Spacecraft attitude control and stabilization: Applications of geometric control theory to rigid body models,” _IEEE Transactions on Automatic Control_ , Vol. 29, No. 4, 1984, pp. 321–331. https://doi.org/10.1109/TAC.1984.1103519. 

- [34] Petersen, C. D., Leve, F., and Kolmanovsky, I., “Model Predictive Control of an Underactuated Spacecraft with Two Reaction Wheels,” _Journal of Guidance, Control, and Dynamics_ , Vol. 40, No. 2, 2017, pp. 320–332. https://doi.org/10.2514/1.G000320, URL https://doi.org/10.2514/1.G000320. 

- [35] Grüne L., P. J., _Nonlinear Model Predictive Control_ , Springer, London, 2011. 

- [36] Seguchi, H., and Ohtsuka, T., “Nonlinear receding horizon control of an underactuated hovercraft,” _International Journal of Robust and Nonlinear Control_ , Vol. 13, No. 3-4, 2003, pp. 381–398. https://doi.org/10.1002/rnc.824, URL https: //onlinelibrary.wiley.com/doi/abs/10.1002/rnc.824. 

- [37] Huang, M., Nakada, H., Butts, K., and Kolmanovsky, I., “Nonlinear Model Predictive Control of a Diesel Engine Air Path: A Comparison of Constraint Handling and Computational Strategies,” _IFAC-PapersOnLine_ , Vol. 48, No. 23, 2015, pp. 372 – 379. https://doi.org/https://doi.org/10.1016/j.ifacol.2015.11.308, URL http://www.sciencedirect.com/science/article/pii/ S2405896315025926, 5th IFAC Conference on Nonlinear Model Predictive Control NMPC 2015. 

- [38] Bryson, A. E., Ho, Y., and Siouris, G. M., “Applied Optimal Control: Optimization, Estimation, and Control,” _IEEE Transactions on Systems, Man, and Cybernetics_ , Vol. 9, No. 6, 1979, pp. 366–367. https://doi.org/10.1109/TSMC.1979.4310229. 

- [39] Ohtsuka, T., and Fujii, H. A., “Real-time optimization algorithm for nonlinear receding-horizon control,” _Automatica_ , Vol. 33, No. 6, 1997, pp. 1147 – 1154. https://doi.org/https://doi.org/10.1016/S0005-1098(97)00005-8, URL http://www.sciencedirect. com/science/article/pii/S0005109897000058. 

- [40] Gillet, N., Barrois, O., and Finlay, C., “Stochastic forecasting of the geomagnetic field from the COV-OBS.x1 geomagnetic field model, and candidate models for IGRF-12,” _Earth, Planets and Space_ , Vol. 67, 2015. https://doi.org/10.1186/s40623-015-0225-z. 

- [41] Cubas, J., Farrahi, A., and Pindado, S., “Magnetic Attitude Control for Satellites in Polar or Sun-Synchronous Orbits,” _Journal of Guidance, Control, and Dynamics_ , Vol. 38, No. 10, 2015, pp. 1947–1958. https://doi.org/10.2514/1.G000751, URL https://doi.org/10.2514/1.G000751. 
