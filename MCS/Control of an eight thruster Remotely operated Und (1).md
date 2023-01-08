Control of an eight thruster Remotely operated Underwater Vehicle(ROV) model using LQR controller.

The PID controller has been one of the most commonly used controllers for a really long time. There have been numerous PID tuning techniques, such as the Ziegler-Nichols method but are insufficient for high-performance control applications. The Linear Quadratic Regulator (LQR) is an optimal control method based on full state feedback. It aims to minimise the quadratic cost function and is then applied to linear systems, hence the name Linear Quadratic Regulator. 

**Why LQR controller?**

The use of LQR  over PID control comes from the higher robustness of the former in terms of tuning the parameters with varying conditions. PID control uses the error in the input parameter of the closed loop system and tunes the parameters to reduce the error to zero. LQR, on the other hand, uses the state space model of the system and takes complete state feedback: -Kx, to calculate the error. LQR uses the method of cost function to calculate the control input cost vs the importance of achieving desired states.

**State-Space Model with Full State Feedback Gain:**

![fc09bc78a5d891afa4282765192504b5.png](../_resources/f8edeebb53274bfb8def84a0017c67a8.png)


Ẋ = AX + BU
y = CX+DU

**Cost Function:**
The cost function defined by the system equations is minimised by the LQR controller using an optimal control algorithm. The cost function involves the system's state parameters and input (control) parameters, along with the Q and R matrices. For the optimal LQR solution, the overall cost function must be as low as possible. The weights given to the state and control parameters are represented by the Q and R matrices, which act as knobs whose values can be varied to adjust the total value of the cost function. 

The system must have a linearized state-space model to solve the LQR optimization problem. The cost function to be optimised is given by

𝐽 = ∫ (𝑋T𝑄𝑋 + 𝑈T𝑅𝑈)𝑑t

**Algebraic Riccati Equation and Its Solution(S matrix):**

The Q and R matrices are used to solve the Algebraic Riccati Equation (ARE) to compute the full state feedback matrix.

𝐴T𝑆 + 𝑆𝐴 − 𝑆𝐵𝑅-1𝐵T𝑆 + 𝑄 = 0

On solving the above equation, we obtain the matrix 𝑆.

**Feedback Gain (K) and Eigen Values from S matrix:**

The matrix 𝑆 obtained from the above ARE is used to find the full state feedback gain matrix K using the relation,

𝐾 = 𝑅-1𝐵T𝑆

The control matrix U is then given by

U = - KX

**Linearisation of a state-space model:**

The linearization of a state-space model is needed while using the LQR technique since it works on linear systems. In our case, the state space model is in the form: ẋ = f(x); where f(x) is a nonlinear function of x. In such cases, to linearise the equation, we use the concept of linearising about a fixed point. The steps for the same are as follows:
Find the fixed points x̄ ; where  f(x̄) = 0.
Linearise about an x̄, by calculating the Jacobian of dynamics at the fixed point x̄; where the latter can be represented as:

![748fccdb27d3c763a7e3767457795874.png](../_resources/dc6404ca0d9a487c814cd4c0451bfe46.png)

where each term is a partial derivative of the dynamics with respect to a variable.
This step is executed since the dynamics of a nonlinear system behave linearly at the fixed point, or, in a small neighborhood around the fixed point. 

Changing the frame of reference to one with x̄ as the origin:
 ẋ -ẋ̄  = f(x)
         = f(x̄) + J.(x -x̄) + J2.(x-x̄)2 + . . . . .  

where   J  is calculated at x̄.
The higher order terms from the third term  J2.(x-x̄)2 are neglected since they are really small, hence the equation reduces to:
 
  ∆ẋ = 0 +J.∆x + 0 
 ⇒ ∆ẋ = J.∆x  

This is in the form  ẋ = Ax

**Note:** The above method for linearization works only when the fixed point satisfies the condition for linearising a system given by the Hartman Grobman Theorem, which states that:
“the behaviour of a dynamical system in a domain near a hyperbolic equilibrium point is qualitatively the same as the behaviour of its linearisation near this equilibrium point, where hyperbolicity means that no eigenvalue of the linearisation has real part equal to zero. Therefore, when dealing with such dynamical systems one can use the simpler linearisation of the system to analyse its behaviour around equilibria.[1]”

Hence, linearization works only when the fixed point is hyperbolic or put simply, has a non-zero real part.


**Implementation:**

The following is the Simulink model that we built:

![8ea1dd2ddcff1483c74bdd93f16b209b.png](../_resources/f350ff9587fa40a2942756c2f20eafc4.png)

To implement this, we need the following:
State-space model: 
A and B matrices: These relate the derivative of the states with the current states and the control input. But, since our model is non-linear in nature, we cannot get a direct relation consisting of A and B matrices. Also, the LQR controller works only for Linear Systems. So, we have to linearise our system around an operating point. For our system, we took the origin in the world frame (initial point of the bot) as the operating point. Also, we used the position of the bot in the world frame and velocity of the bot in its own frame as the two states. Meaning:

![712086ebf11a62836b19e3b14a6dcfa6.png](../_resources/5d9c384663dc4e89b0e2d67b5c9b7da3.png)

(Here x is the state but not the distance in x direction)

![142a0e6ea79541a35773eccf39171f5c.png](../_resources/beaab453c88c486cb0cdbed7dec1146d.png)

Since,

![c0e5ecf56a58ece1c31cfb62ba91c11d.png](../_resources/41c6f6db8ba94427939ee65ebb8b7b5f.png)

The above state space model is clearly non-linear. So, we have to linearise to find A and B as follows:

![2aae6563a52e45984d0b1a1dcfa2e7db.png](../_resources/00f72a17d7fb446082e933b086767774.png)

![af91d34d2ea4b80fd03a465693c32274.png](../_resources/2b159be790c44f7caa395ac9ca2a7eb6.png)

So, we obtain the A and B as follows:

![91c08a5c76f6fc5ad7ae86eb421cf27d.png](../_resources/f259130897a043afb305707acb8a8aa0.png)

![8e087d19d37c13d3b034a15384ddc4cd.png](../_resources/a08757dfd53d42bdb53eb7ea1bb1cb58.png)

But we obtained the above results by taking weight W and buoyancy B as 110 N and 120 N respectively.

We take the C matrix in state-space (y=CX+DU) as eye(12) so as to send back all the states as feedback to the summing point.

Q and R matrices:
Q stands for the importance of the states to reach their desired values.
R stands for the cost of the control input. 
So, if we have an expensive control input compared to that of our needs to reach the states, we take the Q matrix to be dominating compared to R and vice versa. 

Then, we calculate the full-state feedback matrix, the Algebraic Riccati Equation and the eigenvalues by using the following command:
			[K,S,P] = lqr(A,B,Q,R)

Then, we put the obtained value of the Gain matrix in the state-space model in Simulink. The following results are obtained for different R matrices (Q matrix being a 12x12 identity matrix) and desired states being [1;2;1;0;0;0;0;0;0;0;0;0]:

![4104dd02cea08608622bc89c07daa6a4.png](../_resources/40d3bbd8fa894a248b19fc9b53f9b152.png)

![994c76fe82c7d017fe82243f0bb41fb5.png](../_resources/e578e2b5906a42bd9f5b4860b1a3b73b.png)

For the above, the Poles of the closed loop and open loop transfer function are as follows:

![d3a196bc57240dc8161ac4ea944af02b.png](../_resources/30bbfd184f3f4806abf18ea647a496e6.png)

**Poles:**

Closed loop(With the feedback):

![061fa80a7756df24211d1b4d097fe725.png](../_resources/a2c6f3f9ea8547de92f8a444e8328b5b.png)

Open loop(Without the feedback):

![c0877a5f9abc58098a692e719e769e33.png](../_resources/f311ff15b4034f34b95720fb3c5648b9.png)

**Making ‘R’ more dominant:**

![0a299e2db190fad74d6f9e2f88efd6e1.png](../_resources/6d0af97064c84ba7b8f797024f28beb2.png)

![56f9c54d900167e9d18db63d87d96ee6.png](../_resources/8656a4d3f7814f34bd8ab45108f3a929.png)

**Making ‘R’ less dominant:**

![34181b2ba6567f3ef123a0d634f6901c.png](../_resources/28b22a4f42794f229fde8a5380ae6cf7.png)

![3d8ec25ac866c2b5886716d673be3a53.png](../_resources/6561da1ed4fa4a98a921d2cd8902d0e0.png)

**Results:**
**a) Observed states:**

![bb14c98dc1bd81d35bf7b4b72ec495d0.png](../_resources/d2336d884fee4f06b9e91c43525e012a.png)

**More dominant ‘R’:**

![b09eec3e2c852d80d02da4042192b1c9.png](../_resources/d85a90c729dc42b5aeb4e1ccec83b3d3.png)

**Less dominant ‘R’:**

![793213287588f45a5936f9219b9af761.png](../_resources/d8e226d0f41b4de3af45b807ab0ae9d9.png)

**b)Thrust i/p to the thrusters:**

![2aa69e4f7190c4f7d0a4419b3901a2aa.png](../_resources/5bacd3b2aa6a4422a0810406551c9b07.png)

**More dominant ‘R’:**

![f39fd171ac6ca335df05a27063036221.png](../_resources/dab7c0d1ba0f4d47930f883a9ac385e9.png)

**Less dominant ‘R’:**

![3db8245fd5f095e284e21b4742a5e61c.png](../_resources/5744088635ca460199fc8911f1944d86.png)

**Band Limited White Noise:**
We planned on modelling a system with White Noise. So, we used the following control system:

![bfdb73fd484320597f24964ecbbdc68a.png](../_resources/1a759a49ab594fc8a0e3e659010efe1b.png)

Where:

![55510b503970cac67342e94677db8b3b.png](../_resources/33783a5c8fb54a65a04f573d211342ed.png)

We included a low pass filter for removing noise with frequency greater than 1Hz. With low values of ‘Q’ we are getting really unstable results as the importance to the states is reduced. Later, we increased the value of Q to allow the states to reach quickly. This made the response of our system better. Taking the reference input as: [0;0;2;1.57;1;0.8;0;0;0;0;0;0];

1)Q = eye(12):
**States:**

![3fd5489e8fe53d1272e38c9c7a23d0e6.png](../_resources/cf5ab790e01c4244b08425844e3d11fe.png)

**Thrust provided by each thruster:**

![fbfba474def7cda9bef422f71e9cb89a.png](../_resources/75ab7a69a6d242e9860841008f1515f4.png)

**Poles of this system:**

![dbdcd2edf68d27a124b6081791c89f6f.png](../_resources/7b6eac6befa8438fbc892e3d3b4ecff6.png)

2)Q = 1000*eye(12):
**States:**

![59d1bcdf6acc9cdf751600810927f1b5.png](../_resources/feb61203b15f4354a7bb48e00f8e0b9a.png)

**Thrust provided by each thruster:**

![7a0e73544f01235f28db9fbf3a9d016c.png](../_resources/a6e07695ba6f4999b519a13558938f29.png)

**Poles of this system:**

![f799dfc5b6b2c72c911a440bee7b96f6.png](../_resources/9d636758f7a440acaec699f956736ebc.png)

Clearly, the response of the system is faster in the second case. 
**Note:** In the above two cases, the open loop and closed loop pole plots are drawn without taking the noise into account. The effect of noise is shown only in the ‘Thrust’ and ‘States’ plots. 

Also, in the paper that we referred to, they considered noise caused due to the ‘current velocity’ which has a direct effect on the velocity of the rover instead of the torques. So, for that, a Random Number generator block was included in  the model as follows:

![ecd0084c0d6067a60aa22db5291725a1.png](../_resources/def322c78940424aa63da117cbf61f21.png)

The parameters of the noise block are as follows:

![b48f8981333efba2566f13021a6fc59b.png](../_resources/33acf7dae9994f50a303975e281ae764.png)

Here, noise is added considering the velocity of current as: [0.25,0.25,0.25] and taking the variance as 0.01 in all the three directions.

Taking R matrix as: R=1.2*eye(6)+0.8*ones(6)
And Q matrix as: Q=1000*eye(12);
The results that we obtained for 1min stop time  are:
**States:**

![be82d3ac55f54be0302f73a5b10ffe29.png](../_resources/e2b0399adc6744509cdabb205b4bf7b6.png)

**Thrust provided by each thruster:**

![df475dc8004aaff25c34702615f38cdc.png](../_resources/c6bb1a078b3940d5868cdaad3017fc2c.png)

Clearly, the thrust output involves too much vibrations.

**Analysis of Eigenvalues:**
When we analyse the change in eigen values while considering the open loop and LQR controlled closed loop systems, for the same values of Q,R we see a clear change in their position in the pole zero plot.

![27784a165aef27fed02596dd27ecb713.png](../_resources/af4a4b0d798a48c1a1ea734edaab0e2c.png)

Considering the above plot, we can see a clear shift in the eigen values in the direction of the negative real axis, ensuring more stability of the system. As we write down the eigen values or the poles of the two systems, the difference becomes evident.
                                                                      
For   Q = eye(12) & R = 0.1*ones(12) + 0.9*eye(12) 

![46692c69ab6f1295f167649595df7c68.png](../_resources/e50e14ce1eb1448684a02d55c8921f5c.png)

As it can be observed, in the case of the open loop system, we have four eigen values or poles located at the origin which leads to the fact that the system is marginally stable. Whereas, for the closed loop system, most of  the poles have shifted towards the left side of the imaginary axis with all the poles lying on the left half of the s-plane. This gives enough proof to say that after implementing LQR control, the system has become more stable as compared to the open loop system and hence has helped in increasing performance in terms of reaching the end states. 

**Conclusion:**
Given the results of implementing both PID and LQR control on the ROV system, we can see a better response for an LQR control over the PID when we include disturbances in the environment. Also, the LQR control is far more robust in adapting to the change in buoyancy as compared to the PID control, which broadens the spectrum of usage in the former as compared to the latter. When we have changes in the system, using a PID requires tuning all the three gain values for all the control inputs. Tuning the gain values for multiple parameters is not an easy task and there is a very narrow range of values which give the desired results for a specific situation. On the other hand, when we want to adapt to changes while working on LQR control, our tuning is dependent only on the Q,R cost matrices which can help in changing the  relative importance of the states and control inputs for a given circumstance. Here, our goal is achieved by changing the values of the matrices, so as to select the more important factor among the states and inputs,  where we achieve the desired results for a wide range of values which only differ in terms of speed of transient response and steady state error. Another advantage of LQR over PID control that we have come across was the ability to easily control velocity components along with the position coordinates, which would lead to much more complexity in the case of a PID based controller. For simulating in a noisy environment, we used nonlinear PID control where a water current velocity was added as a disturbance; its value being constant with time and we attained appreciable results. Whereas for the LQR setup, we have used a gaussian white noise as a source of disturbance, which gives a better look at the real world scenario. Upon increasing the Q matrix or simply, the cost of attaining the states, we have observed better performance in attaining the states and less erratic, more uniform thrust outputs produced by the thrusters; which is a more reliable result since it is realisable in a real world environment. 
However, one drawback of using the LQR control is its applicability to only linear systems, whereas most of the real world scenarios tend to have non linearity in them. However, this problem can be overcome by linearising the system near the fixed points and applying the control.

Hence, we have come to the conclusion that using an LQR control is far more reliable than a PID control for the positional and velocity control of a 6-DOF Autonomous Underwater Vehicle.

---