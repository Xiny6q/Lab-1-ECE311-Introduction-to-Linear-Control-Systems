Download link :https://programming.engineering/product/lab-1-ece311-introduction-to-linear-control-systems/


# Lab-1-ECE311-Introduction-to-Linear-Control-Systems
Lab 1 – ECE311 Introduction to Linear Control Systems
Introduction to LTI Systems in Matlab and Simulink

MAIN CONCEPTS OF THIS LAB

How to define state space and transfer function models in Matlab

How to convert between state space and transfer function representations

How to numerically find the time response of an LTI system in Matlab

How to plot solutions

Definition of state space and transfer function blocks in Simulink

Speed control of a permanent magnet DC motor

INTRODUCTION

In this lab you will be introduced to the basic tools required to perform numerical simulation of control systems in Matlab and Simulink. To motivate various constructions, we will use the example of a permanent magnet DC motor, whose model is

Armature: La

dia

+ Ra ia + Ke

˙

= u,

Rotor:

dt

I ¨ + b ˙

θ

(1)

− Kt ia = 0.

θθ

In the differential equations above, θ denotes the angle of the motor shaft, ia the armature current, and u the voltage applied at the motor terminals, our control input. The other parameters and their numerical values are defined in the table below. We choose the state x ∈ R3 given by

x =

x2 =

˙

,

x1

θ

x3

θ

ia

and assume we can measure the angular rate of the motor shaft, ˙, via a tachometer. We take this to be

θ

our output. The control problem is speed control: make the output ˙ converge to a desired constant.

θ

We rewrite the equations above in state-space form

x˙ =

0

− b/I

Kt /I

x +

0

u

0

1

0

0

0

− Ke /L a

− Ra /La

1/La

(2)

= 0 1 0 x.

Motor Parameter

Description

Numerical value

La

Armature inductance

0.02

H

Ra

Armature resistance

3 ohms

Ke

Back emf constant

0.01

V/(rad/sec)

Kt

Motor torque constant

0.01

N m /A

I

Motor moment of inertia

6 · 10−4 N m /(rad/sec2)

b

Viscous friction coefficient

10−4 N m /(rad/sec)

It is common to approximate the model (2) by assuming that the ratio Ra /La is very large, so that the term La dia /dt can be ignored, and the armature equation in (1) can be approximated by

Ra ia + Ke

˙

= u,

θ

in which case we can solve this identity for ia ,

ia = −

Ke

θ

+

1

u.

Ra

˙

Ra

Effectively, in writing this expression we are assuming that the electrical dynamics are much faster than the rotor dynamics, and therefore the transient of the armature current can be ignored. The expression for ia we just found is called a quasi steady-state. We now substitute this expression into the rotor equation, to obtain the simplified model of the DC motor:

I ¨ +

b +

Ke Kt

˙

=

Kt

u.

θ

Ra

θ

Ra

θ

⊤

Letting x1 = θ, x2 = ˙, and x =

x1 x2

, the state space model is now given by

Throughout the lab, you will be guided through a number of steps which will require you to write Matlab code or draw Simulink diagrams. You will write your code in a Matlab script called labx.m, where x is the lab number. Your Simulink diagram will be saved as labx.slx. If there are multiple Simulink files, save them as labx_1.slx,labx_2.slx, and so on. You will submit this code as a group. Your code should provide certain outputs (figures, numbers, and the like). A request for output is highlighted with a shaded area of text, such as the following.

Output 1. Print the poles of the transfer function.

Parts of the text containing directions for the writing of your Matlab code will be highlighted in a different colour, such as this:

Convert the state space model into a transfer function model, then find the poles.

SUBMISSION GUIDELINES AND MARK BREAKDOWN

Marks are assigned to groups. Members of each group get identical marks, unless special circumstances occur. The group lab mark will be made up of three components.

Matlab and Simulink code

4 pts

In person discussion of the lab work with TA

2 pts

Lab report

4 pts

Total

10 pts

Matlab code. Your code should be clean and readable, and it should contain abundant commentary, so that the instructors may follow your development and check its correctness. This component of the mark will be based on the correctness of the code and its readability, and it will be assigned as follows:

what were your findings, show the outputs you obtained, and comment on them. In this presentation, you need to show an understanding of the problem you have worked on, and the outputs you’ve pro-duced. If the lab document requests comments on a particular output, these should be an integral part of the discussion. It’s important that each group member contribute equally to the video. This component of the mark will be assigned as follows:

In Matlab, such a transfer function is defined using the command tf, placing the coefficients ai , bj into two arrays, num and den. Sometimes, it is convenient to factor the numerator and denominator into products of elementary terms, so as to expose the zeroes and poles of the transfer function:

G(s) = K (s − z1 ) · (s − z2 ) · · · (s − zm ) .

(s − p1 ) · (s − p2 ) · · · (s − pn )

In Matlab, the form above is called a ZPK representation, and it is defined by the two arrays Z = [z1 . . . zm ], P = [p1 . . . pn ], and the gain K. The command zpk defines an LTI object from this data.

Defining the motor model. Create a script lab1.m and do the following.

Create motor variables La,Ra,Ke,Kt,I,b with numerical values given by the entries in the table found in Section 1.

Create matrices A,B,C,D representing the motor model in (2) (the scalar D is zero here).

Create matrices A1,B1,C1,D1 representing the simplified motor model in (3) (the scalar D1 is zero).

Using the command ss, define the state space model of the motor (2) in an object named motor, and that of the simplified model (3) in an object named motor_simplified.

Using the command tf, convert the objects motor and motor_simplified to transfer functions, and name the resulting objects G_motor and G_motor_simplified, respectively.

Using the command zpk, convert the object motor to ZPK form, and name the resulting object zpk_motor. You don’t need to do this for the simplified model.

Using the commands zpkdata and cell2mat, extract the list of poles of the transfer function from the object G_motor. You don’t need to do this for the simplified model.

Using the commands tfdata and cell2mat, extract numerator and denominator arrays from the object G_motor, and name them num and den, respectively. You will need these arrays in Section 5. Repeat this procedure for the transfer function object G_motor_simplified, this time creating ar-rays num1 and den1.

Output 1. • Print out the transfer functions G_motor and G_motor_simplified.

Print out the transfer function in ZPK form, zpk_motor.

Print out the poles of the transfer function G_motor. Comment on the location of the two poles, and how are they related to one another: are they close to each other? Is one much smaller in magnitude than the other? Comment on how you expect the motor to behave based on the location of the poles when the input voltage is a unit step.

Print out the numerator and denominator arrays of the transfer functions G_motor and G_motor_simplified. You will need these arrays in Sections 5 and 6.

NUMERICAL SIMULATION OF LTI SYSTEMS

Having defined the motor model in Matlab, you will now numerically simulate it to glean useful infor-mation about the motor behaviour when subjected to step and sinusoidal inputs. For step inputs, you will verify that the simplified model (3) does indeed approximate the full model (2) very well.

MATLAB COMMANDS

linspace(T1,T2,N)

Generates an array of N equally spaced numbers between T1 and

T2.

step(sys,T)

Plots the step response of the LTI object sys at the times in the

array T. If called as [Y,T,X]=step(sys,T), it returns matrices X,Y

containing the state and output samples corresponding to the

times in T. If sys is a transfer function, a state space realization

of it is used in producing X.

lsim(sys,U,T,X0)

Plots the time response of the LTI object sys at the times in the

array T subject to the control input whose samples are in U, and

the initial condition in X0 (this latter is only meaningful if the

model is in state space form).

evalfr(sys,s)

Evaluates the transfer function of the LTI object sys at s.

plot(T,Y)

Plots the samples in Y versus the samples in T.

subplot

Partitions a figure into sub-figures.

xlabel, ylabel

Labels the axes of a graph.

title

Adds a title to a figure.

Continue working on your lab1.m script.

Define an array T of 1000 time samples, equally spaced between 0 and 30 seconds.

Using the command subplot(311), create a figure with three subplots, vertically aligned.

Using the command step, find the step response of the state space model motor at the time samples in T, save it in an array Y1, and plot it in the first subplot versus T.

Find the step response of the simplified state space model motor_simplified at the time samples in T, save it in an array Y2, and using subplot(312) plot it in the second subplot versus T.

Plot Y1-Y2 versus T in the third subplot using subplot(313), and verify that the step response of the simplified model approximates very well that of the full model.

The last entry of the output vector Y1 that you’ve just computed is an approximation of the motor ’s asymptotic speed for a unit step voltage. Using the Final Value Theorem and the transfer function G_motor, determine the theoretical asymptotic value of the motor speed in response to a unit step.

In a second figure, plot the armature current (third component of the state) of the full motor model (2) versus time when the input is a step. For this, you need to use the state space model motor.

In a third figure, using the command lsim and the LTI object motor, find the output response with initial condition X0=[0;-1;.5] and input signal sin(T), and plot it versus time.

From the figure you just plotted, determine the approximate amplitude of oscillation in steady-state of the motor speed in response to the input signal sin(T).

Using the command evalfr(G_motor,s), evaluate the motor transfer function at s = i (the fre-quency of the sinusoidal input is 1 rad/sec), and verify that the amplitude you found is approx-imately equal to |G(i)|. This is the manifestation of a general property of the frequency response that we shall review later on in the course.

Output 2. • Produce three figures as explained above. Label the axes of each (sub-) figure, and add titles explaining what the (sub-) figure represents.

Compare the step responses of the motor model and its simplified version. How close are they to each other? What is the maximum error between the two?

Print the approximate asymptotic value of the motor speed in response to a unit step.

Print the theoretical asymptotic value of the motor speed in response to a unit step. Verify that the approximate value above is indeed very close to the theoretical value.

Print the approximate amplitude of oscillation of the motor speed in response to a sinusoidal input.

Print the theoretical amplitude of oscillation of the motor speed in response to a sinusoidal input, and compare it to the approximate value above.

DEFINITION OF LTI SYSTEM BLOCKS IN SIMULINK AND NUMERICAL SIMULATION

In this section you will define the motor model and its simplified version in Simulink. You will then compare their outputs and verify once again that the simplified model offers an accurate approximation.

SIMULINK BLOCKS (INSIDE THE LIBRARY BROWSER)

Continuous → State-space define a state space LTI object

Continuous → Transfer Fcn define a transfer function LTI object

Sources → Step generate unit step signal

Sources → Sine wave generate sinusoidal signal

Sinks → Scope plot signal vs time

Signal routing → Mux create a vector signal from two scalar signals

Signal routing → Manual switch manually switch between two signals

Open Simulink and the Library Browser. Create a blank diagram called lab1_1.slx.

Using the blocks listed above, draw a Simulink diagram like the one depicted below. In it, the same input signal (a step or a sine wave) is fed simultaneously to two different transfer function models. The outputs of these models are then plotted versus time.

Full motor TF model

motor output

Simplified motor TF model

Edit the first transfer function block. Inside the boxes labelled num, den, write num and den. This will tell Simulink to use the numerator and denominator arrays of the motor transfer function that you’ve previously defined in the workspace. If you have erased them, run your lab1.m script first.

Note. You could replace this transfer function block by a state space block using the matrices A,B,C,D that you’ve previously defined in the workspace. The results of the following comparison would be identical, except that with the state space block you can set a nonzero initial condition.

Edit the second transfer function block, and this time enter num1 and den1 in the numerator and denominator boxes. Thus the second transfer function represents the simplified motor model.

Edit the Step block and make sure of the following: the initial value of the step is 0, the final value is 1, the step time is zero.

Edit the Sine Wave block and make sure that the frequency is 1 rad/sec, the amplitude is 1, the phase is 0 and the bias is zero.

Open the Model settings and under the Solver menu, set both relative and absolute tolerances to 10−10. Make sure that the solver is variable-step, and set the stop time to 30 seconds.

Run the Simulink diagram. Plot the output of the motor models when the input is a unit step first, then a sine wave, and verify that in both cases the outputs of the two transfer functions give almost identical results. Save the results into two figures, one for the step input, one for the sinusoidal input. To save the Simulink scope, select File → Print to Figure, then save the figure in the format of your choice. Do not take screenshots.

Output 3. • Produce the two figures described above.

Comment on how well the simplified transfer function model approximates the full transfer func-tion model for the two given input signals.

PROPORTIONAL CONTROL OF THE PERMANENT MAGNET DC MOTOR

Having verified that the simplified model G_motor_simplified well approximates the full motor model G_motor, we now work with G_motor_simplified, and try our hand at the speed control problem. The problem in question is to make the output y (the speed of the motor shaft) converge to a desired constant value.

To achieve this control objective, we use the simplest kind of controller, a proportional one. A proportional controller is one where u(t) = Ke(t), where e(t) denotes the tracking error, and K > 0 is the control gain. We don’t yet have the theoretical tools to check whether and to what extent this type of controller is appropriate for our speed control problem, but we’ll nonetheless go ahead and try it out. Later in our lectures we’ll justify the viability of this controller and understand its limitations, limitations that you’ll have an opportunity to observe in a moment.

SIMULINK BLOCKS (INSIDE THE LIBRARY BROWSER)

Sources → Signal Generator

generates various input signal classes

Math

Operations → Add

summing block

Math

Operations → Gain

constant gain block

Sinks → To Workspace

saves signal to the Matlab workspace

Create a blank Simulink diagram named lab1_2.slx.

Using the blocks described above, draw in Simulink the block diagram depicted below. There are two scopes in the diagram. One scope monitors the tracking error, while the second scope monitors the output signal superimposed to the reference signal.

motor speed and reference
