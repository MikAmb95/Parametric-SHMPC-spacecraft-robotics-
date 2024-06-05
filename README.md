# Parametric-SHMPC-spacecraft-robotics-
In this folder, an example of parametric Shrinking Horizon Model Predictive Control is shown. The system is assumed to be under the effect of a feedback linearization. Subsequently, the control input gains are optimised by solving an optimal control problem. 

## Model Description ##

To derive the equation of motion of the chaser spacecraft equipped with a robotic arm, we first define the generalized system coordinates that can be expressed as

```math
q(t) = \begin{bmatrix} q^\top_c(t),& q^\top_r(t) \end{bmatrix}^\top,
```
where 
```math
q^\top_c(t) = \begin{bmatrix} x_c(t),\;y_c(t),\;z_c(t),\;\phi_c(t),\;\theta_c(t),\;\psi_c(t)\end{bmatrix}
```
 contains the generalized coordinates of the chaser (i.e. the base), $x_c(t), y_c(t), z_c(t)$ are the position coordinates of the CoM of the spacecraft while its orientation is described by 
$\phi_c(t), \theta_c(t), \psi_c(t)$. 

The generalized coordinates of the robotic arm are 
```math
q^\top_r(t) = \begin{bmatrix}q_1(t),...,q_{nr}(t) \end{bmatrix}$ 
```
where $nr$ is the number of DoFs (or joints) of the arm.
The equation of the dynamics of the Spacecraft Manipulator can be expressed as
```math
    \begin{bmatrix}
        M_c(q(t))&M_{c,r}(q(t))\\M^\top_{c,r}(q(t))&M_r(q(t))
    \end{bmatrix}\begin{bmatrix}
        \ddot{q}^\top_c(t)\\\ddot{q}^\top_r(t)
    \end{bmatrix}+\begin{bmatrix}
        C_c(q(t),\dot{q}(t))\\C_r(q(t),\dot{q}(t)) 
    \end{bmatrix}= \begin{bmatrix}
         u_c(t) \\u_r(t)   
        \end{bmatrix},        
```
where $M_c(q(t))\in \mathbb R^{6+nr} \rightarrow \mathbb R^{6+nr}$ is the chaser inertia matrix, $M_r(q(t))\in \mathbb R^{6+nr} \rightarrow \mathbb R^{nr+nr}$ is the spacecraft inertia matrix, and $M_{c,r}(q(t))\in \mathbb R^{6+nr} \rightarrow \mathbb R^{6+nr}$ is the interaction inertia matrix between the chaser and the robotic arm. The vectors $C_c(q(t),\dot{q}(t)) \in \mathbb R^{6+nr} \times \mathbb R^{6+nr} \rightarrow \mathbb R^{6}$ and $C_r(q(t),\dot{q}(t)) \in \mathbb R^{6+nr} \times \mathbb R^{6+nr} \rightarrow \mathbb R^{nr}$ represent the Coriolis and centrifugal components of the chaser and the arm, respectively. Moreover, $u_c(t) \in \mathbb R^6$ and $u_r(t) \in \mathbb R^{nr}$, are the inputs for the chaser and for the robot, respectively. More precisely, $u_c(t)$ contains the force and torques applied on to chaser using reaction wheels and thrusters, while $u_r(t)$ represents the torque applied to each joint of the manipulator. The previous dynamic model can be written in compact matrix form as 

```math
    M(q)\ddot{q} + C(q,\dot{q}) = u.      
```

For a 3DoFs robotic arm mounted on a spacecraft base, the generalized coordinates are 

```math
    q_c(t) = \begin{bmatrix}
        x_c(t),y_c(t),\psi_c(t)
    \end{bmatrix}^\top, \quad q_r(t) = \begin{bmatrix}
        q_1(t),q_2(t),q_3(t)
    \end{bmatrix}^\top.
```


## Control Description ##

Starting from 
```math
    M(q)\ddot{q} + C(q,\dot{q}) = u,      
```
we first perform a feedback linearization considering

```math
    u = M(q)v + C(q,\dot{q}).      
```
where $v$ is an additional control input. The linearized dynamics turned out to be

```math
    \ddot{q} = v.      
```
Then, we desing the additional control input as 

```math
 v = K_p (x_f - x) - K_d\dot{x} ,
```
where $K_p$ is the proportional gain, $K_d$ is the derivative gain, $x_d$ is the desired reference (i.e $x_{c,r}(t),y_{c,r}(t),\psi_{c,r}(t),q_{1,r}(t),q_{3,r}(t),q_{3,r}(t)$ ), $x$ is the current measured state (e.g $x_c(t),y_c(t),\psi_c(t),q_1(t),q_3(t),q_r(t)$ ), and $\dot{x}$ the current velocity. 

In the script pSHMPC_5Dofs.m

1. The function _computeControl_ implement the additional control input
2. Saturation on the additional inputs are also considered. 
3. The system is assumed to be linearized before. Therefore, the dynamics expressed in the function _FSS_dynamic_model_ are linear. 

### Optimal Control Problem ###

1. The additional control input contains twelve paramenters, six proprortional gains (i.e. $K_{P1}$, $K_{P2}$, $K_{P3}$, $K_{P4}$, $K_{P5}$, $K_{P6}$) and six derivative gains (i.e. $K_{D1}$, $K_{D2}$, $K_{D3}$, $K_{D4}$, $K_{D5}$, $K_{D6}$).
2. Instead of using as decision variables the six additional control input $v$, in this work we used the gain parameters as variables to be optimized by the solver.
3. We fix two gains (i.e. $K_{P1}$ and $K_{D1}$) to suboptimal values. The other ten can be optimised by the solver.
4. The solver minimises a certain quadratic cost function in order to guide the system towards the desired reference and at the same time ensure that constraints are satisfied (e.g. avoidance of an obstacle). 



## File Description ##

1. pSHMPC_5Dofs.m  : main script that contains simulations
2. print_system_config.m : script to print system motion
3. (Additional) Example_without_obstacle_avoidance.m : simulation script without obstacle avoidance

## How to run ##

Run the script: pSHMPC_5Dofs.m 
Note: from the script print_system_config.m set the flag printMotion to false if you do not wish to display the motion history
