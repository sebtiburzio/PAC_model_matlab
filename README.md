# PAC_model_matlab

This repository contains Matlab code relating to my MSc Robotics [thesis](https://repository.tudelft.nl/islandora/object/uuid%3A7f543ce2-b7a5-456c-ab7b-81171c353de2?collection=education) on manipulation of deformable linear objects using Affine Curvature modelling, subsequently developed into [this paper](https://arxiv.org/abs/2402.16114).

See [here](https://github.com/sebtiburzio/floating_PAC_model) for the code dealing with processing the data recorded in the experiments as well as a less developed Python implementation of the model using Sympy.

There are two models:

## Fixed Base Model

This is a single affine curvature segment with a fixed end, as conceived in [this paper](https://ieeexplore.ieee.org/abstract/document/9303976).

The main scripts are:

### model_gen_fixed_base.m

Generates functions for the forward kinematics and terms in the dynamic EOM, which are saved in the `automatically_generated` directory. 

**Note:** There are some differences in how the model is defined in the code compared with the thesis report. In the formal derivation the model is defined in the XY plane (Y upwards), however in the matlab code upwards is defined as Z to align with the manipulator base frame for experiments. Positive rotation in the formal model is anticlockwise in the XY plane (around the Z-axis); in the code it is clockwise in the XZ plane (around the Y-axis). 

Also the z-coordinate of the forward kinematics is negated so that at 0 curvature the object points downwards instead of upwards, without an additional 180 degree rotation (which the B' frame introduces in the report).

### fwd_sim.m

Framework for forward simulation of the dynamic EOM, which are implemented in `f_fcn.m` and `dynamics.slx`. 

Using the variable gravitational field direction, it is also possible to use this model to simulate the equilibrium state of the object when held in different orientations. This approach is used extensively for steady state comparisons; `fss_fcn.m` and `ss_solver.slx` implement simplified dynamics allowing much faster determination of the equilibrium by forward simulation.

<img src="https://github.com/sebtiburzio/PAC_model_matlab/assets/95340175/58d2dc43-d4e2-4572-8ead-384c175d8b2e" height="330">
<img src="https://github.com/sebtiburzio/PAC_model_matlab/assets/95340175/032b2827-417c-4a03-b66c-cf0406c78b72" height="330">

### param_id.m & param_id_static.m

A collection of parameter ID formulations to determine the object properties. 

## Floating Base Model

Here the model is extended with 3 additional DOF at the base: X, Z and Phi representating a planar manipulator TCP.

### model_gen.m

The same considerations as for the fixed base forward kinematics apply: when all states are 0, the base of the object is at (0,0), and the tip is at (0,-L). Positive Phi will rotate the object clockwise in the XZ plane, similarly positive Theta will bend it clockwise.

### fwd_sim.m

In the full dynamic model input force and torque at the base is applied as the third argument of `f_fcn.m`: F = (Fx, Fz, Ty). This can be applied directly in `dynamics.slx` or by importing a signal into the matrix variable `actuation`.

**Note:** When simulating the model the solver may hang at some stage, probably due to hitting a singularity (to be investigated). In this case, the state evolution will still be saved in `q_ev.mat` if the execution is interrupted.

https://github.com/sebtiburzio/PAC_model_matlab/assets/95340175/1fb80a96-2ca4-42a0-a4a4-adbc72e858b8

### solve_static.m

Examples of setting up nonlinear optimisation problems to generate manipulator solutions for steady state control of the object endpoint position and orientation. 

https://github.com/sebtiburzio/PAC_model_matlab/assets/95340175/9968affd-c4e2-4f4b-82fa-2cf3014d43c5

## Object Properties
The object properties determined through the parameter identification experiments are saved in the `object_parameters` directory, and should be loaded in the script initialisation. There are two sets: those in the main directory were determined with the 2 stage process (k and theta_bar from static data, then beta from dynamic data), while those in the sub directory `full_dynamic_id` were determined from dynamic data only
