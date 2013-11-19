# Gazebo Multi-Physics Engine Support

For robot simulation research and applications in areas such as controls, navigation and mobile manipulation, the simulated robot's dynamical actuation response or physical interaction with objects in the environment can often be obtained with varying degrees of realism. Tradeoffs between physics realism and simulation performance means simulation platforms user have to make tough choice between fast simulation or solutions that *sufficiently converges* to the exact answers when the model complexities are increased.

The governing equations behind dynamical systems of constrained rigid bodies is often described by a system of algebraic or ordinary differential equations. Proofs for solution existence and uniqueness have been diligently derived by many researchers over the years [cite](...). It is also a well known fact that increased model complexity such as redundant constraints, kinematic loops, realistic impact and non-linear frictional forces can render the solution non-unique or otherwise robust algorithms devoid of their solution existence or uniqueness properties [cite](...). In other word, increasing physical realism in simulations can easily make the solution process slow or non-trivial.

Further, for any particular set of governing equations of motion chosen by the simulatee, there exists many variables of simulation such as time discretization, modeling abstraction, numerical solution technique, each can have huge impact on the outcome of simulation. Gazebo tries to alleviate this problem by expanding the available physics toolset at hand by providing common physics abstraction interface into several well know physics engines. Details of supported engines are described in the [sections below](link to Gazebo Supported Physics Engines).


## Installation

At first (as of Gazebo 2.1, 3.0), multi-physics support will be available from source installs of Gazebo, with partial support (Bullet only) using binary install.

***TODO: Provide links to installation instructions here.  Check if the above statements are true for each engine as of Gazebo 2.1.***

We plan to make full multi-physics engine support available through Ubuntu debian binary installs with the future release of Gazebo 4.0.

***TODO: double check release planning.***

Run-time support for multiplexing multiple physics engines at once will not be available until Gazebo 5.0 or later.

***TODO: Check the Gazebo release numbers for each of the new features described above.***

## Gazebo Supported Physics Engines

This section summarizes details of supported physics engines (list maybe incomplete).

### Bullet
[Bullet 2](http://bulletphysics.org/wordpress/) is an Open Source physics engine written by Erwin Cousman, utilizing
iterative maximal coordinate system solver similar to
[ODE's quickstep solver](https://bitbucket.org/osrf/gazebo/src/default/deps/opende/src/quickstep.cpp).

Recently, several direct LCP solvers as well as support for single DOF Featherstone joint systems have been added
to the source code for testing and near future release.

In the pipeline is [Bullet 3](https://github.com/erwincoumans/bullet3), successor to Bullet 2.  This new engine
will be heavily GPU optimized with OpenCL support.

### DART
[DART](https://github.com/dartsim/dart) (Dynamic Animation and Robotics Toolkit) is a collaborative, cross-platform, open source library created by the Georgia Tech [Graphics Lab](http://www.cc.gatech.edu/graphics/) and [Humanoid Robotics Lab](http://www.golems.org/). The library provides data structures and algorithms for kinematic and dynamic applications in robotics and computer animation.  For more information, please visit the [official DART website](http://dartsim.github.io/).

### Open Dynamics Engine (ODE)
[Open Dynamics Engine (ODE)](http://opende.sourceforge.net/) is an Open Source physics engine authored by Russel Smith.
It's a well tested engine that has been used by many robotics and non-robotics projects alike....


### Simbody
[Simbody](https://simtk.org/home/simbody/) is a bio-mechanical inspired rigid multibody dynamics engine.
It is the solver used by [OpenSim](https://simtk.org/???), a popular muscular-skeletal simualtion front-end
commonly used by researchers in the bio-mechanical community.

Simbody was recently moved to [github repository](https://github.com/simbody/).


## Differences Between Different Physics Engines By Components

Each physics engine may hold very different views on components of a complete dynamics solver.  For example,

* how model properties and system states are described and stored,
* how time flow is controlled,
* what constraint solvers are used,
* what collision detection algorithms are used,
* how are collisions resolved / handled,
* what integrators or differencing schemes are used for advancing system states in time,
* how are intermediate states (e.g. joint or constraint Jacobians) and modeling variables (e.g. joint wrench, articulated body inertia) updated and accessed and
* what kind of error control is exposed to the end users, etc.

We'll attempt to highlight some of the more distinguished design or implementation differences between each of the physics engines. But as mentioned before, expected simulation outcome can be sensitive to various modeling and solver parameters, so this section can only serve as a general guideline, neglecting many solution technique details.  Given each physics engine is developed independently, the slightest difference in code can result in drastic divergence in simulation outcome.

### Constraint Solvers
**TODO Put a summary here:**

![Picture](https://dl.dropboxusercontent.com/s/0xril5vpmdr39dw/CollisionResponse.png)

 * Bullet
     * [btSequentialImpulseConstraintSolver](https://code.google.com/p/bullet/source/browse/tags/bullet-2.82/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h) and [Experimental Featherstone Solvers (not yet enabled through Gazebo)](https://code.google.com/p/bullet/source/browse/tags/bullet-2.82/src/#src%2FBulletDynamics%2FFeatherstone).
 * DART formulates the constraint problem as a velocity-based LCP, and solves the problem using the [Dantzig](https://github.com/dartsim/dart/blob/master/src/lcpsolver/lcp.h) or Lemke algorithm.  
 * ODE
     * [quickstep solver](https://bitbucket.org/osrf/gazebo/src/default/deps/opende/src/quickstep.cpp)
     * [worldstep solver](https://bitbucket.org/osrf/gazebo/src/default/deps/opende/src/lcp.cpp)
 * Simbody
     * [Elastic Foundation Solver](https://github.com/simbody/simbody/blob/master/Simbody/src/ElasticFoundationForceImpl.h)
     * [Featherstone Algorithm Implementation](https://github.com/simbody/simbody/blob/master/Simbody/src/MobilizedBody.cpp).

### Integrators
Describe integrators used or offered by each physics engine.

 * Bullet
 * DART supports first-order explicit Euler and RK4.
 * ODE
    * First order semi-implicit (semi-explicit) Euler.  The formulation as described in Anitescu Potra paper is a first order...
    * [Integrate velocity to position](https://bitbucket.org/osrf/gazebo/src/a676255484971eb823e037492a76fff1cfb66436/deps/opende/src/util.cpp?at=default#cl-274)
    * [Integrate force to velocity](https://bitbucket.org/osrf/gazebo/src/a676255484971eb823e037492a76fff1cfb66436/deps/opende/src/quickstep.cpp?at=default#cl-1964)
 * Simbody supports following [integrators](https://github.com/simbody/simbody/tree/simbody-3.3/SimTKmath/Integrators/include/simmath):
    * SimTK::RungeKuttaMersonIntegrator(system)
    * SimTK::RungeKutta3Integrator(system);
    * SimTK::RungeKutta2Integrator(system);
    * SimTK::SemiExplicitEuler2Integrator(system);

## Performance
There is not a universally objective way of quantifying performances for different
physics engines yet, but for specific use case such as the DRCSim, we have the following benchmark results.
Note performance results are volatile as they are sensitive to many factors outside of algorithm performance, things like software implementation, collision detection, adjustable physics parameters can greatly change the outcome of existing benchmarks.

 * Bullet - 
 * DART - 
 * ODE - 
 * Simbody - 



## Examples

These example simulation test cases demonstrates why one engine might be preferred over another due to simulation setup and requirements.

### By Robots

Pioneer for Navigation
Atlas (maybe PR2) for Mobile Manipulation at Near Real-Time
Swarm of Box-bots
Car Simulation without Dynamic with Sensor Integration

### Robust to Large External Perturbations without Strict Accuracy Requirement
If your robot is expected to "bang around" (the environment interaction is unpredictable and potentially violent), and when accuracy is not as critical (don't need 6 digits of precision) as your CAD simulation, and a rough estimation of forces and velocities will do, then iterative solvers are probably a better choice.

### Constraints or Joints with High Inertia Ratio
LCP iterative solvers such as PGS subject to low convergence problems when links constrained together have large inertia ratios in the direction of the constraint.  In this case, it is better to use Featherstone methods.

% ### I don't Need Dynamics, just using SetPosition or SetVelocity to get 3D sensor data integration
% ### I Just Need "Looks Right" Dynamics 99% of the Time (2 decimal points accuracy)

### Small Time Step Sizes, Requires Near Real-Time Simulation Performance
When the real-time step size is small, if near real-time performance is required, a simpler solver with less overhead is ideal.  ODE or Bullet can solve relatively quickly in this use case.

### Elastic Collision vs. Inelastic Collision
Only simbody can "bounce"

### Strict Accuracy Requirements
High Speed Controller Tuning


## FAQ / Overview

* How are multiple physics engines supported in Gazebo?
    * Several forward dynamics engines have been integrated as library dependencies by Gazebo.  Gazebo provides users with a unified modeling interface that transparently supports access to the physics and dynamics updates from each physics engine.  This is made possible by Gazebo's core physics API.
    * As of Gazebo-2.1, a user can pass in a commandline argument to tell gazebo which engine to use.  The ability to switch between different engines dynamically will be added in subsequent Gazebo release.

* Why is support for multiple physics engines useful?
    * There lack a research platform where drastically different engines are supported and tested thoroughly.  Testing and maintenance is a major effort, and requires a top down approach to make sure everything is treated fairly.  Sometimes custom attention is paid to a particular aspect of a physics engine because of its capabilities and differences with other physics engines, making comparison difficult.  For example, error measurement can be hard to compare differently between maximal and reduced coordinate system solvers, the problem varies a lot based on many factors such as requirements in performance, accuracy or simplifying assumptions imposed on the model.
    * A unified interface to multiple physics engines also provides a level playing field for comparing accuracy and performance of the physics engines.  These comparisons may help the upstream developers of the engines to identify bugs in their code or places where computational speed can be improved. It can also be used to compare algorithms for dynamic simulation.
    * Some solvers have restrictions on the underlying model structure.  For example, Featherstone solvers require simulated models to have tree structured kinematics, and contacts or kinematic loop closures need to be modeled with LCP constraints.
    * Different dynamic solver algorithms can have drastically different behaviors in terms of accuracy, performance, robustness to errors and perturbation when simulated models or simulation parameters are changed.
    For example, Projected Gauss Seidel (PGS)-type constraint solvers are much more forgiving to large errors and perturbations in mixed linear complementarity problems (MLCP),
    but at the same time, un-preconditioned PGS solvers are well known for non-convergence in certain situations.
    On the other hand, dynamics solvers using Featherstone's articulated body algorithm to model articulated system's
    internal joints will always have zero joint error, but a system's interaction with its environment modeled by constraints may also pose convergence difficulties (as demonstrated by [this example](some example)).
    * Diverse feature support for different physics engines.
    * Fine tuning individual simulation scenarios requires insight to how the system is being modeled and the specific type of solver used.
    Tutorials in the [examples section](somewhere below) below will attempt to demonstrate differences in simulation results for different physics engines under different scenarios.
    * Emphasize existing features in gazebo outside of physics, e.g. camera, laser sensors, plugin api, ros integration.
* What criteria to consider when choosing a physics engine? (highlight a few corner use cases where certain needs are met by certain physics engines)
    * Highly accurate:
        * Designing and tuning controllers.
        * Model validation.
        * Different error characteristics for differrent algorithms.
        * Numerical robustness to large errors or perturbations.
        * Flexible error control.
    * Ultra-fast simulation for predictive control:
        * Evolving a controller or robot design based on a genetic algorithms, hunreds of thousands of simulations are needed to
        perform sensitivity analysis.
        * Predictive control, where the user wants to know what might happen when teleoperating robot.
    * Low latency, synchronization with real-time controllers:
        * Real-time teleoperation testing.
        * Testing software that are running asynchronously, and expects a "real robot" hardware.
    * Scalable:
        * Swarm research, thousands of robots.
    * Feature support:
        * Heightmap
        * Deformable bodies / cloth (not working yet)
        * Contact modeling (e.g. Simbody elastic foundation, DART deformable bodies).
        * Access to physics engine internal states (Jacobians, ABI, etc.)


