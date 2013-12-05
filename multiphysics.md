# Gazebo Multi-Physics Engine Support

Gazebo is a simulation platform for robotics research and applications development.
Simulated robot dynamics is often computed with varying degrees of realism.
Whether the simulator is used for modeling actuation response in controller design, testing executives in navigation scenarios or predicting environment interactions for mobile manipulation research, balancing tradeoff decisions between physics realism and simulator performance is often a major undertaking.
Especially with increasing robot dynamic capability and model complexities, getting both fast simulation and results that *sufficiently converges* to the theoretical solution can be difficult if not impossible at times.

The governing equations behind modeling constrained rigid multibody dynamics is often described by a system of algebraic or ordinary differential equations.
For simple frictionless systems, solution existence and uniqueness proofs have been diligently derived by researchers over the years [cite](...).
However, for systems with increased physics fidelity or model complexity,
existence or uniqueness properties are no longer guaranteed [cite](...).
For example, simulating mechanisms with redundant constraints, kinematic loops, impact or non-linear frictional forces can render many otherwise robust algorithms inefficient or ineffective.
Further, tunable simulation parameters such as temporal discretization, model material properties, applied numerical integration techniques, etc. can often affect simulation outcomes significantly.

Gazebo tries to alleviate these issues by expanding the available physics toolsets at hand, providing a common physics abstraction interface for several well know physics engines with different solution approaches.
The list of Gazebo supported physics engines currently includes [Open Dynamics Engine (ODE)](...), [Bullet](...), [DART](...) and [Simbody](...).

More details of each supported engines can be found in the [sections below](link to Gazebo Supported Physics Engines).

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

Each physics engine may hold very different views / solution approaches to different components in a complete dynamics solver.  Example components that maybe treated differently by physics engines are listed below,

* how model properties and system states are described and stored,
* how time flow is controlled,
* what constraint solvers are used,
* what collision detection algorithms are used,
* how are collisions resolved / handled,
* what integrators or differencing schemes are used for advancing system states in time,
* how are intermediate states (e.g. joint or constraint Jacobians) and modeling variables (e.g. joint wrench, articulated body inertia) updated and accessed and
* what kind of error control is exposed to the end users, etc.

We'll attempt to highlight some of the more distinguished design or implementation differences between each of the physics engines. But as mentioned before, expected simulation outcome can be extremely sensitive to various modeling and solver parameters, many of the parameters are not mentioned here. And the fact that the slightest difference in code can result in drastically diverging simulation outcome. With that in mind, this section can only serve as an approximate guideline, blantantly neglecting many solver details due to wiki editor's ignorance.  Fortunately, community wiki contribution can potentially make this document more complete and less erroneous over time.

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


