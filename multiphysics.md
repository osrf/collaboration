# Gazebo Multi-Physics Engine Support

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

## Gazebo Supported Physics Engines
This section summarizes each physics engine.

### Bullet
[Bullet 2](http://bulletphysics.org/wordpress/) is an Open Source physics engine written by Erwin Cousman, utilizing
iterative maximal coordinate system solver similar to
[ODE's quickstep solver](https://bitbucket.org/osrf/gazebo/src/default/deps/opende/src/quickstep.cpp).

Recently, several direct LCP solvers as well as support for single DOF Featherstone joint systems have been added
to the source code for testing and near future release.

In the pipeline is [Bullet 3](https://github.com/erwincoumans/bullet3), successor to Bullet 2.  This new engine
will be heavily GPU optimized with OpenCL support.

### DART
[DART](https://github.com/dartsim/dart), a collaborative project based on [RTQL8](https://bitbucket.org/karenliu/rtql8),
is a physics engine developed at Georgia Institute of Technology under the direction of Karen Liu and Michael Stilman.

DART uses Featherstone solver for parts of the models that can be described by a kinematic tree, and uses
Dantzig method (similar to
[ODE's worldstep solver](https://bitbucket.org/osrf/gazebo/src/default/deps/opende/src/lcp.cpp)) for all other
constraints.

Currently...

### Open Dynamics Engine (ODE)
[Open Dynamics Engine (ODE)](http://opende.sourceforge.net/) is an Open Source physics engine authored by Russel Smith.
It's a well tested engine that has been used by many robotics and non-robotics projects alike....


### Simbody
[Simbody](https://simtk.org/home/simbody/) is a bio-mechanical inspired rigid multibody dynamics engine.
It is the solver used by [OpenSim](https://simtk.org/???), a popular muscular-skeletal simualtion front-end
commonly used by researchers in the bio-mechanical community.

Simbody was recently moved to [github repository](https://github.com/simbody/).


## Differences Between Different Physics Engines By Components
We'll highlight differences between each of the physics engines by components.

### Dynamics and Constraint Solvers

 * Bullet
     * [btSequentialImpulseConstraintSolver](https://code.google.com/p/bullet/source/browse/tags/bullet-2.82/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h) and [Experimental Featherstone Solvers (not yet enabled through Gazebo)](https://code.google.com/p/bullet/source/browse/tags/bullet-2.82/src/#src%2FBulletDynamics%2FFeatherstone).
 * DART
     * [Dantzig Constraint Solver](https://github.com/dartsim/dart/blob/master/src/lcpsolver/lcp.h)
     * [Featherstone Algorithm Implementation](https://github.com/dartsim/dart/blob/master/src/dynamics/Skeleton.h).
 * ODE
     * [quickstep solver](https://bitbucket.org/osrf/gazebo/src/default/deps/opende/src/quickstep.cpp)
     * [worldstep solver](https://bitbucket.org/osrf/gazebo/src/default/deps/opende/src/lcp.cpp)
 * Simbody
     * [Elastic Foundation Solver](https://github.com/simbody/simbody/blob/master/Simbody/src/ElasticFoundationForceImpl.h)
     * [Featherstone Algorithm Implementation](https://github.com/simbody/simbody/blob/master/Simbody/src/MobilizedBody.cpp).

### Integrators
Describe integrators used or offered by each physics engine.

 * Bullet
 * DART
 * ODE
    * First order semi-implicit (semi-explicit) Euler.  The formulation as described in Anitescu Potra paper is a first order...
    * [Integrate velocity to position](https://bitbucket.org/osrf/gazebo/src/a676255484971eb823e037492a76fff1cfb66436/deps/opende/src/util.cpp?at=default#cl-274)
    * [Integrate force to velocity](https://bitbucket.org/osrf/gazebo/src/a676255484971eb823e037492a76fff1cfb66436/deps/opende/src/quickstep.cpp?at=default#cl-1964)
 * Simbody [integrators](https://github.com/simbody/simbody/tree/simbody-3.3/SimTKmath/Integrators/include/simmath):
    * SimTK::RungeKuttaMersonIntegrator(system)
    * SimTK::RungeKutta3Integrator(system);
    * SimTK::RungeKutta2Integrator(system);
    * SimTK::SemiExplicitEuler2Integrator(system);

## Performance
There is not a universally objective way of quantifying performances for different
physics engines yet, but for specific use case such as the DRCSim, we have the following benchmark results.
Note that these benchmark results are preliminary, as we continue on integration and optimization
of each engine, the results may change.

 * Bullet - contacts are still unstable right now.
 * DART - no error correction.
 * ODE - small overhead at 1kHz (typical for robotics applications with real-time effort controllers).
 * Simbody - slow but accurate.  rigid body contact will speed this up greatly.













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








# Summary Table

Note: table worked ok on Gazebo wiki, but not in bitbucket.

| Physics Engine || Reduced Coordinate System || Contact Constraint Resolution || Variable Time Stepping
|-
| ODE || No || LCP with Dantzig or PGS || No
|-
| Bullet || No (but under development) || LCP with PGS || No
|-
| Simbody || Mobilizers || Hertz Contact Model || Supported
|-
| DART || Joints || LCP with Dantzig || Supported
