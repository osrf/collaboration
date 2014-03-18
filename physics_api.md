# Existing Physics API

## Discussion 2014-03-18
Nate, Evan, Steve, John

 * Nate: Simplify Load/Init into single Load or generalize multi-step loading process.
 * Evan: It's difficult to map objects in one engine to gazebo's abstraction if the conceptial abstractions are different. Alternatively, the physics engine need to instantiate from SDF by itself.
     * what about engines without SDF loaders?
 * Evan: Force reference frames definition are unclear in Gazebo C++ API.
 * Get list of "capabilities" for each physics engine.
 * Do we allow users to change gravity at runtime?
 * Do we allow users to change inertia at runtime?
 * Stragegy: start simple.  Keep existing integration intact and separate from new physics plugin system.
 * What if physics engines want to handle collision internally?  Simbody/DART/ODE/Bullet allows third party collision engine callbacks.  Moby would like to have total control over collision handling due to continuous collision detection capabilities.  This is needed to prevent interpenetration.  (Mirtich's approach).
 * Should Gazebo have it's own collision detection library.
 
TODO:

 * Decide between generalized coordinate support vs. absolute coordinate support.
     * Steve: find a subset of exiting API that works with both.
        * GetWorldPose, Joint::SetEffort are ok
        * Link::SetLinearVel maybe not
 * Decide initial simulation scenarios:
     * box on plane
     * pendulum
     * pioneer
 * Testing for physics plugin api.
 * Repeat physics tests for plugin physics.
 * Create OpenPL repository
     * Define initial API
* Example PAL / OPAL (http://sourceforge.net/p/pal/code/HEAD/tree/pal/pal_i/bullet/bullet_pal.cpp)
* Look at OpenGL API

## High Level Description

Current gazebo physics engine

### Initialization and Shutdown

 * [PhysicsEngine::Load](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/PhysicsEngine.hh?at=default#cl-54): This function loads physics engine description from SDF without initializing the engine itself.
 * [PhysicsEngine::Init](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/PhysicsEngine.hh?at=default#cl-57): Initialize underlying physics engine.
 * [PhysicsEngine::Reset](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/PhysicsEngine.hh?at=default#cl-63)
 * [PhysicsEngine::Fini](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/PhysicsEngine.hh?at=default#cl-60): Multi-step shutdown behavior.
 * [PhysicsEngine::InitForThread](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/PhysicsEngine.hh?at=default#cl-66): Move this into Load or Init.  Prepare physics engine for multi-threaded access.

### Below can happen anytime during simulation after Init. 
 * CreateModel
 * CreateLink
 * CreateCollision
 * CreateShape
 * CreateJoint

### Runtime Modifications Getters and Setters

 * [PhysicsEngine::SetSeed](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/PhysicsEngine.hh?at=default#cl-77): Set seed of engine random seed.
 * [PhysicsEngine::SetTargetRealTimeFactor](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/PhysicsEngine.hh?at=default#cl-97)
 * GetGravity
 * ...
 
### Operational

 * [PhyiscsEngine::UpdateCollision](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/PhysicsEngine.hh?at=default#cl-69)
 * [PhysicsEngine::UpdatePhysics](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/PhysicsEngine.hh?at=default#cl-108)


# New Physics API
 
## [Entity.hh](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/Entity.hh?at=default)

### Set States

      /// \brief Set the world pose of the entity.
      /// \param[in] _pose The new world pose.
      /// \param[in] _notify True = tell children of the pose change.
      /// \param[in] _publish True to publish the pose.
      public: void SetWorldPose(const math::Pose &_pose,
                                bool _notify = true,
                                bool _publish = true);

      /// \brief Set angular and linear rates of an physics::Entity.
      /// \param[in] _linear Linear twist.
      /// \param[in] _angular Angular twist.
      /// \param[in] _updateChildren True to pass this update to child
      /// entities.
      public: void SetWorldTwist(const math::Vector3 &_linear,
                                 const math::Vector3 &_angular,
                                 bool _updateChildren = true);                                

      /// \brief Set the initial pose.
      /// \param[in] _pose The initial pose.
      public: void SetInitialRelativePose(const math::Pose &_pose);

      /// \brief Set the pose of the entity relative to its parent.
      /// \param[in] _pose The new pose.
      /// \param[in] _notify True = tell children of the pose change.
      /// \param[in] _publish True to publish the pose.
      public: void SetRelativePose(const math::Pose &_pose,
                                   bool _notify = true,
                                   bool _publish = true);

### Get States

      /// \brief Get the absolute pose of the entity.
      /// \return The absolute pose of the entity.
      public: inline const math::Pose &GetWorldPose() const
              {return this->worldPose;}

      /// \brief Get the linear velocity of the entity in the world frame.
      /// \return A math::Vector3 for the linear velocity.
      public: virtual math::Vector3 GetWorldLinearVel() const
              {return math::Vector3();}

      /// \brief Get the angular velocity of the entity in the world frame.
      /// \return A math::Vector3 for the velocity.
      public: virtual math::Vector3 GetWorldAngularVel() const
              {return math::Vector3();}

      /// \brief Get the linear acceleration of the entity in the world frame.
      /// \return A math::Vector3 for the acceleration.
      public: virtual math::Vector3 GetWorldLinearAccel() const
              {return math::Vector3();}

      /// \brief Get the angular acceleration of the entity in the world frame.
      /// \return A math::Vector3 for the acceleration.
      public: virtual math::Vector3 GetWorldAngularAccel() const
              {return math::Vector3();}

      /// \brief Get the initial relative pose.
      /// \return The initial relative pose.
      public: math::Pose GetInitialRelativePose() const;

      /// \brief Get the pose of the entity relative to its parent.
      /// \return The pose of the entity relative to its parent.
      public: math::Pose GetRelativePose() const;

      /// \brief Get the linear velocity of the entity.
      /// \return A math::Vector3 for the linear velocity.
      public: virtual math::Vector3 GetRelativeLinearVel() const
              {return math::Vector3();}

      /// \brief Get the angular velocity of the entity.
      /// \return A math::Vector3 for the velocity.
      public: virtual math::Vector3 GetRelativeAngularVel() const
              {return math::Vector3();}

      /// \brief Get the linear acceleration of the entity.
      /// \return A math::Vector3 for the acceleration.
      public: virtual math::Vector3 GetRelativeLinearAccel() const
              {return math::Vector3();}

      /// \brief Get the angular acceleration of the entity.
      /// \return A math::Vector3 for the acceleration.
      public: virtual math::Vector3 GetRelativeAngularAccel() const
              {return math::Vector3();}

## [Link.hh](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/Link.hh?at=default)

### Set States

      /// \brief Set the linear velocity of the body.
      /// \param[in] _vel Linear velocity.
      public: virtual void SetLinearVel(const math::Vector3 &_vel) = 0;

      /// \brief Set the angular velocity of the body.
      /// \param[in] _vel Angular velocity.
      public: virtual void SetAngularVel(const math::Vector3 &_vel) = 0;

      /// \brief Set the linear acceleration of the body.
      /// \param[in] _accel Linear acceleration.
      public: void SetLinearAccel(const math::Vector3 &_accel);

      /// \brief Set the angular acceleration of the body.
      /// \param[in] _accel Angular acceleration.
      public: void SetAngularAccel(const math::Vector3 &_accel);

### Get States

      /// \brief Get the pose of the body's center of gravity in the world
      ///        coordinate frame.
      /// \return Pose of the body's center of gravity in the world coordinate
      ///         frame.
      public: math::Pose GetWorldCoGPose() const;

      /// \brief Get the linear velocity of a point on the body in the world
      ///        frame, using an offset expressed in a body-fixed frame. If
      ///        no offset is given, the velocity at the origin of the Link
      ///        frame will be returned.
      /// \param[in] _offset Offset of the point from the origin of the Link
      ///                    frame, expressed in the body-fixed frame.
      /// \return Linear velocity of the point on the body
      public: virtual math::Vector3 GetWorldLinearVel(
          const math::Vector3 &_offset = math::Vector3(0, 0, 0)) const = 0;

      /// \brief Get the linear velocity of a point on the body in the world
      ///        frame, using an offset expressed in an arbitrary frame.
      /// \param[in] _offset Offset from the origin of the link frame expressed
      ///                    in a frame defined by _q.
      /// \param[in] _q Describes the rotation of a reference frame relative to
      ///               the world reference frame.
      /// \return Linear velocity of the point on the body in the world frame.
      public: virtual math::Vector3 GetWorldLinearVel(
                  const math::Vector3 &_offset,
                  const math::Quaternion &_q) const = 0;

      /// \brief Get the linear velocity at the body's center of gravity in the
      ///        world frame.
      /// \return Linear velocity at the body's center of gravity in the world
      ///         frame.
      public: virtual math::Vector3 GetWorldCoGLinearVel() const = 0;

      /// \brief Get the linear velocity of the body.
      /// \return Linear velocity of the body.
      public: math::Vector3 GetRelativeLinearVel() const;

      /// \brief Get the angular velocity of the body.
      /// \return Angular velocity of the body.
      public: math::Vector3 GetRelativeAngularVel() const;

      /// \brief Get the linear acceleration of the body.
      /// \return Linear acceleration of the body.
      public: math::Vector3 GetRelativeLinearAccel() const;

      /// \brief Get the linear acceleration of the body in the world frame.
      /// \return Linear acceleration of the body in the world frame.
      public: math::Vector3 GetWorldLinearAccel() const;

      /// \brief Get the angular acceleration of the body.
      /// \return Angular acceleration of the body.
      public: math::Vector3 GetRelativeAngularAccel() const;

      /// \brief Get the angular acceleration of the body in the world
      /// frame.
      /// \return Angular acceleration of the body in the world frame.
      public: math::Vector3 GetWorldAngularAccel() const;

### Set Forces / Torques

      /// \brief Set the force applied to the body.
      /// \param[in] _force Force value.
      public: virtual void SetForce(const math::Vector3 &_force) = 0;

      /// \brief Set the torque applied to the body.
      /// \param[in] _torque Torque value.
      public: virtual void SetTorque(const math::Vector3 &_torque) = 0;

      /// \brief Add a force to the body.
      /// \param[in] _force Force to add.
      public: virtual void AddForce(const math::Vector3 &_force) = 0;

      /// \brief Add a force to the body, components are relative to the
      /// body's own frame of reference.
      /// \param[in] _force Force to add.
      public: virtual void AddRelativeForce(const math::Vector3 &_force) = 0;

      /// \brief Add a force to the body using a global position.
      /// \param[in] _force Force to add.
      /// \param[in] _pos Position in global coord frame to add the force.
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                  const math::Vector3 &_pos) = 0;

      /// \brief Add a force to the body at position expressed to the body's
      /// own frame of reference.
      /// \param[in] _force Force to add.
      /// \param[in] _relPos Position on the link to add the force.
      public: virtual void AddForceAtRelativePosition(
                  const math::Vector3 &_force,
                  const math::Vector3 &_relPos) = 0;

      /// \brief Add a torque to the body.
      /// \param[in] _torque Torque value to add to the link.
      public: virtual void AddTorque(const math::Vector3 &_torque) = 0;

      /// \brief Add a torque to the body, components are relative to the
      /// body's own frame of reference.
      /// \param[in] _torque Torque value to add.
      public: virtual void AddRelativeTorque(const math::Vector3 &_torque) = 0;

### Get Forces / Torques

      /// \brief Get the force applied to the body.
      /// \return Force applied to the body.
      public: math::Vector3 GetRelativeForce() const;

      /// \brief Get the force applied to the body in the world frame.
      /// \return Force applied to the body in the world frame.
      public: virtual math::Vector3 GetWorldForce() const = 0;

      /// \brief Get the torque applied to the body.
      /// \return Torque applied to the body.
      public: math::Vector3 GetRelativeTorque() const;

      /// \brief Get the torque applied to the body in the world frame.
      /// \return Torque applied to the body in the world frame.
      public: virtual math::Vector3 GetWorldTorque() const = 0;


## [Joint.hh](https://bitbucket.org/osrf/gazebo/src/577847c43d021f7edc838a30c0eafc99ea312571/gazebo/physics/Joint.hh?at=default#cl-302)

### Set States

      /// \brief Set the velocity of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \param[in] _vel Velocity.
      public: virtual void SetVelocity(int _index, double _vel) = 0;
      
      /// \brief If the Joint is static, Gazebo stores the state of
      /// this Joint as a scalar inside the Joint class, so
      /// this call will NOT move the joint dynamically for a static Model.
      /// But if this Model is not static, then it is updated dynamically,
      /// all the conencted children Link's are moved as a result of the
      /// Joint angle setting.  Dynamic Joint angle update is accomplished
      /// by calling JointController::SetJointPosition.
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle Angle to set the joint to.
      public: void SetAngle(int _index, math::Angle _angle);

### Get States

      /// \brief Get the rotation rate of an axis(index)
      /// \param[in] _index Index of the axis.
      /// \return The rotaional velocity of the joint axis.
      public: virtual double GetVelocity(int _index) const = 0;

      /// \brief Get the angle of rotation of an axis(index)
      /// \param[in] _index Index of the axis.
      /// \return Angle of the axis.
      public: math::Angle GetAngle(int _index) const;

### Set Forces / Torques

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  Force is additive (multiple calls
      /// to SetForce to the same joint in the same time
      /// step will accumulate forces on that Joint).
      /// Forces are truncated by effortLimit before applied.
      /// \param[in] _index Index of the axis.
      /// \param[in] _effort Force value.
      public: virtual void SetForce(int _index, double _effort) = 0;

### Get Forces / Torques

      /// \brief @todo: not yet implemented.
      /// Get external forces applied at this Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] _index Index of the axis.
      /// \return The force applied to an axis.
      public: virtual double GetForce(unsigned int _index);

      /// \brief get internal force and torque values at a joint.
      ///
      ///   The force and torque values are returned in  a JointWrench
      ///   data structure.  Where JointWrench.body1Force contains the
      ///   force applied by the parent Link on the Joint specified in
      ///   the parent Link frame, and JointWrench.body2Force contains
      ///   the force applied by the child Link on the Joint specified
      ///   in the child Link frame.  Note that this sign convention
      ///   is opposite of the reaction forces of the Joint on the Links.
      ///
      ///   FIXME TODO: change name of this function to something like:
      ///     GetNegatedForceTorqueInLinkFrame
      ///   and make GetForceTorque call return non-negated reaction forces
      ///   in perspective Link frames.
      ///
      ///   Note that for ODE you must set
      ///     <provide_feedback>true<provide_feedback>
      ///   in the joint sdf to use this.
      ///
      /// \param[in] _index Not used right now
      /// \return The force and torque at the joint, see above for details
      /// on conventions.
      public: virtual JointWrench GetForceTorque(unsigned int _index) = 0;

      /// \brief Get the forces applied to the center of mass of a physics::Link
      /// due to the existence of this Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] index The index of the link(0 or 1).
      /// \return Force applied to the link.
      public: virtual math::Vector3 GetLinkForce(unsigned int _index) const = 0;

      /// \brief Get the torque applied to the center of mass of a physics::Link
      /// due to the existence of this Joint.
      /// Note that the unit of torque should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] index The index of the link(0 or 1)
      /// \return Torque applied to the link.
      public: virtual math::Vector3 GetLinkTorque(
                  unsigned int _index) const = 0;



      
## Discussions 2013

### URDF Loops Brainstorming

#### Steve

urdf: tree
sdformat: graph

fallback strategies for putting graphs in a tree

 - break the loops
 - freeze looped joints so that the loop becomes like a single link

~~~
<urdf>
  <link/>
  <joint/>
  <graph>
    <sdf>
    </sdf>
  </graph>
</urdf>
~~~

# Discussions 2014-03-18

## Open Issues

 * Should we allow the choice between single and double precision?
 * Rename Joint::SetAngle and Joint::GetAngle.  Change input param and return type from math::Angle to something that works with both sliders and hinge joints. Instead of Angle, it could be configuration, state, or position.
 * Rename Joint::SetForce to SetEffort.
 * Write tests for each API to indicate how we expect it to work.
 * Joint::SetVelocity is persistant but Joint::SetForce is not.  Consider renaming to Joint::SetForce(duration, start time) or creating Joint::SetImpulse.
 * Reference frames
    * Consider attaching reference frames to state getters and setters (currently mostly world or "relative?" frame).
    * Consider attaching Entity pointer to transforms (Pose, Vector3, Quaternion, Wrench) to specify a reference frame (as suggested by Jesper for Wrenches).
    * What to do with Relative Get and Set API, currently it's not so intuitive.
 * Merging URDF and SDF 
   * Packaging: merging urdfdom_headers, urdfdom, sdformat
   * Merge URDF and SDF way of specifying kinematics.  URDF: tree, recursive hierarchy, SDF: graph, flat hierarchy.
       * Define joints as **breakable**, ignore joint if needed.
       * Define <graph> or <superlinks>, designating groups of links to be fused if needed.
       * Use joint mimic to simplify graphs.  See for example the [JointMimic](https://github.com/ros/urdfdom_headers/blob/master/urdf_model/include/urdf_model/joint.h#L149) class.  This can be simulated GearBox joints.
 * Introduce **prescribed motion**.
 * Introduce motor controlled joints.
 * Different types of SetPose()
   * Setting Gazebo Link state independent of physic engines (disregarding dynamics, constraints and joints).
   * Set link pose/twist in Cartesian solver case, pose/twist are not transferred to child links if the desired velocity is in the null space of the constraint.
   * Set link pose/twist in Internal coordinate system, pose/twist are propagated to child links as if they are welded.
 * Add an "AssemblySolver" (Sherm).  Both for prescribed link/joint constraints.
   * For obtaining IK solutions.
   * Using position teleport (split impulse).
 * Joint::SetMaxForce and Joint::GetMaxForce does not make sense in internal coordinate sovlers.
