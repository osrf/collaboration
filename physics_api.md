# New Physics API


## Entity.hh

### Set State (Existing)

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

### Get State (Existing)

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

## Link.hh





