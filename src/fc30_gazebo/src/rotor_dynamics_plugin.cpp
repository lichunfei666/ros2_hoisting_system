#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Force.hh>
#include <gz/sim/components/Torque.hh>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
  class RotorDynamicsPlugin : public gz::sim::System, 
                              public gz::sim::ISystemConfigure, 
                              public gz::sim::ISystemPreUpdate
  {
  public:
    RotorDynamicsPlugin() : gz::sim::System()
    {
    }

    void Configure(const gz::sim::Entity &_entity, 
                   const std::shared_ptr<const sdf::Element> &_sdf, 
                   gz::sim::EntityComponentManager &_ecm, 
                   gz::sim::EventManager &_eventMgr) override
    {
      // Store the model entity
      this->modelEntity = _entity;

      // Get rotor joint name from SDF
      if (_sdf->HasElement("jointName"))
      {
        this->jointName = _sdf->Get<std::string>("jointName");
      }
      else
      {
        gzerr << "No jointName specified in SDF." << std::endl;
        return;
      }

      // Get rotor link name from SDF
      if (_sdf->HasElement("linkName"))
      {
        this->linkName = _sdf->Get<std::string>("linkName");
      }
      else
      {
        gzerr << "No linkName specified in SDF." << std::endl;
        return;
      }

      // Get rotor parameters from SDF
      this->rotorRadius = _sdf->Get<double>("rotorRadius", 0.1);
      this->rotorMaxSpeed = _sdf->Get<double>("rotorMaxSpeed", 1000.0);
      this->rotorMaxThrust = _sdf->Get<double>("rotorMaxThrust", 7.0);
      this->rotorDragCoeff = _sdf->Get<double>("rotorDragCoeff", 0.01);
      this->rotorDirection = _sdf->Get<int>("rotorDirection", 1);

      // Find the joint and link entities
      auto model = gz::sim::Model(this->modelEntity);
      this->jointEntity = model.JointByName(_ecm, this->jointName);
      this->linkEntity = model.LinkByName(_ecm, this->linkName);

      if (this->jointEntity == gz::sim::kNullEntity)
      {
        gzerr << "Joint with name \"" << this->jointName << "\" not found." << std::endl;
        return;
      }

      if (this->linkEntity == gz::sim::kNullEntity)
      {
        gzerr << "Link with name \"" << this->linkName << "\" not found." << std::endl;
        return;
      }

      // Initialize ROS node handle
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "rotor_dynamics_plugin", ros::init_options::NoSigintHandler);
      }
      this->nh = std::make_shared<ros::NodeHandle>();

      // Create ROS subscriber for rotor velocity command
      std::string topicName = "/fc30/" + this->linkName + "/cmd_vel";
      this->sub = this->nh->subscribe(topicName, 10, &RotorDynamicsPlugin::VelocityCallback, this);

      // Initialize rotor velocity
      this->rotorVelocity = 0.0;

      gzmsg << "Rotor Dynamics Plugin loaded for joint: " << this->jointName << std::endl;
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info, 
                   gz::sim::EntityComponentManager &_ecm) override
    {
      // Calculate thrust based on rotor velocity (quadratic relationship)
      double thrust = this->rotorMaxThrust * std::pow(this->rotorVelocity / this->rotorMaxSpeed, 2);

      // Calculate torque based on thrust and rotor drag
      double torque = thrust * this->rotorDragCoeff * this->rotorDirection;

      // Apply thrust to the rotor link
      gz::sim::Link link(this->linkEntity);
      link.AddWorldForce(_ecm, gz::math::Vector3d(0, 0, thrust));

      // Apply torque to the rotor joint (opposite to rotation direction)
      gz::sim::Joint joint(this->jointEntity);
      joint.AddForce(_ecm, -torque);

      // Apply reaction torque to the base link
      gz::sim::Model model(this->modelEntity);
      auto baseLinkEntity = model.LinkByName(_ecm, "base_link");
      if (baseLinkEntity != gz::sim::kNullEntity)
      {
        gz::sim::Link baseLink(baseLinkEntity);
        baseLink.AddWorldTorque(_ecm, gz::math::Vector3d(0, 0, torque));
      }
    }

    void VelocityCallback(const std_msgs::Float64::ConstPtr& msg)
    {
      // Set rotor velocity (clamped to max speed)
      this->rotorVelocity = std::max(-this->rotorMaxSpeed, std::min(this->rotorMaxSpeed, msg->data));
    }

  private:
    // Model, joint, and link entities
    gz::sim::Entity modelEntity;
    gz::sim::Entity jointEntity;
    gz::sim::Entity linkEntity;

    // Rotor parameters
    std::string jointName;
    std::string linkName;
    double rotorRadius;
    double rotorMaxSpeed;
    double rotorMaxThrust;
    double rotorDragCoeff;
    int rotorDirection;

    // Rotor state
    double rotorVelocity;

    // ROS node handle and subscriber
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Subscriber sub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(RotorDynamicsPlugin)
}