/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <gazebo_ros/node.hpp>

#include <gazebo/msgs/msgs.hh>

// #include <gazebo_msgs/msg/contact_state.hpp>
// #include <gazebo_msgs/msg/contacts_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ignition/common/Profiler.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo/physics/ode/ODETypes.hh>

#include <gazebo/physics/JointState.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Subscriber.hh>

#include <gazebo/common/common.hh>
#include "ht_nav_gazebo_wheel_surface_plugin.hh"
#include "ht_nav_config.hh"

#include "gazebo/physics/Contact.hh"

#include <memory>
#include <string>
#include <vector>

#define DEBUG_PRINT 0

namespace gazebo
{
  namespace physics
  {
    typedef boost::weak_ptr<physics::Joint> JointWeakPtr;
    typedef boost::weak_ptr<physics::Link> LinkWeakPtr;
    typedef boost::weak_ptr<physics::Model> ModelWeakPtr;
    typedef boost::weak_ptr<physics::ODESurfaceParams> ODESurfaceParamsWeakPtr;
  }

  class HTNavGazeboWheelSurfacePluginPrivate
  {
    /// A pointer to the GazeboROS node.
    public: gazebo_ros::Node::SharedPtr ros_node_;
      /// Contact mesage publisher.
    public: std::array<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr, 4> publishers;
    // public: rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_{nullptr};
    /// Subscriber to elevator commands
    // public: rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr sub_;

    // Listen to Gazebo world_stats topic
    public:  transport::SubscriberPtr fl_subptr_;
    public:  transport::SubscriberPtr fr_subptr_;
    public:  transport::SubscriberPtr rl_subptr_;
    public:  transport::SubscriberPtr rr_subptr_;

    /// Indicates which link
    public: enum
    {
      FRONT_LEFT,  // Front left wheel
      FRONT_RIGHT, // Front right wheel
      REAR_LEFT,   // Rear left wheel
      REAR_RIGHT,  // Rear right wheel
      IMU_LINK,    // IMU LINK
    };

    public: class WheelNavParams
    {
      public: double position[3] = {0,0,0};
      public: double velocity[3] = {0,0,0};
      public: double vel_body[3] = {0,0,0};
      public: double euler[3]    = {0,0,0};
    };

    public: std::map<int, WheelNavParams> mapWheelNavParams;

    public: class LinkSurfaceParams
    {
      /// \brief Pointer to wheel spin joint.
      public: physics::JointWeakPtr joint;

      /// \brief Pointer to ODESurfaceParams object.
      public: physics::ODESurfaceParamsWeakPtr surface;

      /// \brief Wheel normal force estimate used to compute slip
      /// compliance for ODE, which takes units of 1/N.
      public: double wheelNormalForce = 0;

      /// \brief Wheel radius extracted from collision shape if not
      /// specified as xml parameter.
      public: double wheelRadius = 0;

      /// \brief Publish slip for each wheel.
      public: transport::PublisherPtr slipPub;
    };

    public: class LinkSurfaceConfigParams
    {
      /// Set the elastic modulus.
      public: double ElasticModulus = 0;

      /// Set the friction coefficient in the primary direction.
      public: double MuPrimary = 0;

      /// Set the friction coefficient in the secondary direction.
      public: double MuSecondary = 0;

      /// Set the torsional friction coefficient.
      public: double MuTorsion = 0;

      /// Set the torsional patch radius.
      public: double PatchRadius = 0;

      /// Set the Poisson's ratio.
      public: double PoissonsRatio = 0;

      /// Set the torsional surface radius.
      public: double SurfaceRadius = 0;

      /////Set whether to use the surface radius.
      public: double UsePatchRadius = 0;

      /// bounce restitution coefficient [0,1], 
      /// with 0 being inelastic, and 1 being perfectly elastic.
      public: double bounce = 0;

      /// minimum contact velocity for bounce to take effect, 
      /// otherwise the collision is treated as an inelastic collision
      public: double bounceThreshold = 0;

      /// Constraint Force Mixing parameter.
      public: double cfm = 0;

      /// Error Reduction Parameter.
      public: double erp = 0;

      /// spring damping constant equivalents of a contact 
      /// as a function of SurfaceParams::cfm and SurfaceParams::erp.
      public: double kd = 0;

      /// spring constant equivalents of a contact 
      /// as a function of SurfaceParams::cfm and SurfaceParams::erp
      public: double kp = 0;

      /// Maximum interpenetration error correction velocity. 
      public: double maxVel = 0;

      /// Minimum depth before ERP takes effect.
      public: double minDepth = 0;

      /// Artificial contact slip in the primary friction direction.
      public: double slip1 = 0;

      /// Artificial contact slip in the secondary friction dirction.
      public: double slip2 = 0;

      /// \brief Unitless wheel slip compliance in lateral direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLateral = 0;

      /// \brief Unitless wheel slip compliance in longitudinal direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLongitudinal = 0;
    };

    /// \brief Initial gravity direction in parent model frame.
    public: ignition::math::Vector3d initialGravityDirection;

    /// \brief Model pointer.
    public: physics::ModelWeakPtr model;
    public: physics::ModelPtr model_;

    /// \brief Protect data access during transport callbacks
    public: std::mutex mutex;

    /// \brief Gazebo communication node
    /// \todo: Transition to ignition-transport in gazebo8
    public: transport::NodePtr gzNode;

    /// \brief Link and surface pointers to update.
    public: std::map<physics::LinkWeakPtr,
                        LinkSurfaceParams> mapLinkSurfaceParams;

    public: LinkSurfaceConfigParams mapLinkSurfaceConfigParams;

    /// \brief Link names and their pointers
    public: std::map<std::string,
            physics::LinkWeakPtr> mapLinkNames;

    /// \brief Lateral slip compliance subscriber.
    /// \todo: Transition to ignition-transport in gazebo8.
    public: transport::SubscriberPtr lateralComplianceSub;

    /// \brief Longitudinal slip compliance subscriber.
    /// \todo: Transition to ignition-transport in gazebo8.
    public: transport::SubscriberPtr longitudinalComplianceSub;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;

    public: double lin_vel_[4] = {0,0,0,0};
    public: double lin_vel_avg[4] = {0,0,0,0};
    public: double lin_vel_min_[4] = {0,0,0,0};
    public: double lin_vel_min_two_[4] = {0,0,0,0};
    public: double lin_vel_min_three_[4] = {0,0,0,0};

    public: double ang_vel_[4] = {0,0,0,0};
    public: double ang_vel_avg[4] = {0,0,0,0};
    public: double ang_vel_min_[4] = {0,0,0,0};
    public: double ang_vel_min_two_[4] = {0,0,0,0};
    public: double ang_vel_min_three_[4] = {0,0,0,0};

    public: double steer_angle_[4] = {0,0,0,0};
    public: double F_z_[4] = {0,0,0,0};
    public: double sigma_x_[4] = {0,0,0,0};
    public: double alpha_x_[4] = {0,0,0,0};
    public: double F_x_pacejka_[4] = {0,0,0,0};
    public: double F_y_pacejka_[4] = {0,0,0,0};

    public: int contact_counter_[4] = {0,0,0,0};
    public: double contact_force_min_[4][3]     = {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    public: double contact_force_min_two_[4][3] = {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    public: double contact_force_avg[4][3]      = {0,0,0, 0,0,0, 0,0,0, 0,0,0};
     
    public: int counter_ = 0;
    public: int init_flag_ = 0;
    public: double sim_time_ = 0.0;

    public: double last_fl_upd_sim_time_ = 0.0;
    public: double last_fr_upd_sim_time_ = 0.0;
    public: double last_rl_upd_sim_time_ = 0.0;
    public: double last_rr_upd_sim_time_ = 0.0;

    public: double contact_force_update_freq_   = 100;
    public: double contact_force_update_period_ = 0;

    public: physics::LinkPtr IMULinkptr_;

    public: FILE *fptr;

    public: FILE *fl_fptr;
    public: FILE *fr_fptr;
    public: FILE *rl_fptr;
    public: FILE *rr_fptr;

    public: int fl_init_flag_ = 0;
    public: int fr_init_flag_ = 0;
    public: int rl_init_flag_ = 0;
    public: int rr_init_flag_ = 0;


  };
}

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(HTNavGazeboWheelSurfacePlugin)

/////////////////////////////////////////////////
HTNavGazeboWheelSurfacePlugin::HTNavGazeboWheelSurfacePlugin()
  : dataPtr(new HTNavGazeboWheelSurfacePluginPrivate)
{
}

/////////////////////////////////////////////////
HTNavGazeboWheelSurfacePlugin::~HTNavGazeboWheelSurfacePlugin()
{
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::Fini()
{
  this->dataPtr->updateConnection.reset();


  this->dataPtr->fl_subptr_.reset();
  this->dataPtr->fr_subptr_.reset();
  this->dataPtr->rl_subptr_.reset();
  this->dataPtr->rr_subptr_.reset();
  this->dataPtr->lateralComplianceSub.reset();
  this->dataPtr->longitudinalComplianceSub.reset();
  for (auto linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    linkSurface.second.slipPub.reset();
  }
  if (this->dataPtr->gzNode)
    this->dataPtr->gzNode->Fini();
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->ros_node_ = gazebo_ros::Node::Get(_sdf);

  GZ_ASSERT(_model, "HTNavGazeboWheelSurfacePlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "HTNavGazeboWheelSurfacePlugin sdf pointer is NULL");

  this->dataPtr->model_ = _model;
  this->dataPtr->model = _model;
  auto world = _model->GetWorld();

  gazebo::common::Time current_time =  this->dataPtr->model_->GetWorld()->SimTime();
  this->dataPtr->sim_time_ = current_time.sec*1e6 + current_time.nsec*1e-3; // us

  this->dataPtr->last_fl_upd_sim_time_ = this->dataPtr->sim_time_;
  this->dataPtr->last_fr_upd_sim_time_ = this->dataPtr->sim_time_;
  this->dataPtr->last_rl_upd_sim_time_ = this->dataPtr->sim_time_;
  this->dataPtr->last_rr_upd_sim_time_ = this->dataPtr->sim_time_;

  GZ_ASSERT(world, "world pointer is NULL");
  {
    ignition::math::Vector3d gravity = world->Gravity();
    ignition::math::Quaterniond initialModelRot =
        _model->WorldPose().Rot();
    this->dataPtr->initialGravityDirection =
        initialModelRot.RotateVectorReverse(gravity.Normalized());
  }

  if (!_sdf->HasElement("config_params"))
  {
    gzerr << "Config Paramters are not not specified, plugin is disabled!" << std::endl;
    RCLCPP_ERROR(this->dataPtr->ros_node_->get_logger(), "Config Paramters are not not specified, plugin is disabled!");
    return;
  }

  if (!_sdf->HasElement("wheel"))
  {
    gzerr << "No wheel tags specified, plugin is disabled" << std::endl;
    RCLCPP_ERROR(this->dataPtr->ros_node_->get_logger(), "No wheel tags specified, plugin is disabled!");
    return;
  }

  if (!_sdf->HasElement("imu_link"))
  {
    gzerr << "IMU Link is not spesifed, plugin is disabled!" << std::endl;
    RCLCPP_ERROR(this->dataPtr->ros_node_->get_logger(), "IMU Link is not spesifed, plugin is disabled!");
    return;
  }

  auto imuElem = _sdf->GetElement("imu_link");

  if (!imuElem->HasAttribute("link_name"))
  {
    gzerr << "imu element missing link_name attribute" << std::endl;
    RCLCPP_ERROR(this->dataPtr->ros_node_->get_logger(), "imu element missing link_name attribute, plugin is disabled!");
    return;
  }

  // Get IMU Link
  auto IMUlinkName = imuElem->Get<std::string>("link_name");
  this->dataPtr->IMULinkptr_ = _model->GetLink(IMUlinkName);

  // Read Config Parameters
  auto configElem = _sdf->GetElement("config_params");

  HTNavGazeboWheelSurfacePluginPrivate::LinkSurfaceConfigParams config_params;

  // Contact Force Update Freq;
  if (configElem->HasElement("ContactForceUpdateFreq"))
  {
    this->dataPtr->contact_force_update_freq_ = configElem->Get<double>("ContactForceUpdateFreq");
    this->dataPtr->contact_force_update_period_ = 1 / this->dataPtr->contact_force_update_freq_;
  }
  else{
    this->dataPtr->contact_force_update_freq_ = 100;  
    this->dataPtr->contact_force_update_period_ = 1 / this->dataPtr->contact_force_update_freq_;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <ContactForceUpdateFreq>, set to default: 100 ");
  }
  
  if (configElem->HasElement("ElasticModulus"))
  {
    config_params.ElasticModulus = configElem->Get<double>("ElasticModulus");
  }
  else{
    config_params.ElasticModulus = -1;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <ElasticModulus>, set to default: -1 ");
  }

  if (configElem->HasElement("MuPrimary"))
  {
    config_params.MuPrimary = configElem->Get<double>("MuPrimary");
  }
  else{
    config_params.MuPrimary = 1;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <MuPrimary>, set to default: 1 ");
  }

  if (configElem->HasElement("MuSecondary"))
  {
    config_params.MuSecondary = configElem->Get<double>("MuSecondary");
  }
  else{
    config_params.MuSecondary = 1;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <MuSecondary>, set to default: 1 ");
  }

  if (configElem->HasElement("MuTorsion"))
  {
    config_params.MuTorsion = configElem->Get<double>("MuTorsion");
  }
  else{
    config_params.MuTorsion = 1;
      RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <MuTorsion>, set to default: 1 ");
  }

  if (configElem->HasElement("PatchRadius"))
  {
    config_params.PatchRadius = configElem->Get<double>("PatchRadius");
  }
  else{
    config_params.PatchRadius = 0;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <PatchRadius>, set to default: 0.0 ");
  }

  if (configElem->HasElement("PoissonsRatio"))
  {
    config_params.PoissonsRatio = configElem->Get<double>("PoissonsRatio");
  }
  else{
    config_params.PoissonsRatio = 0.29999999999999999;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <PoissonsRatio>, set to default: 0.29999999999999999 ");
  }

  if (configElem->HasElement("SurfaceRadius"))
  {
    config_params.SurfaceRadius = configElem->Get<double>("SurfaceRadius");
  }
  else{
    config_params.SurfaceRadius = 0;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <SurfaceRadius>, set to default: 0.0 ");
  }

  if (configElem->HasElement("UsePatchRadius"))
  {
    config_params.UsePatchRadius = configElem->Get<double>("UsePatchRadius");
  }
  else{
    config_params.UsePatchRadius = 1;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <UsePatchRadius>, set to default: 1 (true) ");
  }

  if (configElem->HasElement("bounce"))
  {
    config_params.bounce = configElem->Get<double>("bounce");
  }
  else{
    config_params.bounce = 1e5;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <bounce>, set to default: 1e5 ");
  }

  if (configElem->HasElement("bounceThreshold"))
  {
    config_params.bounceThreshold = configElem->Get<double>("bounceThreshold");
  }
  else{
    config_params.bounceThreshold = 0;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <bounceThreshold>, set to default: 0.0 ");
  }

  if (configElem->HasElement("cfm"))
  {
    config_params.cfm = configElem->Get<double>("cfm");
  }
  else{
    config_params.cfm = 0;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <cfm>, set to default: 0.0 ");
  }

  if (configElem->HasElement("erp"))
  {
    config_params.erp = configElem->Get<double>("erp");
  }
  else{
    config_params.erp = 0.2;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <erp>, set to default: 0.2 ");
  }


  if (configElem->HasElement("kd"))
  {
    config_params.kd = configElem->Get<double>("kd");
  }
  else{
    config_params.kd = 1;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <kd>, set to default: 1 ");
  }

  if (configElem->HasElement("kp"))
  {
    config_params.kp = configElem->Get<double>("kp");
  }
  else{
    config_params.kp = 1e12;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <kp>, set to default: 1e12 ");
  }

  if (configElem->HasElement("maxVel"))
  {
    config_params.maxVel = configElem->Get<double>("maxVel");
  }
  else{
    config_params.maxVel = 0.01;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <maxVel>, set to default: 0.01 ");
  }

  if (configElem->HasElement("minDepth"))
  {
    config_params.minDepth = configElem->Get<double>("minDepth");
  }
  else{
    config_params.minDepth = 0;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <minDepth>, set to default: 0.0 ");
  }      


  if (configElem->HasElement("slip1"))
  {
    config_params.slip1 = configElem->Get<double>("slip1");
  }
  else{
    config_params.slip1 = 0;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <slip1>, set to default: 0.0 ");
  }     

  if (configElem->HasElement("slip2"))
  {
    config_params.slip2 = configElem->Get<double>("slip2");
  }
  else{
    config_params.slip2 = 0;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <slip2>, set to default: 0.0 ");
  }     

  if (configElem->HasElement("slipComplianceLateral"))
  {
    config_params.slipComplianceLateral = configElem->Get<double>("slipComplianceLateral");
  }
  else{
    config_params.slipComplianceLateral = 0.0003;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <slipComplianceLateral>, set to default: 0.0003 ");
  }     

  if (configElem->HasElement("slipComplianceLongitudinal"))
  {
    config_params.slipComplianceLongitudinal = configElem->Get<double>("slipComplianceLongitudinal");
  }
  else{
    config_params.slipComplianceLongitudinal = 0.0003;
    RCLCPP_WARN_STREAM(this->dataPtr->ros_node_->get_logger(), "missing <slipComplianceLongitudinal>, set to default: 0.0003 ");
  }     

  this->dataPtr->mapLinkSurfaceConfigParams = config_params;

  RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      "config_params read");

  // Read each wheel element
  for (auto wheelElem = _sdf->GetElement("wheel"); wheelElem;
      wheelElem = wheelElem->GetNextElement("wheel"))
  {
    if (!wheelElem->HasAttribute("link_name"))
    {
      gzerr << "wheel element missing link_name attribute" << std::endl;
      continue;
    }

    // Get link name
    auto linkName = wheelElem->Get<std::string>("link_name");

    HTNavGazeboWheelSurfacePluginPrivate::LinkSurfaceParams params;
    
    int LINK_IND = 0;

    if(linkName == "front_left_wheel"){
      LINK_IND = this->dataPtr->FRONT_LEFT;
    }
    else if(linkName == "front_right_wheel"){
      LINK_IND = this->dataPtr->FRONT_RIGHT;
    }
    else if(linkName == "rear_left_wheel") {
      LINK_IND = this->dataPtr->REAR_LEFT;
    }
    else if(linkName == "rear_right_wheel"){
      LINK_IND = this->dataPtr->REAR_RIGHT;
    }
    else{
      LINK_IND = 0;
    }

    if (wheelElem->HasElement("wheel_normal_force"))
    {
      params.wheelNormalForce = wheelElem->Get<double>("wheel_normal_force");
      this->dataPtr->F_z_[LINK_IND] = params.wheelNormalForce;
    }

    if (wheelElem->HasElement("wheel_radius"))
    {
      params.wheelRadius = wheelElem->Get<double>("wheel_radius");
    }

    auto link = _model->GetLink(linkName);
    if (link == nullptr)
    {
      gzerr << "Could not find link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    auto collisions = link->GetCollisions();
    if (collisions.empty() || collisions.size() != 1)
    {
      gzerr << "There should be 1 collision in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << ", but " << collisions.size() << " were found"
            << std::endl;
      continue;
    }
    auto collision = collisions.front();
    if (collision == nullptr)
    {
      gzerr << "Could not find collision in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    auto surface = collision->GetSurface();
    auto odeSurface =
      boost::dynamic_pointer_cast<physics::ODESurfaceParams>(surface);
    if (odeSurface == nullptr)
    {
      gzerr << "Could not find ODE Surface "
            << "in collision named [" << collision->GetName()
            << "] in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }
    params.surface = odeSurface;

    auto joints = link->GetParentJoints();
    if (joints.empty() || joints.size() != 1)
    {
      gzerr << "There should be 1 parent joint for link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << ", but " << joints.size() << " were found"
            << std::endl;
      continue;
    }
    auto joint = joints.front();
    if (joint == nullptr)
    {
      gzerr << "Could not find parent joint for link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }
    params.joint = joint;

    if (params.wheelRadius <= 0)
    {
      // get collision shape and extract radius if it is a cylinder or sphere
      auto shape = collision->GetShape();
      if (shape->HasType(physics::Base::CYLINDER_SHAPE))
      {
        auto cyl = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);
        if (cyl != nullptr)
        {
          params.wheelRadius = cyl->GetRadius();
        }
      }
      else if (shape->HasType(physics::Base::SPHERE_SHAPE))
      {
        auto sphere = boost::dynamic_pointer_cast<physics::SphereShape>(shape);
        if (sphere != nullptr)
        {
          params.wheelRadius = sphere->GetRadius();
        }
      }
      else
      {
        gzerr << "A positive wheel radius was not specified in the"
              << " [wheel_radius] parameter, and the the wheel radius"
              << " could not be identified automatically because a"
              << " sphere or cylinder collision shape could not be found."
              << " Skipping link [" << linkName << "]."
              << std::endl;
        continue;
      }

      // if that still didn't work, skip this link
      if (params.wheelRadius <= 0)
      {
        gzerr << "Found wheel radius [" << params.wheelRadius
              << "], which is not positive"
              << " in link named [" << linkName
              << "] in model [" << _model->GetScopedName() << "]"
              << std::endl;
        continue;
      }
    }

    if (params.wheelNormalForce <= 0)
    {
      gzerr << "Found wheel normal force [" << params.wheelNormalForce
            << "], which is not positive"
            << " in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    this->dataPtr->mapLinkSurfaceParams[link] = params;
    this->dataPtr->mapLinkNames[link->GetName()] = link;

    auto link_name = link->GetName();

    RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(), "Going to control link %s", link_name.c_str());
  }

  if (this->dataPtr->mapLinkSurfaceParams.empty())
  {
    gzerr << "No ODE links and surfaces found, plugin is disabled" << std::endl;
    return;
  }

  // Subscribe to slip compliance updates
  this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
  this->dataPtr->gzNode->Init(world->Name());

  // add publishers
  for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    auto link = linkSurface.first.lock();
    GZ_ASSERT(link, "link should still exist inside Load");
    auto &params = linkSurface.second;
    params.slipPub = this->dataPtr->gzNode->Advertise<msgs::Vector3d>(
        "~/" + _model->GetName() + "/wheel_slip/" + link->GetName());
  }

  this->dataPtr->lateralComplianceSub = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/wheel_slip/lateral_compliance",
      &HTNavGazeboWheelSurfacePlugin::OnLateralCompliance, this);

  this->dataPtr->longitudinalComplianceSub = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/wheel_slip/longitudinal_compliance",
      &HTNavGazeboWheelSurfacePlugin::OnLongitudinalCompliance, this);

  const gazebo_ros::QoS & qos = this->dataPtr->ros_node_->get_qos();

  // this->dataPtr->sub_ = this->dataPtr->ros_node_->create_subscription<gazebo_msgs::msg::ContactsState>(
  //   "kobra_mk5/rear_right_contact_forces", 10,
  //   std::bind(&HTNavGazeboWheelSurfacePlugin::Update, this));

// /gazebo/default/kobra_mk5/front_left_wheel/fl_bumper_plugin/contacts
// /gazebo/default/kobra_mk5/front_right_wheel/fr_bumper_plugin/contacts
// /gazebo/default/kobra_mk5/rear_left_wheel/rl_bumper_plugin/contacts
// /gazebo/default/kobra_mk5/rear_right_wheel/rr_bumper_plugin/contacts

  this->dataPtr->publishers[this->dataPtr->FRONT_LEFT] = this->dataPtr->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      _model->GetName() + "/front_left_wheel_contact_states", qos.get_publisher_qos(_model->GetName() + "/front_left_wheel_contact_states", rclcpp::QoS(1000)));
  this->dataPtr->publishers[this->dataPtr->FRONT_RIGHT] = this->dataPtr->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      _model->GetName() + "/front_right_wheel_contact_states", qos.get_publisher_qos(_model->GetName() + "/front_right_wheel_contact_states", rclcpp::QoS(1000)));
  this->dataPtr->publishers[this->dataPtr->REAR_LEFT] = this->dataPtr->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      _model->GetName() + "/rear_left_wheel_contact_states", qos.get_publisher_qos(_model->GetName() + "/rear_left_wheel_contact_states", rclcpp::QoS(1000)));
  this->dataPtr->publishers[this->dataPtr->REAR_RIGHT] = this->dataPtr->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      _model->GetName() + "/rear_right_wheel_contact_states", qos.get_publisher_qos(_model->GetName() + "/rear_right_wheel_contact_states", rclcpp::QoS(1000)));
  
  this->dataPtr->fl_subptr_ = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/front_left_wheel/fl_bumper_plugin/contacts",
      &HTNavGazeboWheelSurfacePlugin::OnFLContacts, this);
      
  this->dataPtr->fr_subptr_ = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/front_right_wheel/fr_bumper_plugin/contacts",
      &HTNavGazeboWheelSurfacePlugin::OnFRContacts, this);
  
  this->dataPtr->rl_subptr_ = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/rear_left_wheel/rl_bumper_plugin/contacts",
      &HTNavGazeboWheelSurfacePlugin::OnRLContacts, this);
  
  this->dataPtr->rr_subptr_ = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/rear_right_wheel/rr_bumper_plugin/contacts",
      &HTNavGazeboWheelSurfacePlugin::OnRRContacts, this);

  // Connect to the update event
    RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      "Pre Update");
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HTNavGazeboWheelSurfacePlugin::Update, this));
}

/////////////////////////////////////////////////
physics::ModelPtr HTNavGazeboWheelSurfacePlugin::GetParentModel() const
{
  return this->dataPtr->model.lock();
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::GetSlips(
        std::map<std::string, ignition::math::Vector3d> &_out) const
{
  auto model = this->GetParentModel();
  if (!model)
  {
    gzerr << "Parent model does not exist" << std::endl;
    return;
  }
  auto modelWorldPose = model->WorldPose();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (const auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    auto link = linkSurface.first.lock();
    if (!link)
      continue;
    auto params = linkSurface.second;

    // Compute wheel velocity in parent model frame
    auto wheelWorldLinearVel = link->WorldLinearVel();
    auto wheelModelLinearVel =
        modelWorldPose.Rot().RotateVectorReverse(wheelWorldLinearVel);
    // Compute wheel spin axis in parent model frame
    auto joint = params.joint.lock();
    if (!joint)
      continue;
    auto wheelWorldAxis = joint->GlobalAxis(0).Normalized();
    auto wheelModelAxis =
        modelWorldPose.Rot().RotateVectorReverse(wheelWorldAxis);
    // Estimate longitudinal direction as cross product of initial gravity
    // direction with wheel spin axis.
    auto longitudinalModelAxis =
        this->dataPtr->initialGravityDirection.Cross(wheelModelAxis);

    double spinSpeed = params.wheelRadius * joint->GetVelocity(0);
    double lateralSpeed = wheelModelAxis.Dot(wheelModelLinearVel);
    double longitudinalSpeed = longitudinalModelAxis.Dot(wheelModelLinearVel);

    ignition::math::Vector3d slip;
    slip.X(longitudinalSpeed - spinSpeed);
    slip.Y(lateralSpeed);
    slip.Z(spinSpeed);

    auto name = link->GetName();
    _out[name] = slip;
  }
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::OnLateralCompliance(ConstGzStringPtr &_msg)
{
  RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      "OnLateralCompliance: %.3e", _msg->data());
  
  try
  {
    this->SetSlipComplianceLateral(std::stod(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Invalid slip compliance data[" << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::OnLongitudinalCompliance(ConstGzStringPtr &_msg)
{
  try
  {
    this->SetSlipComplianceLongitudinal(std::stod(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Invalid slip compliance data[" << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::SetSlipComplianceLateral(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  // {
  //   linkSurface.second.slipComplianceLateral = _compliance;
  // }
  this->dataPtr->mapLinkSurfaceConfigParams.slipComplianceLateral = _compliance;
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::SetSlipComplianceLateral(std::string _wheel_name, const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->mapLinkNames.count(_wheel_name) > 0)
  {
    auto link = this->dataPtr->mapLinkNames[_wheel_name];
    // this->dataPtr->mapLinkSurfaceParams[link].slipComplianceLateral = _compliance;
    this->dataPtr->mapLinkSurfaceConfigParams.slipComplianceLateral = _compliance;
  }
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::SetSlipComplianceLongitudinal(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  // {
  //   linkSurface.second.slipComplianceLongitudinal = _compliance;
  // }
  this->dataPtr->mapLinkSurfaceConfigParams.slipComplianceLongitudinal = _compliance;
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::SetSlipComplianceLongitudinal(std::string _wheel_name, const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->mapLinkNames.count(_wheel_name) > 0)
  {
    auto link = this->dataPtr->mapLinkNames[_wheel_name];
    // this->dataPtr->mapLinkSurfaceParams[link].slipComplianceLongitudinal = _compliance;
    this->dataPtr->mapLinkSurfaceConfigParams.slipComplianceLongitudinal = _compliance;
  }
}

/////////////////////////////////////////////////
std::map<std::string, ignition::math::Vector2d> HTNavGazeboWheelSurfacePlugin::GetFrictionCoefficients()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  std::map<std::string, ignition::math::Vector2d> frictionCoeffs;
  for (const auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    auto link = linkSurface.first.lock();
    auto surface = linkSurface.second.surface.lock();
    if (!link || !surface)
      continue;

    ignition::math::Vector2d friction;
    friction.X(surface->FrictionPyramid()->MuPrimary());
    friction.Y(surface->FrictionPyramid()->MuSecondary());

    frictionCoeffs[link->GetName()] = friction;
  }

  return frictionCoeffs;
}

/////////////////////////////////////////////////
bool HTNavGazeboWheelSurfacePlugin::SetMuPrimary(const std::string &_wheel_name, double _mu)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->mapLinkNames.count(_wheel_name) == 0)
    return false;

  auto link = this->dataPtr->mapLinkNames[_wheel_name];
  auto surface = this->dataPtr->mapLinkSurfaceParams[link].surface.lock();
  if (surface == nullptr)
    return false;

  surface->FrictionPyramid()->SetMuPrimary(_mu);
  return true;
}

/////////////////////////////////////////////////
bool HTNavGazeboWheelSurfacePlugin::SetMuSecondary(const std::string &_wheel_name, double _mu)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->mapLinkNames.count(_wheel_name) == 0)
    return false;

  auto link = this->dataPtr->mapLinkNames[_wheel_name];
  auto surface = this->dataPtr->mapLinkSurfaceParams[link].surface.lock();
  if (surface == nullptr)
    return false;

  surface->FrictionPyramid()->SetMuSecondary(_mu);
  return true;
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSurfacePlugin::OnFLContacts(ConstContactsPtr &_msg)
{
  gazebo::common::Time current_time =  this->dataPtr->model_->GetWorld()->SimTime();
  double local_sim_time = current_time.sec*1e6 + current_time.nsec*1e-3; // us

  // RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
  //   "OnRLContacts Time");  

  if (this->dataPtr->fl_init_flag_ == 0){
    this->dataPtr->fl_fptr = fopen(base_path"fl_contact_forces_gazebo.txt", "w");
    if (this->dataPtr->fl_fptr == NULL){
      RCLCPP_ERROR(this->dataPtr->ros_node_->get_logger(), "Could not open FL file !");
      return;
    }
    this->dataPtr->fl_init_flag_ = 1;
  }

  ContactStateSolver(_msg, this->dataPtr->FRONT_LEFT); 

  if ( (local_sim_time - this->dataPtr->last_fl_upd_sim_time_ )*1e-6 >  this->dataPtr->contact_force_update_period_ ){
    fprintf(this->dataPtr->fl_fptr,"%lf\t", local_sim_time);  // us

    for (int iz = 0; iz < 3; ++iz)
    {
      fprintf(this->dataPtr->fl_fptr,"%lf\t", this->dataPtr->contact_force_avg[this->dataPtr->FRONT_LEFT][iz] );                                                 // unitless 
    }

    fprintf(this->dataPtr->fl_fptr,"\n");

    current_time =  this->dataPtr->model_->GetWorld()->SimTime();
    this->dataPtr->last_fl_upd_sim_time_ = current_time.sec*1e6 + current_time.nsec*1e-3;

    // Populate message
    sensor_msgs::msg::JointState contact_msg;
    contact_msg.header.stamp.sec = current_time.sec;
    contact_msg.header.stamp.nanosec = current_time.nsec;
    ContactMessageConstr(&contact_msg, this->dataPtr->FRONT_LEFT);
    this->dataPtr->publishers[this->dataPtr->FRONT_LEFT]->publish(contact_msg);

    return;
  }
  else{
    return;
  }
}

void HTNavGazeboWheelSurfacePlugin::OnFRContacts(ConstContactsPtr &_msg)
{
  gazebo::common::Time current_time =  this->dataPtr->model_->GetWorld()->SimTime();
  double local_sim_time = current_time.sec*1e6 + current_time.nsec*1e-3; // us

  // RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
  //   "OnRRContacts Time");  

  if (this->dataPtr->fr_init_flag_ == 0){
    this->dataPtr->fr_fptr = fopen(base_path"fr_contact_forces_gazebo.txt", "w");
    if (this->dataPtr->fr_fptr == NULL){
      RCLCPP_ERROR(this->dataPtr->ros_node_->get_logger(), "Could not open FR file !");
      return;
    }
    this->dataPtr->fr_init_flag_ = 1;
  }

  ContactStateSolver(_msg, this->dataPtr->FRONT_RIGHT); 

  if ( (local_sim_time - this->dataPtr->last_fr_upd_sim_time_ )*1e-6 >  this->dataPtr->contact_force_update_period_ ){
    fprintf(this->dataPtr->fr_fptr,"%lf\t", local_sim_time);  // us

    for (int iz = 0; iz < 3; ++iz)
    {
      fprintf(this->dataPtr->fr_fptr,"%lf\t", this->dataPtr->contact_force_avg[this->dataPtr->FRONT_RIGHT][iz] );                                                 // unitless 
    }

    fprintf(this->dataPtr->fr_fptr,"\n");

    current_time =  this->dataPtr->model_->GetWorld()->SimTime();
    this->dataPtr->last_fr_upd_sim_time_ = current_time.sec*1e6 + current_time.nsec*1e-3;

    // Populate message
    sensor_msgs::msg::JointState contact_msg;
    contact_msg.header.stamp.sec = current_time.sec;
    contact_msg.header.stamp.nanosec = current_time.nsec;
    ContactMessageConstr(&contact_msg, this->dataPtr->FRONT_RIGHT);
    this->dataPtr->publishers[this->dataPtr->FRONT_RIGHT]->publish(contact_msg);

    return;
  }
  else{
    return;
  }
}

void HTNavGazeboWheelSurfacePlugin::OnRLContacts(ConstContactsPtr &_msg)
{
  gazebo::common::Time current_time =  this->dataPtr->model_->GetWorld()->SimTime();
  double local_sim_time = current_time.sec*1e6 + current_time.nsec*1e-3; // us

  // RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
  //   "OnRLContacts Time");  

  if (this->dataPtr->rl_init_flag_ == 0){
    this->dataPtr->rl_fptr = fopen(base_path"rl_contact_forces_gazebo.txt", "w");
    if (this->dataPtr->rl_fptr == NULL){
      RCLCPP_ERROR(this->dataPtr->ros_node_->get_logger(), "Could not open RL file !");
      return;
    }
    this->dataPtr->rl_init_flag_ = 1;
  }

  ContactStateSolver(_msg, this->dataPtr->REAR_LEFT); 

  if ( (local_sim_time - this->dataPtr->last_rl_upd_sim_time_ )*1e-6 >  this->dataPtr->contact_force_update_period_ ){
    fprintf(this->dataPtr->rl_fptr,"%lf\t", local_sim_time);  // us

    for (int iz = 0; iz < 3; ++iz)
    {
      fprintf(this->dataPtr->rl_fptr,"%lf\t", this->dataPtr->contact_force_avg[this->dataPtr->REAR_LEFT][iz] );                                                 // unitless 
    }

    fprintf(this->dataPtr->rl_fptr,"\n");

    current_time =  this->dataPtr->model_->GetWorld()->SimTime();
    this->dataPtr->last_rl_upd_sim_time_ = current_time.sec*1e6 + current_time.nsec*1e-3;

    // Populate message
    sensor_msgs::msg::JointState contact_msg;
    contact_msg.header.stamp.sec = current_time.sec;
    contact_msg.header.stamp.nanosec = current_time.nsec;
    ContactMessageConstr(&contact_msg, this->dataPtr->REAR_LEFT);
    this->dataPtr->publishers[this->dataPtr->REAR_LEFT]->publish(contact_msg);

    return;
  }
  else{
    return;
  }
}


void HTNavGazeboWheelSurfacePlugin::OnRRContacts(ConstContactsPtr &_msg)
{
  gazebo::common::Time current_time =  this->dataPtr->model_->GetWorld()->SimTime();
  double local_sim_time = current_time.sec*1e6 + current_time.nsec*1e-3; // us

  // RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
  //   "OnRRContacts Time");  

  if (this->dataPtr->rr_init_flag_ == 0){
    this->dataPtr->rr_fptr = fopen(base_path"rr_contact_forces_gazebo.txt", "w");
    if (this->dataPtr->rr_fptr == NULL){
      RCLCPP_ERROR(this->dataPtr->ros_node_->get_logger(), "Could not open RR file !");
      return;
    }
    this->dataPtr->rr_init_flag_ = 1;
  }

  ContactStateSolver(_msg, this->dataPtr->REAR_RIGHT); 

  if ( (local_sim_time - this->dataPtr->last_rr_upd_sim_time_ )*1e-6 >  this->dataPtr->contact_force_update_period_ ){
    fprintf(this->dataPtr->rr_fptr,"%lf\t", local_sim_time);  // us

    for (int iz = 0; iz < 3; ++iz)
    {
      fprintf(this->dataPtr->rr_fptr,"%lf\t", this->dataPtr->contact_force_avg[this->dataPtr->REAR_RIGHT][iz] );                                                 // unitless 
    }

    fprintf(this->dataPtr->rr_fptr,"\n");

    current_time =  this->dataPtr->model_->GetWorld()->SimTime();
    this->dataPtr->last_rr_upd_sim_time_ = current_time.sec*1e6 + current_time.nsec*1e-3;

    // Populate message
    sensor_msgs::msg::JointState contact_msg;
    contact_msg.header.stamp.sec = current_time.sec;
    contact_msg.header.stamp.nanosec = current_time.nsec;
    ContactMessageConstr(&contact_msg, this->dataPtr->REAR_RIGHT);
    this->dataPtr->publishers[this->dataPtr->REAR_RIGHT]->publish(contact_msg);

    return;
  }
  else{
    return;
  }
}

void HTNavGazeboWheelSurfacePlugin::Update()
{
  // RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
  //     "Update Time");

  // this->dataPtr->model = _model;
  // auto world = _model->GetWorld();
  gazebo::common::Time current_time =  this->dataPtr->model_->GetWorld()->SimTime();

  double time_sec = current_time.sec;
  double time_nces = current_time.nsec;
  this->dataPtr->sim_time_ = time_sec*1e6 + time_nces*1e-3; // us

#if DEBUG_PRINT
  // RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
  //   "Sim Time: %lf", time_sec );
  if (this->dataPtr->init_flag_ == 0){
    this->dataPtr->fptr = fopen(base_path"rr_wheel_slips.txt", "w");
    if (this->dataPtr->fptr == NULL){
      RCLCPP_ERROR(this->dataPtr->ros_node_->get_logger(), "Could not open file !");
      return;
    }
    this->dataPtr->init_flag_ = 1;
  }
#endif
  IGN_PROFILE("HTNavGazeboWheelSurfacePlugin::OnUpdate");
  IGN_PROFILE_BEGIN("Update");
  // Get slip data so it can be published later
  std::map<std::string, ignition::math::Vector3d> slips;
  this->GetSlips(slips);

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
#if DEBUG_PRINT
  fprintf(this->dataPtr->fptr,"%lf\t", this->dataPtr->sim_time_);  // us
#endif
  const auto config_params = this->dataPtr->mapLinkSurfaceConfigParams;
  for (const auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    const auto &params = linkSurface.second;

    // get user-defined normal force constant
    double force = params.wheelNormalForce;

    // get link angular velocity parallel to joint axis
    ignition::math::Vector3d wheelAngularVelocity;
    auto link = linkSurface.first.lock();
    auto link_name = link->GetName();

    if (link)
      wheelAngularVelocity = link->WorldAngularVel();

    ignition::math::Vector3d jointAxis;
    auto joint = params.joint.lock();
    
    // Since the front wheel joints are combined joints, use the secondary axis
    if(link_name == "front_left_wheel"){
      if (joint){
        jointAxis = joint->GlobalAxis(1);
      }
    }
    else if(link_name == "front_right_wheel"){
      if (joint){
        jointAxis = joint->GlobalAxis(1);
      }
    }
    else if(link_name == "rear_left_wheel") {
      if (joint){
        jointAxis = joint->GlobalAxis(0);
      }
    }
    else if(link_name == "rear_right_wheel"){
      if (joint){
        jointAxis = joint->GlobalAxis(0);
      }
    }
    else{
      if (joint){
        jointAxis = joint->GlobalAxis(0);
      }
    }

    double spinAngularVelocity = wheelAngularVelocity.Dot(jointAxis);

    auto surface = params.surface.lock();
    if (surface)
    {
      // As discussed in ht_nav_gazebo_wheel_surface_plugin.hh, the ODE slip1 and slip2
      // parameters have units of inverse viscous damping:
      // [linear velocity / force] or [m / s / N].
      // Since the slip compliance parameters supplied to the plugin
      // are unitless, they must be scaled by a linear speed and force
      // magnitude before being passed to ODE.
      // The force is taken from a user-defined constant that should roughly
      // match the steady-state normal force at the wheel.
      // The linear speed is computed dynamically at each time step as
      // radius * spin angular velocity.
      // This choice of linear speed corresponds to the denominator of
      // the slip ratio during acceleration (see equation (1) in
      // Yoshida, Hamano 2002 DOI 10.1109/ROBOT.2002.1013712
      // "Motion dynamics of a rover with slip-based traction model").
      // The acceleration form is more well-behaved numerically at low-speed
      // and when the vehicle is at rest than the braking form,
      // so it is used for both slip directions.
      double speed = params.wheelRadius * std::abs(spinAngularVelocity);
      double pos[3], vel[3], vel_body[3], euler[3];
      double imu_pos[3], imu_vel[3], imu_vel_body[3], imu_euler[3];
      double steer_angle = 0.0;

      LinkStateSolver(pos, vel, vel_body, euler, link);
      LinkStateSolver(imu_pos, imu_vel, imu_vel_body, imu_euler, this->dataPtr->IMULinkptr_);
      SteeringAngleCalc(&steer_angle, euler, imu_euler);
      
    // public: enum
    // {
    //   FRONT_LEFT,  // Front left wheel
    //   FRONT_RIGHT, // Front right wheel
    //   REAR_LEFT,   // Rear left wheel
    //   REAR_RIGHT,  // Rear right wheel
    // };

    int LINK_IND = 0;

    if(link_name == "front_left_wheel"){
      LINK_IND = this->dataPtr->FRONT_LEFT;
    }
    else if(link_name == "front_right_wheel"){
      LINK_IND = this->dataPtr->FRONT_RIGHT;
    }
    else if(link_name == "rear_left_wheel") {
      LINK_IND = this->dataPtr->REAR_LEFT;
    }
    else if(link_name == "rear_right_wheel"){
      LINK_IND = this->dataPtr->REAR_RIGHT;
    }
    else{
      LINK_IND = 0;
    }

    for (int ix = 0; ix < 3; ix++)
		{
      this->dataPtr->mapWheelNavParams[LINK_IND].position[ix] = pos[ix];
      this->dataPtr->mapWheelNavParams[LINK_IND].velocity[ix] = vel[ix];
      this->dataPtr->mapWheelNavParams[LINK_IND].vel_body[ix] = vel_body[ix];
      this->dataPtr->mapWheelNavParams[LINK_IND].euler[ix]    = euler[ix];
		}  

    this->dataPtr->lin_vel_[LINK_IND] = vel_body[0];
    this->dataPtr->ang_vel_[LINK_IND] = std::abs(spinAngularVelocity);

    if( this->dataPtr->counter_ == 0){
      this->dataPtr->lin_vel_min_[LINK_IND]     = vel_body[0];
      this->dataPtr->lin_vel_min_two_[LINK_IND] = vel_body[0];
      this->dataPtr->lin_vel_min_three_[LINK_IND] = vel_body[0];

      this->dataPtr->ang_vel_min_[LINK_IND]     = std::abs(spinAngularVelocity);
      this->dataPtr->ang_vel_min_two_[LINK_IND] = std::abs(spinAngularVelocity);
      this->dataPtr->ang_vel_min_three_[LINK_IND] = std::abs(spinAngularVelocity);
    }

    this->dataPtr->lin_vel_avg[LINK_IND] = (this->dataPtr->lin_vel_[LINK_IND] + this->dataPtr->lin_vel_min_[LINK_IND] + this->dataPtr->lin_vel_min_two_[LINK_IND] + this->dataPtr->lin_vel_min_three_[LINK_IND])/4;
    // ang_vel_avg[LINK_IND] = (this->dataPtr->ang_vel_[LINK_IND] + this->dataPtr->ang_vel_min_[LINK_IND] + this->dataPtr->ang_vel_min_two_[LINK_IND] + this->dataPtr->ang_vel_min_three_[LINK_IND])/4;
    double temp_ang_vel_avg = (this->dataPtr->ang_vel_[LINK_IND] + this->dataPtr->ang_vel_min_[LINK_IND] + this->dataPtr->ang_vel_min_two_[LINK_IND])/3; 
    if(this->dataPtr->counter_ % 3 == 0){
      this->dataPtr->ang_vel_avg[LINK_IND] = temp_ang_vel_avg;
    }

    this->dataPtr->lin_vel_min_three_[LINK_IND] = this->dataPtr->lin_vel_min_two_[LINK_IND];
    this->dataPtr->lin_vel_min_two_[LINK_IND] = this->dataPtr->lin_vel_min_[LINK_IND];
    this->dataPtr->lin_vel_min_[LINK_IND]     = this->dataPtr->lin_vel_[LINK_IND];

    this->dataPtr->ang_vel_min_three_[LINK_IND] = this->dataPtr->ang_vel_min_two_[LINK_IND];
    this->dataPtr->ang_vel_min_two_[LINK_IND] = this->dataPtr->ang_vel_min_[LINK_IND];
    this->dataPtr->ang_vel_min_[LINK_IND]     = this->dataPtr->ang_vel_[LINK_IND];


    // if(link_name == front_left_wheel){

    // }
    // else if(link_name == front_right_wheel){

    // }
    // else if(link_name == rear_left_wheel) {

    // }
    // else if(link_name == rear_right_wheel){
      
    // }
    // else{

    // }

      // The true slip is calculated as follows: Consider changing
      // // double R_eff = params.wheelRadius
      // // double w_tk = spinAngularVelocity;
      // // double v_tk_x = vel_body[0];
      double sigma_x = 0.0;
      double alpha_x = 0.0;
      // sigma_x = speed / std::abs(vel_body[0]) - 1.0;

      /* link->collision->surface->bounce */
      // surface->bounce = config_params.bounce; 
      // surface->bounceThreshold = config_params.bounceThreshold;

      /* link->collision->surface->friction->ode */
      surface->FrictionPyramid()->SetMuPrimary(config_params.MuPrimary);
      surface->FrictionPyramid()->SetMuSecondary(config_params.MuSecondary);
      // surface->slip1 = config_params.slip1;
      // surface->slip2 = config_params.slip2;

      /* Slip Calculation */
      // surface->slip1 = speed / force * params.slipComplianceLateral;
      // surface->slip2 = speed / force * params.slipComplianceLongitudinal;
      double wheel_speed = 0.0;
      sigma_x = std::abs(this->dataPtr->ang_vel_avg[LINK_IND]) * params.wheelRadius / std::abs(this->dataPtr->lin_vel_avg[LINK_IND]) - 1.0;
      alpha_x = euler[2] - std::atan2(vel[1] , vel[0]);
      if(alpha_x > 6.0){
            alpha_x = alpha_x - 2*M_PI;
      }
      else if(alpha_x < -6.0){
        alpha_x = alpha_x + 2*M_PI;
      }

      if (sigma_x > 1.0){
        wheel_speed = std::abs(this->dataPtr->lin_vel_avg[LINK_IND]);
        sigma_x = 1.0;
      }
      else{
        wheel_speed = std::abs(this->dataPtr->ang_vel_avg[LINK_IND]) * params.wheelRadius;
      }

      this->dataPtr->alpha_x_[LINK_IND] = alpha_x;
      this->dataPtr->sigma_x_[LINK_IND] = sigma_x;

      if (this->dataPtr->counter_ > 75){
        this->dataPtr->F_z_[LINK_IND] = std::abs(this->dataPtr->contact_force_avg[LINK_IND][2]);
      }

      double F_x_pacejka, F_y_pacejka;
      CalcPacejkaModel(&F_x_pacejka, &F_y_pacejka, LINK_IND);

      this->dataPtr->F_x_pacejka_[LINK_IND] = F_x_pacejka;
      this->dataPtr->F_y_pacejka_[LINK_IND] = F_y_pacejka;
      
      
      // RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      // "F_z: %lf %d", F_z , this->dataPtr->counter_ );

      double ster_factor = 0.0;
      this->dataPtr->steer_angle_[LINK_IND] = steer_angle;
      ster_factor = ( (this->dataPtr->steer_angle_[ this->dataPtr->FRONT_LEFT] + this->dataPtr->steer_angle_[ this->dataPtr->FRONT_RIGHT]) / 2) / this->dataPtr->F_z_[LINK_IND] * 9 ;
      // double wheel_slip = wheel_speed / this->dataPtr->F_z_[LINK_IND] / 8; // Defence Jury Edition
      double wheel_slip = wheel_speed / this->dataPtr->F_z_[LINK_IND] / 8 * 1.2;
      double wheel_slip_ang = steer_angle / this->dataPtr->F_z_[LINK_IND] * 9;

#if DEBUG_PRINT
      // fprintf(this->dataPtr->fptr,"%lf\t", std::abs(this->dataPtr->ang_vel_avg[LINK_IND]) * params.wheelRadius );    // m/s
      // fprintf(this->dataPtr->fptr,"%lf\t", std::abs(this->dataPtr->lin_vel_avg[LINK_IND]) );                         // m/s

      fprintf(this->dataPtr->fptr,"%d\t", LINK_IND );                                                 // unitless 
      fprintf(this->dataPtr->fptr,"%lf\t", sigma_x );                                                 // unitless 
      fprintf(this->dataPtr->fptr,"%lf\t", alpha_x );                                                 // unitless 
      
      // fprintf(this->dataPtr->fptr,"%lf\t", wheel_slip);                                                 // unitless 
      // fprintf(this->dataPtr->fptr,"%lf\t", wheel_slip_ang);                                                 // unitless 
      fprintf(this->dataPtr->fptr,"%lf\t", this->dataPtr->contact_force_avg[LINK_IND][0]);                                                 // unitless 
      fprintf(this->dataPtr->fptr,"%lf\t", F_x_pacejka);                                                 // unitless 
      fprintf(this->dataPtr->fptr,"%lf\t", this->dataPtr->contact_force_avg[LINK_IND][1]);                                                 // unitless 
      fprintf(this->dataPtr->fptr,"%lf\t", F_y_pacejka);                                                 // unitless                                              // unitless 
#endif
      // surface->slip1 = wheel_slip;
      
      // double slip_12 = (ster_factor / 256 + wheel_slip_ang / 2 + 2 * wheel_slip)/2; 
      // double slip_12 = (ster_factor / 16 + wheel_slip_ang / 16 + 2 * wheel_slip)/2; 
      // double slip_12 = (ster_factor / 32 + wheel_slip_ang / 32 + 2 * wheel_slip)/2;
      // double slip_12 = (ster_factor / 64 + wheel_slip_ang / 64 + 2 * wheel_slip)/2;

      // First Commited Version
      // double slip_12 = (wheel_slip_ang + 2 * wheel_slip)/2;
      // Second Commited Version
      // double slip_12 = (ster_factor / 256 + wheel_slip_ang / 8 + 2 * wheel_slip)/2;

      double slip_12 = 0.0;
      if(LINK_IND == this->dataPtr->FRONT_LEFT || LINK_IND == this->dataPtr->FRONT_RIGHT){
        // Third Commited Version
        slip_12 = ( 3 * wheel_slip_ang / 128 + wheel_slip_ang / 8 + 2 * wheel_slip)/2;
      }
      else{
      // Second Commited Version
        slip_12 = (ster_factor / 256 + wheel_slip_ang / 8 + 2 * wheel_slip)/2;
      }

      if (std::abs(this->dataPtr->lin_vel_avg[LINK_IND]) < 0.05){
        surface->slip1 = config_params.slip1;
        surface->slip2 = config_params.slip2;      
      }
      else{
        surface->slip1 = slip_12;
        surface->slip2 = slip_12;    
        // surface->slip1 = (wheel_slip_ang + 2 * wheel_slip)/2;
        // surface->slip2 = (wheel_slip_ang + 2 * wheel_slip)/2;
      }

      // double coeff_1 = sigma_x/surface->slip1;
      // double coeff_2 = sigma_x/surface->slip2;
      // double coeff_3 = alpha_x/surface->slip1;
      // double coeff_4 = alpha_x/surface->slip2;

      /* link->collision->surface->friction->contact */
      surface->FrictionPyramid()->SetPoissonsRatio(config_params.PoissonsRatio);
      surface->FrictionPyramid()->SetElasticModulus(config_params.ElasticModulus);
      
      /* link->collision->surface->friction->contact->ode */
      surface->cfm = config_params.cfm;
      surface->erp = config_params.erp;
      surface->kd = config_params.kd;
      surface->kp = config_params.kp;
      surface->maxVel = config_params.maxVel;
      surface->minDepth = config_params.minDepth;

      /* link->collision->surface->friction->torsional */
      surface->FrictionPyramid()->SetMuTorsion(config_params.MuTorsion);
      surface->FrictionPyramid()->SetSurfaceRadius(config_params.SurfaceRadius);
      surface->FrictionPyramid()->SetPatchRadius(config_params.PatchRadius);
      bool usepatch = false;
      if (config_params.UsePatchRadius == 1)
      {
        usepatch = true;
      }
      surface->FrictionPyramid()->SetUsePatchRadius(usepatch);
      // surface->FrictionPyramid()->SetMuPrimary(0.9);
      // surface->FrictionPyramid()->SetMuSecondary(0.9);
    }

    // Try to publish slip data for this wheel
    if (link)
    {
      msgs::Vector3d msg;
      auto name = link->GetName();
      msg = msgs::Convert(slips[name]);
      if (params.slipPub)
        params.slipPub->Publish(msg);
    }
  }
#if DEBUG_PRINT
  fprintf(this->dataPtr->fptr,"\n");
#endif
  this->dataPtr->counter_ = this->dataPtr->counter_+1;
  IGN_PROFILE_END();
}

void HTNavGazeboWheelSurfacePlugin::rr_contact_cb(){
  RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      "Subscribed to RR Contact Forces");
}

void HTNavGazeboWheelSurfacePlugin::LinkStateSolver(double position_NED[3], double velocity_NED[3], double velocity_body[3], double euler[3], physics::LinkPtr link)
{
  ignition::math::Vector3d velocity;
  ignition::math::Pose3d position;
          
  velocity = link->WorldLinearVel();        
  position = link->WorldPose();

  double c_nb[3][3];
  // double position_NED[3], velocity_NED[3];
  double euler_temp[3];

  position_NED[0] = -position.Y();
  position_NED[1] = -position.X();
  position_NED[2] = -position.Z();
  
  velocity_NED[0] = -velocity.Y();
  velocity_NED[1] = -velocity.X();
  velocity_NED[2] = -velocity.Z();

  // euler[0] =  position.Rot().Roll();
  // euler[1] =  position.Rot().Pitch();
  // euler[2] =  position.Rot().Yaw();

  euler[0] =  -position.Rot().Pitch();
  euler[1] =  -position.Rot().Roll();
  euler[2] =  -position.Rot().Yaw();

  euler_temp[0] =  0.0;
  euler_temp[1] =  0.0;
  euler_temp[2] =  euler[2];
  
  Euler2Cnb(c_nb, euler_temp);
  // MatrixVectorMult(position_body, c_nb, position_NED);
  MatrixVectorMult(velocity_body, c_nb, velocity_NED);

}


void HTNavGazeboWheelSurfacePlugin::ContactStateSolver(ConstContactsPtr &_msg, int LINK_IND){
  double body_1_force[3];
  double euler_tw[3], contact_force[3], c_wt[3][3], c_tw[3][3];
  // double sim_time;
  // double body_2_force[3], body_1_torque[3], body_2_torque[3];
  // int time_sec, time_nsec;
  // double contact_time = 0;
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    for (int j = 0; j < _msg->contact(i).position_size(); ++j)
    {

      body_1_force[0] = - _msg->contact(i).wrench(j).body_1_wrench().force().y();
      body_1_force[1] = - _msg->contact(i).wrench(j).body_1_wrench().force().x();
      body_1_force[2] = - _msg->contact(i).wrench(j).body_1_wrench().force().z();

      euler_tw[0] =  this->dataPtr->mapWheelNavParams[LINK_IND].euler[0];
      euler_tw[1] =  this->dataPtr->mapWheelNavParams[LINK_IND].euler[1];
      euler_tw[2] =  0.0;
      
      Euler2Cnb(c_wt, euler_tw);
      MatrixTranspose(c_tw,c_wt);
      MatrixVectorMult(contact_force, c_tw, body_1_force);

    for (int iy = 0; iy < 3; ++iy){
      if( this->dataPtr->contact_counter_[LINK_IND] == 0){
        this->dataPtr->contact_force_min_[LINK_IND][iy] = contact_force[iy];
        this->dataPtr->contact_force_min_two_[LINK_IND][iy] = contact_force[iy];
      }
      this->dataPtr->contact_force_avg[LINK_IND][iy] = (contact_force[iy] + this->dataPtr->contact_force_min_[LINK_IND][iy] + this->dataPtr->contact_force_min_two_[LINK_IND][iy])/3; 

      this->dataPtr->contact_force_min_two_[LINK_IND][iy] = this->dataPtr->contact_force_min_[LINK_IND][iy];
      this->dataPtr->contact_force_min_[LINK_IND][iy]     = contact_force[iy];
    }

    this->dataPtr->contact_counter_[LINK_IND] = this->dataPtr->contact_counter_[LINK_IND] + 1;

    // time_sec = _msg->contact(i).time().sec();
    // time_nsec = _msg->contact(i).time().nsec();
    // contact_time = time_sec + time_nsec*1e-9;

    // body_2_force[0] = _msg->contact(i).wrench(j).body_2_wrench().force().x();
    // body_2_force[1] = _msg->contact(i).wrench(j).body_2_wrench().force().y();
    // body_2_force[2] = _msg->contact(i).wrench(j).body_2_wrench().force().z();

    // body_1_torque[0] = _msg->contact(i).wrench(j).body_1_wrench().torque().x();
    // body_1_torque[1] = _msg->contact(i).wrench(j).body_1_wrench().torque().y();
    // body_1_torque[2] = _msg->contact(i).wrench(j).body_1_wrench().torque().z();

    // body_2_torque[0] = _msg->contact(i).wrench(j).body_2_wrench().torque().x();
    // body_2_torque[1] = _msg->contact(i).wrench(j).body_2_wrench().torque().y();
    // body_2_torque[2] = _msg->contact(i).wrench(j).body_2_wrench().torque().z();
   }
  } 

  return;
}

void HTNavGazeboWheelSurfacePlugin::SteeringAngleCalc(double *steer_ang, double euler[3], double imu_euler[3])
{
  double c_nb[3][3];
  double c_nt[3][3];
  double c_tn[3][3];
  double c_tb[3][3];

  double euler_temp[3], euler_temp2[3];

  euler_temp2[0] =  0.0;
  euler_temp2[1] =  0.0;
  euler_temp2[2] =  euler[2];

  double imu_euler_temp2[3];

  imu_euler_temp2[0] =  0.0;
  imu_euler_temp2[1] =  0.0;
  imu_euler_temp2[2] =  imu_euler[2];
  
  Euler2Cnb(c_nb, euler_temp2);
  Euler2Cnb(c_nt, imu_euler_temp2);
  MatrixTranspose(c_tn, c_nt);

  // MatrixVectorMult(position_body, c_nb, position_NED);
  MatrixMatrixMult(c_tb, c_nb, c_tn);
  Cbn2Euler(c_tb, euler_temp);

  *steer_ang = euler_temp[2];
  return;
}

void HTNavGazeboWheelSurfacePlugin::Euler2Cnb(double c_nb[3][3], double euler_in[3])
{
    c_nb[0][0] = cos(euler_in[1]) * cos(euler_in[2]);
    c_nb[0][1] = cos(euler_in[1]) * sin(euler_in[2]);
    c_nb[0][2] = -sin(euler_in[1]);
    c_nb[1][0] = -cos(euler_in[0]) * sin(euler_in[2]) + sin(euler_in[0]) * sin(euler_in[1]) * cos(euler_in[2]);
    c_nb[1][1] = cos(euler_in[0]) * cos(euler_in[2]) + sin(euler_in[0]) * sin(euler_in[1]) * sin(euler_in[2]);
    c_nb[1][2] = sin(euler_in[0]) * cos(euler_in[1]);
    c_nb[2][0] = sin(euler_in[0]) * sin(euler_in[2]) + cos(euler_in[0]) * sin(euler_in[1]) * cos(euler_in[2]);
    c_nb[2][1] = -sin (euler_in[0]) * cos(euler_in[2]) + cos(euler_in[0]) * sin(euler_in[1]) * sin(euler_in[2]);
    c_nb[2][2] = cos(euler_in[0]) * cos(euler_in[1]);
}

void HTNavGazeboWheelSurfacePlugin::Cbn2Euler(double c_bn[3][3], double euler_out[3]){
  euler_out[0] = std::atan2(c_bn[2][1], c_bn[2][2]);
  euler_out[1] = - std::asin(c_bn[2][0]);
  euler_out[2] = std::atan2(c_bn[1][0], c_bn[0][0]);
  
	return;
}

void HTNavGazeboWheelSurfacePlugin::MatrixVectorMult(double vector_res[3], double matrix1[3][3], double vector2[3]) {
	for (int i = 0; i < 3; i++)
	{
		vector_res[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			vector_res[i] = vector_res[i] + matrix1[i][j]*vector2[j];
		}
	}
	return;
}

void HTNavGazeboWheelSurfacePlugin::MatrixTranspose(double matrix_res[3][3], double matrix1[3][3]) {
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
  		matrix_res[i][j] = matrix1[j][i];
		}
	}
	return;
}

void HTNavGazeboWheelSurfacePlugin::MatrixMatrixMult(double matrix_res[3][3], double matrix1[3][3], double matrix2[3][3]) {
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			matrix_res[i][j] = 0;
			for (int k = 0; k < 3; k++)
			{
				matrix_res[i][j] = matrix_res[i][j] + matrix1[i][k] * matrix2[k][j];
			}
		}
	}
	return;
}

void HTNavGazeboWheelSurfacePlugin::CalcPacejkaModel(double *F_x0, double *F_y0, int LINK_IND){
  // Coeffs
  double alpha, sigma_k;
  double B_x0, B_y0;
  double C_x, C_y;
  double C_F_alpha, C_F_sigma;
  double D_x0, D_y0;
  double E_x0, E_y0;
  double E_x, E_y;
  double mu = this->dataPtr->mapLinkSurfaceConfigParams.MuPrimary;
  double F_z = this->dataPtr->F_z_[LINK_IND];

  alpha   = this->dataPtr->alpha_x_[LINK_IND];
  sigma_k = this->dataPtr->sigma_x_[LINK_IND];

  D_x0 = mu * F_z; 
  D_y0 = mu * F_z; 

  // wheel_slip_ang / 2 
  // First Commited Version
  // C_F_alpha = 4.15 * F_z; // c_1 * c_2 * sin(2*atan(F_z/c_2/F_z0)) * F_z0;
  // C_F_sigma = 8 * F_z;     // c_8 * F_z;
 
  // wheel_slip_ang / 16 
  // C_F_alpha = 6.5 * F_z; // c_1 * c_2 * sin(2*atan(F_z/c_2/F_z0)) * F_z0;
  // C_F_sigma = 7.75 * F_z;     // c_8 * F_z;
  
  // wheel_slip_ang / 32 
  // C_F_alpha = 7.15 * F_z; // c_1 * c_2 * sin(2*atan(F_z/c_2/F_z0)) * F_z0;
  // C_F_sigma = 7.85 * F_z;     // c_8 * F_z;

  // wheel_slip_ang / 64
  // C_F_alpha = 8.15 * F_z; // c_1 * c_2 * sin(2*atan(F_z/c_2/F_z0)) * F_z0;
  // C_F_sigma = 7.95 * F_z;     // c_8 * F_z;

  // wheel_slip_ang / 128
  // Second Commited Version
  // C_F_alpha = 8.0 * F_z; // c_1 * c_2 * sin(2*atan(F_z/c_2/F_z0)) * F_z0;
  // C_F_sigma = 8.0 * F_z;     // c_8 * F_z;

  // Second & Third Commited Version
  C_F_alpha = 8.0 * F_z; // c_1 * c_2 * sin(2*atan(F_z/c_2/F_z0)) * F_z0;
  C_F_sigma = 8.0 * F_z;     // c_8 * F_z;

  // C_F_sigma = 5.25 * F_z;     // c_8 * F_z;
  // C_F_gamma = F_z;         % c_5 * F_z;

  // C_F_alpha_0 = C_F_alpha;
  // C_F_sigma_0 = C_F_sigma;
  // C_x = 0.8;
  C_x = 1.44;
  // C_y = 0.2;
  C_y = 1;
  
  B_x0 = C_F_sigma / C_x / D_x0;
  B_y0 = C_F_alpha / C_y / D_y0;

  E_x0 = -1;
  E_y0 = -1;

  E_x = E_x0;
  E_y = E_y0;

  // % Combined Slips
  // alpha = alpha_k + C_F_gamma / C_F_alpha * gamma_k;

  // sigma_cx = sigma_k / (1 + sigma_k);
  // sigma_cy = tan(alpha) / (1 + sigma_k);

  // sigma = sqrt(sigma_cx^2 + sigma_cy^2);

  // - 


  *F_x0 = D_x0 * sin(C_x * atan(B_x0 * sigma_k - E_x * (B_x0 * sigma_k - atan(B_x0 * sigma_k)) ) );

  *F_y0 = D_y0 * sin(C_y * atan(B_y0 * alpha - - E_y * (B_y0 * alpha - atan(B_y0 * alpha)) ) );
  
  // *F_x0 = D_x0 * sin(C_x * atan(B_x0 * sigma_k - E_x0 * sigma_k - E_x * (B_x0 * sigma_k - atan(B_x0 * sigma_k)) ) );

  // *F_y0 = D_y0 * sin(C_y * atan(B_y0 * alpha - E_y0 * alpha - E_y * (B_y0 * alpha - atan(B_y0 * alpha)) ) );

  return;
}


void HTNavGazeboWheelSurfacePlugin::ContactMessageConstr(sensor_msgs::msg::JointState *msg, int LINK_IND)
{
  msg->name.resize(3);
  msg->position.resize(3);
  msg->velocity.resize(3);
  msg->effort.resize(3);

  auto link_name = "front_left_wheel";

  if( LINK_IND == this->dataPtr->FRONT_LEFT){
    link_name = "front_left_wheel";
  }
  else if(LINK_IND == this->dataPtr->FRONT_RIGHT){
    link_name = "front_right_wheel";
  }
  else if(LINK_IND == this->dataPtr->REAR_LEFT) {
    link_name = "rear_left_wheel";
  }
  else if(LINK_IND == this->dataPtr->REAR_RIGHT){
    link_name = "rear_right_wheel";
  }
  else{
    link_name = "front_left_wheel";
  }

  msg->name[0] = link_name;
  msg->name[1] = link_name;
  msg->name[2] = link_name;
  msg->position[0] = this->dataPtr->contact_force_avg[LINK_IND][0];
  msg->position[1] = this->dataPtr->contact_force_avg[LINK_IND][1];
  msg->position[2] = this->dataPtr->contact_force_avg[LINK_IND][2];
  msg->velocity[0] = this->dataPtr->alpha_x_[LINK_IND];
  msg->velocity[1] = this->dataPtr->sigma_x_[LINK_IND];
  msg->velocity[2] = this->dataPtr->ang_vel_avg[LINK_IND];
  msg->effort[0] =  this->dataPtr->F_x_pacejka_[LINK_IND];
  msg->effort[1] =  this->dataPtr->F_y_pacejka_[LINK_IND];
  msg->effort[2] =  this->dataPtr->lin_vel_avg[LINK_IND];

  return;
}
