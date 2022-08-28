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

#include <gazebo_msgs/msg/contact_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>

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

#include "ht_nav_gazebo_wheel_slip_plugin.hh"

namespace gazebo
{
  namespace physics
  {
    typedef boost::weak_ptr<physics::Joint> JointWeakPtr;
    typedef boost::weak_ptr<physics::Link> LinkWeakPtr;
    typedef boost::weak_ptr<physics::Model> ModelWeakPtr;
    typedef boost::weak_ptr<physics::ODESurfaceParams> ODESurfaceParamsWeakPtr;
  }

  class HTNavGazeboWheelSlipPluginPrivate
  {
    /// A pointer to the GazeboROS node.
    public: gazebo_ros::Node::SharedPtr ros_node_;
    /// Subscriber to elevator commands
    public: rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr sub_;

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
  };
}

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(HTNavGazeboWheelSlipPlugin)

/////////////////////////////////////////////////
HTNavGazeboWheelSlipPlugin::HTNavGazeboWheelSlipPlugin()
  : dataPtr(new HTNavGazeboWheelSlipPluginPrivate)
{
}

/////////////////////////////////////////////////
HTNavGazeboWheelSlipPlugin::~HTNavGazeboWheelSlipPlugin()
{
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSlipPlugin::Fini()
{
  this->dataPtr->updateConnection.reset();

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
void HTNavGazeboWheelSlipPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->ros_node_ = gazebo_ros::Node::Get(_sdf);

  GZ_ASSERT(_model, "HTNavGazeboWheelSlipPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "HTNavGazeboWheelSlipPlugin sdf pointer is NULL");

  this->dataPtr->model = _model;
  auto world = _model->GetWorld();
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

  auto configElem = _sdf->GetElement("config_params");

  HTNavGazeboWheelSlipPluginPrivate::LinkSurfaceConfigParams config_params;

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

    HTNavGazeboWheelSlipPluginPrivate::LinkSurfaceParams params;
    
    if (wheelElem->HasElement("wheel_normal_force"))
    {
      params.wheelNormalForce = wheelElem->Get<double>("wheel_normal_force");
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

    RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      "wheel read");
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
      &HTNavGazeboWheelSlipPlugin::OnLateralCompliance, this);

  this->dataPtr->longitudinalComplianceSub = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/wheel_slip/longitudinal_compliance",
      &HTNavGazeboWheelSlipPlugin::OnLongitudinalCompliance, this);

  const gazebo_ros::QoS & qos = this->dataPtr->ros_node_->get_qos();

  // this->dataPtr->sub_ = this->dataPtr->ros_node_->create_subscription<gazebo_msgs::msg::ContactsState>(
  //   "kobra_mk5/rear_right_contact_forces", 10,
  //   std::bind(&HTNavGazeboWheelSlipPlugin::Update, this));

  // Connect to the update event
    RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      "Pre Update");
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HTNavGazeboWheelSlipPlugin::Update, this));
}

/////////////////////////////////////////////////
physics::ModelPtr HTNavGazeboWheelSlipPlugin::GetParentModel() const
{
  return this->dataPtr->model.lock();
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSlipPlugin::GetSlips(
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
void HTNavGazeboWheelSlipPlugin::OnLateralCompliance(ConstGzStringPtr &_msg)
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
void HTNavGazeboWheelSlipPlugin::OnLongitudinalCompliance(ConstGzStringPtr &_msg)
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
void HTNavGazeboWheelSlipPlugin::SetSlipComplianceLateral(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  // {
  //   linkSurface.second.slipComplianceLateral = _compliance;
  // }
  this->dataPtr->mapLinkSurfaceConfigParams.slipComplianceLateral = _compliance;
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSlipPlugin::SetSlipComplianceLateral(std::string _wheel_name, const double _compliance)
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
void HTNavGazeboWheelSlipPlugin::SetSlipComplianceLongitudinal(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  // {
  //   linkSurface.second.slipComplianceLongitudinal = _compliance;
  // }
  this->dataPtr->mapLinkSurfaceConfigParams.slipComplianceLongitudinal = _compliance;
}

/////////////////////////////////////////////////
void HTNavGazeboWheelSlipPlugin::SetSlipComplianceLongitudinal(std::string _wheel_name, const double _compliance)
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
std::map<std::string, ignition::math::Vector2d> HTNavGazeboWheelSlipPlugin::GetFrictionCoefficients()
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
bool HTNavGazeboWheelSlipPlugin::SetMuPrimary(const std::string &_wheel_name, double _mu)
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
bool HTNavGazeboWheelSlipPlugin::SetMuSecondary(const std::string &_wheel_name, double _mu)
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
void HTNavGazeboWheelSlipPlugin::Update()
{
  // RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
  //     "Update Time");
  
  IGN_PROFILE("HTNavGazeboWheelSlipPlugin::OnUpdate");
  IGN_PROFILE_BEGIN("Update");
  // Get slip data so it can be published later
  // std::map<std::string, ignition::math::Vector3d> slips;
  // this->GetSlips(slips);

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  const auto config_params = this->dataPtr->mapLinkSurfaceConfigParams;
  for (const auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    const auto &params = linkSurface.second;

    // get user-defined normal force constant
    double force = params.wheelNormalForce;

    // get link angular velocity parallel to joint axis
    ignition::math::Vector3d wheelAngularVelocity;
    auto link = linkSurface.first.lock();
    if (link)
      wheelAngularVelocity = link->WorldAngularVel();

    ignition::math::Vector3d jointAxis;
    auto joint = params.joint.lock();
    if (joint)
      jointAxis = joint->GlobalAxis(0);

    double spinAngularVelocity = wheelAngularVelocity.Dot(jointAxis);

    auto surface = params.surface.lock();
    if (surface)
    {
      // As discussed in HTNavGazeboWheelSlipPlugin.hh, the ODE slip1 and slip2
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

      // // double pos_body[3], vel_body[3], euler_body[3];
      // // LinkStateSolver(pos_body, vel_body, euler_body, link);

      // surface->slip1 = 0.0;
      // surface->slip2 = 0.0;

      // surface->slip1 = speed / force * params.slipComplianceLateral;
      // surface->slip2 = speed / force * params.slipComplianceLongitudinal;

      // surface->slip1 = 0.001 + speed / 3e-5;
      // surface->slip2 = 0.001 + speed / 3e-5;

      // surface->bounce = config_params.bounce; 
      // surface->bounceThreshold = config_params.bounceThreshold;
      // surface->cfm = config_params.cfm;
      // surface->erp = config_params.erp;
      // surface->kd = config_params.kd;
      // surface->kp = config_params.kp;
      // surface->maxVel = config_params.maxVel;
      // surface->minDepth = config_params.minDepth;
      // surface->slip1 = config_params.slip1;
      // surface->slip2 = config_params.slip2;

      // surface->FrictionPyramid()->SetMuPrimary(config_params.MuPrimary);

      // surface->FrictionPyramid()->SetElasticModulus(config_params.ElasticModulus);
      // surface->FrictionPyramid()->SetMuPrimary(config_params.MuPrimary);
      // surface->FrictionPyramid()->SetMuSecondary(config_params.MuSecondary);
      // surface->FrictionPyramid()->SetMuTorsion(config_params.MuTorsion);
      // surface->FrictionPyramid()->SetPatchRadius(config_params.PatchRadius);
      // surface->FrictionPyramid()->SetPoissonsRatio(config_params.PoissonsRatio);
      // surface->FrictionPyramid()->SetSurfaceRadius(config_params.SurfaceRadius);
      // bool usepatch = false;
      // if (config_params.UsePatchRadius == 1)
      // {
      //   usepatch = true;
      // }
      // surface->FrictionPyramid()->SetUsePatchRadius(usepatch);

      // surface->FrictionPyramid()->SetMuPrimary(0.9);
      // surface->FrictionPyramid()->SetMuSecondary(0.9);
    }

    // Try to publish slip data for this wheel
    // if (link)
    // {
    //   msgs::Vector3d msg;
    //   auto name = link->GetName();
    //   msg = msgs::Convert(slips[name]);
    //   if (params.slipPub)
    //     params.slipPub->Publish(msg);
    // }
  }
  IGN_PROFILE_END();
}

void HTNavGazeboWheelSlipPlugin::rr_contact_cb(){
  RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      "Subscribed to RR Contact Forces");
}

void HTNavGazeboWheelSlipPlugin::LinkStateSolver(double position_body[3], double velocity_body[3], double euler[3], physics::LinkPtr link)
{
  ignition::math::Vector3d velocity;
  ignition::math::Pose3d position;

  velocity = link->WorldLinearVel();        
  position = link->WorldPose();

  double c_nb[3][3];
  double position_NED[3], velocity_NED[3];


  position_NED[0] = -position.Y();
  position_NED[1] = -position.X();
  position_NED[2] = -position.Z();
  
  velocity_NED[0] = -velocity.Y();
  velocity_NED[1] = -velocity.X();
  velocity_NED[2] = -velocity.Z();

  euler[0] =  position.Rot().Roll();
  euler[1] =  position.Rot().Pitch();
  euler[2] =  position.Rot().Yaw();

  Euler2Cnb(c_nb, euler);
  MatrixVectorMult(position_body, c_nb, position_NED);
  MatrixVectorMult(velocity_body, c_nb, velocity_NED);

}

void HTNavGazeboWheelSlipPlugin::Euler2Cnb(double c_nb[3][3], double euler_in[3])
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


void HTNavGazeboWheelSlipPlugin::MatrixVectorMult(double vector_res[3], double matrix1[3][3], double vector2[3]) {
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
