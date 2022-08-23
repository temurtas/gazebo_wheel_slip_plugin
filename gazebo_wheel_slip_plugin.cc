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

#include "gazebo_wheel_slip_plugin.hh"

namespace gazebo
{
  namespace physics
  {
    typedef boost::weak_ptr<physics::Joint> JointWeakPtr;
    typedef boost::weak_ptr<physics::Link> LinkWeakPtr;
    typedef boost::weak_ptr<physics::Model> ModelWeakPtr;
    typedef boost::weak_ptr<physics::ODESurfaceParams> ODESurfaceParamsWeakPtr;
  }

  class GazeboWheelSlipPluginPrivate
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

      /// \brief Wheel normal force estimate used to compute slip
      /// compliance for ODE, which takes units of 1/N.
      public: double wheelNormalForce = 0;

      /// \brief Wheel radius extracted from collision shape if not
      /// specified as xml parameter.
      public: double wheelRadius = 0;

      /// \brief Publish slip for each wheel.
      public: transport::PublisherPtr slipPub;
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
GZ_REGISTER_MODEL_PLUGIN(GazeboWheelSlipPlugin)

/////////////////////////////////////////////////
GazeboWheelSlipPlugin::GazeboWheelSlipPlugin()
  : dataPtr(new GazeboWheelSlipPluginPrivate)
{
}

/////////////////////////////////////////////////
GazeboWheelSlipPlugin::~GazeboWheelSlipPlugin()
{
}

/////////////////////////////////////////////////
void GazeboWheelSlipPlugin::Fini()
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
void GazeboWheelSlipPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->ros_node_ = gazebo_ros::Node::Get(_sdf);

  GZ_ASSERT(_model, "GazeboWheelSlipPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "GazeboWheelSlipPlugin sdf pointer is NULL");

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

  if (!_sdf->HasElement("wheel"))
  {
    gzerr << "No wheel tags specified, plugin is disabled" << std::endl;
    return;
  }

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

    GazeboWheelSlipPluginPrivate::LinkSurfaceParams params;
    if (wheelElem->HasElement("slip_compliance_lateral"))
    {
      params.slipComplianceLateral =
        wheelElem->Get<double>("slip_compliance_lateral");
    }
    if (wheelElem->HasElement("slip_compliance_longitudinal"))
    {
      params.slipComplianceLongitudinal =
        wheelElem->Get<double>("slip_compliance_longitudinal");
    }

    RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      "New lateral slip compliance: %.3e", params.slipComplianceLateral);

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
      &GazeboWheelSlipPlugin::OnLateralCompliance, this);

  this->dataPtr->longitudinalComplianceSub = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/wheel_slip/longitudinal_compliance",
      &GazeboWheelSlipPlugin::OnLongitudinalCompliance, this);

  const gazebo_ros::QoS & qos = this->dataPtr->ros_node_->get_qos();

  this->dataPtr->sub_ = this->dataPtr->ros_node_->create_subscription<gazebo_msgs::msg::ContactsState>(
    "kobra_mk5/rear_right_contact_forces", 10,
    std::bind(&GazeboWheelSlipPlugin::Update, this));

  // Connect to the update event
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboWheelSlipPlugin::Update, this));
}

/////////////////////////////////////////////////
physics::ModelPtr GazeboWheelSlipPlugin::GetParentModel() const
{
  return this->dataPtr->model.lock();
}

/////////////////////////////////////////////////
void GazeboWheelSlipPlugin::GetSlips(
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
void GazeboWheelSlipPlugin::OnLateralCompliance(ConstGzStringPtr &_msg)
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
void GazeboWheelSlipPlugin::OnLongitudinalCompliance(ConstGzStringPtr &_msg)
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
void GazeboWheelSlipPlugin::SetSlipComplianceLateral(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    linkSurface.second.slipComplianceLateral = _compliance;
  }
}

/////////////////////////////////////////////////
void GazeboWheelSlipPlugin::SetSlipComplianceLateral(std::string _wheel_name, const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->mapLinkNames.count(_wheel_name) > 0)
  {
    auto link = this->dataPtr->mapLinkNames[_wheel_name];
    this->dataPtr->mapLinkSurfaceParams[link].slipComplianceLateral = _compliance;
  }
}

/////////////////////////////////////////////////
void GazeboWheelSlipPlugin::SetSlipComplianceLongitudinal(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    linkSurface.second.slipComplianceLongitudinal = _compliance;
  }
}

/////////////////////////////////////////////////
void GazeboWheelSlipPlugin::SetSlipComplianceLongitudinal(std::string _wheel_name, const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->mapLinkNames.count(_wheel_name) > 0)
  {
    auto link = this->dataPtr->mapLinkNames[_wheel_name];
    this->dataPtr->mapLinkSurfaceParams[link].slipComplianceLongitudinal = _compliance;
  }
}

/////////////////////////////////////////////////
std::map<std::string, ignition::math::Vector2d> GazeboWheelSlipPlugin::GetFrictionCoefficients()
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
bool GazeboWheelSlipPlugin::SetMuPrimary(const std::string &_wheel_name, double _mu)
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
bool GazeboWheelSlipPlugin::SetMuSecondary(const std::string &_wheel_name, double _mu)
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
void GazeboWheelSlipPlugin::Update()
{
  // RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
  //     "Update Time");
  
  IGN_PROFILE("GazeboWheelSlipPlugin::OnUpdate");
  IGN_PROFILE_BEGIN("Update");
  // Get slip data so it can be published later
  std::map<std::string, ignition::math::Vector3d> slips;
  this->GetSlips(slips);

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
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
      // As discussed in GazeboWheelSlipPlugin.hh, the ODE slip1 and slip2
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

      double pos_body[3], vel_body[3], euler_body[3];
      LinkStateSolver(pos_body, vel_body, euler_body, link);



      surface->slip1 = speed / force * params.slipComplianceLateral;
      surface->slip2 = speed / force * params.slipComplianceLongitudinal;

      surface->FrictionPyramid()->SetMuPrimary(0.9);
      surface->FrictionPyramid()->SetMuPrimary(0.9);
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
  IGN_PROFILE_END();
}

void GazeboWheelSlipPlugin::rr_contact_cb(){
  RCLCPP_INFO(this->dataPtr->ros_node_->get_logger(),
      "Subscribed to RR Contact Forces");
}

void GazeboWheelSlipPlugin::LinkStateSolver(double position_body[3], double velocity_body[3], double euler[3], physics::LinkPtr link)
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

void GazeboWheelSlipPlugin::Euler2Cnb(double c_nb[3][3], double euler_in[3])
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


void GazeboWheelSlipPlugin::MatrixVectorMult(double vector_res[3], double matrix1[3][3], double vector2[3]) {
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
