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
#ifndef GAZEBO_PLUGINS_HTNAVGAZEBOWHEELSURFACEPLUGIN_HH_
#define GAZEBO_PLUGINS_HTNAVGAZEBOWHEELSURFACEPLUGIN_HH_

#include <map>
#include <memory>
#include <string>

#include <ignition/math/Vector3.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

#include "gazebo/physics/Contact.hh"

namespace gazebo
{
  // Forward declare private data class
  class HTNavGazeboWheelSurfacePluginPrivate;

  /// \brief A plugin that updates ODE wheel slip parameters based
  /// on linear wheel spin velocity (radius * spin rate).
  /// It currently assumes that the fdir1 friction parameter is set
  /// parallel to the joint axis (often [0 0 1]) and that the link
  /// origin is on the joint axis.
  /// The ODE slip parameter is documented as Force-Dependent Slip
  /// (slip1, slip2) in the ODE user guide:
  /// http://ode.org/ode-latest-userguide.html#sec_7_3_7
  /// and it has units of velocity / force (m / s / N),
  /// similar to the inverse of a viscous damping coefficient.
  /// The slip_compliance parameters specified in this plugin
  /// are unitless, representing the lateral or longitudinal slip ratio
  /// (see https://en.wikipedia.org/wiki/Slip_(vehicle_dynamics) )
  /// to tangential force ratio (tangential / normal force).
  /// Note that the maximum force ratio is the friction coefficient.
  /// At each time step, these compliance are multipled by
  /// the linear wheel spin velocity and divided by the wheel_normal_force
  /// parameter specified below in order to match the units of the ODE
  /// slip parameters.
  ///
  /// A graphical interpretation of these parameters is provided below
  /// for a positive value of slip compliance.
  /// The horizontal axis corresponds to the slip ratio at the wheel,
  /// and the vertical axis corresponds to the tangential force ratio
  /// (tangential / normal force).
  /// As wheel slip increases, the tangential force increases until
  /// it reaches the maximum set by the friction coefficient.
  /// The slip compliance corresponds to the inverse of the slope
  /// of the force before it reaches the maximum value.
  /// A slip compliance of 0 corresponds to a completely vertical
  /// portion of the plot below.
  /// As slip compliance increases, the slope decreases.
  ///
  /** \verbatim
        |                                            .
        |      _________ friction coefficient        .
        |     /                                      .
        |    /|                                      .
        |   /-┘ slope is inverse of                  .
        |  /    slip compliance                      .
        | /                                          .
        |/                                           .
      --+-------------------------- slipRatio
        |

    <plugin filename="libht_nav_gazebo_wheel_surface_plugin.so" name="wheel_surface_plugin">
      <config_params>
        <!-- link->collision->surface->bounce -->
        <bounce>0.0</bounce>
        <bounceThreshold>0.0</bounceThreshold> 
        <!-- link->collision->surface->friction->ode -->
        <MuPrimary>0.6</MuPrimary> 
        <MuSecondary>0.6</MuSecondary>    
        <slip1>0.0003</slip1>
        <slip2>0.0003</slip2>
        <!-- link->collision->surface->friction->torsional  -->
        <MuTorsion>10.0</MuTorsion> 
        <SurfaceRadius>0.31265</SurfaceRadius> 
        <PatchRadius>0</PatchRadius> 
        <UsePatchRadius>0</UsePatchRadius> 
        <!-- link->collision->surface->friction->contact -->
        <PoissonsRatio>0.5</PoissonsRatio> 
        <ElasticModulus>1e7</ElasticModulus> 
        <minDepth>0.005</minDepth>
        <kp>1e15</kp>
        <kd>1e2</kd>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
        <maxVel>0.0</maxVel>
        <!-- slip complience -->
        <slipComplianceLateral>0.1</slipComplianceLateral>
        <slipComplianceLongitudinal>0.1</slipComplianceLongitudinal>
      </config_params>

      <wheel link_name="front_left_wheel">
        <wheel_normal_force>3315</wheel_normal_force>
      </wheel>
      <wheel link_name="front_right_wheel">
        <wheel_normal_force>3315</wheel_normal_force>
      </wheel>
      <wheel link_name="rear_left_wheel">
        <wheel_normal_force>3315</wheel_normal_force>
      </wheel>
      <wheel link_name="rear_right_wheel">
        <wheel_normal_force>3315</wheel_normal_force>
      </wheel>
    </plugin>

  \endverbatim */

  class GZ_PLUGIN_VISIBLE HTNavGazeboWheelSurfacePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: HTNavGazeboWheelSurfacePlugin();

    /// \brief Destructor.
    public: virtual ~HTNavGazeboWheelSurfacePlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
    public: virtual void Fini();

    /// \brief Get parent model.
    /// \return pointer to parent model.
    public: physics::ModelPtr GetParentModel() const;

    /// \brief Get wheel slip measurements.
    /// \param[out] _out Map of wheel name to a Vector3 of slip velocities.
    /// The Vector3.X value is the longitudinal slip in m/s,
    /// the Vector3.Y value is the lateral slip in m/s, and
    /// the Vector3.Z value is the product of radius and spin rate in m/s.
    public: void GetSlips(std::map<std::string, ignition::math::Vector3d> &_out)
            const;

    /// \brief Set unitless lateral slip compliance for all wheels.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLateral(const double _compliance);

    /// \brief Set unitless lateral slip compliance for a particular wheel.
    /// \param[in] _wheel_name name of the wheel link on which _compliance should be set.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLateral(std::string _wheel_name, const double _compliance);

    /// \brief Set unitless longitudinal slip compliance for all wheels.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLongitudinal(const double _compliance);

    /// \brief Set unitless longitudinal slip compliance for a particular wheel.
    /// \param[in] _wheel_name name of the wheel link on which _compliance should be set.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLongitudinal(std::string _wheel_name, const double _compliance);

    /// \brief Transport callback for setting lateral slip compliance.
    /// \param[in] _msg Slip compliance encoded as string.
    private: void OnLateralCompliance(ConstGzStringPtr &_msg);

    /// \brief Transport callback for setting longitudinal slip compliance.
    /// \param[in] _msg Slip compliance encoded as string.
    private: void OnLongitudinalCompliance(ConstGzStringPtr &_msg);

    /// brief Get friction coefficients for each wheel
    /// \return Map of wheel name to a Vector2 of friction coefficients
    /// The Vector2.X value is the friction coefficient in the primary direction and
    /// the Vector2.Y value is the friction coefficient in the secondary direction.
    public: std::map<std::string, ignition::math::Vector2d> GetFrictionCoefficients();

    /// \brief Set the friction coefficient in the primary direction for a particular wheel.
    /// \param[in] _wheel_name name of the wheel link on which _mu should be set.
    /// \param[in] _mu Friction coefficient.
    /// \return True if the friction coefficient was successfully set. False otherwise.
    public: bool SetMuPrimary(const std::string &_wheel_name, double _mu);

    /// \brief Set the friction coefficient in the secondary direction for a particular wheel.
    /// \param[in] _wheel_name name of the wheel link on which _mu should be set.
    /// \param[in] _mu Friction coefficient.
    /// \return True if the friction coefficient was successfully set. False otherwise.
    public: bool SetMuSecondary(const std::string &_wheel_name, double _mu);

    /// \brief Update the plugin. This is updated every iteration of
    /// simulation.
    private: void Update();
    public: void OnFLContacts(ConstContactsPtr &_msg);
    public: void OnFRContacts(ConstContactsPtr &_msg);
    public: void OnRLContacts(ConstContactsPtr &_msg);
    public: void OnRRContacts(ConstContactsPtr &_msg);

    public: void rr_contact_cb();

    public: void LinkStateSolver(double position_NED[3], double velocity_NED[3], double velocity_body[3], double euler[3], physics::LinkPtr link);
    public: void ContactStateSolver(ConstContactsPtr &_msg, int LINK_IND);
    public: void SteeringAngleCalc(double *steer_ang, double euler[3], double imu_euler[3]);
    public: void Euler2Cnb(double c_nb[3][3], double euler_in[3]);
    public: void MatrixVectorMult(double vector_res[3], double matrix1[3][3], double vector2[3]);
    public: void MatrixMatrixMult(double matrix_res[3][3], double matrix1[3][3], double matrix2[3][3]);
    public: void Cbn2Euler(double c_bn[3][3], double euler_out[3]);
    public: void MatrixTranspose(double matrix_res[3][3], double matrix1[3][3]);
    public: void CalcPacejkaModel(double *F_x0, double *F_y0, int LINK_IND);
    public: void ContactMessageConstr(sensor_msgs::msg::JointState *msg, int LINK_IND);

    /// \brief Private data pointer.
    private: std::unique_ptr<HTNavGazeboWheelSurfacePluginPrivate> dataPtr;
  };
}
#endif
