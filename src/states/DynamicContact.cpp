/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Yuquan Wang, Arnaud Tanguy and Pierre Gergondet
 *
 * 
 *
 * mc_impact_pusher is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * mc_impact_pusher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with mc_impact_pusher. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "DynamicContact.h"

#include <mc_tasks/MetaTaskLoader.h>

#include "../mc_r_arm_push_unknown_wall.h"

void DynamicContactState::configure(const mc_rtc::Configuration & config)
{
  state_conf_.load(config);
}

void DynamicContactState::start(mc_control::fsm::Controller & ctlInput)
{

  LOG_SUCCESS("Starting DynamicContactState");

  auto & ctl = static_cast<Controller &>(ctlInput);

  const auto & conf = ctl.config()("hrp4");
  const auto & efName = conf("rightEfTaskDynamic")("bodyName");

  rPosTaskPtr_ = std::make_shared<mc_tasks::EndEffectorTask>(
      efName, ctl.robots(), 0, conf("rightEfTaskDynamic")("stiffness"), conf("rightEfTaskDynamic")("weight"));
  ctl.solver().addTask(rPosTaskPtr_);

  Eigen::Vector3d referenceVelocity, pushDepth, currentPos;
  pushDepth = ctl.config()("states")("Contact")("pushDepth");
  referenceVelocity = ctl.config()("states")("Contact")("contactVelocity");

  ctl.logger().addLogEntry("ee_Vel_target", [this]() { return rPosTaskPtr_->positionTask->refVel(); });

  currentPos = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName(efName)].translation();
  // Align hand to vertical wall
  Eigen::Matrix3d desiredRotation;
  desiredRotation << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  sva::PTransformd X_0_target(desiredRotation, currentPos + pushDepth);
  // Set the target position, which is supposed to be far away.
  rPosTaskPtr_->set_ef_pose(X_0_target);
  // Set the reference velocity
  rPosTaskPtr_->positionTask->refVel(referenceVelocity);

  // Set the damping gain (set it much much higher than the position
  // gain[stiffness] to ensure that velocity is properly tracked
  if(state_conf_.has("rightEfStiffness"))
  {
    rPosTaskPtr_->positionTask->stiffness(static_cast<double>(state_conf_("rightEfStiffness")));
  }
  if(state_conf_.has("rightEfDamping"))
  {
    rPosTaskPtr_->positionTask->damping(static_cast<double>(state_conf_("rightEfDamping")));
  }

  run(ctlInput);
}

bool DynamicContactState::run(mc_control::fsm::Controller & ctlInput)
{
  auto & ctl = static_cast<Controller &>(ctlInput);

  ctl.setOsd()->update();

  if(ctl.config()("qpEstimator")("on"))
  {
    ctl.qpEstimatorPtr->update();
    ctl.osdQpEstimatorPtr->update();
    ctl.jsdQpEstimatorPtr->update();
    ctl.ecQpEstimatorPtr->update();
  }

  if(ctl.config()("lcp")("on"))
  {
    if(ctl.lcpSolverPtr->getDim() == 1)
    {
      std::map<std::string, Eigen::Vector3d> contactSurfaceNormals;
      Eigen::Vector3d groundSurfaceNormal;
      groundSurfaceNormal << 0, 0, 1;

      sva::PTransformd X_0_lSole = ctl.robot().bodyPosW("l_sole");
      contactSurfaceNormals["l_sole"] = X_0_lSole.rotation() * groundSurfaceNormal + X_0_lSole.translation();
      sva::PTransformd X_0_rSole = ctl.robot().bodyPosW("r_sole");
      contactSurfaceNormals["r_sole"] = X_0_rSole.rotation() * groundSurfaceNormal + X_0_rSole.translation();

      ctl.lcpSolverPtr->update(contactSurfaceNormals);
    }
    else
    {
      ctl.lcpSolverPtr->update();
    }
  }
  if(ctl.rArmInContact() && !removedConstraint_)
  {
    LOG_SUCCESS("Contact Detected, stoping DynamicContactState");
    removedConstraint_ = true;
    rPosTaskPtr_->reset();
    rPosTaskPtr_->positionTask->refVel(Eigen::Vector3d::Zero());

    // FIXME: when removing these tasks, the hand keep moving forward in
    // IdleState.... Why? ctl.solver().removeTask(rPosTaskPtr_);
    // ctl.solver().removeTask(ctl.comTaskPtr);

    if(ctl.config()("impact")("constraints")("zmpWithImpulse")("on"))
    {
      ctl.solver().removeConstraint(ctl.zmpImpulse_.get());
    }

    if(ctl.config()("impact")("constraints")("copWithImpulse")("on"))
    {
      ctl.solver().removeConstraint(ctl.copImpulseLeftFoot_.get());
      ctl.solver().removeConstraint(ctl.copImpulseRightFoot_.get());
    }

    if(ctl.config()("impact")("constraints")("frictionWithImpulse"))
    {
      ctl.solver().removeConstraint(ctl.frictionImpulseLeftFoot_.get());
      ctl.solver().removeConstraint(ctl.frictionImpulseRightFoot_.get());
    }

    if(ctl.config()("impact")("constraints")("jointTorque")("on"))
    {
      ctl.solver().removeConstraint(ctl.boundTorqueJump_.get());
    }

    if(ctl.config()("impact")("constraints")("jointVelocity")("on"))
    {
      ctl.solver().removeConstraint(ctl.boundVelocityJump_.get());
    }
    ctl.solver().updateConstrSize();

    LOG_INFO("remaining tasks");
    for(const auto task : ctl.solver().tasks())
    {
      LOG_INFO("task: " << task->name());
    }
    output("OK");
    return true;
  }

  return false;
}

void DynamicContactState::teardown(mc_control::fsm::Controller & ctl_)
{
  LOG_SUCCESS("Completed DynamicContactState");
  auto & ctl = static_cast<Controller &>(ctl_);
  ctl.solver().removeTask(rPosTaskPtr_);
  ctl.logger().removeLogEntry("ee_Vel_target");
}

EXPORT_SINGLE_STATE("DynamicContact", DynamicContactState)
