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

#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>

#include <mc_impact_constraints/BoundJointTorqueJump.h>
#include <mc_impact_constraints/BoundJointVelocityJump.h>
#include <mc_impact_constraints/COPInsideContactAreaWithImpulse.h>
#include <mc_impact_constraints/ZeroSlippageWithImpulse.h>

struct DynamicContactState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller &) override;

  bool run(mc_control::fsm::Controller &) override;

  void teardown(mc_control::fsm::Controller &) override;

protected:
  // tasks to raise the robot hands
  std::shared_ptr<mc_tasks::EndEffectorTask> rPosTaskPtr_; // right hand
  sva::PTransformd rTransformZero_;
  double contactVelocity_;
  mc_rtc::Configuration state_conf_;

  bool removedConstraint_ = false;

  std::unique_ptr<mc_impact::BoundJointTorqueJump> boundTorqueJump_;
  std::unique_ptr<mc_impact::BoundJointVelocityJump> boundVelocityJump_;
  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> zeroSlippageLeftFoot_;
  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> zeroSlippageRightFoot_;
};
