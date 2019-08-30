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
#include <mc_control/fsm/Controller.h>
#include <mc_prediction/mi_lcp.h>
#include <mc_prediction/mi_qpEstimator.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>

#include <mc_impact_constraints/BoundJointTorqueJump.h>
#include <mc_impact_constraints/BoundJointVelocityJump.h>
#include <mc_impact_constraints/COPInsideContactAreaWithImpulse.h>
#include <mc_impact_constraints/ZeroSlippageWithImpulse.h>
#include <mc_impact_constraints/copWithImpulse.h>
#include <mc_impact_constraints/frictionWithImpulse.h>
#include <mc_impact_constraints/zmpWithImpulse.h>

struct Controller : public mc_control::fsm::Controller
{
  Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf);

  void reset(const mc_control::ControllerResetData & data) override;

  bool run() override;

  std::shared_ptr<mc_tasks::MetaTask> comTaskPtr;

  std::unique_ptr<mc_impact::BoundJointTorqueJump> boundTorqueJump_;
  std::unique_ptr<mc_impact::BoundJointVelocityJump> boundVelocityJump_;
  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> zeroSlippageLeftFoot_;
  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> zeroSlippageRightFoot_;
  std::unique_ptr<mc_impact::COPInsideContactAreaWithImpulse> COPImpulseLeftFoot_;
  std::unique_ptr<mc_impact::COPInsideContactAreaWithImpulse> COPImpulseRightFoot_;
  std::unique_ptr<mc_impact::frictionWithImpulse> frictionImpulseRightFoot_;
  std::unique_ptr<mc_impact::frictionWithImpulse> frictionImpulseLeftFoot_;
  std::unique_ptr<mc_impact::copWithImpulse> copImpulseLeftFoot_;
  std::unique_ptr<mc_impact::copWithImpulse> copImpulseRightFoot_;
  std::unique_ptr<mc_impact::zmpWithImpulse> zmpImpulse_;

  // Force sensor threshold
  double forceThreshold = 3.0;

  bool firstContact = true;

  bool rArmInContact();

  std::unique_ptr<mc_impact::mi_lcp> lcpSolverPtr;
  std::unique_ptr<mc_impact::mi_qpEstimator> qpEstimatorPtr;
  std::unique_ptr<mc_impact::mi_qpEstimator> jsdQpEstimatorPtr;
  std::unique_ptr<mc_impact::mi_qpEstimator> osdQpEstimatorPtr;
  std::unique_ptr<mc_impact::mi_qpEstimator> ecQpEstimatorPtr;

  const mc_rbdyn::Contact & getContact(const std::string & s);

  const std::shared_ptr<mc_impact::mi_osd> getOsd()
  {
    return miOsdPtr_;
  }

  std::shared_ptr<mc_impact::mi_osd> & setOsd()
  {
    return miOsdPtr_;
  }

private:
  const mc_rbdyn::Robot & realRobot() const;

  std::shared_ptr<mc_impact::mi_osd> miOsdPtr_;
  double impactIndicator_;

  size_t iter_ = 0;
  // mc_rtc::Configuration state_conf_;
};
