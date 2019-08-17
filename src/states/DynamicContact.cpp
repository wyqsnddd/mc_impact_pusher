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
  // ctl.miPredictorPtr->resetDataStructure();

  run(ctlInput);
}

bool DynamicContactState::run(mc_control::fsm::Controller & ctlInput)
{
  auto & ctl = static_cast<Controller &>(ctlInput);

  // ctl.miOsdPtr->update();
  Eigen::Vector3d surfaceNormal;
  surfaceNormal << 1, 0, 0;
  // Convert surfaceNormal to the local frame of the right wrist.
  sva::PTransformd X_0_ee = ctl.robot().bodyPosW("r_wrist");

  // ctl.miPredictorPtr->run(X_0_ee.rotation() * surfaceNormal + X_0_ee.translation());

  //Eigen::Vector3d r_wrist_surfaceNormal = X_0_ee.rotation() * surfaceNormal + X_0_ee.translation();

  //r_wrist_surfaceNormal = X_0_ee.rotation().transpose() * r_wrist_surfaceNormal;
  std::map<std::string, Eigen::Vector3d> surfaceNormals;

  Eigen::Vector3d bodySurfaceNormal = X_0_ee.rotation().transpose() * surfaceNormal; 
  bodySurfaceNormal.normalize();

  surfaceNormals["r_wrist"] = bodySurfaceNormal; 
  /*
  Eigen::Vector3d l_surfaceNormal;
  l_surfaceNormal << 0, 1, 0;
  // Convert surfaceNormal to the local frame of the right wrist.
  sva::PTransformd X_0_lee = ctl.robot().bodyPosW("l_wrist");
  surfaceNormals["l_wrist"] =  X_0_lee.rotation() * l_surfaceNormal + X_0_lee.translation();
*/
  // ctl.miPredictorPtr->run(X_0_ee.rotation() * surfaceNormal + X_0_ee.translation());

  ctl.multiImpactPredictorPtr->run(surfaceNormals);

  if(ctl.config()("qpEstimator")("on"))
  {
    ctl.qpEstimatorPtr->update(surfaceNormals);
    ctl.osdQpEstimatorPtr->update(surfaceNormals);
    ctl.jsdQpEstimatorPtr->update(surfaceNormals);
    ctl.ecQpEstimatorPtr->update(surfaceNormals);
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

    // FIXME: when removing these tasks, the hand keep moving forward in IdleState.... Why?
    // ctl.solver().removeTask(rPosTaskPtr_);
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
}

EXPORT_SINGLE_STATE("DynamicContact", DynamicContactState)
