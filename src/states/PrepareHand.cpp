#include "PrepareHand.h"

#include <mc_tasks/MetaTaskLoader.h>

#include "../mc_r_arm_push_unknown_wall.h"

void PrepareHandState::configure(const mc_rtc::Configuration & config)
{
  config("EfThreshold", efThreshold_);
}

void PrepareHandState::start(mc_control::fsm::Controller & ctlInput)
{

  auto & ctl = static_cast<Controller &>(ctlInput);

  rEfTaskPtr_ = std::make_shared<mc_tasks::EndEffectorTask>(
      ctl.config()("hrp4")("rightEfTask")("bodyName"), ctl.robots(), 0,
      ctl.config()("hrp4")("rightEfTask")("stiffness"), ctl.config()("hrp4")("rightEfTask")("weight"));

  std::vector<std::string> right_joints = {"CHEST_Y",   "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y",
                                           "R_ELBOW_P", "R_WRIST_Y",    "R_WRIST_P",    "R_WRIST_R"};
  rEfTaskPtr_->selectActiveJoints(ctl.solver(), right_joints);

  ctl.solver().addTask(rEfTaskPtr_);

  // Set the contacts between the feet and the ground
  ctl.solver().setContacts({{ctl.solver().robots(), 0, 2, "LeftFoot", "AllGround"},
                            {ctl.solver().robots(), 0, 2, "RightFoot", "AllGround"}});

  ctl.comTaskPtr = mc_tasks::MetaTaskLoader::load(ctl.solver(), ctl.config()("com"));
  ctl.solver().addTask(ctl.comTaskPtr);

  Eigen::Vector3d translation_offset;
  translation_offset = ctl.config()("states")("Prepare")("raiseHandOffset");

  // Align hand to vertical wall
  Eigen::Matrix3d desiredRotation;
  desiredRotation << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  rEfTaskPtr_->set_ef_pose(
      sva::PTransformd(desiredRotation, rEfTaskPtr_->get_ef_pose().translation() + translation_offset));

  run(ctlInput);
  //--------------------------- Joint velocity and torque jump
  bool debugTorque = ctl.config()("impact")("constraints")("jointTorque")("on");
  bool debugVelocity = ctl.config()("impact")("constraints")("jointVelocity")("on");
  if(debugTorque)
  {
    ctl.logger().addLogEntry("qp_boundTau_difference_lower", [&ctl]() { return ctl.boundTorqueJump_->getDiffLower(); });

    ctl.logger().addLogEntry("qp_boundTau_difference_upper", [&ctl]() { return ctl.boundTorqueJump_->getDiffUpper(); });

    ctl.logger().addLogEntry("qp_boundTau_delta_tau", [&ctl]() { return ctl.boundTorqueJump_->getDeltaTau(); });

    ctl.logger().addLogEntry("qp_boundTau_delta_tau_compare", [&ctl]() {
      return (Eigen::VectorXd)(ctl.boundTorqueJump_->getDeltaTau() - ctl.ecQpEstimatorPtr->getTauJump())
          .segment(6, ctl.ecQpEstimatorPtr->getDof() - 6);
    });
  }
  if(debugVelocity)
  {
    ctl.logger().addLogEntry("qp_boundJointVel_difference_lower",
                             [&ctl]() { return ctl.boundVelocityJump_->getDiffLower(); });
    ctl.logger().addLogEntry("qp_boundJointVel_difference_upper",
                             [&ctl]() { return ctl.boundVelocityJump_->getDiffUpper(); });
    ctl.logger().addLogEntry("qp_boundJointVel_delta_vel", [&ctl]() { return ctl.boundVelocityJump_->getDeltaVel(); });
    ctl.logger().addLogEntry("qp_boundJointVel_delta_vel_compare", [&ctl]() {
      return (Eigen::VectorXd)(ctl.boundVelocityJump_->getDeltaVel() - ctl.ecQpEstimatorPtr->getJointVelJump())
          .segment(6, ctl.ecQpEstimatorPtr->getDof() - 6);
    });
  }

  //--------------------------- CoP constraint:
  // Suppose the contact area is a 5 by 5 cm area:

  ctl.logger().addLogEntry("l_ankle_CoP_Constraint_sensor", [&ctl]() {
    Eigen::MatrixXd A_cop;
    A_cop.resize(4, 6);
    A_cop.setZero();

    A_cop(0, 5) = -0.10;
    A_cop(0, 1) = -1;
    A_cop(1, 5) = -0.10;
    A_cop(1, 1) = 1;
    A_cop(2, 5) = -0.05;
    A_cop(2, 0) = 1;
    A_cop(3, 5) = -0.05;
    A_cop(3, 0) = -1;

    auto sensorForce = ctl.robot().forceSensor("LeftFootForceSensor").wrench();
    // auto sensorForce = ctl.solver().desiredContactForce(ctl.getContact("LeftFoot"));
    Eigen::VectorXd result = A_cop * (sensorForce.vector());
    return result;
  });
  ctl.logger().addLogEntry("r_ankle_CoP_Constraint_sensor", [&ctl]() {
    Eigen::MatrixXd A_cop;
    A_cop.resize(4, 6);
    A_cop.setZero();

    A_cop(0, 5) = -0.10;
    A_cop(0, 1) = -1;
    A_cop(1, 5) = -0.10;
    A_cop(1, 1) = 1;
    A_cop(2, 5) = -0.05;
    A_cop(2, 0) = 1;
    A_cop(3, 5) = -0.05;
    A_cop(3, 0) = -1;

    auto sensorForce = ctl.robot().forceSensor("RightFootForceSensor").wrench();
    // auto sensorForce = ctl.solver().desiredContactForce(ctl.getContact("RightFoot"));
    Eigen::VectorXd result = A_cop * (sensorForce.vector());
    return result;
  });

  if(ctl.config()("impact")("constraints")("zmpWithImpulse")("on"))
  {

    ctl.logger().addLogEntry("ZMP_perturbation_pconstraint",
                             [&ctl]() { return ctl.zmpImpulse_->getZMP_perturbation(); });

    ctl.logger().addLogEntry("ZMP_sensor_constraint", [&ctl]() { return ctl.zmpImpulse_->getZMP_sensor(); });

    ctl.logger().addLogEntry("ZMP_constraint_difference", [&ctl]() {
      Eigen::VectorXd difference = ctl.zmpImpulse_->getZMP_constraint_difference();
      return difference;
    });

    ctl.logger().addLogEntry("ZMP_prediction_allforce_constraint",
                             [&ctl]() { return ctl.zmpImpulse_->getZMP_prediction_allforce(); });
    ctl.logger().addLogEntry("ZMP_prediction_feetforce_constraint",
                             [&ctl]() { return ctl.zmpImpulse_->getZMP_prediction_feetforce(); });

    ctl.logger().addLogEntry("ZMP_Constraint_test", [&ctl]() {
      Eigen::VectorXd result = ctl.zmpImpulse_->getA() * rbd::dofToVector(ctl.robot().mb(), ctl.robot().mbc().alphaD)
                               - ctl.zmpImpulse_->getb();
      return result;
    });
  }

  //-------------------------- zero slippage:
  //

  ctl.logger().addLogEntry("r_ankle_tangential_force", [&ctl]() {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d nullProjector = Eigen::MatrixXd::Identity(3, 3) - normal * normal.transpose();

    Eigen::Vector3d f_qp = (ctl.robot().forceSensor("RightFootForceSensor").force());
    return (nullProjector * f_qp).norm();
  });
  ctl.logger().addLogEntry("r_ankle_tangential_max_friction", [&ctl]() {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d projector = normal * normal.transpose();
    double mu = mc_rbdyn::Contact::defaultFriction;

    Eigen::Vector3d f_qp = (ctl.robot().forceSensor("RightFootForceSensor").force());
    return (projector * f_qp * mu).norm();
  });

  ctl.logger().addLogEntry("l_ankle_tangential_force", [&ctl]() {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d nullProjector = Eigen::MatrixXd::Identity(3, 3) - normal * normal.transpose();

    Eigen::Vector3d f_qp = (ctl.robot().forceSensor("LeftFootForceSensor").force());
    return (nullProjector * f_qp).norm();
  });
  ctl.logger().addLogEntry("l_ankle_tangential_max_friction", [&ctl]() {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d projector = normal * normal.transpose();
    double mu = mc_rbdyn::Contact::defaultFriction;

    Eigen::Vector3d f_qp = (ctl.robot().forceSensor("LeftFootForceSensor").force());
    return (projector * f_qp * mu).norm();
  });

  ctl.logger().addLogEntry("l_ankle_zero_slippage_sensor", [&ctl]() {
    double mu = mc_rbdyn::Contact::defaultFriction;
    Eigen::MatrixXd multiplier_;
    multiplier_.resize(2, 3);
    multiplier_.setZero();
    multiplier_(0, 0) = 1.0;
    multiplier_(0, 2) = -mu;
    multiplier_(1, 1) = 1.0;
    multiplier_(1, 2) = -mu;
    Eigen::Vector3d f_qp = (ctl.robot().forceSensor("LeftFootForceSensor").force());
    Eigen::Vector2d result = multiplier_ * f_qp;
    return result;
  });

  ctl.logger().addLogEntry("r_ankle_zero_slippage_sensor", [&ctl]() {
    double mu = mc_rbdyn::Contact::defaultFriction;
    Eigen::MatrixXd multiplier_;
    multiplier_.resize(2, 3);
    multiplier_.setZero();
    multiplier_(0, 0) = 1.0;
    multiplier_(0, 2) = -mu;
    multiplier_(1, 1) = 1.0;
    multiplier_(1, 2) = -mu;
    Eigen::Vector3d f_qp = (ctl.robot().forceSensor("RightFootForceSensor").force());
    Eigen::Vector2d result = multiplier_ * f_qp;
    return result;
  });
}

bool PrepareHandState::run(mc_control::fsm::Controller & ctlInput)
{

  auto & ctl = static_cast<Controller &>(ctlInput);
  // std::cout << "PrepareState: the right ef error is: " << rEfTaskPtr_->eval().norm() << std::endl;

  ctl.setOsd()->update();

  if(ctl.config()("lcp")("on"))
  {
    if(ctl.lcpSolverPtr->getDim() == 1)
    {
      std::map<std::string, Eigen::Vector3d> contactSurfaceNormals;
      Eigen::Vector3d groundSurfaceNormal;
      groundSurfaceNormal << 0, 0, 1;

      sva::PTransformd X_0_lSole = ctl.robot().bodyPosW("l_ankle");
      contactSurfaceNormals["l_ankle"] = X_0_lSole.rotation() * groundSurfaceNormal + X_0_lSole.translation();
      sva::PTransformd X_0_rSole = ctl.robot().bodyPosW("r_ankle");
      contactSurfaceNormals["r_ankle"] = X_0_rSole.rotation() * groundSurfaceNormal + X_0_rSole.translation();

      ctl.lcpSolverPtr->update(contactSurfaceNormals);
    }
    else
    {
      ctl.lcpSolverPtr->update();
    }
  }

  if(ctl.config()("qpEstimator")("on"))
  {
    ctl.qpEstimatorPtr->update();
    ctl.osdQpEstimatorPtr->update();
    ctl.jsdQpEstimatorPtr->update();
    ctl.ecQpEstimatorPtr->update();
  }
  if(rEfTaskPtr_->eval().norm() <= efThreshold_)
  {
    // Output the transition signal such that we can move on according to the transitions
    output("RightHandReady");
    return true;
  }

  return false;
}

void PrepareHandState::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Controller &>(ctl_);
  ctl.solver().removeTask(rEfTaskPtr_);
}

EXPORT_SINGLE_STATE("PrepareHand", PrepareHandState)
