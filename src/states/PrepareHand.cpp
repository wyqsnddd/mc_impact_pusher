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

  std::vector<std::string> right_joints = {"CHEST_Y", "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",
                                           "R_WRIST_Y",    "R_WRIST_P",    "R_WRIST_R"};
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
  desiredRotation << 0,1,0,
                     1,0,0,
                     0,0,-1;

  rEfTaskPtr_->set_ef_pose(sva::PTransformd(desiredRotation, rEfTaskPtr_->get_ef_pose().translation()+translation_offset));

  run(ctlInput);

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
 
  if(ctl.config()("impact")("constraints")("zmpWithImpulse")("on")){

    ctl.logger().addLogEntry("ZMP_perturbation_zmpconstraint", [&ctl]() {
      return ctl.zmpImpulse_->getZMP_perturbation();
    });
    ctl.logger().addLogEntry("ZMP_sensor_zmpconstraint", [&ctl]() {
      return ctl.zmpImpulse_->getZMP_sensor();
    });
    ctl.logger().addLogEntry("ZMP_constraint_difference", [&ctl]() {
      Eigen::VectorXd difference = ctl.zmpImpulse_->getZMP_constraint_difference();
      return difference;
    });


    ctl.logger().addLogEntry("ZMP_prediction_zmpconstraint", [&ctl]() {
		 /*
    Eigen::MatrixXd sumJac;
    Eigen::Vector6d sumWrench;
    ctl.zmpImpulse_->getComItems(sumJac, sumWrench);

    Eigen::MatrixXd A_;  
    Eigen::Vector4d b_;
    Eigen::VectorXd alpha_;
    int nDof = ctl.miPredictorPtr->getRobot().mb().nrDof();
    A_.resize(4, nDof);
    A_.setZero();
    b_.setZero();
    A_ = (ctl.timeStep/ ctl.timeStep) * ctl.zmpImpulse_->getZMP()* sumJac;
  // std::cout<<"size of A_: "<<A_.rows()<<", "<<A_.cols()<<std::endl;
    rbd::paramToVector(ctl.miPredictorPtr->getRobot().mbc().alpha, alpha_);

  // std::cout<<"size of alpha_"<<alpha_.rows()<<std::endl;
    b_ = -ctl.zmpImpulse_->getZMP()*(sumWrench 
		    + sumJac* alpha_ / ctl.timeStep);

    //Eigen::VectorXd result = A_*rbd::dofToVector(robot().mb(), robot().mbc().alphaD) - b_;
    */
/*
    Eigen::MatrixXd sumJac;
    Eigen::Vector6d sumWrench;
    ctl.zmpImpulse_->getComItems(sumJac, sumWrench);
    Eigen::VectorXd temp =
        (rbd::dofToVector(ctl.robot().mb(), ctl.robot().mbc().alpha)
         + rbd::dofToVector(ctl.robot().mb(), ctl.robot().mbc().alphaD) * ctl.miPredictorPtr->getImpactDuration_());

    Eigen::VectorXd result = (1/ctl.miPredictorPtr->getImpactDuration_())*sumJac*temp; 

    double denominator = ( sumWrench(5) + result(5));

    Eigen::Vector3d tempZMP = Eigen::Vector3d::Zero();
    tempZMP.x() = - ( sumWrench(1) + result(1))/denominator;
    tempZMP.y() =   ( sumWrench(0) + result(0))/denominator;
    */
    return ctl.zmpImpulse_->getZMP_prediction();
    });


    ctl.logger().addLogEntry("ZMP_Constraint_test", [&ctl]() {
		 /*
    Eigen::MatrixXd sumJac;
    Eigen::Vector6d sumWrench;
    ctl.zmpImpulse_->getComItems(sumJac, sumWrench);

    Eigen::MatrixXd A_;
    Eigen::Vector4d b_;
    Eigen::VectorXd alpha_;
    int nDof = ctl.miPredictorPtr->getRobot().mb().nrDof();
    A_.resize(4, nDof);
    A_.setZero();
    b_.setZero();
    A_ = (ctl.timeStep/ ctl.timeStep) * ctl.zmpImpulse_->getZMP()* sumJac;
  // std::cout<<"size of A_: "<<A_.rows()<<", "<<A_.cols()<<std::endl;
    rbd::paramToVector(ctl.miPredictorPtr->getRobot().mbc().alpha, alpha_);

  // std::cout<<"size of alpha_"<<alpha_.rows()<<std::endl;
    b_ = -ctl.zmpImpulse_->getZMP()*(sumWrench
		    + sumJac* alpha_ / ctl.timeStep);

    //Eigen::VectorXd result = A_*rbd::dofToVector(robot().mb(), robot().mbc().alphaD) - b_;
    */
    Eigen::VectorXd result = ctl.zmpImpulse_->getA()*rbd::dofToVector(ctl.robot().mb(), ctl.robot().mbc().alphaD) - ctl.zmpImpulse_->getb();
    return result;
  });
  }

  ctl.logger().addLogEntry("l_ankle_CoP_Constraint", [&ctl]() {
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

    sva::PTransformd X_0_lSole = ctl.robot().bodyPosW("l_sole");
    sva::PTransformd X_0_rSole = ctl.robot().bodyPosW("r_sole");
    sva::PTransformd X_0_ee = ctl.robot().bodyPosW("r_wrist");

    sva::PTransformd X_rSole_lSole = X_0_lSole * X_0_rSole.inv();
    sva::ForceVecd r_impulse =
        X_rSole_lSole.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), ctl.miPredictorPtr->getImpulsiveForce("r_sole")));
    sva::ForceVecd l_impulse = sva::ForceVecd(Eigen::Vector3d::Zero(), ctl.miPredictorPtr->getImpulsiveForce("l_sole"));

    ctl.miPredictorPtr->getImpulsiveForce("l_sole");
    sva::PTransformd X_ee_lSole = X_0_lSole * X_0_ee.inv();
    sva::ForceVecd f_ee =
        X_ee_lSole.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), ctl.miPredictorPtr->getImpulsiveForce()));

    auto sensorForce = ctl.robot().forceSensor("LeftFootForceSensor").wrench();

    // auto sensorForce = ctl.solver().desiredContactForce(ctl.getContact("LeftFoot"));
    // Eigen::VectorXd result = A_cop * (sensorForce.vector() + r_impulse.vector() + l_impulse.vector() + f_ee.vector());
    Eigen::VectorXd result = A_cop * (sensorForce.vector() + l_impulse.vector());

    return result;
  });

  ctl.logger().addLogEntry("r_ankle_CoP_Constraint", [&ctl]() {
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

    sva::PTransformd X_0_lSole = ctl.robot().bodyPosW("l_sole");
    sva::PTransformd X_0_rSole = ctl.robot().bodyPosW("r_sole");
    sva::PTransformd X_0_ee = ctl.robot().bodyPosW("r_wrist");

    sva::PTransformd X_lSole_rSole = X_0_rSole * X_0_lSole.inv();

    sva::ForceVecd l_impulse =
        X_lSole_rSole.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), ctl.miPredictorPtr->getImpulsiveForce("l_sole")));

    sva::ForceVecd r_impulse = sva::ForceVecd(Eigen::Vector3d::Zero(), ctl.miPredictorPtr->getImpulsiveForce("r_sole"));

    sva::PTransformd X_ee_rSole = X_0_rSole * X_0_ee.inv();
    sva::ForceVecd f_ee =
        X_ee_rSole.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), ctl.miPredictorPtr->getImpulsiveForce()));

    auto sensorForce = ctl.robot().forceSensor("RightFootForceSensor").wrench();

    // auto sensorForce = ctl.solver().desiredContactForce(ctl.getContact("RightFoot"));
    // Eigen::VectorXd result = A_cop * (sensorForce.vector() + r_impulse.vector() + l_impulse.vector() + f_ee.vector());
    Eigen::VectorXd result = A_cop * (sensorForce.vector() + r_impulse.vector());

    return result;
  });

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
  ctl.logger().addLogEntry("l_ankle_zero_slippage", [&ctl]() {
    // auto tempContact  = ctl.getContact("LeftFoot");
    // Eigen::Vector3d normal =
    // tempContact.X_0_r2s(ctl.solver().robots().robot(tempContact.r2Index())).rotation().row(2).transpose();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double mu = mc_rbdyn::Contact::defaultFriction;
    Eigen::Matrix3d multiplier = (Eigen::MatrixXd::Identity(3, 3) - (1 + mu) * normal * normal.transpose());
    // Eigen::Vector3d f_qp = ctl.solver().desiredContactForce(ctl.getContact("LeftFoot")).force();
    Eigen::Vector3d f_qp = (ctl.robot().forceSensor("LeftFootForceSensor").force());
    // Eigen::Vector3d result = multiplier * (f_qp + ctl.miPredictorPtr->getImpulsiveForce("l_sole"));
    // Eigen::Vector3d result = multiplier * (f_qp);
    // return result;
    Eigen::MatrixXd multiplier_;
    multiplier_.resize(2, 3);
    multiplier_.setZero();
    multiplier_(0, 0) = 1.0;
    multiplier_(0, 2) = -mu;
    multiplier_(1, 1) = 1.0;
    multiplier_(1, 2) = -mu;

    Eigen::Vector2d result = multiplier_ * (f_qp - ctl.miPredictorPtr->getImpulsiveForce("l_sole"));
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

  ctl.logger().addLogEntry("r_ankle_zero_slippage", [&ctl]() {
    auto & tempContact = ctl.getContact("RightFoot");
    // Eigen::Vector3d normal =
    // tempContact.X_0_r2s(ctl.solver().robots().robot(tempContact.r2Index())).rotation().row(2).transpose();
    auto cid = tempContact.contactId(ctl.solver().robots());
    // ContactWrenchMatrixToLambdaMatrix transformer(solver, cid);
    Eigen::Vector3d normal = tempContact.X_0_r2s(ctl.solver().robots()).rotation().row(2).transpose();

    // Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double mu = mc_rbdyn::Contact::defaultFriction;
    Eigen::Matrix3d multiplier = (Eigen::MatrixXd::Identity(3, 3) - (1 + mu) * normal * normal.transpose());
    // Eigen::Vector3d f_qp = ctl.solver().desiredContactForce(ctl.getContact("RightFoot")).force();
    Eigen::Vector3d f_qp = (ctl.robot().forceSensor("RightFootForceSensor").force());
    // Eigen::Vector3d result = multiplier * (f_qp + ctl.miPredictorPtr->getImpulsiveForce("r_sole"));
    Eigen::MatrixXd multiplier_;
    multiplier_.resize(2, 3);
    multiplier_.setZero();
    multiplier_(0, 0) = 1.0;
    multiplier_(0, 2) = -mu;
    multiplier_(1, 1) = 1.0;
    multiplier_(1, 2) = -mu;

    Eigen::Vector2d result = multiplier_ * (f_qp - ctl.miPredictorPtr->getImpulsiveForce("r_sole"));
    return result;
  });
}

bool PrepareHandState::run(mc_control::fsm::Controller & ctlInput)
{

  auto & ctl = static_cast<Controller &>(ctlInput);
  //std::cout << "PrepareState: the right ef error is: " << rEfTaskPtr_->eval().norm() << std::endl;

  ctl.miOsdPtr->update();

  Eigen::Vector3d surfaceNormal;
  surfaceNormal << 1, 0, 0;

  // Assert:

  sva::PTransformd X_0_ee = ctl.robot().bodyPosW("r_wrist");
  // ctl.miPredictorPtr->run(surfaceNormal);
  ctl.miPredictorPtr->run(X_0_ee.rotation() * surfaceNormal + X_0_ee.translation());

  std::map<std::string, Eigen::Vector3d> surfaceNormals;
  surfaceNormals["r_wrist"] =  X_0_ee.rotation() * surfaceNormal + X_0_ee.translation();
  ctl.multiImpactPredictorPtr->run(surfaceNormals);

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
  // ctl.miPredictorPtr->resetDataStructure();
}

EXPORT_SINGLE_STATE("PrepareHand", PrepareHandState)
