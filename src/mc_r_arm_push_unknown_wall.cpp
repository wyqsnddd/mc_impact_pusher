#include "mc_r_arm_push_unknown_wall.h"

Controller::Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf)
: mc_control::fsm::Controller(rm, dt, conf)
{

  forceThreshold = config()("contact_detection")("ForceThreshold");
  impactIndicator_ = -20.0;
  std::cout << "Kinematics and Dynamics constraints are created." << std::endl;
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(dynamicsConstraint);

  logger().addLogEntry("CoP_LeftFoot_World", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.copW("LeftFoot");
  });
  logger().addLogEntry("CoP_RightFoot_World", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.copW("RightFoot");
  });

  logger().addLogEntry("ZMP_World", [this]() {
    auto & robot = this->realRobots().robot();
    Eigen::Vector3d zmp;
    try
    {
      zmp = robot.zmp({"LeftFootForceSensor", "RightFootForceSensor"}, Eigen::Vector3d::Zero(),
                      Eigen::Vector3d{0., 0., 1.}, 100);
      zmp.z() = 0;
    }
    catch(...)
    {
      zmp.setZero();
    }
    return zmp;
  });
  logger().addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });

  gui()->addElement({"Forces"},
                    mc_rtc::gui::Force("LeftCoPForce_real", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                                       [this]() { return realRobot().surfaceWrench("LeftFoot"); },
                                       [this]() {
                                         auto & robot = realRobot();
                                         Eigen::Vector3d cop = robot.copW("LeftFoot");
                                         const sva::PTransformd X_0_s = robot.surface("LeftFoot").X_0_s(robot);
                                         sva::PTransformd surface(X_0_s.rotation(), cop);
                                         return surface;
                                       }),
                    mc_rtc::gui::Force("RightCoPForce_real", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                                       [this]() { return realRobot().surfaceWrench("RightFoot"); },
                                       [this]() {
                                         auto & robot = this->realRobots().robot();
                                         Eigen::Vector3d cop = robot.copW("RightFoot");
                                         const sva::PTransformd X_0_s = robot.surface("RightFoot").X_0_s(robot);
                                         sva::PTransformd surface(X_0_s.rotation(), cop);
                                         return surface;
                                       }),
                    mc_rtc::gui::Point3D("ZMP_real", [this]() {
                      auto & robot = this->realRobots().robot();
                      Eigen::Vector3d zmp;
                      try
                      {
                        zmp = robot.zmp({"LeftFootForceSensor", "RightFootForceSensor"}, Eigen::Vector3d::Zero(),
                                        Eigen::Vector3d{0., 0., 1.}, 100);
                        zmp.z() = 0;
                      }
                      catch(...)
                      {
                        zmp.setZero();
                      }
                      return zmp;
                    }));

  std::string impactBodyString(config()("impact")("estimation")("impactBodyName"));
  miPredictorPtr.reset(new mi_impactPredictor(
      robot(), impactBodyString, config()("impact")("estimation")("useLinearJacobian"),
      timeStep, // This is the controller time step.
      // solver().dt(),
      // config()("impact")("estimation")("delta_dt"),
      config()("impact")("estimation")("coeFrictionDeduction"), config()("impact")("estimation")("coeRestitution")));

  std::vector<std::string> eeNameVector = config()("impact")("estimation")("end-effectors");

  miPredictorPtr->initializeDataStructure(static_cast<int>(eeNameVector.size()));
  miPredictorPtr->resetDataStructure();

  for(auto index = eeNameVector.begin(); index != eeNameVector.end(); ++index)
  {

    // std::string eeName = ctl.config()("states")("Contact")("end-effectors")(index);
    std::cout << "Processing end-effector: " << *index << std::endl;

    if(!miPredictorPtr->addEndeffector(*index))
    {
      throw std::runtime_error("Impact predictor failed to add endeffector!");
    }
    else
    {
      std::cout << "End-effector: " << *index << "is added to the impact-predictor. " << std::endl;
    }
  }

  std::cout << "Operational space dynamics Predictor is created." << std::endl;

  miPredictorPtr->setContact("l_sole");
  miPredictorPtr->setContact("r_sole");

  //----------------------------------- Jump on different branch:
  logger().addLogEntry("ee_Vel_impact_jump", [this]() {
    // Eigen::VectorXd eeVelJump =
    return miPredictorPtr->getEeVelocityJump();
  });
  //------------------------------ Test Jacobian_delta_f
  logger().addLogEntry("test_left_Jacobian_f", [this]() {
    Eigen::VectorXd temp =
        (rbd::dofToVector(robot().mb(), robot().mbc().alpha)
         + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * miPredictorPtr->getImpactDuration_());
    Eigen::Vector3d delta_f =
        ((1 / miPredictorPtr->getImpactDuration_()) * miPredictorPtr->getJacobianDeltaF("l_sole") * temp);

    return delta_f;
  });
  logger().addLogEntry("test_right_Jacobian_f", [this]() {
    Eigen::VectorXd temp =
        (rbd::dofToVector(robot().mb(), robot().mbc().alpha)
         + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * miPredictorPtr->getImpactDuration_());
    Eigen::Vector3d delta_f =
        ((1 / miPredictorPtr->getImpactDuration_()) * miPredictorPtr->getJacobianDeltaF("r_sole") * temp);

    return delta_f;
  });

  // ---------------------------- constructor: add modified constraints
  logger().addLogEntry("ee_Vel_impact_jump", [this]() {
    // Eigen::VectorXd eeVelJump =
    return miPredictorPtr->getEeVelocityJump();
  });
  logger().addLogEntry("ee_Vel", [this]() {
    return realRobots().robot().mbc().bodyVelW[realRobots().robot().mb().bodyIndexByName("r_wrist")].linear();
  });

  logger().addLogEntry("ee_dq", [this]() { return miPredictorPtr->getBranchJointVelJump("r_wrist"); });
  logger().addLogEntry("ee_dtau", [this]() { return miPredictorPtr->getBranchTauJump("r_wrist"); });

  logger().addLogEntry("l_ankle_Vel_impact_jump", [this]() { return miPredictorPtr->getEeVelocityJump("l_sole"); });

  logger().addLogEntry("l_ankle_Vel", [this]() {
    return realRobots().robot().mbc().bodyVelW[realRobots().robot().mb().bodyIndexByName("l_sole")].linear();
  });

  logger().addLogEntry("l_ankle_dq", [this]() { return miPredictorPtr->getBranchJointVelJump("l_sole"); });
  logger().addLogEntry("l_ankle_dtau", [this]() { return miPredictorPtr->getBranchTauJump("l_sole"); });

  logger().addLogEntry("r_ankle_Vel_impact_jump", [this]() { return miPredictorPtr->getEeVelocityJump("r_sole"); });

  logger().addLogEntry("r_ankle_Vel", [this]() {
    return realRobots().robot().mbc().bodyVelW[realRobots().robot().mb().bodyIndexByName("r_sole")].linear();
  });

  logger().addLogEntry("r_ankle_dq", [this]() { return miPredictorPtr->getBranchJointVelJump("r_sole"); });
  logger().addLogEntry("r_ankle_dtau", [this]() { return miPredictorPtr->getBranchTauJump("r_sole"); });

  logger().addLogEntry("ee_impact_impulse", [this]() { return miPredictorPtr->getImpulsiveForce(); });
  logger().addLogEntry("ee_impact_impulse_COM_test", [this]() {
    sva::PTransformd X_0_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_0_ee = robot().bodyPosW("r_wrist");
    sva::PTransformd X_ee_CoM = X_0_CoM * X_0_ee.inv();

    Eigen::VectorXd f_ee =
        X_ee_CoM.dualMatrix() * sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce()).vector();
    return f_ee;
  });

  logger().addLogEntry("ee_impact_impulse_COM", [this]() {
    sva::PTransformd X_0_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_0_ee = robot().bodyPosW("r_wrist");
    sva::PTransformd X_ee_CoM = X_0_CoM * X_0_ee.inv();

    sva::ForceVecd f_ee =
        X_ee_CoM.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce()));
    return f_ee;
  });

  logger().addLogEntry("l_ankle_predict_impact_impulse",
                       [this]() { return miPredictorPtr->getImpulsiveForce("l_sole"); });

  logger().addLogEntry("r_ankle_predict_impact_impulse",
                       [this]() { return miPredictorPtr->getImpulsiveForce("r_sole"); });
  logger().addLogEntry(
      "q_position", [this]() { return rbd::paramToVector(realRobots().robot().mb(), realRobots().robot().mbc().q); });

  logger().addLogEntry(
      "q_vel", [this]() { return rbd::dofToVector(realRobots().robot().mb(), realRobots().robot().mbc().alpha); });

  logger().addLogEntry(
      "q_acc", [this]() { return rbd::dofToVector(realRobots().robot().mb(), realRobots().robot().mbc().alphaD); });

  // logger().addLogEntry("tau", [this]() { return rbd::dofToVector(realRobots().robot().mb(),
  // realRobots().robot().mbc().jointTorque); });
  logger().addLogEntry("tau_QP", [this]() { return rbd::dofToVector(robot().mb(), robot().mbc().jointTorque); });

  logger().addLogEntry("delta_q_vel", [this]() { return miPredictorPtr->getJointVelocityJump(); });

  logger().addLogEntry("delta_tau", [this]() { return miPredictorPtr->getTauJump(); });

  logger().addLogEntry("test_delta_tau_norm", [this]() { return miPredictorPtr->getTauJump().norm(); });

  logger().addLogEntry("test_delta_tau_jacobian", [this]() {
    Eigen::VectorXd temp =
        (rbd::dofToVector(robot().mb(), robot().mbc().alpha)
         + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * miPredictorPtr->getImpactDuration_());
    return ((1 / miPredictorPtr->getImpactDuration_()) * miPredictorPtr->getJacobianDeltaTau() * temp).norm();
  });

  logger().addLogEntry("diff_delta_tau", [this]() {
    Eigen::VectorXd temp =
        (rbd::dofToVector(robot().mb(), robot().mbc().alpha)
         + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * miPredictorPtr->getImpactDuration_());
    return ((1 / miPredictorPtr->getImpactDuration_()) * miPredictorPtr->getJacobianDeltaTau() * temp
            - rbd::dofToVector(robot().mb(), robot().mbc().jointTorque))
        .norm();
  });

  logger().addLogEntry("delta_alpha_norm", [this]() { return miPredictorPtr->getJointVelocityJump().norm(); });

  logger().addLogEntry("diff_delta_alpha", [this]() {
    Eigen::VectorXd temp =
        (rbd::dofToVector(robot().mb(), robot().mbc().alpha)
         + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * miPredictorPtr->getImpactDuration_());
    return (miPredictorPtr->getJacobianDeltaAlpha() * temp - miPredictorPtr->getJointVelocityJump()).norm();
  });

  //----------------------------- check the joint velocity jump
  logger().addLogEntry("test_delta_q_vel_upper-diff-direct", [this]() {
    Eigen::VectorXd result =
        rbd::dofToVector(miPredictorPtr->getRobot().mb(), miPredictorPtr->getRobot().vu())
        - (miPredictorPtr->getJointVelocityJump() + rbd::dofToVector(robot().mb(), robot().mbc().alpha));

    return result;
  });
  logger().addLogEntry("test_delta_q_vel_lower-diff-direct", [this]() {
    Eigen::VectorXd result =
        (miPredictorPtr->getJointVelocityJump() + rbd::dofToVector(robot().mb(), robot().mbc().alpha))
        - rbd::dofToVector(miPredictorPtr->getRobot().mb(), miPredictorPtr->getRobot().vl());

    return result;
  });

  logger().addLogEntry("test_delta_q_vel_upper-diff", [this]() {
    Eigen::VectorXd upper =
        rbd::dofToVector(miPredictorPtr->getRobot().mb(), miPredictorPtr->getRobot().vu())
        - miPredictorPtr->getJacobianDeltaAlpha() * rbd::dofToVector(robot().mb(), robot().mbc().alpha)
        - rbd::dofToVector(robot().mb(), robot().mbc().alpha);
    Eigen::VectorXd middle =
        miPredictorPtr->getJacobianDeltaAlpha() * rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * timeStep;
    //* miPredictorPtr->getImpactDuration_();
    Eigen::VectorXd result = upper - middle;
    return result;
  });
  logger().addLogEntry("test_delta_q_vel_lower-diff", [this]() {
    Eigen::VectorXd lower =
        rbd::dofToVector(miPredictorPtr->getRobot().mb(), miPredictorPtr->getRobot().vl())
        - miPredictorPtr->getJacobianDeltaAlpha() * rbd::dofToVector(robot().mb(), robot().mbc().alpha)
        - rbd::dofToVector(robot().mb(), robot().mbc().alpha);

    Eigen::VectorXd middle =
        miPredictorPtr->getJacobianDeltaAlpha() * rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * timeStep;
    // * miPredictorPtr->getImpactDuration_();
    Eigen::VectorXd result = middle - lower;
    return result;
  });

  logger().addLogEntry("test_delta_q_vel_upper", [this]() {
    Eigen::VectorXd upper =
        rbd::dofToVector(miPredictorPtr->getRobot().mb(), miPredictorPtr->getRobot().vu())
        - miPredictorPtr->getJacobianDeltaAlpha() * rbd::dofToVector(robot().mb(), robot().mbc().alpha)
        - rbd::dofToVector(robot().mb(), robot().mbc().alpha);
    return upper;
  });
  logger().addLogEntry("test_delta_q_vel_lower", [this]() {
    Eigen::VectorXd lower =
        rbd::dofToVector(miPredictorPtr->getRobot().mb(), miPredictorPtr->getRobot().vl())
        - miPredictorPtr->getJacobianDeltaAlpha() * rbd::dofToVector(robot().mb(), robot().mbc().alpha)
        - rbd::dofToVector(robot().mb(), robot().mbc().alpha);
    return lower;
  });
  logger().addLogEntry("test_delta_q_vel_middle", [this]() {
    Eigen::VectorXd middle = miPredictorPtr->getJacobianDeltaAlpha()
                             * rbd::dofToVector(robot().mb(), robot().mbc().alphaD)
                             * miPredictorPtr->getImpactDuration_();
    return middle;
  });

  //----------------------------- check the impulsive induced jump to COP

  logger().addLogEntry("CoP_LeftFoot_local", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.cop("LeftFoot");
  });
  logger().addLogEntry("CoP_RightFoot_local", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.cop("RightFoot");
  });
  logger().addLogEntry("CoP_LeftFoot_impulse_jump_local_new", [this]() {
    Eigen::Vector3d impulse = miPredictorPtr->getImpulsiveForce("l_sole");
    sva::ForceVecd wrench = robot().forceSensor("LeftFootForceSensor").wrench();
    double denominator = (impulse.z() + wrench.force().z()) * wrench.force().z();

    Eigen::Vector3d tempZMP;
    tempZMP.x() = (impulse.z() * wrench.moment().y()) / denominator;
    tempZMP.y() = -(impulse.x() * wrench.moment().x()) / denominator;
    tempZMP.z() = 0;
    return tempZMP;
  });
  logger().addLogEntry("CoP_RightFoot_impulse_jump_local_new", [this]() {
    Eigen::Vector3d impulse = miPredictorPtr->getImpulsiveForce("r_sole");
    sva::ForceVecd wrench = robot().forceSensor("RightFootForceSensor").wrench();
    double denominator = (impulse.z() + wrench.force().z()) * wrench.force().z();

    Eigen::Vector3d tempZMP;
    tempZMP.x() = (impulse.z() * wrench.moment().y()) / denominator;
    tempZMP.y() = -(impulse.x() * wrench.moment().x()) / denominator;
    tempZMP.z() = 0;
    return tempZMP;
  });

  logger().addLogEntry("CoP_LeftFoot_impulse_jump_local", [this]() {
    sva::PTransformd X_0_lSole = robot().bodyPosW("l_sole");
    sva::PTransformd X_0_rSole = robot().bodyPosW("r_sole");
    sva::PTransformd X_0_ee = robot().bodyPosW("r_wrist");

    sva::PTransformd X_rSole_lSole = X_0_lSole * X_0_rSole.inv();
    sva::ForceVecd r_impulse =
        X_rSole_lSole.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce("r_sole")));
    Eigen::Vector3d l_impulse = miPredictorPtr->getImpulsiveForce("l_sole");
    sva::PTransformd X_ee_lSole = X_0_lSole * X_0_ee.inv();
    sva::ForceVecd f_ee =
        X_ee_lSole.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce()));

    double denominator = (f_ee.force().z() + r_impulse.force().z() + l_impulse.z()
                          + robot().forceSensor("LeftFootForceSensor").force().z());

    Eigen::Vector3d tempZMP;
    tempZMP.x() = -(f_ee.moment().y() + r_impulse.moment().y()) / denominator;
    tempZMP.y() = (f_ee.moment().x() + r_impulse.moment().x()) / denominator;
    tempZMP.z() = 0;
    return tempZMP;
  });

  logger().addLogEntry("CoP_RightFoot_impulse_jump", [this]() {
    sva::PTransformd X_0_lSole = robot().bodyPosW("l_sole");
    sva::PTransformd X_0_rSole = robot().bodyPosW("r_sole");
    sva::PTransformd X_0_ee = robot().bodyPosW("r_wrist");

    sva::PTransformd X_lSole_rSole = X_0_rSole * X_0_lSole.inv();
    sva::ForceVecd l_impulse =
        X_lSole_rSole.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce("l_sole")));

    Eigen::Vector3d r_impulse = miPredictorPtr->getImpulsiveForce("r_sole");
    sva::PTransformd X_ee_rSole = X_0_rSole * X_0_ee.inv();
    sva::ForceVecd f_ee =
        X_ee_rSole.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce()));

    double denominator = (f_ee.force().z() + l_impulse.force().z() + r_impulse.z()
                          + robot().forceSensor("RightFootForceSensor").force().z());

    Eigen::Vector3d tempZMP;
    tempZMP.x() = -(f_ee.moment().y() + l_impulse.moment().y()) / denominator;
    tempZMP.y() = (f_ee.moment().x() + l_impulse.moment().x()) / denominator;
    tempZMP.z() = 0;
    return tempZMP;
  });
  //----------------------------- check the impulsive joint torque

  logger().addLogEntry("test_delta_tau_upper", [this]() {
    Eigen::VectorXd upper = 5.0 * rbd::dofToVector(miPredictorPtr->getRobot().mb(), miPredictorPtr->getRobot().tu())
                            - (1 / miPredictorPtr->getImpactDuration_()) * miPredictorPtr->getJacobianDeltaTau()
                                  * rbd::dofToVector(robot().mb(), robot().mbc().alpha);
    return upper;
  });

  logger().addLogEntry("test_delta_tau_lower", [this]() {
    Eigen::VectorXd lower = 5.0 * rbd::dofToVector(miPredictorPtr->getRobot().mb(), miPredictorPtr->getRobot().tl())
                            - (1 / miPredictorPtr->getImpactDuration_()) * miPredictorPtr->getJacobianDeltaTau()
                                  * rbd::dofToVector(robot().mb(), robot().mbc().alpha);
    return lower;
  });

  logger().addLogEntry("test_delta_tau_middle", [this]() {
    Eigen::VectorXd middle =
        miPredictorPtr->getJacobianDeltaTau() * rbd::dofToVector(robot().mb(), robot().mbc().alphaD);
    return middle;
  });
  //----------------------------- check the perturbed ZMP
  /*
   logger().addLogEntry("ee_predict_acc_force", [this]() {
      return  miPredictorPtr->getAccForce("r_wrist");
   });

   logger().addLogEntry("l_ankle_predict_OSD_force", [this]() {
       Eigen::VectorXd force = miPredictorPtr->getOsdForce("l_sole");
       return force;
   });


  logger().addLogEntry("l_ankle_predict_acc_force", [this]() {
      return  miPredictorPtr->getAccForce("l_sole");
   });

  logger().addLogEntry("r_ankle_predict_OSD_force", [this]() {
      Eigen::VectorXd force = miPredictorPtr->getOsdForce("r_sole");
      return force;
   });

 logger().addLogEntry("r_ankle_predict_acc_force", [this]() {
     return miPredictorPtr->getAccForce("r_sole");
   });
 */

  //----------------------------------------
  logger().addLogEntry("l_ankle_predict_impact_impulse_COM", [this]() {
    sva::PTransformd X_b_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_b_lSole = robot().surface("LeftFoot").X_0_s(robot());
    sva::PTransformd X_lSole_CoM = X_b_CoM * X_b_lSole.inv();

    sva::ForceVecd temp =
        X_lSole_CoM.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce("l_sole")));
    return temp;
  });

  logger().addLogEntry("r_ankle_predict_impact_impulse_COM", [this]() {
    sva::PTransformd X_b_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_b_rSole = robot().surface("RightFoot").X_0_s(robot());
    sva::PTransformd X_rSole_CoM = X_b_CoM * X_b_rSole.inv();

    sva::ForceVecd temp =
        X_rSole_CoM.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce("r_sole")));
    return temp;
  });

  logger().addLogEntry("ZMP_QP", [this]() {
    sva::PTransformd X_0_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_0_rSole = robot().bodyPosW("r_sole");
    sva::PTransformd X_rSole_CoM = X_0_CoM * X_0_rSole.inv();
    sva::PTransformd X_0_lSole = robot().bodyPosW("l_sole");
    sva::PTransformd X_lSole_CoM = X_0_CoM * X_0_lSole.inv();

    sva::ForceVecd F_r_qp = X_rSole_CoM.dualMul(solver().desiredContactForce(getContact("RightFoot")));

    sva::ForceVecd F_l_qp = X_lSole_CoM.dualMul(solver().desiredContactForce(getContact("LeftFoot")));

    double denominator = (F_l_qp.force().z() + F_r_qp.force().z());
    Eigen::Vector3d tempZMP;
    tempZMP.x() = -(F_r_qp.force().y() + F_l_qp.force().y()) / denominator;
    tempZMP.y() = -(F_r_qp.force().x() + F_l_qp.force().x()) / denominator;

    return tempZMP;
  });

  logger().addLogEntry("ZMP_Constraint", [this]() {
    sva::PTransformd X_0_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_0_rSole = robot().bodyPosW("r_sole");
    sva::PTransformd X_rSole_CoM = X_0_CoM * X_0_rSole.inv();

    sva::ForceVecd F_r_sole = miPredictorPtr->getImpulsiveForceCOM("r_sole");
    /*
    sva::ForceVecd F_r_sole =
        X_rSole_CoM.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce("r_sole")));
*/
    sva::PTransformd X_0_lSole = robot().bodyPosW("l_sole");
    sva::PTransformd X_lSole_CoM = X_0_CoM * X_0_lSole.inv();

    sva::ForceVecd F_l_sole = miPredictorPtr->getImpulsiveForceCOM("l_sole");
    /*
    sva::ForceVecd F_l_sole =
        X_lSole_CoM.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce("l_sole")));
*/
    sva::PTransformd X_0_ee = robot().bodyPosW("r_wrist");
    sva::PTransformd X_ee_CoM = X_0_CoM * X_0_ee.inv();

    sva::ForceVecd F_ee = miPredictorPtr->getImpulsiveForceCOM();
    /*
  sva::ForceVecd F_ee =
      X_ee_CoM.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce()));
*/
    // sva::ForceVecd f_ee = miPredictorPtr->getImpulsiveForceCOM();

    sva::ForceVecd F_r_qp = X_rSole_CoM.dualMul(solver().desiredContactForce(getContact("RightFoot")));

    sva::ForceVecd F_r_sensor = robot().forceSensor("RightFootForceSensor").wrench();
    sva::ForceVecd F_l_qp = X_lSole_CoM.dualMul(solver().desiredContactForce(getContact("LeftFoot")));

    sva::ForceVecd F_l_sensor = robot().forceSensor("LeftFootForceSensor").wrench();
    Eigen::Vector6d F_sum = F_ee.vector() + F_r_sole.vector() + F_l_sole.vector() + F_r_qp.vector() + F_l_qp.vector();

    Eigen::MatrixXd A_ZMP;
    A_ZMP.resize(4, 6);
    A_ZMP.setZero();

    A_ZMP(0, 5) = -0.12;
    A_ZMP(0, 1) = -1;
    A_ZMP(1, 5) = -0.12;
    A_ZMP(1, 1) = 1;
    A_ZMP(2, 5) = -0.06;
    A_ZMP(2, 0) = 1;
    A_ZMP(3, 5) = -0.06;
    A_ZMP(3, 0) = -1;

    Eigen::VectorXd result = A_ZMP * F_sum;
    return result;
  });

  logger().addLogEntry("ZMP_pertubation", [this]() {
    sva::PTransformd X_0_CoM = sva::PTransformd(robot().com());
    // sva::PTransformd X_0_rSole = robot().surface("RightFoot").X_0_s(robot());
    sva::PTransformd X_0_rSole = robot().bodyPosW("r_sole");
    sva::PTransformd X_rSole_CoM = X_0_CoM * X_0_rSole.inv();

    sva::ForceVecd f_r_sole =
        X_rSole_CoM.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce("r_sole")));

    // sva::PTransformd X_lSole_b = robot().bodyTransform("l_sole");
    // sva::PTransformd X_0_lSole = robot().surface("LeftFoot").X_0_s(robot());
    sva::PTransformd X_0_lSole = robot().bodyPosW("l_sole");
    sva::PTransformd X_lSole_CoM = X_0_CoM * X_0_lSole.inv();

    sva::ForceVecd f_l_sole =
        X_lSole_CoM.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce("l_sole")));

    // sva::ForceVecd f_l_sole = miPredictorPtr->getImpulsiveForceCOM("l_sole");
    // sva::ForceVecd f_r_sole = miPredictorPtr->getImpulsiveForceCOM("r_sole");
    // sva::PTransformd X_lSole_b = robot().bodyTransform("l_sole");
    sva::PTransformd X_0_ee = robot().bodyPosW("r_wrist");
    sva::PTransformd X_ee_CoM = X_0_CoM * X_0_ee.inv();

    sva::ForceVecd f_ee =
        X_ee_CoM.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), miPredictorPtr->getImpulsiveForce()));

    // sva::ForceVecd f_ee = miPredictorPtr->getImpulsiveForceCOM();

    sva::ForceVecd F_r_qp = X_rSole_CoM.dualMul(solver().desiredContactForce(getContact("RightFoot")));

    sva::ForceVecd F_l_qp = X_lSole_CoM.dualMul(solver().desiredContactForce(getContact("LeftFoot")));
    /*
        double denominator =
            (F_l_qp.force().z() + F_r_qp.force().z() + f_r_sole.force().z() + f_l_sole.force().z() + f_ee.force().z());
    */

    double denominator = (robot().forceSensor("RightFootForceSensor").force().z()
                          + robot().forceSensor("LeftFootForceSensor").force().z() + f_r_sole.force().z()
                          + f_l_sole.force().z() + f_ee.force().z());

    Eigen::Vector3d tempZMP;
    // tempZMP.x() = -(f_r_sole.moment().y() + f_l_sole.moment().y() + f_ee.moment().y()) / denominator;
    // tempZMP.x() = -(f_r_sole.moment().y() + f_l_sole.moment().y() ) / denominator;
    tempZMP.x() = -(f_ee.moment().y()) / denominator;
    // tempZMP.y() = (f_r_sole.moment().x() + f_l_sole.moment().x() + f_ee.moment().x()) / denominator;
    // tempZMP.y() = (f_r_sole.moment().x() + f_l_sole.moment().x()) / denominator;
    tempZMP.y() = (f_ee.moment().x()) / denominator;
    tempZMP.z() = 0;
    return tempZMP;
  });

  //----------------------------- F_COP
  //----------------------------- F_QP

  logger().addLogEntry("l_ankle_wrench_COP", [this]() { return miPredictorPtr->getImpulsiveForceCOP("l_sole"); });
  logger().addLogEntry("r_ankle_wrench_COP", [this]() { return miPredictorPtr->getImpulsiveForceCOP("r_sole"); });

  logger().addLogEntry("l_ankle_wrench_QP", [this]() {
    // sva::ForceVecd F =

    return solver().desiredContactForce(getContact("LeftFoot"));

    // return F;
  });

  logger().addLogEntry("r_ankle_wrench_QP", [this]() {
    // sva::ForceVecd F =

    return solver().desiredContactForce(getContact("RightFoot"));
    // return F;
  });

  logger().addLogEntry("impact_time_indicator", [this]() {
    return impactIndicator_;
    // return F;
  });

  //-----------------------------
  logger().addLogEntry("robot_com", [this]() { return robot().com(); });

  logger().addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });

  gui()->addElement({"Forces"},
                    mc_rtc::gui::Force("LeftCoPForce_real", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                                       [this]() { return realRobot().surfaceWrench("LeftFoot"); },
                                       [this]() {
                                         auto & robot = realRobot();
                                         Eigen::Vector3d cop = robot.copW("LeftFoot");
                                         const sva::PTransformd X_0_s = robot.surface("LeftFoot").X_0_s(robot);
                                         sva::PTransformd surface(X_0_s.rotation(), cop);
                                         return surface;
                                       }),
                    mc_rtc::gui::Force("RightCoPForce_real", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                                       [this]() { return realRobot().surfaceWrench("RightFoot"); },
                                       [this]() {
                                         auto & robot = this->realRobots().robot();
                                         Eigen::Vector3d cop = robot.copW("RightFoot");
                                         const sva::PTransformd X_0_s = robot.surface("RightFoot").X_0_s(robot);
                                         sva::PTransformd surface(X_0_s.rotation(), cop);
                                         return surface;
                                       }),
                    mc_rtc::gui::Point3D("ZMP_real", [this]() {
                      auto & robot = this->realRobots().robot();
                      Eigen::Vector3d zmp;
                      try
                      {
                        zmp = robot.zmp({"LeftFootForceSensor", "RightFootForceSensor"}, Eigen::Vector3d::Zero(),
                                        Eigen::Vector3d{0., 0., 1.}, 100);
                        zmp.z() = 0;
                      }
                      catch(...)
                      {
                        zmp.setZero();
                      }
                      return zmp;
                    }));
  // std::cout << "Operational space dynamics Predictor is about to be created." << std::endl;
  // Add constriants:
}

const mc_rbdyn::Robot & Controller::realRobot() const
{
  return realRobots().robot();
}

bool Controller::rArmInContact()
{

  if(robot().forceSensor("RightHandForceSensor").force().z() > forceThreshold)
  {
    impactIndicator_ = 20;
    return true;
  }
  else
    return false;
}

bool Controller::run()
{
  bool r = mc_control::fsm::Controller::run();
  if(iter_++ == 0)
  {

    forceThreshold = config()("contact_detection")("ForceThreshold");

    if(config()("impact")("constraints")("zmpWithImpulse"))
    {
       std::vector<mc_impact::supportContact> supports;
       supports.push_back({"r_sole", "RightFootForceSensor"});
       supports.push_back({"l_sole", "LeftFootForceSensor"});

       zmpImpulse_.reset(new mc_impact::zmpWithImpulse
		       (*miPredictorPtr, 
			supports,
			timeStep, timeStep, 
			{-0.10, 0.10, -0.15, 0.15} /// This is the supportPolygon
			)
		       );

      solver().addConstraint(zmpImpulse_.get());
    }
    if(config()("impact")("constraints")("copWithImpulse"))
    {
      copImpulseRightFoot_.reset(new mc_impact::copWithImpulse(*miPredictorPtr, "r_sole", "RightFootForceSensor",
                                                               timeStep, timeStep, {-0.10, 0.10, -0.05, 0.05}));
      solver().addConstraint(copImpulseRightFoot_.get());

      copImpulseLeftFoot_.reset(new mc_impact::copWithImpulse(*miPredictorPtr, "l_sole", "LeftFootForceSensor",
                                                              timeStep, timeStep, {-0.10, 0.10, -0.05, 0.05}));
      solver().addConstraint(copImpulseLeftFoot_.get());
    }
    if(config()("impact")("constraints")("frictionWithImpulse"))
    {
      frictionImpulseLeftFoot_.reset(new mc_impact::frictionWithImpulse(
          *miPredictorPtr, "l_sole", "LeftFootForceSensor", getContact("LeftFoot"), timeStep, timeStep));
      solver().addConstraint(frictionImpulseLeftFoot_.get());

      frictionImpulseRightFoot_.reset(new mc_impact::frictionWithImpulse(
          *miPredictorPtr, "r_sole", "RightFootForceSensor", getContact("RightFoot"), timeStep, timeStep));
      solver().addConstraint(frictionImpulseRightFoot_.get());
    }

    if(config()("impact")("constraints")("jointTorque"))
    {
      boundTorqueJump_.reset(new mc_impact::BoundJointTorqueJump(*miPredictorPtr, timeStep, timeStep, 3.5));
      solver().addConstraint(boundTorqueJump_.get());
    }

    if(config()("impact")("constraints")("jointVelocity"))
    {
      boundVelocityJump_.reset(new mc_impact::BoundJointVelocityJump(*miPredictorPtr, timeStep));
      solver().addConstraint(boundVelocityJump_.get());
    }

    if(config()("impact")("constraints")("SlippageLeft"))
    {
      zeroSlippageLeftFoot_.reset(
          new mc_impact::ZeroSlippageWithImpulse(solver(), getContact("LeftFoot"), *miPredictorPtr, "l_sole"));
      solver().addConstraint(zeroSlippageLeftFoot_.get());
    }
    if(config()("impact")("constraints")("SlippageRight"))
    {
      zeroSlippageRightFoot_.reset(
          new mc_impact::ZeroSlippageWithImpulse(solver(), getContact("RightFoot"), *miPredictorPtr, "r_sole"));
      solver().addConstraint(zeroSlippageRightFoot_.get());
    }

    if(config()("impact")("constraints")("CoPLeft"))
    {
      COPImpulseLeftFoot_.reset(new mc_impact::COPInsideContactAreaWithImpulse(
          solver(), getContact("LeftFoot"), {-0.10, 0.10, -0.05, 0.05}, *miPredictorPtr, "l_sole"));
      solver().addConstraint(COPImpulseLeftFoot_.get());
    }

    if(config()("impact")("constraints")("CoPRight"))
    {
      COPImpulseRightFoot_.reset(new mc_impact::COPInsideContactAreaWithImpulse(
          solver(), getContact("RightFoot"), {-0.10, 0.10, -0.05, 0.05}, *miPredictorPtr, "r_sole"));
      solver().addConstraint(COPImpulseRightFoot_.get());
    }
    solver().updateConstrSize();
  }
  return r;
}

void Controller::reset(const mc_control::ControllerResetData & data)
{

  /** First reset to get correct initial position of main robot */
  mc_control::MCController::reset(data);

  // Open the grippers
  for(auto & g : grippers)
  {
    g.second->setTargetOpening(1.0);
  }

  /** Initialize FSM stuff */
  mc_control::fsm::Controller::reset(data);
}

const mc_rbdyn::Contact & Controller::getContact(const std::string & s)
{
  for(const auto & c : solver().contacts())
  {
    if((c.r1Index() == 0 && c.r1Surface()->name() == s) || (c.r2Index() == 0 && c.r2Surface()->name() == s))
    {
      return c;
    }
  }
  LOG_ERROR_AND_THROW(std::runtime_error, "Failed to find contact id for " << s)
}

CONTROLLER_CONSTRUCTOR("RArmPushUnknownWall", Controller)
