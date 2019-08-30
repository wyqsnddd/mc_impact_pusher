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

#include "mc_r_arm_push_unknown_wall.h"

Controller::Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf)
: mc_control::fsm::Controller(rm, dt, conf)
{

  forceThreshold = config()("contact_detection")("ForceThreshold");
  impactIndicator_ = -20.0;
  std::cout << "Kinematics and Dynamics constraints are created." << std::endl;
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  // ------------------------- Test the new bodyWrench function:

  logger().addLogEntry("bodyWrench_LeftHandForceSensor", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.bodyWrench("l_wrist");
  });

  logger().addLogEntry("bodyWrench_RightHandForceSensor", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.bodyWrench("r_wrist");
  });

  logger().addLogEntry("bodyWrench_LeftFootForceSensor", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.bodyWrench("l_ankle");
  });

  logger().addLogEntry("bodyWrench_RightFootForceSensor", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.bodyWrench("r_ankle");
  });

  logger().addLogEntry("worldWrench_RightFootForceSensor", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.forceSensor("RightFootForceSensor").worldWrench(robot);
  });

  logger().addLogEntry("worldWrench_LeftFootForceSensor", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.forceSensor("LeftFootForceSensor").worldWrench(robot);
  });

  logger().addLogEntry("worldWrench_LeftHandForceSensor", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.forceSensor("LeftHandForceSensor").worldWrench(robot);
  });

  logger().addLogEntry("worldWrench_RightHandForceSensor", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.forceSensor("RightHandForceSensor").worldWrench(robot);
  });
  // -----------------------C-res

  logger().addLogEntry("C_res", [this]() {
    Eigen::Vector3d surfaceNormal = ecQpEstimatorPtr->getImpactModel("r_wrist")->getSurfaceNormal();
    double tempVel = surfaceNormal.transpose() * ecQpEstimatorPtr->getImpactModel("r_wrist")->getContactVel();
    // Eigen::Vector3d tempVel =
    // ecQpEstimatorPtr->getImpactModel("r_wrist")->getContactVel();

    Eigen::MatrixXd jacobian = ecQpEstimatorPtr->getImpactModel("r_wrist")->getJacobian();

    Eigen::Vector3d deltaV = jacobian * getOsd()->getInvMassMatrix() * jacobian.transpose()
                             * robot().bodyWrench("r_wrist").force() * timeStep;

    double tempDeltaV = surfaceNormal.transpose() * deltaV;
    double c_res = (double)(tempDeltaV + tempVel) / (double)tempVel;
    if(std::abs(c_res) > 10.0)
      return 10.0;
    else
      return c_res;
  });

  logger().addLogEntry("C_deltaV_forceSensor", [this]() {
    Eigen::VectorXd jacobian = ecQpEstimatorPtr->getImpactModel("r_wrist")->getJacobian();

    Eigen::Vector3d deltaV = jacobian * getOsd()->getInvMassMatrix() * jacobian.transpose()
                             * robot().bodyWrench("r_wrist").force() * timeStep;

    return deltaV;
  });

  logger().addLogEntry("C_V_plus_forceSensor", [this]() {
    Eigen::Vector3d tempVel = ecQpEstimatorPtr->getImpactModel("r_wrist")->getContactVel();
    Eigen::VectorXd jacobian = ecQpEstimatorPtr->getImpactModel("r_wrist")->getJacobian();

    Eigen::Vector3d deltaV = jacobian * getOsd()->getInvMassMatrix() * jacobian.transpose()
                             * robot().bodyWrench("r_wrist").force() * timeStep;

    return (Eigen::Vector3d)(deltaV + tempVel);
  });

  logger().addLogEntry("C_V_minus_forceSensor", [this]() {
    Eigen::Vector3d tempVel = ecQpEstimatorPtr->getImpactModel("r_wrist")->getContactVel();

    return tempVel;
  });

  // -----------------------------------------------------------
  logger().addLogEntry("CoP_LeftFoot_World", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.copW("LeftFoot");
  });
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

  logger().addLogEntry("RightWrist", [this]() { return robot().mbc().bodyPosW[robot().bodyIndexByName("r_wrist")]; });
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

  logger().addLogEntry("constraints_jointVelocity_on", [this]() -> double {
    return static_cast<bool>(config()("impact")("constraints")("jointVelocity")("on"));
  });
  logger().addLogEntry("constraints_jointVelocity_multiplier",
                       [this]() -> double { return config()("impact")("constraints")("jointVelocity")("multiplier"); });
  logger().addLogEntry("constraints_jointTorque_on", [this]() -> double {
    return static_cast<bool>(config()("impact")("constraints")("jointTorque")("on"));
  });
  logger().addLogEntry("constraints_jointTorque_multiplier",
                       [this]() -> double { return config()("impact")("constraints")("jointTorque")("multiplier"); });
  logger().addLogEntry("constraints_frictionWithImpulse_on", [this]() -> double {
    return static_cast<bool>(config()("impact")("constraints")("frictionWithImpulse"));
  });
  logger().addLogEntry("constraints_copWithImpulse_on", [this]() -> double {
    return static_cast<bool>(config()("impact")("constraints")("copWithImpulse")("on"));
  });
  logger().addLogEntry("constraints_zmpWithImpulse_on", [this]() -> double {
    return static_cast<bool>(config()("impact")("constraints")("zmpWithImpulse")("on"));
  });
  logger().addLogEntry("constraints_zmpWithImpulse_allforce", [this]() -> double {
    return static_cast<bool>(config()("impact")("constraints")("zmpWithImpulse")("allforce"));
  });

  std::string impactBodyString(config()("impact")("estimation")("impactBodyName"));

  miOsdPtr_ = std::make_shared<mc_impact::mi_osd>(robot(), true);
  std::vector<std::string> eeNameVector = config()("impact")("estimation")("end-effectors");

  miOsdPtr_->initializeDataStructure(static_cast<int>(eeNameVector.size()));
  miOsdPtr_->resetDataStructure();

  for(auto index = eeNameVector.begin(); index != eeNameVector.end(); ++index)
  {

    // std::string eeName =
    // ctl.config()("states")("Contact")("end-effectors")(index);
    std::cout << "Processing end-effector: " << *index << std::endl;

    if(!miOsdPtr_->addEndeffector(*index))
    {
      throw std::runtime_error("OSD failed to add endeffector!");
    }
    else
    {
      std::cout << "End-effector: " << *index << "is added to the OSD model. " << std::endl;
    }
  }

  std::vector<std::string> contactBodies = config()("impact")("estimation")("contactBodies");
  miOsdPtr_->setContact(contactBodies);

  std::cout << "Operational space dynamics Predictor is created." << std::endl;

  if(config()("lcp")("on"))
  {
    lcpSolverPtr.reset(new mc_impact::mi_lcp(robot(), realRobots().robot(), getOsd(), config()("lcp")("dim"),
                                             config()("lcp")("solver"), config()("lcp")("convergenceThreshold")));
    std::cout << "lcp solver is created" << std::endl;
  }

  if(config()("qpEstimator")("on"))
  {
    mc_impact::qpEstimatorParameter qpParams;
    qpParams.Qweight = config()("qpEstimator")("Qweight");

    qpParams.impactDuration = config()("impact")("estimation")("delta_dt");
    qpParams.coeFrictionDeduction = config()("impact")("estimation")("coeFrictionDeduction");
    qpParams.coeRes = config()("impact")("estimation")("coeRestitution");
    qpParams.dim = config()("qpEstimator")("dim");
    qpParams.timeStep = timeStep;

    std::string impactBodyName = config()("impact")("estimation")("impactBodyName");
    Eigen::Vector3d inertial_normal = config()("impact")("estimation")("inertialFrame_impactNormal");
    qpParams.impactNameAndNormals.insert(std::pair<std::string, Eigen::Vector3d>(impactBodyName, inertial_normal));
    qpParams.useLagrangeMultiplier = false;
    qpParams.useJsd = true;
    qpParams.useOsd = true;

    qpEstimatorPtr.reset(new mc_impact::mi_qpEstimator(robot(), getOsd(), qpParams));

    qpParams.useJsd = true;
    qpParams.useOsd = false;
    jsdQpEstimatorPtr.reset(new mc_impact::mi_qpEstimator(robot(), getOsd(), qpParams));

    qpParams.useJsd = false;
    qpParams.useOsd = true;
    osdQpEstimatorPtr.reset(new mc_impact::mi_qpEstimator(robot(), getOsd(), qpParams));

    qpParams.useJsd = false;
    qpParams.useOsd = true;
    qpParams.useLagrangeMultiplier = true;
    ecQpEstimatorPtr.reset(new mc_impact::mi_qpEstimator(robot(), getOsd(), qpParams));

    std::cout << "qp solver is created" << std::endl;

    std::vector<std::string> eeNameVector = config()("impact")("estimation")("end-effectors");
    /*
    for(auto idx = eeNameVector.begin(); idx != eeNameVector.end(); ++idx)
    {
      qpEstimatorPtr->addEndeffector(*idx);
      osdQpEstimatorPtr->addEndeffector(*idx);
      jsdQpEstimatorPtr->addEndeffector(*idx);
      ecQpEstimatorPtr->addEndeffector(*idx);
    }
    */
    jsdQpEstimatorPtr->print();
    osdQpEstimatorPtr->print();
    ecQpEstimatorPtr->print();
    qpEstimatorPtr->print();
    std::cout << "QP impulse estimator has added the endeffectors." << std::endl;
  }

  // ------------------------------Contact velocity

  // ---------------------------- constructor: add modified constraints

  logger().addLogEntry("ee_Vel_real", [this]() {
    return realRobots().robot().mbc().bodyVelB[realRobots().robot().mb().bodyIndexByName("r_wrist")];
  });
  logger().addLogEntry("ee_Vel_robot",
                       [this]() { return robot().mbc().bodyVelB[robot().mb().bodyIndexByName("r_wrist")]; });

  logger().addLogEntry("ee_Vel_qp",
                       [this]() { return robot().mbc().bodyVelB[robot().mb().bodyIndexByName("r_wrist")]; });
  logger().addLogEntry("l_ankle_Vel_robot",
                       [this]() { return robot().mbc().bodyVelB[robot().mb().bodyIndexByName("l_ankle")].linear(); });

  logger().addLogEntry("l_ankle_Vel_real", [this]() {
    return realRobots().robot().mbc().bodyVelB[realRobots().robot().mb().bodyIndexByName("l_ankle")].linear();
  });

  logger().addLogEntry("r_ankle_Vel_robot",
                       [this]() { return robot().mbc().bodyVelB[robot().mb().bodyIndexByName("r_ankle")].linear(); });

  logger().addLogEntry("r_ankle_Vel_real", [this]() {
    return realRobots().robot().mbc().bodyVelB[realRobots().robot().mb().bodyIndexByName("r_ankle")].linear();
  });
  logger().addLogEntry("COM_RightFootForceSensor", [this]() {
    sva::PTransformd X_0_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_0_ee = robot().bodyPosW("r_ankle");
    sva::PTransformd X_ee_CoM = X_0_CoM * X_0_ee.inv();
    sva::ForceVecd f_ee = X_ee_CoM.dualMul(robot().forceSensor("RightFootForceSensor").wrench());
    return f_ee;
  });

  logger().addLogEntry("COM_LeftFootForceSensor", [this]() {
    sva::PTransformd X_0_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_0_ee = robot().bodyPosW("l_ankle");
    sva::PTransformd X_ee_CoM = X_0_CoM * X_0_ee.inv();
    sva::ForceVecd f_ee = X_ee_CoM.dualMul(robot().forceSensor("LeftFootForceSensor").wrench());
    return f_ee;
  });

  logger().addLogEntry("COM_RightHandForceSensor", [this]() {
    sva::PTransformd X_0_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_0_ee = robot().bodyPosW("r_wrist");
    sva::PTransformd X_ee_CoM = X_0_CoM * X_0_ee.inv();
    sva::ForceVecd f_ee = X_ee_CoM.dualMul(robot().forceSensor("RightHandForceSensor").wrench());
    return f_ee;
  });

  logger().addLogEntry("COM_sum_sensor_wrench", [this]() {
    sva::PTransformd X_0_CoM = sva::PTransformd(robot().com());
    sva::PTransformd X_0_ee = robot().bodyPosW("r_wrist");
    sva::PTransformd X_ee_CoM = X_0_CoM * X_0_ee.inv();
    sva::ForceVecd f_ee = X_ee_CoM.dualMul(robot().forceSensor("RightHandForceSensor").wrench());

    sva::PTransformd X_0_rSole = robot().bodyPosW("r_ankle");
    sva::PTransformd X_rSole_CoM = X_0_CoM * X_0_rSole.inv();
    sva::ForceVecd f_rSole = X_rSole_CoM.dualMul(robot().forceSensor("RightFootForceSensor").wrench());

    sva::PTransformd X_0_lSole = robot().bodyPosW("l_ankle");
    sva::PTransformd X_lSole_CoM = X_0_CoM * X_0_lSole.inv();
    sva::ForceVecd f_lSole = X_lSole_CoM.dualMul(robot().forceSensor("LeftFootForceSensor").wrench());

    return f_ee + f_rSole + f_lSole;
  });

  logger().addLogEntry(
      "q_position", [this]() { return rbd::paramToVector(realRobots().robot().mb(), realRobots().robot().mbc().q); });

  logger().addLogEntry(
      "q_vel", [this]() { return rbd::dofToVector(realRobots().robot().mb(), realRobots().robot().mbc().alpha); });

  logger().addLogEntry(
      "q_acc", [this]() { return rbd::dofToVector(realRobots().robot().mb(), realRobots().robot().mbc().alphaD); });

  // logger().addLogEntry("tau", [this]() { return
  // rbd::dofToVector(realRobots().robot().mb(),
  // realRobots().robot().mbc().jointTorque); });
  logger().addLogEntry("tau_QP", [this]() { return rbd::dofToVector(robot().mb(), robot().mbc().jointTorque); });
  logger().addLogEntry("impulsive_tau_righthand", [this]() {
    return (Eigen::VectorXd)(ecQpEstimatorPtr->getImpactModel("r_wrist")->getJacobian().transpose()
                             * robot().bodyWrench("r_wrist").force());
  });
  //----------------------------- check the joint velocity jump
  //----------------------------- check the impulsive induced jump to COP

  logger().addLogEntry("CoP_LeftFoot_local", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.cop("LeftFoot");
  });
  logger().addLogEntry("CoP_RightFoot_local", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.cop("RightFoot");
  });
  //----------------------------- check the QP estimated contact force

  // -------------------------- QP estimator solved by Lagrange Multipliers

  if(config()("qpEstimator")("on"))
  {

    logger().addLogEntry("qpOsd_RightFootForce", [this]() {
      return osdQpEstimatorPtr->getEndeffector("r_ankle").estimatedAverageImpulsiveForce;
    });
    logger().addLogEntry("qpOsd_LeftFootForce", [this]() {
      return osdQpEstimatorPtr->getEndeffector("l_ankle").estimatedAverageImpulsiveForce;
    });
    logger().addLogEntry("qpOsd_RightHandForce", [this]() {
      return osdQpEstimatorPtr->getEndeffector("r_wrist").estimatedAverageImpulsiveForce;
    });
    logger().addLogEntry("qpOsd_delta_q_dot", [this]() { return osdQpEstimatorPtr->getJointVelJump(); });
    logger().addLogEntry("qpOsd_delta_tau", [this]() { return osdQpEstimatorPtr->getTauJump(); });

    logger().addLogEntry("qpOsd_delta_v_leftFoot",
                         [this]() { return osdQpEstimatorPtr->getEndeffector("l_ankle").eeVJump; });

    logger().addLogEntry("qpOsd_delta_v_rightFoot",
                         [this]() { return osdQpEstimatorPtr->getEndeffector("r_ankle").eeVJump; });
    logger().addLogEntry("qpOsd_delta_v_rightHand",
                         [this]() { return osdQpEstimatorPtr->getEndeffector("r_wrist").eeVJump; });

    logger().addLogEntry("qpJsd_RightFootForce", [this]() {
      return jsdQpEstimatorPtr->getEndeffector("r_ankle").estimatedAverageImpulsiveForce;
    });
    logger().addLogEntry("qpJsd_LeftFootForce", [this]() {
      return jsdQpEstimatorPtr->getEndeffector("l_ankle").estimatedAverageImpulsiveForce;
    });
    logger().addLogEntry("qpJsd_RightHandForce", [this]() {
      return jsdQpEstimatorPtr->getEndeffector("r_wrist").estimatedAverageImpulsiveForce;
    });
    logger().addLogEntry("qpJsd_delta_q_dot", [this]() { return jsdQpEstimatorPtr->getJointVelJump(); });
    logger().addLogEntry("qpJsd_delta_tau", [this]() { return jsdQpEstimatorPtr->getTauJump(); });

    logger().addLogEntry("qpJsd_delta_v_leftFoot",
                         [this]() { return jsdQpEstimatorPtr->getEndeffector("l_ankle").eeVJump; });

    logger().addLogEntry("qpJsd_delta_v_rightFoot",
                         [this]() { return jsdQpEstimatorPtr->getEndeffector("r_ankle").eeVJump; });
    logger().addLogEntry("qpJsd_delta_v_rightHand",
                         [this]() { return jsdQpEstimatorPtr->getEndeffector("r_wrist").eeVJump; });
    logger().addLogEntry("qpEc_check_RightFootForce",
                         [this]() { return ecQpEstimatorPtr->getEndeffector("r_ankle").checkForce; });
    logger().addLogEntry("qpEc_check_LeftFootForce",
                         [this]() { return ecQpEstimatorPtr->getEndeffector("l_ankle").checkForce; });
    logger().addLogEntry("qpEc_check_RightHandForce",
                         [this]() { return ecQpEstimatorPtr->getEndeffector("r_wrist").checkForce; });

    logger().addLogEntry("qpEc_RightFootForce", [this]() {
      return ecQpEstimatorPtr->getEndeffector("r_ankle").estimatedAverageImpulsiveForce;
    });
    logger().addLogEntry("qpEc_LeftFootForce", [this]() {
      return ecQpEstimatorPtr->getEndeffector("l_ankle").estimatedAverageImpulsiveForce;
    });
    logger().addLogEntry("qpEc_RightHandForce", [this]() {
      return ecQpEstimatorPtr->getEndeffector("r_wrist").estimatedAverageImpulsiveForce;
    });
    logger().addLogEntry("qp_RightHand_impact_normal",
                         [this]() { return ecQpEstimatorPtr->getImpactModel("r_wrist")->getSurfaceNormal(); });

    logger().addLogEntry("qpEc_delta_q_dot", [this]() { return ecQpEstimatorPtr->getJointVelJump(); });
    logger().addLogEntry("qpEc_delta_tau", [this]() { return ecQpEstimatorPtr->getTauJump(); });

    logger().addLogEntry("qpEc_delta_v_leftFoot",
                         [this]() { return ecQpEstimatorPtr->getEndeffector("l_ankle").eeVJump; });

    logger().addLogEntry("qpEc_delta_v_rightFoot",
                         [this]() { return ecQpEstimatorPtr->getEndeffector("r_ankle").eeVJump; });
    logger().addLogEntry("qpEc_delta_v_rightHand",
                         [this]() { return ecQpEstimatorPtr->getEndeffector("r_wrist").eeVJump; });
    logger().addLogEntry("qpEc_delta_v_impact",
                         [this]() { return ecQpEstimatorPtr->getImpactModel("r_wrist")->getEeVelocityJump(); });

    logger().addLogEntry("qpEc_ee_v_predict",
                         [this]() { return ecQpEstimatorPtr->getImpactModel("r_wrist")->getEeVelocity(); });

    logger().addLogEntry("qpEc_ee_v_normal",
                         [this]() { return ecQpEstimatorPtr->getImpactModel("r_wrist")->getSurfaceNormal(); });

    // test the qpestimator Jacobians:

    // ecQpEstimatorPtr
    logger().addLogEntry("qpEc_compare_delta_dq", [this]() {
      Eigen::VectorXd temp = rbd::dofToVector(robot().mb(), robot().mbc().alpha)
                             + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * timeStep;

      // return (ecQpEstimatorPtr->getJointVelJump() -
      // ecQpEstimatorPtr->getJacobianDeltaAlpha() * temp).norm();
      return (ecQpEstimatorPtr->getJointVelJump()
              - ecQpEstimatorPtr->getJacobianDeltaAlpha() * ecQpEstimatorPtr->getImpactModel("r_wrist")->getJointVel())
          .norm();
    });

    logger().addLogEntry("qpEc_compare_RightFootForce", [this]() {
      Eigen::VectorXd temp = rbd::dofToVector(robot().mb(), robot().mbc().alpha)
                             + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * timeStep;

      double inv_dt = 1.0 / ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration();
      Eigen::Vector3d predictedForce = inv_dt * (ecQpEstimatorPtr->getJacobianDeltaF("r_ankle") * temp);
      // Eigen::Vector3d predictedForce =
      // inv_dt*(ecQpEstimatorPtr->getJacobianDeltaF("r_ankle")*ecQpEstimatorPtr->getImpactModel()->getEeVelocityJump());
      return predictedForce;
    });

    logger().addLogEntry("qpEc_compare_LeftFootForce", [this]() {
      Eigen::VectorXd temp = rbd::dofToVector(robot().mb(), robot().mbc().alpha)
                             + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * timeStep;

      double inv_dt = 1.0 / ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration();
      Eigen::Vector3d predictedForce = inv_dt * (ecQpEstimatorPtr->getJacobianDeltaF("l_ankle") * temp);
      // Eigen::Vector3d predictedForce =
      // inv_dt*(ecQpEstimatorPtr->getJacobianDeltaF("l_ankle")*ecQpEstimatorPtr->getImpactModel()->getEeVelocityJump());
      return predictedForce;
    });
    logger().addLogEntry("qpEc_compare_RightHandForce", [this]() {
      /*
            Eigen::Vector3d temp = rbd::dofToVector(robot().mb(),
         robot().mbc().alpha)
                                   + rbd::dofToVector(robot().mb(),
         robot().mbc().alphaD) * timeStep;

                                   */
      double inv_dt = 1.0 / ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration();
      // Eigen::Vector3d predictedForce =
      // inv_dt*(ecQpEstimatorPtr->getJacobianDeltaF("r_wrist")*ecQpEstimatorPtr->getImpactModel()->getJointVel());
      // Eigen::Vector3d predictedForce = inv_dt *
      // (ecQpEstimatorPtr->getJacobianDeltaF("r_wrist")) * temp;
      Eigen::Vector3d predictedForce = inv_dt * (ecQpEstimatorPtr->getJacobianDeltaF("r_wrist"))
                                       * ecQpEstimatorPtr->getImpactModel("r_wrist")->getJointVel();
      return predictedForce;
    });

    /*
      logger().addLogEntry("qpEc_compare_RightHandForce", [this]() {
        Eigen::Vector3d temp =
            rbd::dofToVector(robot().mb(), robot().mbc().alpha)
             + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) *timeStep;

        double inv_dt =
      1.0/ecQpEstimatorPtr->getImpactModel()->getImpactDuration();
        Eigen::Vector3d predictedForce =
      inv_dt*(ecQpEstimatorPtr->getJacobianDeltaF("r_wrist")*temp);
        //Eigen::Vector3d predictedForce =
      inv_dt*(ecQpEstimatorPtr->getJacobianDeltaF("r_wrist")*ecQpEstimatorPtr->getImpactModel()->getEeVelocityJump());
        return predictedForce;
        });
        */
    logger().addLogEntry("qpEc_compare_delta_tau", [this]() {
      Eigen::VectorXd temp = rbd::dofToVector(robot().mb(), robot().mbc().alpha)
                             + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * timeStep;

      double inv_dt = 1 / ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration();
      // return (ecQpEstimatorPtr->getTauJump() - inv_dt *
      // ecQpEstimatorPtr->getJacobianDeltaTau() * temp).norm();
      return (Eigen::VectorXd)(ecQpEstimatorPtr->getTauJump()
                               - inv_dt * ecQpEstimatorPtr->getJacobianDeltaTau()
                                     * ecQpEstimatorPtr->getImpactModel("r_wrist")->getJointVel());
    });

    logger().addLogEntry("qpEc_compare_delta_v", [this]() {
      Eigen::VectorXd temp = rbd::dofToVector(robot().mb(), robot().mbc().alpha)
                             + rbd::dofToVector(robot().mb(), robot().mbc().alphaD) * timeStep;
      Eigen::Vector3d diff = ecQpEstimatorPtr->getImpactModel("r_wrist")->getEeVelocityJump()
                             - ecQpEstimatorPtr->getImpactModel("r_wrist")->getProjector()
                                   * ecQpEstimatorPtr->getImpactModel("r_wrist")->getJointVel();
      return diff;
    });

    // -------------------------- QP estimator solved by mini QP
    logger().addLogEntry("qp_RightFootForce",
                         [this]() { return qpEstimatorPtr->getEndeffector("r_ankle").estimatedAverageImpulsiveForce; });
    logger().addLogEntry("qp_LeftFootForce",
                         [this]() { return qpEstimatorPtr->getEndeffector("l_ankle").estimatedAverageImpulsiveForce; });
    logger().addLogEntry("qp_RightHandForce",
                         [this]() { return qpEstimatorPtr->getEndeffector("r_wrist").estimatedAverageImpulsiveForce; });

    logger().addLogEntry("qp_delta_q_dot", [this]() { return qpEstimatorPtr->getJointVelJump(); });
    logger().addLogEntry("qp_delta_tau", [this]() { return qpEstimatorPtr->getTauJump(); });

    logger().addLogEntry("qp_delta_v_leftFoot", [this]() { return qpEstimatorPtr->getEndeffector("l_ankle").eeVJump; });

    logger().addLogEntry("qp_delta_v_rightFoot",
                         [this]() { return qpEstimatorPtr->getEndeffector("r_ankle").eeVJump; });

    logger().addLogEntry("qp_delta_v_rightHand",
                         [this]() { return qpEstimatorPtr->getEndeffector("r_wrist").eeVJump; });
    logger().addLogEntry("qp_delta_v_impact",
                         [this]() { return qpEstimatorPtr->getImpactModel("r_wrist")->getEeVelocityJump(); });
  }
  //----------------------------- check the LCP estimated contact force

  if(config()("lcp")("on"))
  {
    logger().addLogEntry("lcp_RightFootForce", [this]() { return lcpSolverPtr->getPredictedContactForce("r_ankle"); });
    logger().addLogEntry("lcp_LeftFootForce", [this]() { return lcpSolverPtr->getPredictedContactForce("l_ankle"); });
  }
  //----------------------------- check the impulsive joint torque
  //----------------------------- check the perturbed ZMP
  logger().addLogEntry("ZMP_QP", [this]() {
    sva::PTransformd X_rSole_0 = robot().bodyPosW("r_sole").inv();
    sva::PTransformd X_lSole_0 = robot().bodyPosW("l_sole").inv();

    sva::ForceVecd F_r_qp = X_rSole_0.dualMul(solver().desiredContactForce(getContact("RightFoot")));
    sva::ForceVecd F_l_qp = X_lSole_0.dualMul(solver().desiredContactForce(getContact("LeftFoot")));

    double denominator = (F_l_qp.force().z() + F_r_qp.force().z());
    Eigen::Vector3d tempZMP;
    tempZMP.x() = -(F_r_qp.force().y() + F_l_qp.force().y()) / denominator;
    tempZMP.y() = -(F_r_qp.force().x() + F_l_qp.force().x()) / denominator;

    return tempZMP;
  });
  logger().addLogEntry("ZMP_test_calculation_allforce", [this]() {
    sva::PTransformd X_rSole_0 = robot().bodyPosW("r_ankle").inv();
    sva::PTransformd X_lSole_0 = robot().bodyPosW("l_ankle").inv();
    sva::PTransformd X_ree_0 = robot().bodyPosW("r_wrist").inv();

    // sva::ForceVecd fr_sensor =
    // robot().forceSensor("RightFootForceSensor").wrench();
    sva::ForceVecd fr_sensor = robot().bodyWrench("r_ankle");
    sva::ForceVecd fr_0 = X_rSole_0.dualMul(fr_sensor);

    // sva::ForceVecd fl_sensor =
    // robot().forceSensor("LeftFootForceSensor").wrench();
    sva::ForceVecd fl_sensor = robot().bodyWrench("l_ankle");
    sva::ForceVecd fl_0 = X_lSole_0.dualMul(fl_sensor);

    sva::ForceVecd free_sensor = robot().bodyWrench("r_wrist");
    sva::ForceVecd free_0 = X_ree_0.dualMul(free_sensor);

    double denominator = fr_0.force().z() + fl_0.force().z() + free_0.force().z();

    Eigen::Vector3d tempZMP;
    tempZMP.x() = -(fr_0.couple().y() + fl_0.couple().y() + free_0.couple().y()) / denominator;
    tempZMP.y() = (fr_0.couple().x() + fl_0.couple().x() + free_0.couple().x()) / denominator;
    tempZMP.z() = 0;
    return tempZMP;
  });

  logger().addLogEntry("ZMP_test_calculation", [this]() {
    sva::PTransformd X_rSole_0 = robot().bodyPosW("r_ankle").inv();
    sva::PTransformd X_lSole_0 = robot().bodyPosW("l_ankle").inv();

    // sva::ForceVecd fr_sensor =
    // robot().forceSensor("RightFootForceSensor").wrench();
    sva::ForceVecd fr_sensor = robot().bodyWrench("r_ankle");
    sva::ForceVecd fr_0 = X_rSole_0.dualMul(fr_sensor);

    sva::ForceVecd fl_sensor = robot().bodyWrench("l_ankle");
    sva::ForceVecd fl_0 = X_lSole_0.dualMul(fl_sensor);

    double denominator = fr_0.force().z() + fl_0.force().z();

    Eigen::Vector3d tempZMP;
    tempZMP.x() = -(fr_0.moment().y() + fl_0.moment().y()) / denominator;
    tempZMP.y() = (fr_0.moment().x() + fl_0.moment().x()) / denominator;
    tempZMP.z() = 0;
    return tempZMP;
  });
  //----------------------------- F_COP
  //----------------------------- F_QP

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
  // Calculate the impulsive torque:
  /*
  logger().addLogEntry("impulsive_torque_r_wrist", [this]() {
    return
  (Eigen::VectorXd)(ecQpEstimatorPtr->getOsd()->getJacobian("r_wrist").transpose()*robot().forceSensor("RightHandForceSensor").force()).segment(6,
  ecQpEstimatorPtr->getDof()-6);
  });
*/
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
  // Add constriants:
}

const mc_rbdyn::Robot & Controller::realRobot() const
{
  return realRobots().robot();
}

bool Controller::rArmInContact()
{

  if(robot().forceSensor("RightHandForceSensor").force().norm() > forceThreshold)
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
    if(config()("impact")("constraints")("zmpWithImpulse")("on"))
    {
      std::vector<mc_impact::supportContact> supports;
      supports.push_back({"r_ankle", "RightFootForceSensor"});
      supports.push_back({"l_ankle", "LeftFootForceSensor"});

      std::vector<double> zmpVector = config()("impact")("constraints")("zmpWithImpulse")("supportPolygon");

      bool debug = config()("impact")("constraints")("zmpWithImpulse")("debug");
      bool allforce = config()("impact")("constraints")("zmpWithImpulse")("allforce");

      zmpImpulse_.reset(new mc_impact::zmpWithImpulse(
          *ecQpEstimatorPtr, supports, timeStep, ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration(),
          {zmpVector[0], zmpVector[1], zmpVector[2], zmpVector[3]}, /// This is the supportPolygon
          allforce, debug));

      solver().addConstraint(zmpImpulse_.get());

      logger().addLogEntry("ZMP_x_lower_bound", [this]() { return (double)zmpImpulse_->area_.min_x; });

      logger().addLogEntry("ZMP_x_upper_bound", [this]() { return (double)zmpImpulse_->area_.max_x; });

      logger().addLogEntry("ZMP_y_lower_bound", [this]() { return (double)zmpImpulse_->area_.min_y; });

      logger().addLogEntry("ZMP_y_upper_bound", [this]() { return (double)zmpImpulse_->area_.max_y; });
    }

    if(config()("impact")("constraints")("copWithImpulse")("on"))
    {

      std::vector<double> contactArea = config()("impact")("constraints")("copWithImpulse")("contactArea");
      copImpulseRightFoot_.reset(
          new mc_impact::copWithImpulse(*ecQpEstimatorPtr, "r_ankle", "RightFootForceSensor", timeStep,
                                        ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration(),
                                        {contactArea[0], contactArea[1], contactArea[2], contactArea[3]}));
      solver().addConstraint(copImpulseRightFoot_.get());

      copImpulseLeftFoot_.reset(
          new mc_impact::copWithImpulse(*ecQpEstimatorPtr, "l_ankle", "LeftFootForceSensor", timeStep,
                                        ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration(),
                                        {contactArea[0], contactArea[1], contactArea[2], contactArea[3]}));
      solver().addConstraint(copImpulseLeftFoot_.get());

      logger().addLogEntry("CoP_LeftFoot_local_constraint_perturb",
                           [this]() { return copImpulseLeftFoot_->getCopPerturb(); });

      logger().addLogEntry("CoP_RightFoot_local_constraint_perturb",
                           [this]() { return copImpulseRightFoot_->getCopPerturb(); });

      logger().addLogEntry("CoP_LeftFoot_local_constraint", [this]() { return copImpulseLeftFoot_->getCop(); });

      logger().addLogEntry("CoP_RightFoot_local_constraint", [this]() { return copImpulseRightFoot_->getCop(); });

      logger().addLogEntry("CoP_LeftFoot_local_constraint_allforce",
                           [this]() { return copImpulseLeftFoot_->getCopPerturbWhole(); });

      logger().addLogEntry("CoP_RightFoot_local_constraint_allforce",
                           [this]() { return copImpulseRightFoot_->getCopPerturbWhole(); });

      logger().addLogEntry("CoP_x_lower_bound", [this]() { return (double)copImpulseLeftFoot_->area_.min_x; });

      logger().addLogEntry("CoP_x_upper_bound", [this]() { return (double)copImpulseLeftFoot_->area_.max_x; });

      logger().addLogEntry("CoP_y_lower_bound", [this]() { return (double)copImpulseLeftFoot_->area_.min_y; });

      logger().addLogEntry("CoP_y_upper_bound", [this]() { return (double)copImpulseLeftFoot_->area_.max_y; });
    }
    if(config()("impact")("constraints")("frictionWithImpulse"))
    {
      frictionImpulseLeftFoot_.reset(new mc_impact::frictionWithImpulse(
          *ecQpEstimatorPtr, "l_ankle", "LeftFootForceSensor", getContact("LeftFoot"), timeStep,
          ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration()));
      solver().addConstraint(frictionImpulseLeftFoot_.get());

      frictionImpulseRightFoot_.reset(new mc_impact::frictionWithImpulse(
          *ecQpEstimatorPtr, "r_ankle", "RightFootForceSensor", getContact("RightFoot"), timeStep,
          ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration()));
      solver().addConstraint(frictionImpulseRightFoot_.get());
    }

    if(config()("impact")("constraints")("jointTorque")("on"))
    {

      bool debugTorque = config()("impact")("constraints")("jointTorque")("on");
      boundTorqueJump_.reset(new mc_impact::BoundJointTorqueJump(
          *ecQpEstimatorPtr, timeStep, ecQpEstimatorPtr->getImpactModel("r_wrist")->getImpactDuration(),
          config()("impact")("constraints")("jointTorque")("multiplier"), debugTorque));
      solver().addConstraint(boundTorqueJump_.get());

      logger().addLogEntry("qpEc_delta_tau_upper_bound", [this]() {
        double multiplier = config()("impact")("constraints")("jointTorque")("multiplier");
        return (Eigen::VectorXd)(multiplier * rbd::dofToVector(robot().mb(), robot().tu()));
      });

      logger().addLogEntry("qpEc_delta_tau_lower_bound", [this]() {
        double multiplier = config()("impact")("constraints")("jointTorque")("multiplier");
        return (Eigen::VectorXd)(multiplier * rbd::dofToVector(robot().mb(), robot().tl()));
      });
    }

    if(config()("impact")("constraints")("jointVelocity")("on"))
    {
      bool debugVelocity = config()("impact")("constraints")("jointVelocity")("debug");
      double deductFactor = config()("impact")("constraints")("jointVelocity")("multiplier");

      boundVelocityJump_.reset(
          new mc_impact::BoundJointVelocityJump(*ecQpEstimatorPtr, timeStep, deductFactor, debugVelocity));
      solver().addConstraint(boundVelocityJump_.get());

      logger().addLogEntry("qpEc_delta_q_dot_lower_bound", [this]() {
        double multiplier = config()("impact")("constraints")("jointVelocity")("multiplier");
        return (Eigen::VectorXd)(multiplier * rbd::dofToVector(robot().mb(), robot().vl()));
      });

      logger().addLogEntry("qpEc_delta_q_dot_upper_bound", [this]() {
        double multiplier = config()("impact")("constraints")("jointVelocity")("multiplier");
        return (Eigen::VectorXd)(multiplier * rbd::dofToVector(robot().mb(), robot().vu()));
      });
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
