#pragma once 

# include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>



struct Controller : public mc_control::fsm::Controller
{
	Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf);

	void reset(const mc_control::ControllerResetData & data) override;
	
	std::shared_ptr<mc_tasks::MetaTask> comTaskPtr;
	mc_rbdyn::Robots robotsGRF_;

	// Force sensor threshold
	double forceThreshold = 3.0;

	bool firstContact = true;

	bool rArmInContact();
	
};
