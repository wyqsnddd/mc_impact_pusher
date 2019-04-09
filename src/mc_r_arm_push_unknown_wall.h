#pragma once 

# include <mc_control/fsm/Controller.h>


struct Controller : public mc_control::fsm::Controller
{
	Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf);

	void reset(const mc_control::ControllerResetData & data) override;
	
	std::shared_ptr<mc_tasks::MetaTask> comTaskPtr;

	// Force sensor threshold
	double forceThreshold = 3.0;

	bool firstContact = true;

	bool rArmInContact();
};
