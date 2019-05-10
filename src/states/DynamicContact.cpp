#include "DynamicContact.h"

#include "../mc_r_arm_push_unknown_wall.h"

#include <mc_tasks/MetaTaskLoader.h>

void DynamicContactState::configure(const mc_rtc::Configuration & config)
{
  state_conf_ = config;
}


void DynamicContactState::start(mc_control::fsm::Controller & ctlInput)
{
	auto & ctl = static_cast<Controller&>(ctlInput);

  const auto & conf = ctl.config()("hrp4");

	rPosTaskPtr_ = std::make_shared<mc_tasks::PositionTask>(
			conf("rightEfTask")("bodyName"),
			ctl.robots(),
			0,
			conf("rightEfTask")("stiffness"),
			conf("rightEfTask")("weight")
			);
	ctl.solver().addTask(rPosTaskPtr_);

	Eigen::Vector3d referenceVelocity, pushDepth, currentPos;
	pushDepth= ctl.config()("states")("Contact")("pushDepth");
	referenceVelocity= ctl.config()("states")("Contact")("contactVelocity");

	currentPos = rPosTaskPtr_->position();
	// Set the target position, which is supposed to be far away.
	rPosTaskPtr_->position(currentPos + pushDepth);
	// Set the reference velocity
	rPosTaskPtr_->refVel(referenceVelocity);

  // Set the damping gain (set it much much higher than the position
  // gain[stiffness] to ensure that velocity is properly tracked
  if(state_conf_.has("rightEfStiffness"))
  {
    rPosTaskPtr_->stiffness(static_cast<double>(state_conf_("rightEfStiffness")));
  }
  if(state_conf_.has("rightEfDamping"))
  {
    rPosTaskPtr_->damping(static_cast<double>(state_conf_("rightEfDamping")));
  }
}

bool DynamicContactState::run(mc_control::fsm::Controller & ctlInput)
{
	// std::cout<<"The left ef error is: "<< lEfTaskPtr_->eval().norm() << ", the right ef error is: " << rEfTaskPtr_->eval().norm() <<std::endl;
 	auto & ctl = static_cast<Controller&>(ctlInput);

	ctl.miPredictorPtr->run();

	if( rPosTaskPtr_->eval().norm() <= 0.01)
	{
		// Output the transition signal such that we can move on according to the transitions
		// ctl.solver().removeTask(rPosTaskPtr_);
		output("RightHandFinished");
		return true;
	}

	return false;
}

void DynamicContactState::teardown(mc_control::fsm::Controller & ctl_ )
{
	auto & ctl = static_cast<Controller&>(ctl_);
	ctl.solver().removeTask(rPosTaskPtr_);
	ctl.solver().removeTask(ctl.comTaskPtr);


}


EXPORT_SINGLE_STATE("DynamicContact", DynamicContactState)
