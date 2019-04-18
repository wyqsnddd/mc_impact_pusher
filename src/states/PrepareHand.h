# pragma once

# include <mc_control/fsm/State.h>
# include <mc_tasks/EndEffectorTask.h>


struct PrepareHandState: mc_control::fsm::State{
	void configure(const mc_rtc::Configuration & config) override;
	void start(mc_control::fsm::Controller&) override;
	bool run(mc_control::fsm::Controller&) override;
	void teardown(mc_control::fsm::Controller&) override;

	protected:
	std::shared_ptr<mc_tasks::EndEffectorTask> rEfTaskPtr_;
	sva::PTransformd rTransformZero_;
	double efThreshold_;
  double rEfStiffness_;
};
