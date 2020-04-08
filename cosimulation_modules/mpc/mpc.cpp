#include "mpc.hpp"

void MPC::solve(const Trajectory& desired_states, const TrajectoryWeights&  desired_states_weights,
                const double& wheelbase, std::vector<double>& accelerations, std::vector<double>& steering_rates) {
	// TODO: check dimensions

	// make parameter_block compatible with ceres
	std::vector<double *> parameter_block;
	parameter_block.push_back(&accelerations[0]);
	parameter_block.push_back(&steering_rates[0]);

	ceres::DynamicAutoDiffCostFunction<TrajectoryFunctor, StateDefinition::MIN_STATE_SIZE> *cost_function =
	  new ceres::DynamicAutoDiffCostFunction<TrajectoryFunctor, StateDefinition::MIN_STATE_SIZE>(new TrajectoryFunctor(desired_states, desired_states_weights, wheelbase));

	cost_function->SetNumResiduals(1);
	cost_function->AddParameterBlock(desired_states.rows() - 1);
	cost_function->AddParameterBlock(desired_states.rows() - 1);

	ceres::Problem problem;
	problem.AddResidualBlock(cost_function, new ceres::TrivialLoss(), parameter_block);

	ceres::Solve(options_, &problem, &summary_);
}