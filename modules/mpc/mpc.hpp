#include <vector>
#include <iostream>
#include "ceres/ceres.h"
#include <Eigen/Core>

#include "cost_function.hpp"

// TODO: use definition in BARK
using Trajectory = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using TrajectoryWeights = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

class MPC {
 public:
	MPC(const int& interations = 4000, const double& function_tolerance = 10e-8) {
		options_.max_num_iterations = interations;
		options_.function_tolerance = function_tolerance;

		options_.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
		options_.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;

		options_.use_inner_iterations = false;
		options_.logging_type = ceres::SILENT;
		options_.minimizer_progress_to_stdout = false;
	}

	void solve(const Trajectory& desired_states, const TrajectoryWeights&  desired_states_weights,
	           const double& wheelbase, std::vector<double>& accelerations, std::vector<double>& steering_rate);

 private:
	ceres::Solver::Options options_;
	ceres::Solver::Summary summary_;
};