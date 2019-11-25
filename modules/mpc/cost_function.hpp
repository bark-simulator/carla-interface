#include <vector>
#include <iostream>
#include "ceres/ceres.h"
#include <Eigen/Core>

// TODO: use definition in BARK
using Trajectory = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using TrajectoryWeights = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
typedef enum StateDefinition : int {
	TIME_POSITION = 0,  // unit is seconds
	X_POSITION = 1,  // unit is meter
	Y_POSITION = 2,  // unit is meter
	THETA_POSITION = 3,  // unit is rad
	VEL_POSITION = 4,  // unit is meter/second
	MIN_STATE_SIZE = 5,
	Z_POSITION = 6  // only placeholder, not used at the moment
} StateDefinition;

class TrajectoryFunctor {
 public:
	TrajectoryFunctor(Trajectory desired_trajectory, TrajectoryWeights weights, double Lf):
		desired_trajectory_(desired_trajectory),
		weights_(weights) ,
		Lf_(Lf) {}

	template <typename T>
	bool operator()(T const *const *parameters, T *residual);

 private:
	Trajectory desired_trajectory_;
	TrajectoryWeights weights_;
	double Lf_;

	template <typename T>
	inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> KinematicModel(T const *const *parameters,
	    const Trajectory& desired_trajectory, const double& Lf_);
};