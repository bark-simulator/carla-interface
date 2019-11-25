#include "cost_function.hpp"

template <typename T>
bool TrajectoryFunctor::operator()(T const *const *parameters, T *residual) {

	T cost = T(0.0);
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> traj = KinematicModel(parameters, desired_trajectory_, Lf_);
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> desired_trajectory = desired_trajectory_.cast<T>();
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> weights = weights_.cast<T>();

	// offsets
	for (int i = 1 ; i < traj.rows(); i++) {
		T xDiff, yDiff, thetaDiff, velDiff;
		xDiff = traj(i, StateDefinition::X_POSITION) - desired_trajectory(i, StateDefinition::X_POSITION);
		yDiff = traj(i, StateDefinition::Y_POSITION) - desired_trajectory(i, StateDefinition::Y_POSITION);
		thetaDiff = traj(i, StateDefinition::THETA_POSITION) - desired_trajectory(i, StateDefinition::THETA_POSITION);
		velDiff = traj(i, StateDefinition::VEL_POSITION) - desired_trajectory(i, StateDefinition::VEL_POSITION);

		cost += weights(i, StateDefinition::X_POSITION) * xDiff * xDiff;
		cost += weights(i, StateDefinition::Y_POSITION) * yDiff * yDiff;
		cost += weights(i, StateDefinition::THETA_POSITION) * thetaDiff * thetaDiff;
		cost += weights(i, StateDefinition::VEL_POSITION) * velDiff * velDiff;
	}

	// differential values
	// TODO(fortiss): seperate weights
	T weights_acc = T(1);
	T weights_delta = T(1);
	for (int i = 0; i < traj.rows() - 1; i++) {
		cost += weights_acc * parameters[0][i] * parameters[0][i];    // acceleration
		cost += weights_delta * parameters[1][i] * parameters[1][i];  // steering
	}

	residual[0] = cost / (weights.sum() + (weights_acc + weights_delta) * T(traj.rows()));

	return true;
}

template <typename T>
inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> TrajectoryFunctor::KinematicModel(
  T const *const *parameters,  const Trajectory& desired_trajectory, const double& Lf_) {

	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> traj = desired_trajectory.cast<T>();

	T Lf = T(Lf_);
	T dt = T(0.05);

	for (int i = 1; i < traj.rows(); i++) {
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> f_dot(1, 5);

		// fill matrix
		f_dot << T(1),                                                                                         // (0) t' = 1
		      traj(i - 1, StateDefinition::VEL_POSITION) * cos(traj(i - 1, StateDefinition::THETA_POSITION)),  // (1) x_dot = v*cos(psi)
		      traj(i - 1, StateDefinition::VEL_POSITION) * sin(traj(i - 1, StateDefinition::THETA_POSITION)),  // (2) y_dot = v*sin(psi)
		      tan(parameters[1][i - 1]) / Lf,                                                                  // (3) theta_dot = tan(delta)/Lf
		      parameters[0][i - 1];                                                                            // (4) v_dot = a

		traj.block(i, 0, 1, 5) = traj.block(i - 1, 0, 1, 5) + T(dt) * f_dot;
	}
	return traj;
}