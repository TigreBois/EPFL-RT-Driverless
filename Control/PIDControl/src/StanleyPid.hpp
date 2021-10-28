//
// Created by Tudor Oancea on 08/10/2021.
// Copyright (c) 2021 Tudor Oancea & Mattéo Berthet. All rights reserved.
// Licensed under the MIT license (see https://github.com/tudoroancea/pid_controller/blob/develop/LICENSE)
//

#ifndef PID_CONTROLLER_STANLEYPID_HPP
#define PID_CONTROLLER_STANLEYPID_HPP

#include <Eigen/Eigen>
#include <chrono>
#include <utility>
#include <vector>
#include <numeric>

#include "spline.hpp"
#include "utils.hpp"

typedef std::pair<tk::spline, tk::spline> Path;


/**
 * @brief at t=t_0 the state is the initial state and the inputs are all 0
 * At t=t_k we compute the inputs for t=t_k+1 and those are directly implemented after the computation, giving a new state at time t=t_k+1
 * At time t=t_k
 */
class StanleyPID {
private:
	static constexpr double k_d = 0.0;
	static constexpr double k_Delta = 0.0;
	static constexpr double k_s = 0.0;
	static constexpr double delta_max = 45.0;
	static constexpr double K_D = 0.0;
	static constexpr double K_I = 0.0;
	static constexpr double K_P = 0.0;

	static constexpr double timeStep = 0.05;

	std::pair<tk::spline, tk::spline> m_path;

	std::function<double(double)> m_v_ref;

	std::vector<State> m_states;
	std::vector<Input> m_inputs;
	std::vector<ComputationHelpers> m_helpers;

public:
	StanleyPID(State const& initialState, Path path, std::function<double(double)> v_ref)
			: m_states(1, initialState),
			  m_path(std::move(path)),
			  m_v_ref(std::move(v_ref)) {}


	void computeInput();

	void addState(State const& newState);

	[[nodiscard]] std::vector<State> const& states() const;

	[[nodiscard]] std::vector<Input> const& inputs() const;

};


#endif //PID_CONTROLLER_STANLEYPID_HPP
