//
// Created by Tudor Oancea on 08/10/2021.
// Copyright (c) 2021 Tudor Oancea & Mattéo Berthet. All rights reserved.
// Licensed under the MIT license (see https://github.com/tudoroancea/pid_controller/blob/develop/LICENSE)
//

#include "StanleyPid.hpp"


void StanleyPID::addState(const State& newState) {
	m_states.push_back(newState);
}

void StanleyPID::computeInput() {
	double theta_proj_k = projection(m_path, m_states.back().pos(), m_helpers.back().theta);

	// compute the new speed control input u
	double error_k = m_v_ref(theta_proj_k) - m_states.back().v;
	double u_k = (K_P * m_states.back().v + 5) * error_k;
	if (m_states.size() >= 2) {
		// not the first computation so we can compute the integral and derivative terms
		double error_prev = m_v_ref(m_helpers.back().theta) - m_states[m_states.size() - 2].v;
		u_k += K_I * m_states.back().v * 0.5 * (error_k + error_prev) * timeStep + K_D * m_states.back().v * (error_k - error_prev) / timeStep;
	}

	// compute steering angle delta
	double e_Delta = (m_path.first(theta_proj_k) - m_states.back().x) * (m_path.first(
			theta_proj_k) - m_states.back().x) + (m_path.second(theta_proj_k) - m_states.back().y) * (m_path.second(
			theta_proj_k) - m_states.back().y);
	double e_phi = std::acos((std::cos(m_states.back().phi) * m_path.first.deriv(1, theta_proj_k) +
			std::sin(m_states.back().phi) * m_path.second.deriv(1, theta_proj_k)) /
			                         (m_path.first.deriv(1, theta_proj_k) * m_path.first.deriv(1, theta_proj_k) +
					                         m_path.second.deriv(1, theta_proj_k) * m_path.second.deriv(1,
					                                                                                    theta_proj_k)));
	double delta_k = e_phi + std::atan(k_Delta * e_Delta / (k_s + k_d * m_states.back().v));

	if (delta_k > delta_max) {
		delta_k = delta_max;
	} else if (delta_k < -delta_max) {
		delta_k = -delta_max;
	}

	m_inputs.emplace_back(delta_k, u_k);
	m_helpers.emplace_back(theta_proj_k, std::chrono::high_resolution_clock::now());
}

std::vector<State> const& StanleyPID::states() const {
	return m_states;
}

std::vector<Input> const& StanleyPID::inputs() const {
	return m_inputs;
}