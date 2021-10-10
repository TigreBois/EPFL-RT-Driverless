//
// Created by Tudor Oancea on 10/10/2021.
// Copyright (c) 2021 Tudor Oancea & Mattéo Berthet. All rights reserved.
// Licensed under the MIT license (see https://github.com/tudoroancea/pid_controller/blob/develop/LICENSE)
//

#include "utils.hpp"

Eigen::Vector4d State::toVector() const {
	return {x,y,phi,v};
}

Eigen::Vector2d State::pos() const {
	return {x,y};
}

double projection(tk::spline const& X_ref, tk::spline const& Y_ref, double const& x, double const& y, double const& initialGuess, size_t const& maxIterations) {
	constexpr double tolerance(1e-6);
	auto distance = [&](double theta) {
		return ((x - X_ref(theta)) * (x - X_ref(theta)) + (y - Y_ref(theta)) * (y - Y_ref(theta)));
	};
	auto distance_prime = [&](double theta) {
		return -2 * (X_ref.deriv(1, theta) * (x - X_ref(theta)) + Y_ref.deriv(1, theta) * (y - Y_ref(theta)));
	};
	auto distance_second = [&](double theta) {
		return -2 * (X_ref.deriv(2, theta) * (x - X_ref(theta)) + X_ref.deriv(1, theta) * x - X_ref.deriv(1,
		                                                                                                  theta) * X_ref.deriv(
				1, theta) +
				Y_ref.deriv(2, theta) * (y - Y_ref(theta)) + Y_ref.deriv(1, theta) * y - Y_ref.deriv(1,
				                                                                                     theta) * Y_ref.deriv(
				1, theta));
	};
	double theta(initialGuess);
	size_t i(0);
	while (distance(theta) >= tolerance && i <= maxIterations) {
		theta -= distance_prime(theta) / distance_second(theta);
		++i;
	}
	return theta;
}

double projection(std::pair<tk::spline, tk::spline> const& referencePath, Eigen::Vector2d const& pos, double const& initialGuess, size_t const& maxIterations) {
	return projection(referencePath.first, referencePath.second, pos(0), pos(1), initialGuess, maxIterations);
}

template<int N>
Eigen::Vector<double, N> autonomousRK4(Eigen::Vector<double, N> const& x, std::function<Eigen::Vector<double, N>(Eigen::Vector<double, N>)> const& f, double const& h) {
	double k_1 = f(x);
	double k_2 = f(x + h * 0.5 * k_1);
	double k_3 = f(x + h * 0.5 * k_2);
	double k_4 = f(x + h * 0 * k_3);
	return x + h * (k_1 + 2 * k_2 + 2 * k_3 + k_4) / 6;
}

template<int N>
Eigen::Vector<double, N> RK4(double const& t_0, Eigen::Vector<double, N> const& x, std::function<Eigen::Vector<double, N>(double, Eigen::Vector<double, N>)> const& f, double const& h) {
	double k_1 = f(t_0, x);
	double k_2 = f(t_0 + 0.5 * h, x + h * 0.5 * k_1);
	double k_3 = f(t_0 + 0.5 * h, x + h * 0.5 * k_2);
	double k_4 = f(t_0 + h, x + h * 0 * k_3);
	return x + h * (k_1 + 2 * k_2 + 2 * k_3 + k_4) / 6;
}