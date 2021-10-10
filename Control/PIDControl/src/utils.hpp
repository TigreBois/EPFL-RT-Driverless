//
// Created by Tudor Oancea on 09/10/2021.
// Copyright (c) 2021 Tudor Oancea & Mattéo Berthet. All rights reserved.
// Licensed under the MIT license (see https://github.com/tudoroancea/pid_controller/blob/develop/LICENSE)
//

#ifndef PID_CONTROLLER_UTILS_HPP
#define PID_CONTROLLER_UTILS_HPP

#include <chrono>
#include <Eigen/Eigen>

#include "spline.hpp"

typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimePoint;

struct State {
	double x;
	double y;
	double phi;
	double v;

	State(double const& x, double const& y, double const& phi, double const& v): x(x), y(y), phi(phi), v(v) {}
	State(Eigen::Vector4d const& values) : x(values(0)), y(values(1)), phi(values(2)), v(values(3)) {}

	[[nodiscard]] Eigen::Vector4d toVector() const;
	[[nodiscard]] Eigen::Vector2d pos() const;
};

struct Input {
	double delta;
	double u;

	Input(double const& delta, double const& v) : delta(delta), u(v) {}
};

struct ComputationHelpers {
	ComputationHelpers(double const& theta, TimePoint const& timePoint) : theta(theta), timepoint(timePoint) {}

	double theta;
	TimePoint timepoint;
};

double projection(tk::spline const& X_ref, tk::spline const& Y_ref, double const& x, double const& y, double const& initialGuess, size_t const& maxIterations = 1000);

double projection(std::pair<tk::spline, tk::spline> const& referencePath, Eigen::Vector2d const& pos, double const& initialGuess, size_t const& maxIterations = 1000);

#include <Eigen/Eigen>
#include <chrono>
#include <utility>
#include <vector>
#include <numeric>
#include "spline.hpp"

/**
 * @brief Runge-Kutta scheme of order 4 for non-autonomous ODEs : of the form dx/dt = f(t,x)
 * @tparam N size of the vectors
 * @param t_0 time at which x is known
 * @param x vector to integrate at time t=t_0
 * @param f function giving the dynamics
 * @param h integration step-size
 * @return x at time t=t_0+h
 */
template<int N>
Eigen::Vector<double, N> RK4(double const& t_0, Eigen::Vector<double, N> const& x, std::function<Eigen::Vector<double, N>(double, Eigen::Vector<double, N>)> const& f, double const& h = 1e-2);

/**
 * @brief Runge-Kutta scheme of order 4 for autonomous ODEs : of the form dx/dt = f(x)
 * @tparam N size of the vectors
 * @param x vector to integrate at time t=t_0
 * @param f function giving the dynamics
 * @param h integration step-size
 * @return x at time t=t_0+h
 */
template<int N>
Eigen::Vector<double, N> autonomousRK4(Eigen::Vector<double, N> const& x, std::function<Eigen::Vector<double, N>(Eigen::Vector<double, N>)> const& f, double const& h = 1e-2);



#endif //PID_CONTROLLER_UTILS_HPP
