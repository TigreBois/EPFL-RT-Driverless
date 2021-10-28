#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-narrowing-conversions"

#include <iostream>
#include <Eigen/Eigen>
#include <chrono>
#include <vector>
#include <numeric>

#include "spline.hpp"
#include "StanleyPid.hpp"
#include "utils.hpp"

using namespace Eigen;

int main() {
	constexpr size_t steps(50);
	std::vector<double> T(steps), Xpoints(steps), Ypoints(steps, 0.0);
	std::iota(T.begin(), T.end(), 0);
	std::iota(Xpoints.begin(), Xpoints.end(), 0);
	StanleyPID controller(State(0.0, 0.0, 0.0, 0.0), {tk::spline(T, Xpoints), tk::spline(T, Ypoints)}, std::function<double(double)>());

	std::function<Eigen::Vector4d(Eigen::Vector4d)> f;

	for (size_t i(1); i <= 10; ++i) {
		State lastState(controller.states().back());
		std::cout << lastState.x << "\t" << lastState.y << "\t" << lastState.phi << "\t" << lastState.v << std::endl;
		controller.computeInput();
		controller.addState(State(autonomousRK4<4>(lastState.toVector(), f)));
	}

	return 0;
}

#pragma clang diagnostic pop