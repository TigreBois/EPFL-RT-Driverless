set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(
    pid_controller
    main.cpp
    spline.hpp
    StanleyPid.cpp
    StanleyPid.hpp
    utils.hpp
    utils.cpp
)

target_link_libraries(pid_controller Eigen3::Eigen)