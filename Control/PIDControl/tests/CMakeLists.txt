set( BIN_NAME tests )

add_executable(${BIN_NAME} 
    main.cpp
    example_add.cpp
    example_subtract.cpp
    )
target_link_libraries( ${BIN_NAME} ${GTEST_LIBRARIES} )
target_link_libraries(${BIN_NAME} 
    example
    )

SETUP_TARGET_FOR_COVERAGE(
    coverage_${BIN_NAME}  # Name for custom target.
    ${BIN_NAME}         # Name of the test driver executable that runs the tests.
    # NOTE! This should always have a ZERO as exit code
    # otherwise the coverage generation will not complete.
    ${BIN_NAME}_coverage_report            # Name of output directory and coverage report.
    )


