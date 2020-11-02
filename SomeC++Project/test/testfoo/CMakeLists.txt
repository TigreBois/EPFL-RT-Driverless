set( BIN_NAME testFoo )

add_executable(${BIN_NAME} 
    main.cpp
    example_add.cpp
    example_subtract.cpp
    )
target_link_libraries( ${BIN_NAME} ${GTEST_LIBRARIES} )
target_link_libraries(${BIN_NAME} 
    gtest_main
    example
    )

#add_test(
#  NAME
#    unit
#  COMMAND
#    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/unit_tests
#  )

SETUP_TARGET_FOR_COVERAGE(
    coverage_${BIN_NAME}  # Name for custom target.
    ${BIN_NAME}         # Name of the test driver executable that runs the tests.
    # NOTE! This should always have a ZERO as exit code
    # otherwise the coverage generation will not complete.
    coverage_${BIN_NAME}_dir            # Name of output directory.
    )
