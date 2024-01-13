project(mars_model_exaples)

# define example executables
macro(package_add_example FILENAME)
    # create an executable in which the tests will be stored
    add_executable(${FILENAME} ${ARGN})

    # link the Google test infrastructure, mocking library, and a default main function to
    # the test executable.  Remove g_test_main if writing your own main function.
    target_link_libraries(${FILENAME}
        mars_model_lib
        ${CMAKE_DL_LIBS}
    )
    
endmacro()

# add examples
package_add_example(unconstrained_model unconstrained_model.cpp)