# Stages the client with a chosen subset of engine modules and checks how it
# behaves: every module is optional, and losing one must switch off only what
# it provides. Run with -DCLIENT=<client executable> -DMODULE_DIRECTORY=<dir>
# -DTEST_ROOT=<scratch dir> -DMODULES=<file names present>
# -DEXPECT_EXIT=<accepted exit codes> and optionally -DEXPECT_OUTPUT=<regex>
# -DARGUMENTS=<client arguments>. Lists are separated by '|'.
cmake_minimum_required(VERSION 4.2)

foreach(required IN ITEMS CLIENT MODULE_DIRECTORY TEST_ROOT EXPECT_EXIT)
    if(NOT DEFINED ${required} OR "${${required}}" STREQUAL "")
        message(FATAL_ERROR "${required} is required")
    endif()
endforeach()
foreach(list_variable IN ITEMS MODULES EXPECT_EXIT ARGUMENTS)
    string(REPLACE "|" ";" ${list_variable} "${${list_variable}}")
endforeach()

file(REMOVE_RECURSE "${TEST_ROOT}")
file(MAKE_DIRECTORY "${TEST_ROOT}")
file(COPY "${CLIENT}" DESTINATION "${TEST_ROOT}")
foreach(module IN LISTS MODULES)
    file(COPY "${MODULE_DIRECTORY}/${module}" DESTINATION "${TEST_ROOT}")
endforeach()

get_filename_component(client_name "${CLIENT}" NAME)
execute_process(
    COMMAND "${TEST_ROOT}/${client_name}" ${ARGUMENTS}
    WORKING_DIRECTORY "${TEST_ROOT}"
    RESULT_VARIABLE exit_code
    OUTPUT_VARIABLE output
    ERROR_VARIABLE output
    TIMEOUT 50)
message(STATUS "client output:\n${output}")
if(NOT exit_code IN_LIST EXPECT_EXIT)
    message(FATAL_ERROR "client exited with ${exit_code}, expected one of: ${EXPECT_EXIT}")
endif()
if(DEFINED EXPECT_OUTPUT AND NOT output MATCHES "${EXPECT_OUTPUT}")
    message(FATAL_ERROR "client output does not match: ${EXPECT_OUTPUT}")
endif()
file(REMOVE_RECURSE "${TEST_ROOT}")
