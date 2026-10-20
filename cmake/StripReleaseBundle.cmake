if(NOT CONFIGURATION STREQUAL "Release")
    return()
endif()
if(NOT DEFINED STRIP_TOOL OR STRIP_TOOL STREQUAL "")
    message(FATAL_ERROR "Release bundle requires a strip tool")
endif()
if(NOT IS_DIRECTORY "${BUNDLE_DIRECTORY}")
    message(FATAL_ERROR "Bundle directory does not exist: ${BUNDLE_DIRECTORY}")
endif()

set(bundle_binaries "${BUNDLE_DIRECTORY}/${EXECUTABLE_NAME}")
file(GLOB bundle_libraries LIST_DIRECTORIES FALSE
    "${BUNDLE_DIRECTORY}/*.so"
    "${BUNDLE_DIRECTORY}/*.so.*"
    "${BUNDLE_DIRECTORY}/*.dylib")
list(APPEND bundle_binaries ${bundle_libraries})
foreach(bundle_binary IN LISTS bundle_binaries)
    if(NOT EXISTS "${bundle_binary}")
        message(FATAL_ERROR "Bundle binary does not exist: ${bundle_binary}")
    endif()
    if(APPLE_PLATFORM)
        set(strip_arguments -x)
    else()
        set(strip_arguments --strip-unneeded)
    endif()
    execute_process(
        COMMAND "${STRIP_TOOL}" ${strip_arguments} "${bundle_binary}"
        RESULT_VARIABLE strip_result
        ERROR_VARIABLE strip_error)
    if(NOT strip_result EQUAL 0)
        message(FATAL_ERROR
            "Could not strip ${bundle_binary}: ${strip_error}")
    endif()
endforeach()
