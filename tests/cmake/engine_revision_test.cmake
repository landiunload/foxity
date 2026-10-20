cmake_minimum_required(VERSION 3.28)

include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/EngineRevision.cmake")

# Each resolver call runs in a subprocess so fatal validation errors are tested
# without aborting the rest of the suite.
if(FOX_REVISION_TEST_CHILD)
    fox_resolve_engine_revision(
        "${FOX_REQUESTED_REVISION}" "${FOX_LOCK_FILE}" actual_revision)
    if(NOT actual_revision STREQUAL FOX_EXPECTED_REVISION)
        message(FATAL_ERROR
            "Resolved '${actual_revision}', expected '${FOX_EXPECTED_REVISION}'")
    endif()
    return()
endif()

if(NOT DEFINED FOX_REVISION_TEST_DIR OR
        NOT IS_ABSOLUTE "${FOX_REVISION_TEST_DIR}")
    message(FATAL_ERROR "FOX_REVISION_TEST_DIR must be an absolute build directory")
endif()
file(MAKE_DIRECTORY "${FOX_REVISION_TEST_DIR}")

set(test_count 0)
function(check_revision name requested write_lock contents expected error_pattern)
    set(lock_file "${FOX_REVISION_TEST_DIR}/${name}.lock")
    if(write_lock)
        file(WRITE "${lock_file}" "${contents}")
    elseif(EXISTS "${lock_file}")
        message(FATAL_ERROR "Missing-lock fixture unexpectedly exists: ${lock_file}")
    endif()
    execute_process(
        COMMAND "${CMAKE_COMMAND}"
            "-DFOX_REVISION_TEST_CHILD=ON"
            "-DFOX_REQUESTED_REVISION=${requested}"
            "-DFOX_LOCK_FILE=${lock_file}"
            "-DFOX_EXPECTED_REVISION=${expected}"
            -P "${CMAKE_CURRENT_FUNCTION_LIST_FILE}"
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE error
        TIMEOUT 10)
    if(error_pattern STREQUAL "")
        if(NOT result STREQUAL "0")
            message(FATAL_ERROR "${name}: resolver failed (${result})\n${output}${error}")
        endif()
    elseif(result STREQUAL "0" OR NOT "${output}${error}" MATCHES "${error_pattern}")
        message(FATAL_ERROR
            "${name}: expected failure matching '${error_pattern}', got (${result})\n${output}${error}")
    endif()
    math(EXPR next_count "${test_count} + 1")
    set(test_count "${next_count}" PARENT_SCOPE)
endfunction()

set(sha "0123456789abcdef0123456789abcdef01234567")
set(other_sha "fedcba9876543210fedcba9876543210fedcba98")
set(invalid_lock "engine.lock must hold exactly one 40-character laiue commit SHA")

# Development must retain a remote-tracking ref, not silently pin engine.lock.
check_revision(remote_main "origin/main" FALSE "" "origin/main" "")
check_revision(remote_main_ignores_lock "origin/main" TRUE "${sha}\n" "origin/main" "")
check_revision(remote_main_ignores_invalid_lock "origin/main" TRUE "not-a-sha\n" "origin/main" "")
check_revision(branch "feature/live-engine" FALSE "" "feature/live-engine" "")
check_revision(tag "refs/tags/v1.2.3" FALSE "" "refs/tags/v1.2.3" "")
check_revision(explicit_sha "${sha}" FALSE "" "${sha}" "")
check_revision(explicit_sha_ignores_duplicate_lock "${sha}" TRUE
    "${sha}\n${other_sha}\n" "${sha}" "")
check_revision(empty_request "" FALSE "" "" "FOX_ENGINE_REVISION must not be empty")

check_revision(locked_plain "locked" TRUE "${sha}" "${sha}" "")
check_revision(locked_newline "locked" TRUE "${sha}\n" "${sha}" "")
check_revision(locked_whitespace "locked" TRUE "\n  ${sha}\t\n\n" "${sha}" "")
check_revision(locked_comments "locked" TRUE
    "# Engine revision\n\n  # Another comment\n${sha}\n# End\n" "${sha}" "")
check_revision(locked_crlf "locked" TRUE "# Windows\r\n\r\n${sha}\r\n" "${sha}" "")
check_revision(locked_missing "locked" FALSE "" ""
    "engine.lock is required for FOX_ENGINE_REVISION=locked")
check_revision(locked_empty "locked" TRUE "" "" "${invalid_lock}")
check_revision(locked_comments_only "locked" TRUE "\n# No revision\n  # Still none\n" "" "${invalid_lock}")
check_revision(locked_short "locked" TRUE "0123456\n" "" "${invalid_lock}")
check_revision(locked_long "locked" TRUE "${sha}0\n" "" "${invalid_lock}")
string(TOUPPER "${sha}" upper_sha)
check_revision(locked_uppercase "locked" TRUE "${upper_sha}\n" "" "${invalid_lock}")
check_revision(locked_nonhex "locked" TRUE "0123456789abcdef0123456789abcdef0123456g\n" "" "${invalid_lock}")
check_revision(locked_inline_comment "locked" TRUE "${sha} # Not a full-line comment\n" "" "${invalid_lock}")
check_revision(locked_duplicate "locked" TRUE "${sha}\n${sha}\n" "" "${invalid_lock}")
check_revision(locked_multiple "locked" TRUE "${sha}\n${other_sha}\n" "" "${invalid_lock}")
check_revision(locked_trailing_invalid "locked" TRUE "${sha}\ninvalid\n" "" "${invalid_lock}")

message(STATUS "Engine revision regression tests passed: ${test_count} cases")
