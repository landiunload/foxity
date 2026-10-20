include_guard(GLOBAL)

function(fox_release_bundle_strip_command
    out_variable bundle_directory executable_target)
    set(strip_command)
    if(NOT WIN32)
        list(APPEND strip_command
            COMMAND "${CMAKE_COMMAND}"
                "-DBUNDLE_DIRECTORY=${bundle_directory}"
                "-DCONFIGURATION=$<CONFIG>"
                "-DSTRIP_TOOL=${CMAKE_STRIP}"
                "-DAPPLE_PLATFORM=${APPLE}"
                "-DEXECUTABLE_NAME=$<TARGET_FILE_NAME:${executable_target}>"
                -P "${PROJECT_SOURCE_DIR}/cmake/StripReleaseBundle.cmake")
    endif()
    set(${out_variable} "${strip_command}" PARENT_SCOPE)
endfunction()

# The game loads engine modules by name from its own directory instead of
# linking them. This copies the listed module artifacts there on every build:
# a prebuilt SDK module can change without relinking the game, and a stale
# copy would hide that change.
function(fox_deploy_engine_modules target_name)
    set(module_files)
    set(module_targets)
    foreach(module IN LISTS ARGN)
        if(NOT TARGET "${module}")
            message(FATAL_ERROR "Unknown engine module target: ${module}")
        endif()
        list(APPEND module_files "$<TARGET_FILE:${module}>")
        get_target_property(module_imported "${module}" IMPORTED)
        if(NOT module_imported)
            list(APPEND module_targets "${module}")
        endif()
    endforeach()
    if(NOT module_files)
        return()
    endif()
    add_custom_target(${target_name}_engine_modules ALL
        COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            ${module_files} "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND_EXPAND_LISTS
        COMMENT "Copying engine modules next to ${target_name}"
        VERBATIM)
    if(module_targets)
        add_dependencies(${target_name}_engine_modules ${module_targets})
    endif()
    set_property(TARGET ${target_name} PROPERTY FOX_ENGINE_MODULE_FILES "${module_files}")
endfunction()
