function(fox_add_unit_test name)
    add_executable(fox_${name}_test ${name}_test.c)
    target_link_libraries(fox_${name}_test PRIVATE ${ARGN})
    fox_configure_c_target(fox_${name}_test)
    add_test(NAME foxity.${name} COMMAND fox_${name}_test)
    set_tests_properties(foxity.${name} PROPERTIES TIMEOUT 60)
endfunction()

fox_add_unit_test(space fox_core)
fox_add_unit_test(planet fox_core)
fox_add_unit_test(network fox_core)
fox_add_unit_test(traffic fox_core)
# The rules of the road, watched in the demo town tick by tick.
fox_add_unit_test(rules fox_core)
fox_add_unit_test(arguments fox_app)
# The tools' drawing assistance needs only the simulation.
fox_add_unit_test(assist fox_core)
target_sources(fox_assist_test PRIVATE "${PROJECT_SOURCE_DIR}/src/client/assist.c")

if(FOX_BUILD_HEADLESS)
    # An accelerated hour of the demo town, with routing where the engine has it
    # and without it.
    add_test(NAME foxity.headless
        COMMAND foxity_headless --ticks 108000 --time-scale 10000 --cars 150)
    add_test(NAME foxity.headless_no_routing
        COMMAND foxity_headless --ticks 18000 --time-scale 10000 --no-routing --drive left)
    add_test(NAME foxity.headless_paced COMMAND foxity_headless --ticks 15)
    add_test(NAME foxity.headless_rejects_options COMMAND foxity_headless --ticks -1)
    set_tests_properties(foxity.headless_rejects_options PROPERTIES WILL_FAIL TRUE)
    set_tests_properties(foxity.headless foxity.headless_no_routing foxity.headless_paced
        foxity.headless_rejects_options PROPERTIES TIMEOUT 120)
    set_tests_properties(foxity.headless PROPERTIES
        PASS_REGULAR_EXPRESSION "stopped at tick 108000")
    # A loaded grid with the full statistics at the end.
    add_test(NAME foxity.headless_grid
        COMMAND foxity_headless --grid 10 --spacing 80 --cars 800 --ticks 9000
                --time-scale 10000 --stats)
    set_tests_properties(foxity.headless_grid PROPERTIES TIMEOUT 120
        PASS_REGULAR_EXPRESSION "0 dead ends.*-- performance")
endif()
if(FOX_BUILD_CLIENT)
    # Presents three frames through the engine modules copied next to the
    # client; fails when no window and graphics provider can present.
    add_test(NAME foxity.client_smoke COMMAND foxity --smoke)
    set_tests_properties(foxity.client_smoke PROPERTIES
        WORKING_DIRECTORY "$<TARGET_FILE_DIR:foxity>"
        RESOURCE_LOCK gpu
        TIMEOUT 60)

    # Every engine module is optional. Each case stages the client with some
    # modules removed and checks that only the matching technology goes away.
    set(fox_window "$<TARGET_FILE_NAME:laiue::window>")
    set(fox_input "$<TARGET_FILE_NAME:laiue::input>")
    set(fox_math "$<TARGET_FILE_NAME:laiue::scene_math>")
    set(fox_mesher "$<TARGET_FILE_NAME:laiue::mesher>")
    set(fox_navigation)
    if(TARGET laiue::navigation)
        set(fox_navigation "$<TARGET_FILE_NAME:laiue::navigation>")
    endif()
    set(fox_graphics)
    foreach(provider IN ITEMS laiue::graphics_d3d12 laiue::graphics_vulkan)
        if(TARGET ${provider})
            list(APPEND fox_graphics "$<TARGET_FILE_NAME:${provider}>")
        endif()
    endforeach()
    set(fox_no_presentation 4)
    function(fox_add_client_modules_test name expected_exit expected_output)
        # A list cannot cross the test command line intact; '|' separates it.
        list(JOIN ARGN "|" modules)
        add_test(NAME foxity.client_modules.${name}
            COMMAND "${CMAKE_COMMAND}"
                "-DCLIENT=$<TARGET_FILE:foxity>"
                "-DMODULE_DIRECTORY=$<TARGET_FILE_DIR:foxity>"
                "-DTEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/client_modules/${name}"
                "-DMODULES=${modules}"
                "-DEXPECT_EXIT=${expected_exit}"
                "-DEXPECT_OUTPUT=${expected_output}"
                "-DARGUMENTS=--smoke"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/client_modules_test.cmake")
        set_tests_properties(foxity.client_modules.${name} PROPERTIES
            RESOURCE_LOCK gpu
            TIMEOUT 60)
    endfunction()
    fox_add_client_modules_test(none ${fox_no_presentation} "no window and graphics provider")
    fox_add_client_modules_test(without_window ${fox_no_presentation}
        "engine module window is off" ${fox_input} ${fox_graphics})
    fox_add_client_modules_test(without_input 0 "input is unavailable.*3 of 3 frames"
        ${fox_window} ${fox_graphics} ${fox_math} ${fox_mesher} ${fox_navigation})
    fox_add_client_modules_test(without_camera_math 0
        "camera math is unavailable.*3 of 3 frames"
        ${fox_window} ${fox_input} ${fox_graphics} ${fox_mesher} ${fox_navigation})
    fox_add_client_modules_test(without_mesher 0 "surface meshing is unavailable.*3 of 3 frames"
        ${fox_window} ${fox_input} ${fox_graphics} ${fox_math} ${fox_navigation})
    fox_add_client_modules_test(without_navigation 0 "navigation is unavailable.*3 of 3 frames"
        ${fox_window} ${fox_input} ${fox_graphics} ${fox_math} ${fox_mesher})
endif()

add_test(NAME foxity.engine_revision
    COMMAND "${CMAKE_COMMAND}"
        "-DFOX_REVISION_TEST_DIR=${CMAKE_CURRENT_BINARY_DIR}/engine_revision"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/engine_revision_test.cmake")
set_tests_properties(foxity.engine_revision PROPERTIES TIMEOUT 30)