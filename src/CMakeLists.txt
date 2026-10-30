# Deterministic simulation: standard C only, no engine. Everything that has to
# compute the same city on a server, every client and a replay lives here.
add_library(fox_core STATIC
    sim/clock.c
    sim/clock.h
    sim/lanes.c
    sim/network.c
    sim/network.h
    sim/network_internal.h
    sim/signals.c
    sim/planet.c
    sim/planet.h
    sim/random.c
    sim/random.h
    sim/router.h
    sim/space.c
    sim/space.h
    sim/traffic.c
    sim/traffic.h
    sim/walkers.c
    sim/walkers.h
    sim/world.c
    sim/world.h)
target_include_directories(fox_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
if(NOT WIN32)
    target_link_libraries(fox_core PUBLIC m)
endif()
fox_configure_c_target(fox_core)

# Composition: process lifecycle, engine modules loaded by name, the router on
# the engine's navigation, logging. The engine is reached only through the
# static bootstrap and service tables.
add_library(fox_app STATIC
    app/arguments.c
    app/arguments.h
    app/city.c
    app/city.h
    app/headless.c
    app/headless.h
    app/log.c
    app/log.h
    app/modules.c
    app/modules.h
    app/navigation_router.c
    app/navigation_router.h)
target_link_libraries(fox_app PUBLIC fox_core laiue::bootstrap)
if(ANDROID)
    # The log goes to logcat.
    target_link_libraries(fox_app PUBLIC log)
endif()
fox_configure_c_target(fox_app)

if(FOX_BUILD_HEADLESS)
    add_executable(foxity_headless app/headless_main.c)
    target_link_libraries(foxity_headless PRIVATE fox_app)
    fox_configure_c_target(foxity_headless)
    set_target_properties(foxity_headless PROPERTIES OUTPUT_NAME "FoxityHeadless")
    fox_deploy_engine_modules(foxity_headless ${FOX_SIMULATION_ENGINE_MODULES})
endif()
# What the mayor sees and steers: the scene, its views, the camera and the
# tools. It reaches the engine only through service tables.
if(FOX_BUILD_CLIENT)
    add_library(fox_client STATIC
        client/camera.c
        client/camera.h
        client/cars_view.c
        client/cars_view.h
        client/assist.c
        client/assist.h
        client/hud.c
        client/hud.h
        client/planet_view.c
        client/planet_view.h
        client/road_mesh.c
        client/road_mesh.h
        client/roads_view.c
        client/roads_view.h
        client/scene.c
        client/scene.h
        client/ui.c
        client/ui.h)
    target_link_libraries(fox_client PUBLIC fox_app)
    fox_configure_c_target(fox_client)

    # Android starts programs from libraries: the engine's window module is
    # the activity and calls this library's main.
    if(ANDROID)
        add_library(foxity SHARED
            client/client.c
            client/client.h
            client/client_main.c)
        # Only main is looked up; everything else, the static libraries
        # included, stays inside.
        set_target_properties(foxity PROPERTIES C_VISIBILITY_PRESET hidden)
        target_link_options(foxity PRIVATE "LINKER:--exclude-libs,ALL")
    else()
        add_executable(foxity WIN32
            client/client.c
            client/client.h
            client/client_main.c)
    endif()
    target_link_libraries(foxity PRIVATE fox_client)
    if(WIN32)
        target_link_libraries(foxity PRIVATE shell32)
    endif()
    fox_configure_c_target(foxity)
    set_target_properties(foxity PROPERTIES
        OUTPUT_NAME "Foxity"
        VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:foxity>")
    if(ANDROID)
        # The APK carries the engine modules as separate libraries, loaded by
        # name as on the desktop. Packaging comes with an engine built from
        # sources; a prepared SDK leaves it to the application.
        if(COMMAND laiue_add_android_apk)
            laiue_add_android_apk(foxity_apk
                MAIN_TARGET foxity
                PACKAGE dev.landiunload.foxity
                LABEL Foxity
                ORIENTATION sensorLandscape
                MODULES ${FOX_CLIENT_ENGINE_MODULES})
        else()
            message(STATUS "Foxity APK: this laiue build cannot package APKs")
        endif()
    else()
        fox_deploy_engine_modules(foxity ${FOX_CLIENT_ENGINE_MODULES})
    endif()

    # For testers only, never shipped: P saves the frame with a comment.
    if(FOX_REVIEW_BUILD)
        add_executable(foxity_review WIN32
            client/client.c
            client/client.h
            client/client_main.c
            client/review.c
            client/review.h)
        target_compile_definitions(foxity_review PRIVATE FOX_REVIEW)
        target_link_libraries(foxity_review PRIVATE fox_client)
        if(WIN32)
            target_link_libraries(foxity_review PRIVATE shell32)
        endif()
        fox_configure_c_target(foxity_review)
        set_target_properties(foxity_review PROPERTIES
            OUTPUT_NAME "FoxityReview"
            VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:foxity_review>")
        fox_deploy_engine_modules(foxity_review ${FOX_CLIENT_ENGINE_MODULES})
    endif()
endif()