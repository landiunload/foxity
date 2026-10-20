# Deterministic simulation: standard C only, no engine. Everything that has to
# compute the same city on a server, every client and a replay lives here.
add_library(fox_core STATIC
    sim/clock.c
    sim/clock.h
    sim/lanes.c
    sim/network.c
    sim/network.h
    sim/network_internal.h
    sim/planet.c
    sim/planet.h
    sim/random.c
    sim/random.h
    sim/router.h
    sim/space.c
    sim/space.h
    sim/traffic.c
    sim/traffic.h
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
        client/planet_view.c
        client/planet_view.h
        client/roads_view.c
        client/roads_view.h
        client/scene.c
        client/scene.h)
    target_link_libraries(fox_client PUBLIC fox_app)
    fox_configure_c_target(fox_client)

    add_executable(foxity WIN32
        client/client.c
        client/client.h
        client/client_main.c)
    target_link_libraries(foxity PRIVATE fox_client)
    if(WIN32)
        target_link_libraries(foxity PRIVATE shell32)
    endif()
    fox_configure_c_target(foxity)
    set_target_properties(foxity PROPERTIES
        OUTPUT_NAME "Foxity"
        VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:foxity>")
    fox_deploy_engine_modules(foxity ${FOX_CLIENT_ENGINE_MODULES})
endif()