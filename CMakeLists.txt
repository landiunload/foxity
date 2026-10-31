cmake_minimum_required(VERSION 4.2)

project(foxity
    VERSION 0.1.0
    DESCRIPTION "Foxity, a city builder on a small planet"
    LANGUAGES C)

set(FOX_ENGINE_SOURCE_DIR "" CACHE PATH
    "Optional laiue source tree for platform superbuild integration")
option(FOX_BUILD_ENGINE_TESTS
    "Build laiue's own tests when using FOX_ENGINE_SOURCE_DIR" OFF)
option(FOX_ENGINE_FETCH
    "Fetch laiue when no source tree or SDK is found" ON)
set(FOX_ENGINE_REVISION "origin/main" CACHE STRING
    "Fetched laiue Git revision; origin/main tracks development, locked uses engine.lock")
option(FOX_BUILD_HEADLESS "Build the headless simulation executable" ON)
option(FOX_REVIEW_BUILD "Build FoxityReview, which saves frames with testers' comments; never shipped" OFF)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Foxity supports only 64-bit targets")
endif()

# Keep distributable game binaries independent of an installed VC Runtime.
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY
        "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(FoxityTarget)
include(RuntimeDeployment)
include(CTest)

# Игра и модули движка, собранные вместе с ней, уходят одним комплектом, и
# процессор должен подходить всему комплекту: уровень x86-64 и настройка под
# процессор у них общие и задаются только переменными FOX_*.
set(LAIUE_X86_64_LEVEL "${FOX_X86_64_LEVEL}" CACHE INTERNAL "")
set(LAIUE_X86_64_TUNE "${FOX_X86_64_TUNE}" CACHE INTERNAL "")

# Движок и игра развиваются вместе, поэтому по умолчанию игра собирает
# соседний `../laiue` из исходников: правка движка сразу видна игре.
# Заранее собранный SDK выбирается явным `laiue_DIR`. Обе дороги дают одни и
# те же цели `laiue::*` и одни и те же пути заголовков.
if(NOT FOX_ENGINE_SOURCE_DIR AND NOT laiue_DIR)
    get_filename_component(FOX_ENGINE_SIBLING_DIR
        "${PROJECT_SOURCE_DIR}/../laiue" ABSOLUTE)
    if(EXISTS "${FOX_ENGINE_SIBLING_DIR}/CMakeLists.txt")
        set(FOX_ENGINE_SOURCE_DIR "${FOX_ENGINE_SIBLING_DIR}")
    endif()
endif()

if(NOT FOX_ENGINE_SOURCE_DIR AND NOT laiue_DIR AND FOX_ENGINE_FETCH)
    # Без соседнего дерева сборка следует текущему движку: FetchContent
    # обновляет origin/main при configure. Для воспроизводимой сборки можно
    # явно выбрать locked или commit SHA.
    include(EngineRevision)
    fox_resolve_engine_revision("${FOX_ENGINE_REVISION}"
        "${PROJECT_SOURCE_DIR}/engine.lock" FOX_ENGINE_FETCH_REVISION)
    message(STATUS
        "laiue: no source tree beside the game, fetching ${FOX_ENGINE_FETCH_REVISION}")

    include(FetchContent)
    FetchContent_Declare(laiue
        GIT_REPOSITORY "https://github.com/landiunload/laiue.git"
        GIT_TAG "${FOX_ENGINE_FETCH_REVISION}")
    set(FOX_GAME_BUILD_TESTING "${BUILD_TESTING}")
    set(BUILD_TESTING ${FOX_BUILD_ENGINE_TESTS})
    FetchContent_MakeAvailable(laiue)
    set(BUILD_TESTING "${FOX_GAME_BUILD_TESTING}")
    set(FOX_ENGINE_SOURCE_DIR "${laiue_SOURCE_DIR}")
    set(FOX_ENGINE_ALREADY_ADDED ON)
endif()

if(FOX_ENGINE_SOURCE_DIR)
    get_filename_component(FOX_ENGINE_SOURCE_DIR
        "${FOX_ENGINE_SOURCE_DIR}" ABSOLUTE BASE_DIR "${PROJECT_SOURCE_DIR}")
    if(NOT EXISTS "${FOX_ENGINE_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "FOX_ENGINE_SOURCE_DIR is not a laiue source tree: "
            "${FOX_ENGINE_SOURCE_DIR}")
    endif()
    message(STATUS "laiue: building from sources at ${FOX_ENGINE_SOURCE_DIR}")
    if(NOT FOX_ENGINE_ALREADY_ADDED)
        set(FOX_GAME_BUILD_TESTING "${BUILD_TESTING}")
        set(BUILD_TESTING ${FOX_BUILD_ENGINE_TESTS})
        add_subdirectory("${FOX_ENGINE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/_engine")
        set(BUILD_TESTING "${FOX_GAME_BUILD_TESTING}")
    endif()
elseif(laiue_DIR)
    message(STATUS "laiue: using the prepared SDK at ${laiue_DIR}")
    find_package(laiue 0.1.0 CONFIG REQUIRED)
else()
    message(FATAL_ERROR
        "No laiue engine available. Clone https://github.com/landiunload/laiue "
        "next to this repository, or point FOX_ENGINE_SOURCE_DIR at a source "
        "tree, or laiue_DIR at a prepared SDK. Re-enable FOX_ENGINE_FETCH to "
        "let the build download FOX_ENGINE_REVISION (origin/main by default).")
endif()

if(NOT TARGET laiue::bootstrap)
    message(FATAL_ERROR "The selected laiue build does not provide laiue::bootstrap")
endif()

# Routing comes from laiue.navigation when the engine has it; without it the
# cars still drive and choose their turns at random.
set(FOX_SIMULATION_ENGINE_MODULES)
if(TARGET laiue::navigation)
    list(APPEND FOX_SIMULATION_ENGINE_MODULES laiue::navigation)
endif()

# The client needs a window, input and at least one graphics provider; camera
# math and surface meshing draw the planet. The game only copies these modules
# next to itself and loads them at run time, so a build that lacks the first
# ones simply has no client, and one without the others shows less.
set(FOX_CLIENT_ENGINE_MODULES ${FOX_SIMULATION_ENGINE_MODULES})
foreach(module IN ITEMS laiue::window laiue::input laiue::graphics_d3d12 laiue::graphics_vulkan
                        laiue::scene_math laiue::mesher)
    if(TARGET ${module})
        list(APPEND FOX_CLIENT_ENGINE_MODULES ${module})
    endif()
endforeach()
set(FOX_CLIENT_AVAILABLE OFF)
if(TARGET laiue::window AND TARGET laiue::input AND
   (TARGET laiue::graphics_d3d12 OR TARGET laiue::graphics_vulkan))
    set(FOX_CLIENT_AVAILABLE ON)
endif()
option(FOX_BUILD_CLIENT "Build the interactive client" ${FOX_CLIENT_AVAILABLE})
if(FOX_BUILD_CLIENT AND NOT FOX_CLIENT_AVAILABLE)
    message(FATAL_ERROR
        "FOX_BUILD_CLIENT needs a laiue build with window, input and a "
        "graphics provider; disable the client for a headless build")
endif()
message(STATUS "Foxity client: ${FOX_BUILD_CLIENT}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/$<CONFIG>")

add_subdirectory(src)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# Self-contained folders to hand out: the executable plus the engine modules
# it loads. Release bundles are stripped outside Windows; Android hands out
# the APK instead.
if(TARGET foxity AND NOT ANDROID)
    set(FOX_BUNDLE_DIRECTORY "${CMAKE_BINARY_DIR}/bundles/Foxity/$<CONFIG>")
    fox_release_bundle_strip_command(FOX_CLIENT_BUNDLE_STRIP_COMMAND
        "${FOX_BUNDLE_DIRECTORY}" foxity)
    get_target_property(FOX_CLIENT_MODULE_FILES foxity FOX_ENGINE_MODULE_FILES)
    add_custom_target(foxity_bundle
        COMMAND "${CMAKE_COMMAND}" -E remove_directory "${FOX_BUNDLE_DIRECTORY}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${FOX_BUNDLE_DIRECTORY}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:foxity>" ${FOX_CLIENT_MODULE_FILES}
            "${FOX_BUNDLE_DIRECTORY}"
        ${FOX_CLIENT_BUNDLE_STRIP_COMMAND}
        DEPENDS foxity foxity_engine_modules
        COMMAND_EXPAND_LISTS
        COMMENT "Staging the Foxity bundle"
        VERBATIM)
endif()

if(TARGET foxity_headless)
    set(FOX_HEADLESS_BUNDLE_DIRECTORY "${CMAKE_BINARY_DIR}/bundles/FoxityHeadless/$<CONFIG>")
    fox_release_bundle_strip_command(FOX_HEADLESS_BUNDLE_STRIP_COMMAND
        "${FOX_HEADLESS_BUNDLE_DIRECTORY}" foxity_headless)
    set(FOX_HEADLESS_MODULE_FILES)
    if(FOX_SIMULATION_ENGINE_MODULES)
        get_target_property(FOX_HEADLESS_MODULE_FILES foxity_headless FOX_ENGINE_MODULE_FILES)
    endif()
    add_custom_target(foxity_headless_bundle
        COMMAND "${CMAKE_COMMAND}" -E remove_directory "${FOX_HEADLESS_BUNDLE_DIRECTORY}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${FOX_HEADLESS_BUNDLE_DIRECTORY}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:foxity_headless>" ${FOX_HEADLESS_MODULE_FILES}
            "${FOX_HEADLESS_BUNDLE_DIRECTORY}"
        ${FOX_HEADLESS_BUNDLE_STRIP_COMMAND}
        DEPENDS foxity_headless
        COMMAND_EXPAND_LISTS
        COMMENT "Staging the Foxity headless bundle"
        VERBATIM)
    if(TARGET foxity_headless_engine_modules)
        add_dependencies(foxity_headless_bundle foxity_headless_engine_modules)
    endif()
endif()
