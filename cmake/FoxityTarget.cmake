include_guard(GLOBAL)

option(FOX_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(FOX_ENABLE_CLANG_TIDY "Run clang-tidy while compiling" OFF)
option(FOX_ENABLE_LTO "Enable interprocedural optimization in Release" ON)
option(FOX_AGGRESSIVE_INLINING "Use MSVC /Ob3 in Release" ON)

set(FOX_TARGET_ARCHITECTURE "")
if(APPLE AND CMAKE_OSX_ARCHITECTURES)
    list(LENGTH CMAKE_OSX_ARCHITECTURES FOX_OSX_ARCHITECTURE_COUNT)
    if(FOX_OSX_ARCHITECTURE_COUNT EQUAL 1)
        list(GET CMAKE_OSX_ARCHITECTURES 0 FOX_TARGET_ARCHITECTURE)
    endif()
endif()
if(NOT FOX_TARGET_ARCHITECTURE AND CMAKE_C_COMPILER_ARCHITECTURE_ID)
    set(FOX_TARGET_ARCHITECTURE "${CMAKE_C_COMPILER_ARCHITECTURE_ID}")
endif()
if(NOT FOX_TARGET_ARCHITECTURE)
    set(FOX_TARGET_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
endif()
string(TOLOWER "${FOX_TARGET_ARCHITECTURE}" FOX_TARGET_ARCHITECTURE)

set(FOX_ARCH_X86_64 OFF)
set(FOX_ARCH_ARM64 OFF)
if(FOX_TARGET_ARCHITECTURE MATCHES "^(x64|amd64|x86_64)$")
    set(FOX_ARCH_X86_64 ON)
    set(FOX_TARGET_ARCHITECTURE "x86_64")
elseif(FOX_TARGET_ARCHITECTURE MATCHES "^(arm64|aarch64)$")
    set(FOX_ARCH_ARM64 ON)
    set(FOX_TARGET_ARCHITECTURE "arm64")
else()
    message(FATAL_ERROR
        "Foxity supports x86_64 and ARM64; detected: "
        "${FOX_TARGET_ARCHITECTURE}")
endif()
set(FOX_EXPECTED_ARCHITECTURE "auto" CACHE STRING
    "Expected target architecture: auto, x86_64 or arm64")
set_property(CACHE FOX_EXPECTED_ARCHITECTURE PROPERTY STRINGS
    auto x86_64 arm64)
if(NOT FOX_EXPECTED_ARCHITECTURE MATCHES "^(auto|x86_64|arm64)$")
    message(FATAL_ERROR
        "FOX_EXPECTED_ARCHITECTURE must be auto, x86_64 or arm64; got: "
        "${FOX_EXPECTED_ARCHITECTURE}")
endif()
if(NOT FOX_EXPECTED_ARCHITECTURE STREQUAL "auto"
   AND NOT FOX_EXPECTED_ARCHITECTURE STREQUAL FOX_TARGET_ARCHITECTURE)
    message(FATAL_ERROR
        "Expected ${FOX_EXPECTED_ARCHITECTURE}, but the compiler targets "
        "${FOX_TARGET_ARCHITECTURE}")
endif()
message(STATUS "Foxity target architecture: ${FOX_TARGET_ARCHITECTURE}")

get_property(FOX_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(FOX_IS_MULTI_CONFIG)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)
else()
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    endif()
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release)
    if(NOT CMAKE_BUILD_TYPE MATCHES "^(Debug|Release)$")
        message(FATAL_ERROR
            "CMAKE_BUILD_TYPE must be Debug or Release, got: "
            "${CMAKE_BUILD_TYPE}")
    endif()
endif()

set(FOX_CLANG_LTO_MODE "full" CACHE STRING
    "Clang Release LTO mode: thin or full")
set_property(CACHE FOX_CLANG_LTO_MODE PROPERTY STRINGS thin full)
if(NOT FOX_CLANG_LTO_MODE MATCHES "^(thin|full)$")
    message(FATAL_ERROR
        "FOX_CLANG_LTO_MODE must be thin or full, got: ${FOX_CLANG_LTO_MODE}")
endif()
# The x86-64 baseline runs on every 64-bit x86 processor, Pentium and Celeron
# chips without AVX included, and measured no slower: the simulation tick on
# a Ryzen 7 8700F took 0.762 and 0.931 ms against 0.753 and 1.107 ms with
# AVX2, and AVX-512 gained nothing either. On Android the NDK compiler raises
# the floor to the ABI's SSE4.2 and POPCNT by itself.
set(FOX_X86_64_LEVEL "sse2" CACHE STRING
    "Release ISA profile: sse2, MSVC AVX2/GNU x86-64-v3, or avx512")
set_property(CACHE FOX_X86_64_LEVEL PROPERTY STRINGS sse2 avx2 avx512)
if(NOT FOX_X86_64_LEVEL MATCHES "^(sse2|avx2|avx512)$")
    message(FATAL_ERROR
        "FOX_X86_64_LEVEL must be sse2, avx2 or avx512, got: "
        "${FOX_X86_64_LEVEL}")
endif()
set(FOX_X86_64_TUNE "generic" CACHE STRING
    "Release CPU scheduling tune: generic or amd_zen4")
set_property(CACHE FOX_X86_64_TUNE PROPERTY STRINGS generic amd_zen4)
if(NOT FOX_X86_64_TUNE MATCHES "^(generic|amd_zen4)$")
    message(FATAL_ERROR
        "FOX_X86_64_TUNE must be generic or amd_zen4, got: "
        "${FOX_X86_64_TUNE}")
endif()

if(NOT FOX_ARCH_X86_64 AND NOT FOX_X86_64_TUNE STREQUAL "generic")
    message(FATAL_ERROR
        "FOX_X86_64_TUNE=${FOX_X86_64_TUNE} cannot be used on "
        "${FOX_TARGET_ARCHITECTURE}")
endif()

if(FOX_ARCH_X86_64)
    if(FOX_X86_64_LEVEL STREQUAL "sse2")
        set(FOX_MSVC_ARCH_FLAG /arch:SSE2)
        set(FOX_CLANG_CL_ARCH_FLAG /clang:-march=x86-64)
        set(FOX_NATIVE_ARCH_FLAG -march=x86-64)
    elseif(FOX_X86_64_LEVEL STREQUAL "avx2")
        set(FOX_MSVC_ARCH_FLAG /arch:AVX2)
        set(FOX_CLANG_CL_ARCH_FLAG /clang:-march=x86-64-v3)
        set(FOX_NATIVE_ARCH_FLAG -march=x86-64-v3)
    else()
        set(FOX_MSVC_ARCH_FLAG /arch:AVX512)
        set(FOX_CLANG_CL_ARCH_FLAG /clang:-march=x86-64-v4)
        set(FOX_NATIVE_ARCH_FLAG -march=x86-64-v4)
    endif()
endif()

# CMake's MSVC Release default spells out /Ob2 after /O2. When the optional
# aggressive profile is selected, remove that directory-scope default so the
# target-scoped /Ob3 does not produce a D9025 override warning.
if(CMAKE_C_COMPILER_ID STREQUAL "MSVC" AND FOX_AGGRESSIVE_INLINING)
    string(REGEX REPLACE
        "(^|[ \t])[-/]Ob2([ \t]|$)"
        " " FOX_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
    set(CMAKE_C_FLAGS_RELEASE "${FOX_C_FLAGS_RELEASE}")
endif()

if(FOX_ENABLE_CLANG_TIDY)
    if(NOT CMAKE_C_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "FOX_ENABLE_CLANG_TIDY requires clang-cl")
    endif()
    find_program(FOX_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    string(REPLACE "/" "[/\\\\]" FOX_TIDY_HEADER_FILTER "${PROJECT_SOURCE_DIR}/src/")
    set(FOX_CLANG_TIDY_COMMAND
        "${FOX_CLANG_TIDY_EXECUTABLE}"
        "--config-file=${PROJECT_SOURCE_DIR}/.clang-tidy"
        "--header-filter=^${FOX_TIDY_HEADER_FILTER}")
endif()

if(FOX_ENABLE_LTO)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        include(CheckCSourceCompiles)
        include(CMakePushCheckState)
        string(TOUPPER "${FOX_CLANG_LTO_MODE}" FOX_CLANG_LTO_MODE_UPPER)
        set(FOX_CLANG_LTO_CHECK
            "FOX_CLANG_${FOX_CLANG_LTO_MODE_UPPER}_LTO_SUPPORTED")
        cmake_push_check_state(RESET)
        set(CMAKE_REQUIRED_FLAGS "-flto=${FOX_CLANG_LTO_MODE}")
        set(CMAKE_REQUIRED_LINK_OPTIONS "-flto=${FOX_CLANG_LTO_MODE}")
        check_c_source_compiles(
            "int main(void) { return 0; }" ${FOX_CLANG_LTO_CHECK})
        cmake_pop_check_state()
        if(NOT ${FOX_CLANG_LTO_CHECK})
            message(FATAL_ERROR
                "Clang does not support -flto=${FOX_CLANG_LTO_MODE}")
        endif()
    else()
        include(CheckIPOSupported)
        check_ipo_supported(
            RESULT FOX_LTO_SUPPORTED
            OUTPUT FOX_LTO_ERROR
            LANGUAGES C)
        if(NOT FOX_LTO_SUPPORTED)
            message(FATAL_ERROR
                "FOX_ENABLE_LTO requires compiler IPO support: ${FOX_LTO_ERROR}")
        endif()
    endif()
    # Archives of LTO objects need the archiver of the same compiler (llvm-ar,
    # gcc-ar). Plain ar indexes bitcode through whatever LTO plugin the system
    # registered, which may belong to another compiler version or be missing.
    if(NOT MSVC AND NOT APPLE AND CMAKE_C_COMPILER_AR AND CMAKE_C_COMPILER_RANLIB)
        set(CMAKE_AR "${CMAKE_C_COMPILER_AR}")
        set(CMAKE_RANLIB "${CMAKE_C_COMPILER_RANLIB}")
    endif()
endif()

function(fox_configure_c_target target_name)
    cmake_parse_arguments(PARSE_ARGV 1 TARGET "FAST_FP" "" "")
    if(TARGET_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR
            "Unknown arguments for ${target_name}: ${TARGET_UNPARSED_ARGUMENTS}")
    endif()

    target_compile_features(${target_name} PRIVATE c_std_17)
    if(FOX_ENABLE_CLANG_TIDY)
        # Game lint rules belong to game targets, not to the source-based SDK.
        set_property(TARGET ${target_name} PROPERTY C_CLANG_TIDY "${FOX_CLANG_TIDY_COMMAND}")
    endif()
    if(WIN32)
        target_compile_definitions(${target_name} PRIVATE
            WIN32_LEAN_AND_MEAN
            NOMINMAX
            UNICODE
            _UNICODE)
    endif()

    if(MSVC)
        target_compile_options(${target_name} PRIVATE
            /W4
            /utf-8
            $<$<CONFIG:Release>:/O2 /Ot /Oi /GF /Gy /Gw /volatile:iso>
            $<$<AND:$<C_COMPILER_ID:MSVC>,$<CONFIG:Release>>:/Zc:inline>
            $<$<AND:$<C_COMPILER_ID:MSVC>,$<CONFIG:Release>,$<BOOL:${FOX_ARCH_X86_64}>>:${FOX_MSVC_ARCH_FLAG}>
            $<$<AND:$<C_COMPILER_ID:Clang>,$<CONFIG:Release>>:/Qvec /clang:-O3 /clang:-fvectorize /clang:-fslp-vectorize /clang:-fstrict-aliasing /clang:-fno-math-errno>
            $<$<AND:$<C_COMPILER_ID:Clang>,$<CONFIG:Release>,$<BOOL:${FOX_ARCH_X86_64}>>:${FOX_CLANG_CL_ARCH_FLAG}>
            $<$<AND:$<C_COMPILER_ID:MSVC>,$<CONFIG:Release>,$<BOOL:${FOX_AGGRESSIVE_INLINING}>>:/Ob3>
            $<$<BOOL:${TARGET_FAST_FP}>:/fp:fast>
            $<$<NOT:$<BOOL:${TARGET_FAST_FP}>>:/fp:strict>
            $<$<BOOL:${FOX_WARNINGS_AS_ERRORS}>:/WX>)
        if(FOX_X86_64_TUNE STREQUAL "amd_zen4")
            target_compile_options(${target_name} PRIVATE
                $<$<AND:$<C_COMPILER_ID:MSVC>,$<CONFIG:Release>>:/favor:AMD64>
                $<$<AND:$<C_COMPILER_ID:Clang>,$<CONFIG:Release>>:/clang:-mtune=znver4>)
        endif()
    else()
        target_compile_options(${target_name} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            $<$<CONFIG:Release>:-O3 -fstrict-aliasing -fno-math-errno>
            $<$<AND:$<CONFIG:Release>,$<PLATFORM_ID:Linux,Android>>:-ffunction-sections -fdata-sections -fno-semantic-interposition>
            $<$<AND:$<CONFIG:Release>,$<BOOL:${FOX_ARCH_X86_64}>>:${FOX_NATIVE_ARCH_FLAG}>
            $<$<BOOL:${TARGET_FAST_FP}>:-ffast-math>
            $<$<NOT:$<BOOL:${TARGET_FAST_FP}>>:-fno-fast-math -ffp-contract=off>
            $<$<BOOL:${FOX_WARNINGS_AS_ERRORS}>:-Werror>)
        if(FOX_X86_64_TUNE STREQUAL "amd_zen4")
            target_compile_options(${target_name} PRIVATE
                $<$<CONFIG:Release>:-mtune=znver4>)
        endif()
    endif()

    if(FOX_ENABLE_LTO)
        if(CMAKE_C_COMPILER_ID MATCHES "Clang")
            target_compile_options(${target_name} PRIVATE
                $<$<CONFIG:Release>:-flto=${FOX_CLANG_LTO_MODE}>)
            if(NOT MSVC)
                target_link_options(${target_name} PRIVATE
                    $<$<CONFIG:Release>:-flto=${FOX_CLANG_LTO_MODE}>)
            endif()
        else()
            set_property(TARGET ${target_name}
                PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
        endif()
    endif()

    get_target_property(target_type ${target_name} TYPE)
    if(MSVC AND target_type STREQUAL "EXECUTABLE")
        target_link_options(${target_name} PRIVATE
            $<$<CONFIG:Release>:/INCREMENTAL:NO /OPT:REF /OPT:ICF=10>
            $<$<AND:$<BOOL:${FOX_ENABLE_LTO}>,$<C_COMPILER_ID:MSVC>,$<CONFIG:Release>>:/LTCG>
            $<$<AND:$<BOOL:${FOX_ENABLE_LTO}>,$<C_COMPILER_ID:Clang>,$<CONFIG:Release>>:/OPT:LLDLTO=3 /OPT:LLDLTOCGO=3>)
    elseif(NOT MSVC AND
           target_type MATCHES "^(EXECUTABLE|SHARED_LIBRARY|MODULE_LIBRARY)$")
        target_link_options(${target_name} PRIVATE
            $<$<AND:$<CONFIG:Release>,$<PLATFORM_ID:Linux,Android>>:-Wl,-O2,--gc-sections,--as-needed>
            $<$<AND:$<CONFIG:Release>,$<PLATFORM_ID:Darwin,iOS,tvOS,visionOS>>:-Wl,-dead_strip>)
    endif()
endfunction()
