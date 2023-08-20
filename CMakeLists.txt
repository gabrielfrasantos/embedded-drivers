cmake_minimum_required(VERSION 3.24)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(DRIVERS_STANDALONE On)
endif()

option(CMAKE_COMPILE_WARNING_AS_ERROR "Enable warnings-as-error" ON)
option(DRIVERS_INCLUDE_DISPLAY_TFT "Include code for display TFT; turn on when providing custom initialization" OFF)

if (DRIVERS_STANDALONE)
    set(EMIL_INCLUDE_FREERTOS ${DRIVERS_BUILD_EXAMPLES_FREERTOS})

    include(FetchContent)

    FetchContent_Declare(
        emil
        GIT_REPOSITORY https://github.com/gabrielfrasantos/embedded-infra-lib.git
        GIT_TAG        af9bda39c192ae9199485cd7a86b6802aa2eb65f # Unreleased
    )

    FetchContent_MakeAvailable(emil)
endif()

project(DRIVERS LANGUAGES C CXX ASM VERSION 2.0.0) # x-release-please-version

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)

set_directory_properties(PROPERTY USE_FOLDERS ON)

# When building HalSt by itself do not exclude any targets from all
# Libraries will only be excluded from all when HalSt is a consumed as a dependency.
if (DRIVERS_STANDALONE AND NOT EMIL_HOST_BUILD)
    set(DRIVERS_EXCLUDE_FROM_ALL "")
else()
    set(DRIVERS_EXCLUDE_FROM_ALL "EXCLUDE_FROM_ALL")
endif()

add_subdirectory(display)

if (DRIVERS_STANDALONE)
    emil_folderize_all_targets()
endif()
