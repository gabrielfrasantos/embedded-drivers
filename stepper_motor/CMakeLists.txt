add_library(drivers.stepper_motor ${HAL_TI_EXCLUDE_FROM_ALL} STATIC)

target_include_directories(drivers.stepper_motor PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(drivers.stepper_motor PUBLIC
    hal.interfaces
    infra.timer
    infra.util
)

target_sources(drivers.stepper_motor PRIVATE
    Drv8711.cpp
    Drv8711.hpp
)
