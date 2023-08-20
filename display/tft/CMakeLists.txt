add_library(drivers.display.tft ${HAL_TI_EXCLUDE_FROM_ALL} STATIC)
install(TARGETS drivers.display.tft EXPORT halTivaTargets)

target_include_directories(drivers.display.tft PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(drivers.display.tft PUBLIC
    hal.interfaces
    infra.timer
    infra.util
)

target_sources(drivers.display.tft PRIVATE
    DisplayTft.hpp
    Ssd2119.cpp
    Ssd2119.hpp
)
