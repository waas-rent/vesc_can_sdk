cmake_minimum_required(VERSION 3.10)
project(vesc_can_sdk VERSION 1.0.0 LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")

# Set library properties
set(LIBRARY_NAME vesc_can_sdk)

# Source files
set(SOURCES
    vesc_buffer.c
    vesc_crc.c
    vesc_can_sdk.c
)

# Header files
set(HEADERS
    vesc_buffer.h
    vesc_crc.h
    vesc_can_sdk.h
)

# Create static library
add_library(${LIBRARY_NAME}_static STATIC ${SOURCES})
set_target_properties(${LIBRARY_NAME}_static PROPERTIES
    OUTPUT_NAME ${LIBRARY_NAME}
    POSITION_INDEPENDENT_CODE ON
)

# Create shared library
add_library(${LIBRARY_NAME}_shared SHARED ${SOURCES})
set_target_properties(${LIBRARY_NAME}_shared PROPERTIES
    OUTPUT_NAME ${LIBRARY_NAME}
    POSITION_INDEPENDENT_CODE ON
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

# Set include directories
target_include_directories(${LIBRARY_NAME}_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${LIBRARY_NAME}_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link math library
target_link_libraries(${LIBRARY_NAME}_static m)
target_link_libraries(${LIBRARY_NAME}_shared m)

# Create example executables
if(BUILD_EXAMPLES)
    # Basic control example
    add_executable(basic_control examples/basic_control.c)
    target_link_libraries(basic_control ${LIBRARY_NAME}_static m)
    target_include_directories(basic_control PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    
    # Linux SocketCAN example
    add_executable(socketcan_example examples/linux_socketcan_example.c)
    target_link_libraries(socketcan_example ${LIBRARY_NAME}_static m pthread)
    target_include_directories(socketcan_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# Installation
install(TARGETS ${LIBRARY_NAME}_static ${LIBRARY_NAME}_shared
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${HEADERS} DESTINATION include)

# Create pkg-config file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/vesc_can_sdk.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/vesc_can_sdk.pc
    @ONLY
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/vesc_can_sdk.pc DESTINATION lib/pkgconfig)

# Create config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${LIBRARY_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${LIBRARY_NAME}
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}ConfigVersion.cmake
    DESTINATION lib/cmake/${LIBRARY_NAME}
)

# Export targets
install(EXPORT ${LIBRARY_NAME}Targets
    FILE ${LIBRARY_NAME}Targets.cmake
    NAMESPACE ${LIBRARY_NAME}::
    DESTINATION lib/cmake/${LIBRARY_NAME}
)

# Testing
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

# Print configuration summary
message(STATUS "VESC CAN SDK Configuration:")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Build examples: ${BUILD_EXAMPLES}")
message(STATUS "  Build testing: ${BUILD_TESTING}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}") 