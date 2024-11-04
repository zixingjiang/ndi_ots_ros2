cmake_minimum_required(VERSION 3.8)
project(ndi_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(OpenSSL REQUIRED)  # Add this line to find OpenSSL

# Include directories
include_directories(
  include
  external/NDI_CAPI_v1.9.7/include
  external/NDI_CAPI_v1.9.7/src/include
)

# Source files
file(GLOB_RECURSE NDI_HARDWARE_SOURCES src/*.cpp)
file(GLOB_RECURSE NDI_CAPI_SOURCES external/NDI_CAPI_v1.9.7/src/*.cpp)

# Add library
add_library(ndi_capi STATIC ${NDI_CAPI_SOURCES})
set_target_properties(ndi_capi PROPERTIES POSITION_INDEPENDENT_CODE ON)

## Setup target
add_library(
  ${PROJECT_NAME}
  SHARED
  ${NDI_HARDWARE_SOURCES}
)

# Add preprocessor definition for OpenSSL
target_compile_definitions(${PROJECT_NAME} PRIVATE OPENSSL)

# Link libraries
target_link_libraries(${PROJECT_NAME} ndi_capi OpenSSL::SSL OpenSSL::Crypto)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface ndi_hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()