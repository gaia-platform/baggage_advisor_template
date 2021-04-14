cmake_minimum_required(VERSION 3.5)
project(baggage_advisor)

include(/opt/gaia/cmake/gaia.cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Using C++17 for Gaia system
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(barcode_msgs REQUIRED)
find_package(image_draw_msgs REQUIRED)

find_package(Threads REQUIRED) # Might not be required, try removing later.

set(GAIA_INC "/opt/gaia/include")
set(GAIA_BIN "/opt/gaia/bin")
set(GAIA_LIB "/opt/gaia/lib")
set(GAIA_COMPILE_FLAGS "-c -Wall -Wextra")
set(GENERATED_OUTPUTS "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY ${GENERATED_OUTPUTS})

# Generate
process_schema(
  DDL_FILE ${PROJECT_SOURCE_DIR}/src/baggage_advisor.ddl
  OUTPUT_FOLDER ${GENERATED_OUTPUTS}
  TARGET_NAME generate_baggage_advisor_headers
)

# Translate
translate_ruleset(
  RULESET_FILE ${PROJECT_SOURCE_DIR}/src/baggage_advisor.ruleset
  OUTPUT_FOLDER ${GENERATED_OUTPUTS}
  TARGET_NAME translate_baggage_advisor_ruleset
  CLANG_PARAMS
    -I /opt/ros/foxy/include
    -I ${GENERATED_OUTPUTS}
    -I ${CMAKE_CURRENT_SOURCE_DIR}/include
    -I ${CMAKE_CURRENT_BINARY_DIR}/../barcode_msgs/rosidl_generator_cpp
    -I ${CMAKE_CURRENT_BINARY_DIR}/../image_draw_msgs/rosidl_generator_cpp
  DEPENDS generate_baggage_advisor_headers
)

# Preparing gaia shared library
add_library(gaia SHARED IMPORTED GLOBAL)
set_target_properties(gaia PROPERTIES IMPORTED_LOCATION ${GAIA_LIB}/libgaia.so)

# ROS2 composable node shared library
add_library(advisor_component SHARED
  src/advisor.cpp
  ${GENERATED_OUTPUTS}/baggage_advisor_ruleset.cpp
)

add_dependencies(advisor_component
  generate_baggage_advisor_headers
  translate_baggage_advisor_ruleset
)

target_link_libraries(advisor_component PRIVATE
  ${CMAKE_THREAD_LIBS_INIT} # Might not be required, try removing later.
  rt # Might not be required, try removing later.
  pthread # Might not be required, try removing later.
  gaia
)

ament_target_dependencies(advisor_component PUBLIC
  rclcpp
  rclcpp_components
  ament_index_cpp
  barcode_msgs
  image_draw_msgs
)

rclcpp_components_register_node(advisor_component
  PLUGIN "baggage_advisor::advisor"
  EXECUTABLE advisor
)

target_include_directories(advisor_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${GAIA_INC}
    ${GENERATED_OUTPUTS}
)

install(TARGETS
  advisor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
