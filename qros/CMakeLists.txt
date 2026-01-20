cmake_minimum_required(VERSION 3.8)

################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  qros_interfaces
  hydraulic_interfaces
  io_interfaces
  nav_msgs
  geometry_msgs
  propulsion_interfaces
  std_srvs
  geographic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

# Qt "packages" to find
set(EXTERNAL_DEPENDS
  Qt6
  Qt6Core
  Qt6Qml
  Qt6Quick
)

# Qt imported targets to link
set(EXTERNAL_DEPENDS_LIB
  Qt6::Core
  Qt6::Gui
  Qt6::Quick
)

# Use directory name as package name (as you do today)
get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Creating Package: ${PACKAGE_NAME}")
project(${PACKAGE_NAME})

add_compile_definitions(CMAKE_PACKAGE_NAME=${PACKAGE_NAME})

# Default to C++17 for Humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###
### Dependencies
###
foreach(DEPEND ${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach()

################
## Find files ##
################

# Public headers (installed)
file(GLOB HDRS CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.h"
  "include/${PROJECT_NAME}/*.hpp"
)

# Common sources compiled ONCE into a library
file(GLOB SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/*.hpp"
  # NOTE: You currently glob include/${PROJECT_NAME}/*.cpp in your file :contentReference[oaicite:1]{index=1}
  # It's unusual, but kept here to match your current layout. Prefer moving these to src/.
  "include/${PROJECT_NAME}/*.cpp"
)

# Node entrypoints: each one becomes an executable
file(GLOB NODES CONFIGURE_DEPENDS
  "nodes/*.cpp"
)

###
### Qt (only needed if you have .ui/.qrc)
###
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB_RECURSE UI CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.ui"
)

file(GLOB QRC CONFIGURE_DEPENDS
  "rc/*.qrc"
)

if(QRC)
  message(STATUS "Adding ${PROJECT_NAME} QRC files:")
  foreach(source ${QRC})
    message(STATUS "  - ${source}")
  endforeach()
endif()

if(UI)
  message(STATUS "Found ${PROJECT_NAME} UI files:")
  foreach(source ${UI})
    message(STATUS "  - ${source}")
  endforeach()
endif()

#############
## Library ##
#############

# Compile the shared code ONCE
add_library(${PROJECT_NAME} ${SRCS} ${HDRS} ${UI} ${QRC})

# Includes:
# - BUILD_INTERFACE: lets targets include from your source tree while building
# - INSTALL_INTERFACE: lets downstream packages include after install
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPENDS})
target_link_libraries(${PROJECT_NAME} ${EXTERNAL_DEPENDS_LIB})

# Export so downstream packages can link to your library if they want
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
ament_export_include_directories(include)

###############
## Executables
###############

set(EXECUTABLES "")

message(STATUS "Building ${PROJECT_NAME} nodes:")
foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message(STATUS "  - ${NODE_NAME}")

  add_executable(${NODE_NAME} ${NODE_FILE})

  # Your node file can include headers from include/${PROJECT_NAME}/...
  target_include_directories(${NODE_NAME}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_target_dependencies(${NODE_NAME} ${ROS_DEPENDS})

  # Link the common library (this is the big RAM/perf fix)
  target_link_libraries(${NODE_NAME}
    ${PROJECT_NAME}
    ${EXTERNAL_DEPENDS_LIB}
  )

  list(APPEND EXECUTABLES ${NODE_NAME})
endforeach()

#############
## Install ##
#############

# Install executables
if(EXECUTABLES)
  install(TARGETS
    ${EXECUTABLES}
    DESTINATION lib/${PROJECT_NAME}/
  )
endif()

# Install headers
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
