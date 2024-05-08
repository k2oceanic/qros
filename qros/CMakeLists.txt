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
    nav_msgs
    io_interfaces
    geometry_msgs
    propulsion_interfaces
  )

set(EXTERNAL_DEPENDS
    Qt5Widgets
    Qt5PrintSupport
    Qt5Qml
    Qt5Quick
    Qt5Gui
    Qt5QuickControls2
  )

set(EXTERNAL_DEPENDS_LIB
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::Core
    Qt5::Qml
    Qt5::Quick
    Qt5::QuickControls2
  )


get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")
project(${PACKAGE_NAME})  ## this package name is the name of the directory this cmake file is in


add_compile_definitions(CMAKE_PACKAGE_NAME=${PACKAGE_NAME})

# Default to C++17 for humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###
###  Add dependencies
###

foreach(DEPEND ${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)


################
## Find files ##
################

file(GLOB HDRS CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/*.hpp"
  "include/${PROJECT_NAME}/*.cpp"
)
file(GLOB NODES CONFIGURE_DEPENDS
  "nodes/*.cpp"
)

###
### QT stuff (required if you want a GUI)
###
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files

#find all the qt UI stuff
file(GLOB_RECURSE UI
    "include/${PROJECT_NAME}/*.ui"
)

## find all resource files
file(GLOB QRC
    "rc/*.qrc"
)

message("Adding ${PROJECT_NAME} QRC files: "  )
foreach(source ${QRC})
    message("  - " ${source})
endforeach(source)

message("Adding ${PROJECT_NAME} UI files: "  )
foreach(source ${UI})
    message("  - " ${source})
endforeach(source)

#make them into headers
qt5_wrap_ui (UI_HDRS  ${UI})

message("Generating ${PROJECT_NAME} UI Headers: " )
foreach(source ${UI_HDRS})
    message("  - " ${source})
endforeach(source)


###########
## Build ##
###########

set(EXECUTABLES "")  # for installing later

message("buliding ${PROJECT_NAME} node: ")

foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})
  list(APPEND EXECUTABLES ${NODE_NAME})
  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS} ${UI} ${QRC})
  ament_target_dependencies(${NODE_NAME} ${ROS_DEPENDS} )
  target_link_libraries(${NODE_NAME} ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")
endforeach(NODE_FILE)


message("  Installing: " ${EXECUTABLES})
foreach(EXECUTABLE ${EXECUTABLES})
    message("  - " ${EXECUTABLE})
endforeach(EXECUTABLE)

install(TARGETS
  ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}/
)

#############
## library ##
#############

include_directories(include/${PROJECT_NAME})
add_library(${PROJECT_NAME} ${SRCS} ${HDRS})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPENDS})
ament_export_dependencies(${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
ament_export_include_directories(include)

target_link_libraries(${PROJECT_NAME} ${EXTERNAL_DEPENDS_LIB})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
