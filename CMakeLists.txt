cmake_minimum_required(VERSION 3.0.2)
project(motor_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        std_msgs
        can_msgs
)

## Generate messages in the 'msg' folder
add_message_files(FILES
        MotorFeedback.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
        std_msgs
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        message_runtime
        roscpp
        std_msgs
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/motor_config.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)



add_executable(motor_control_test
        src/motor_control_test.cpp
)

add_dependencies(motor_control_test 
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(motor_control_test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
        message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
        add_clang_tooling(
                TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
                SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
                CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
                CF_WERROR
        )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Install plugins
install(FILES legged_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

