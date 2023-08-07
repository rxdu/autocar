cmake_minimum_required(VERSION 3.8)
project(autocar_teleop)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(xmotion REQUIRED)

add_library(autocar_teleop SHARED
        src/autocar_js_teleop.cpp)
ament_target_dependencies(autocar_teleop
        rclcpp rclcpp_components
        std_msgs geometry_msgs
        tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(autocar_teleop
        xmotion::interface
        xmotion::input_joystick
        xmotion::motor_vesc)
target_include_directories(autocar_teleop PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include> PRIVATE src)

rclcpp_components_register_node(autocar_teleop
        PLUGIN "xmotion::AutocarJsTeleop"
        EXECUTABLE js_teleop)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(TARGETS
        autocar_teleop
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        js_teleop
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
