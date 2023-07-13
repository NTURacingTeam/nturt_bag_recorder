cmake_minimum_required(VERSION 3.8)
project(nturt_bag_recorder)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2 REQUIRED)

# install python module for this package
ament_python_install_package(${PROJECT_NAME})

include_directories(
        include
)

# libraries
# library: nturt_bag_recorder
add_library(${PROJECT_NAME} SHARED
        src/bag_decoder.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_components
)
# register to node components
# node component: BagDecoder
rclcpp_components_register_nodes(${PROJECT_NAME} "BagDecoder")
set(node_plugins "${node_plugins}BagDecoder;$<TARGET_FILE:nturt_bag_recorder>\n")

# executable
# application: bag_decoder_node
add_executable(bag_decoder_node
        app/bag_decoder_node.cpp
)
target_link_libraries(bag_decoder_node
        ${PROJECT_NAME}
)
ament_target_dependencies(bag_decoder_node
        rclcpp
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install libraries
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# install applications, test executables
install(TARGETS bag_decoder_node
        DESTINATION lib/${PROJECT_NAME}
)

# install bash, python scripts
install(DIRECTORY scripts/
        DESTINATION lib/${PROJECT_NAME}
)

# install scripts
install(PROGRAMS scripts/nturt_bag_recorder.sh
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # comment the line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # comment the line when this package is in a git repo and when
        # a copyright and license is added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

# export package dependencies
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        ament_cmake
        ament_cmake_python
        rclpy
        ros2bag
)

ament_package()
