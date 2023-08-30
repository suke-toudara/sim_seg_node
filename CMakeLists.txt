cmake_minimum_required(VERSION 3.8)
#project(k_means)
project(sim_seg_node)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(segmentation_msg REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
 include
 ${OpenCV_INCLUDE_DIRS} #追加
)

add_executable(k_means src/k_means.cpp)
ament_target_dependencies(k_means 
  sensor_msgs 
  rclcpp 
  cv_bridge
)


ament_target_dependencies(k_means
  rclcpp
  segmentation_msg
)

# ライブラリにアクセスできるようにする必要
target_link_libraries(k_means 
  ${OpenCV_LIBRARIES} 
  ${cv_bridge_LIBRARIES}
)

#指定したターゲットにヘッダーファイルの検索パスを追加する
target_include_directories(k_means PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS k_means
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
