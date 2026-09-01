cmake_minimum_required(VERSION 3.8)
project(robomantis_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  dual_arm_plan_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dual_arm_plan_example
)

ament_package()
