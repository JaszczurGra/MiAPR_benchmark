cmake_minimum_required(VERSION 3.8)
project(mb_moveit_benchmarks)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Python helper scripts (warehouse population, result processing).
install(
  PROGRAMS
    scripts/populate_warehouse.py
    scripts/process_results.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
