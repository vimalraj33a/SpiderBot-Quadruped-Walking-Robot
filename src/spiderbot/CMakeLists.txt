cmake_minimum_required(VERSION 3.8)
project(spiderbot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description config launch worlds meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/control.py
    src/walk.py
    src/moveit_control.py
    src/keyboard_teleop.py
    src/manual_control.py
    src/stabilize.py
    src/moveit_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
