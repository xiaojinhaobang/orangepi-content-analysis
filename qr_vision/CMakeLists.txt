cmake_minimum_required(VERSION 3.8)
project(qr_vision)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/qr_vision_node.py
  scripts/qr_decode.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
