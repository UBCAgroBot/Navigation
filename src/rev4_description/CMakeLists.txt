cmake_minimum_required(VERSION 3.8)
project(rev4_description)

find_package(ament_cmake REQUIRED)

# Install models and launch files
install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

