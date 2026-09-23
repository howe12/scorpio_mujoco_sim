cmake_minimum_required(VERSION 3.8)
project(scorpio_mujoco_sim)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/scorpio_mujoco_bridge.py
  scripts/test_scoring_mujoco.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
