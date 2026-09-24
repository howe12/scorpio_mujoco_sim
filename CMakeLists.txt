cmake_minimum_required(VERSION 3.8)
project(scorpio_mujoco_sim)

find_package(ament_cmake REQUIRED)

# 安装全部 Python 脚本
install(PROGRAMS
  scripts/run_seb_naver_sim.py
  scripts/scorpio_ros2_publisher.py
  scripts/scorpio_mujoco_bridge.py
  scripts/lidar_simulator.py
  scripts/generate_terrain.py
  scripts/scorpio_rl_env.py
  scripts/test_scoring_mujoco.py
  scripts/view_sim_web.py
  DESTINATION lib/${PROJECT_NAME}
)

# 同时把脚本以数据形式放在 share 下, 供 launch 用 python3 直接调用
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.py"
)

# 模型文件
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 场景文件
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# 配置文件 (SLAM / RViz)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
