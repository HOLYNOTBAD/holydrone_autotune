# holydrone_autotune 的 Catkin 构建、安装与测试注册。
cmake_minimum_required(VERSION 3.0.2)
project(holydrone_autotune)

# ============================ 构建依赖 ============================
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  mavros_msgs
  message_generation
  roscpp
  std_msgs
  std_srvs
)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# 注册试验事件消息，供 ROS 发布订阅与离线记录使用。
add_message_files(FILES ExperimentEvent.msg)
# 注册带字符串参数的人工试验命令服务。
add_service_files(FILES ExperimentCommand.srv)
# 生成已注册的消息和服务代码。
generate_messages()

# 导出本包供其他 Catkin 包使用的接口与依赖。
catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3
  CATKIN_DEPENDS geometry_msgs sensor_msgs mavros_msgs message_runtime roscpp std_msgs std_srvs)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# 构建候选参数写入、确认与激励状态机。
add_executable(brct_autotune_fsm src/brct_autotune_fsm.cpp)
target_compile_features(brct_autotune_fsm PRIVATE cxx_std_14)
# 确保本包及 Catkin 依赖的消息/服务代码先生成，再编译节点。
add_dependencies(brct_autotune_fsm
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
# 链接 ROS 通信、YAML 读写和 Boost 文件系统依赖。
target_link_libraries(brct_autotune_fsm
  ${catkin_LIBRARIES}
  yaml-cpp
  ${Boost_LIBRARIES})

# 安装可执行节点，供 rosrun/roslaunch 查找。
install(TARGETS brct_autotune_fsm
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 分析程序属于本 ROS 包；Shell 入口负责选取 Conda 环境。
catkin_install_python(PROGRAMS scripts/autotune_analyze.py scripts/autotune_getPID.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# 包内 Shell 入口和离线脚本保留相对布局，便于源码及安装空间调用。
install(PROGRAMS _scripts/autotune_getPID.sh _scripts/autotune.sh _scripts/autotune_analyze.sh _scripts/kill.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/_scripts)
install(PROGRAMS scripts/autotune_getPID.py scripts/autotune_analyze.py scripts/autotune_session.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
install(FILES environment.yml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/download_flight_ulog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 安装本行列出的公共头文件或运行资源，保留目录结构。
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# 安装本行列出的公共头文件或运行资源，保留目录结构。
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# 安装包说明与资源来源说明。
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# ============================ 测试注册 ============================
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_brct_trial test/test_brct_trial.cpp)
  if(TARGET test_brct_trial)
    target_compile_features(test_brct_trial PRIVATE cxx_std_14)
  endif()
  catkin_add_nosetests(test)
endif()
