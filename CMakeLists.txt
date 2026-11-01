cmake_minimum_required(VERSION 3.0.2)
project(holydrone_autotune)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  message_generation
  roscpp
  std_msgs
  std_srvs
)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_message_files(FILES ExperimentEvent.msg)
add_service_files(FILES ExperimentCommand.srv)
catkin_python_setup()
generate_messages()

catkin_package(
  CATKIN_DEPENDS mavros_msgs message_runtime roscpp std_msgs std_srvs)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(brct_autotune_fsm src/brct_autotune_fsm.cpp)
target_compile_features(brct_autotune_fsm PRIVATE cxx_std_14)
add_dependencies(brct_autotune_fsm
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(brct_autotune_fsm
  ${catkin_LIBRARIES}
  yaml-cpp
  ${Boost_LIBRARIES})

install(TARGETS brct_autotune_fsm
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/brct_autotune_command.py
  scripts/brct_autotune_analyze.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_brct_trial test/test_brct_trial.cpp)
  if(TARGET test_brct_trial)
    target_compile_features(test_brct_trial PRIVATE cxx_std_14)
  endif()
  catkin_add_nosetests(test)
endif()
