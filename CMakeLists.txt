cmake_minimum_required(VERSION 2.8.3)
project(cobvid)

find_package(catkin REQUIRED COMPONENTS )

catkin_python_setup()

catkin_package()


install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY trajectory_goal_data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS helper_scripts/bezier_bernstein.py helper_scripts/calculations_circula_path.py helper_scripts/trajectory_goal_from_joint_states.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/helper_scripts
)

install(PROGRAMS src/cobvid.py src/demo_berlin.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
