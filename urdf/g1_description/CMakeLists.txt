cmake_minimum_required(VERSION 3.8)
project(g1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  images
  inspire_hand
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  g1_23dof.urdf
  g1_23dof.xml
  g1_23dof_rev_1_0.urdf
  g1_23dof_rev_1_0.xml
  g1_29dof.urdf
  g1_29dof.xml
  g1_29dof_with_hand.urdf
  g1_29dof_with_hand.xml
  g1_29dof_rev_1_0.urdf
  g1_29dof_rev_1_0.xml
  g1_29dof_with_hand_rev_1_0.urdf
  g1_29dof_with_hand_rev_1_0.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
