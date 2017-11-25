aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Collision BOX2D_Collision_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Collision/Shapes BOX2D_Shapes_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Common BOX2D_Common_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Dynamics BOX2D_Dynamics_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Dynamics/Contacts BOX2D_Contacts_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Dynamics/Joints BOX2D_Joints_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Rope BOX2D_Rope_SRCS)

add_library(Box2D SHARED
  ${BOX2D_General_HDRS}
  ${BOX2D_Joints_SRCS}
  ${BOX2D_Joints_HDRS}
  ${BOX2D_Contacts_SRCS}
  ${BOX2D_Contacts_HDRS}
  ${BOX2D_Dynamics_SRCS}
  ${BOX2D_Dynamics_HDRS}
  ${BOX2D_Common_SRCS}
  ${BOX2D_Common_HDRS}
  ${BOX2D_Shapes_SRCS}
  ${BOX2D_Shapes_HDRS}
  ${BOX2D_Collision_SRCS}
  ${BOX2D_Collision_HDRS}
  ${BOX2D_Rope_SRCS}
  ${BOX2D_Rope_HDRS}
  )
