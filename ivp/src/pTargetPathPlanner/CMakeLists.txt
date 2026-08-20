#--------------------------------------------------------
# The CMakeLists.txt for:              pTargetPathPlanner
# Author(s):                               m_shield_demo
#--------------------------------------------------------

SET(SRC
  TargetPathPlanner.cpp
  TargetPathPlanner_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTargetPathPlanner ${SRC})

TARGET_LINK_LIBRARIES(pTargetPathPlanner
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   geometry
   mbutil
   apputil
   ${SYSTEM_LIBS}
)
