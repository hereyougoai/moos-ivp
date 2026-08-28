#--------------------------------------------------------
# The CMakeLists.txt for:               pTargetCoordinator
# Author(s):                               m_shield_demo
#--------------------------------------------------------

SET(SRC
  TargetCoordinator.cpp
  TargetCoordinator_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTargetCoordinator ${SRC})

TARGET_LINK_LIBRARIES(pTargetCoordinator
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   contacts
   geometry
   landmodel
   mbutil
   apputil
   ${SYSTEM_LIBS}
)
