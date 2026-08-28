#--------------------------------------------------------
# The CMakeLists.txt for:                      pLandGuard
# Author(s):                                m_shield_demo
#--------------------------------------------------------

SET(SRC
  LandGuard.cpp
  LandGuard_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pLandGuard ${SRC})

TARGET_LINK_LIBRARIES(pLandGuard
   ${MOOS_LIBRARIES}
   geometry
   landmodel
   mbutil
   apputil
   ${SYSTEM_LIBS}
)
