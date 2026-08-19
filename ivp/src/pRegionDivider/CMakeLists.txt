#--------------------------------------------------------
# The CMakeLists.txt for:                  pRegionDivider
# Author(s):                               m_shield_demo
#--------------------------------------------------------

SET(SRC
  RegionDivider.cpp
  RegionDivider_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pRegionDivider ${SRC})

TARGET_LINK_LIBRARIES(pRegionDivider
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   contacts
   geometry
   mbutil
   apputil
   ${SYSTEM_LIBS}
)
