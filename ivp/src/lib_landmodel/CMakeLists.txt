#--------------------------------------------------------
# The CMakeLists.txt for:                   lib_landmodel
# Author(s):                               m_shield_demo
#--------------------------------------------------------

SET(SRC
  LandModel.cpp
)

SET(HEADERS
  LandModel.h
)

ADD_LIBRARY(landmodel ${SRC})

TARGET_LINK_LIBRARIES(landmodel
  mbutil
)
