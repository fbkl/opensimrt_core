# global files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
# file(GLOB tests tests/*.cpp)
file(GLOB tests
  tests/TestCircularBuffer.cpp
  tests/TestLowPassSmoothFilter.cpp
  tests/TestButterWorthFilter.cpp
  tests/TestSyncManager.cpp
  )

# dependencies
include_directories(include/)
include_directories( ${catkin_INCLUDE_DIRS})
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} ${catkin_LIBRARIES})

# dynamic library
set(target Common)
addLibrary(
  KIT ${target}
  SOURCES ${sources}
  INCLUDES ${includes}
  LINKLIBS ${DEPENDENCY_LIBRARIES}
  )

# tests
addTests(
  TESTPROGRAMS ${tests}
  LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
  )
