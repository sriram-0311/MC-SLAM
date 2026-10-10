set(MCSLAM_SOURCES
  src/Backend.cpp
  src/FrontEnd.cpp
  src/EmbeddingTable.cpp
  src/GlobalMap.cpp
  src/LoopCloser.cpp
  src/MultiCameraFrame.cpp
  src/newGPSFactor.cpp
  src/ORBextractor.cpp
  src/ORBVocabulary.cpp
  src/relocalization.cpp
  src/time_measurement.cpp
  src/Triangulation.cpp
  src/VisualInertialAlignment.cpp
  src/Tracking.cpp
)
if(MCSLAM_WITH_VIEWER)
  list(APPEND MCSLAM_SOURCES src/OpenGlViewer.cpp)
endif()

add_library(MCSlamCore ${MCSLAM_SOURCES})
target_include_directories(MCSlamCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(MCSlamCore PUBLIC
  MCDataUtils
  common_utils
  gtsam
  gtsam_unstable
  opengv
  mcslam_bow
  mcslam_fbow
  ${GeographicLib_LIBRARIES}
  Boost::json
  gflags
  Threads::Threads
)
if(MCSLAM_WITH_VIEWER)
  target_link_libraries(MCSlamCore PUBLIC ${Pangolin_LIBRARIES})
  target_include_directories(MCSlamCore SYSTEM PUBLIC ${EPOXY_INCLUDE_DIR})
  target_compile_definitions(MCSlamCore PUBLIC MCSLAM_WITH_VIEWER)
endif()
