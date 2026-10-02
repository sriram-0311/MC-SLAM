add_library(common_utils
  src/tools.cpp
  src/utilities.cpp
)
target_include_directories(common_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(common_utils PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Boost::filesystem
  Boost::serialization
  glog::glog
)
