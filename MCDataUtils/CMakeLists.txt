add_library(MCDataUtils
  src/CamArrayConfig.cpp
  src/DatasetReaderBase.cpp
  src/DatasetReader.cpp
)
target_include_directories(MCDataUtils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(MCDataUtils PUBLIC common_utils)
