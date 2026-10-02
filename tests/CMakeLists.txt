find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(mcslam_tests
  test_triangulation.cpp
  test_factors.cpp
  test_vocabulary.cpp
)
target_link_libraries(mcslam_tests PRIVATE MCSlamCore GTest::gtest_main)
gtest_discover_tests(mcslam_tests)
