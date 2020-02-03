add_executable(Ceres_test Ceres_test.cpp)



target_link_libraries(Ceres_test pthread)
target_link_libraries(Ceres_test ceres)
