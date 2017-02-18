INCLUDE_DIRECTORIES(../src)
INCLUDE_DIRECTORIES(../clientlib)

add_definitions(-DBOOST_ALL_DYN_LINK)
add_definitions(-DDEBUG)

add_executable (sensing_test sensing_test.cpp ../src/ChannelPowerEstimator.cpp ../src/sensing_components.cpp ../src/json_utils.cpp ../src/monitor_components.cpp ../src/usrp_components.cpp)
target_link_libraries(sensing_test
		      ${Boost_LIBRARIES} ${FFTW3F_LIBRARIES}  ${UHD_LIBRARIES}
              ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}  ${PYTHON_LIBRARIES}
              )

add_executable (database_test database_test.cpp ../src/database_comms.cpp)
target_link_libraries(database_test
		      clientlib
		      ${Boost_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                      )

enable_testing()
ADD_TEST(SensingTests sensing_test)
ADD_TEST(DatabaseTests database_test)
