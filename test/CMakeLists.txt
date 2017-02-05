INCLUDE_DIRECTORIES(../src)
INCLUDE_DIRECTORIES(../clientlib)

add_definitions(-DBOOST_ALL_DYN_LINK)

add_executable (database_test database_test.cpp ../src/database_comms.cpp)

target_link_libraries(database_test
		      clientlib
		      ${Boost_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                      )

enable_testing()
ADD_TEST(DatabaseTests database_test)
