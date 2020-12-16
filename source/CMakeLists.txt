cmake_minimum_required(VERSION 2.8)

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "test_PointcloudFunction" )

add_subdirectory(include_FPFH)
add_subdirectory(include_ICP)
add_subdirectory(include_RobotState)
add_subdirectory(include_TimeString)
add_subdirectory(include_PointcloudFunction)
add_subdirectory(include_PointcloudBasic)
add_subdirectory(test_GlobalRegistration)
add_subdirectory(test_PointcloudFunction)
add_subdirectory(test_SLAM)
add_subdirectory(test_03GlobalFeatureRegistration)

