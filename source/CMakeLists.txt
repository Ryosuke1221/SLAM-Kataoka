cmake_minimum_required(VERSION 2.8)

project(Project)

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "test_PointcloudFunction" )

add_subdirectory(include_FPFH)
add_subdirectory(include_ICP)
add_subdirectory(include_TimeString)
add_subdirectory(include_PointcloudFunction)
add_subdirectory(include_PointcloudBasic)
add_subdirectory(include_PointcloudBasicProcess)
add_subdirectory(include_GlobalFeatureRegistration)
add_subdirectory(test_PointcloudFunction)
add_subdirectory(test_HandRegistration)
add_subdirectory(test_PointcloudGeneration)
add_subdirectory(test_02ICPWithFeature)
add_subdirectory(test_03GlobalFeatureRegistration)