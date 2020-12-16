add_executable(test_03GlobalFeatureRegistration 
	main.cpp GlobalFeatureRegistration.cpp GlobalFeatureRegistration.h)

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries(test_03GlobalFeatureRegistration ${PCL_LIBRARIES} )
endif()

#TimeString
include_directories("${CMAKE_SOURCE_DIR}/include_TimeString")
target_link_libraries(test_03GlobalFeatureRegistration lib_TimeString)

#PointcloudBasic
include_directories("${CMAKE_SOURCE_DIR}/include_PointcloudBasic")
target_link_libraries(test_03GlobalFeatureRegistration lib_PointcloudBasic)

#ICP
include_directories("${CMAKE_SOURCE_DIR}/include_ICP")
target_link_libraries(test_03GlobalFeatureRegistration lib_ICP)

#FPFH
include_directories("${CMAKE_SOURCE_DIR}/include_FPFH")
target_link_libraries(test_03GlobalFeatureRegistration lib_FPFH)



