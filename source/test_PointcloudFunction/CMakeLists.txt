add_executable(test_PointcloudFunction main.cpp )

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries(test_PointcloudFunction ${PCL_LIBRARIES} )
endif()

#TimeString
include_directories("${CMAKE_SOURCE_DIR}/include_TimeString")
target_link_libraries(test_PointcloudFunction lib_TimeString)

#PointcloudBasicProcess
include_directories("${CMAKE_SOURCE_DIR}/include_PointcloudBasicProcess")
target_link_libraries(test_PointcloudFunction lib_PointcloudBasicProcess)

#PointcloudFuction
include_directories("${CMAKE_SOURCE_DIR}/include_PointcloudFunction")
target_link_libraries(test_PointcloudFunction lib_PointcloudFunction)