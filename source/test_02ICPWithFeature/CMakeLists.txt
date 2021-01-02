add_executable(test_02ICPWithFeature 
	main.cpp test_ICPWithFeature.cpp test_ICPWithFeature.h)

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries(test_02ICPWithFeature ${PCL_LIBRARIES} )
endif()

#TimeString
include_directories("${CMAKE_SOURCE_DIR}/include_TimeString")
target_link_libraries(test_02ICPWithFeature lib_TimeString)

#lib_PointcloudBasicProcess
include_directories("${CMAKE_SOURCE_DIR}/include_PointcloudBasicProcess")
target_link_libraries(test_02ICPWithFeature lib_PointcloudBasicProcess)

#ICP
include_directories("${CMAKE_SOURCE_DIR}/include_ICP")
target_link_libraries(test_02ICPWithFeature lib_ICP)

#FPFH
include_directories("${CMAKE_SOURCE_DIR}/include_FPFH")
target_link_libraries(test_02ICPWithFeature lib_FPFH)
