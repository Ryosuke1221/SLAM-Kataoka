add_library(lib_PointcloudBasicProcess STATIC 
	PointcloudBasicProcess.cpp PointcloudBasicProcess.h PointVisualization.h)

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries(lib_PointcloudBasicProcess ${PCL_LIBRARIES} )
endif()

#TimeString
include_directories("${CMAKE_SOURCE_DIR}/include_TimeString")
target_link_libraries(lib_PointcloudBasicProcess lib_TimeString)


#PointcloudBasic
target_include_directories(lib_PointcloudBasicProcess PUBLIC "${CMAKE_SOURCE_DIR}/include_PointcloudBasic")
target_link_libraries(lib_PointcloudBasicProcess lib_PointcloudBasic)