add_library(lib_PointcloudFunction STATIC 
	PointcloudFunction.cpp PointcloudFunction.h)

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries(lib_PointcloudFunction ${PCL_LIBRARIES} )
endif()

#TimeString
include_directories("${CMAKE_SOURCE_DIR}/include_TimeString")
#target_link_libraries(lib_PointcloudFunction lib_TimeString)

#PointVisualization(PointcloudBasicProcess)
include_directories("${CMAKE_SOURCE_DIR}/include_PointcloudBasicProcess")
target_link_libraries(lib_PointcloudFunction lib_PointcloudBasicProcess)

#ICP
target_include_directories(lib_PointcloudFunction PUBLIC "${CMAKE_SOURCE_DIR}/include_ICP")
target_link_libraries(lib_PointcloudFunction lib_ICP)

#FPFH
target_include_directories(lib_PointcloudFunction PUBLIC "${CMAKE_SOURCE_DIR}/include_FPFH")
target_link_libraries(lib_PointcloudFunction lib_FPFH)