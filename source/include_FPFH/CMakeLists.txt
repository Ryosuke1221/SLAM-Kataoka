add_library(lib_FPFH STATIC 
	FPFH_PCL.cpp FPFH_PCL.h )

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries( lib_FPFH ${PCL_LIBRARIES} )
endif()

#TimeString
target_include_directories(lib_FPFH PUBLIC "${CMAKE_SOURCE_DIR}/include_TimeString")
target_link_libraries(lib_FPFH lib_TimeString)

#PointcloudBasic
target_include_directories(lib_FPFH PUBLIC "${CMAKE_SOURCE_DIR}/include_PointcloudBasic")
target_link_libraries(lib_FPFH lib_PointcloudBasic)