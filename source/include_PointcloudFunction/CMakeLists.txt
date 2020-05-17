add_library(lib_PointcloudFunction STATIC PointcloudFunction.cpp PointcloudFunction.h )

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries(lib_PointcloudFunction ${PCL_LIBRARIES} )
endif()
