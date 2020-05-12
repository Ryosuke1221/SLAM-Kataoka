add_library(lib_ICP STATIC 
	KataokaPCL.cpp KataokaPCL.h 
	KataokaConvergence.cpp KataokaConvergence.h 
	KataokaCorrespondence.h)

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries( lib_ICP ${PCL_LIBRARIES} )
endif()
