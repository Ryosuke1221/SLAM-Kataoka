add_library(lib_GlobalFeatureRegistration STATIC 
	GlobalFeatureRegistration.cpp GlobalFeatureRegistration.h )

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries( lib_GlobalFeatureRegistration ${PCL_LIBRARIES} )
endif()

#TimeString
target_include_directories(lib_GlobalFeatureRegistration PUBLIC "${CMAKE_SOURCE_DIR}/include_TimeString")
target_link_libraries(lib_GlobalFeatureRegistration lib_TimeString)

#ICP
target_include_directories(lib_GlobalFeatureRegistration PUBLIC "${CMAKE_SOURCE_DIR}/include_ICP")
target_link_libraries(lib_GlobalFeatureRegistration lib_ICP)

#FPFH
target_include_directories(lib_GlobalFeatureRegistration PUBLIC "${CMAKE_SOURCE_DIR}/include_FPFH")
target_link_libraries(lib_GlobalFeatureRegistration lib_FPFH)