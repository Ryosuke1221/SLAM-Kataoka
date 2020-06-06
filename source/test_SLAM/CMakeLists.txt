add_executable(test_SLAM main.cpp 
	SLAM_Kataoka.h SLAM_Kataoka.cpp)

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries(test_SLAM ${PCL_LIBRARIES} )
endif()

##OpenCV 3.4.1
#find_package( OpenCV REQUIRED PATHS "C:/Program Files/OpenCV-3.4.1")
#include_directories(${OpenCV_INCLUDE_DIRS} )
#target_link_libraries(test_SLAM ${OpenCV_LIBS} )

#TimeString
include_directories("${CMAKE_SOURCE_DIR}/include_TimeString")
target_link_libraries(test_SLAM lib_TimeString)

#RobotState
include_directories("${CMAKE_SOURCE_DIR}/include_RobotState")
target_link_libraries(test_SLAM lib_RobotState)

#ICP
include_directories("${CMAKE_SOURCE_DIR}/include_ICP")
target_link_libraries(test_SLAM lib_ICP)

