cmake_minimum_required(VERSION 2.8)
project( stereo-vo )
find_package( OpenCV 3.0 REQUIRED )

set(CMAKE_INCLUE_CURRENT_DIR ON)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CONFIGURATION_TYPES RELEASE)

include_directories( ${OpenCV_INCLUDE_DIRS} )

#add_executable(vo ${viso} )
#target_link_libraries( vo ${OpenCV_LIBS} )

add_executable(vo "src/vo_features.h" "src/visodo.cpp")
target_link_libraries(vo ${OpenCV_LIBS})

add_executable(rectify "src/vo_features.h" "src/test.cpp" "src/cam_stereo.yml")
target_link_libraries(rectify ${OpenCV_LIBS})


#add_executable(testImage ${ti} )
#target_link_libraries( testImage ${OpenCV_LIBS} )
