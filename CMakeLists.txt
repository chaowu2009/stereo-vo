cmake_minimum_required(VERSION 2.8)
project( stereo-vo )
find_package( OpenCV 3.0 REQUIRED )

set(CMAKE_INCLUE_CURRENT_DIR ON)
#set(CMAKE_BUILD_TYPE RELEASE)
#set(CMAKE_CONFIGURATION_TYPES RELEASE)

include_directories( ${OpenCV_INCLUDE_DIRS} )

SET(GCC_COVERAGE_LINK_FLAGS "wiringPi m rt")

add_executable(readBNO "src/readBNO.h" "src/readBNO_test.cpp")
target_link_libraries(readBNO ${OpenCV_LIBS} wiringPi m rt)


add_executable(vo "src/readBNO.h" "src/rotation.h" "src/vo_features.h" "src/visodo.cpp")
target_link_libraries(vo ${OpenCV_LIBS} wiringPi m rt)

add_executable(singleCamera "src/readBNO.h" "src/rotation.h" "src/vo_features.h" "src/singleCamera.cpp")
target_link_libraries(singleCamera ${OpenCV_LIBS} wiringPi m rt)

add_executable(cri "src/rotation.h" "src/vo_features.h" "src/captureRealImage.cpp")
target_link_libraries(cri ${OpenCV_LIBS})

add_executable(rectify "src/rotation.h" "src/vo_features.h" "src/test.cpp" "src/cam_stereo.yml")
target_link_libraries(rectify ${OpenCV_LIBS})

add_executable(test "src/test.cpp" )
target_link_libraries(test ${OpenCV_LIBS})


#add_executable(testImage ${ti} )
#target_link_libraries( testImage ${OpenCV_LIBS} )
