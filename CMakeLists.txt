cmake_minimum_required(VERSION 2.8)
project( stereo-vo )
find_package( OpenCV 3.0 REQUIRED )

set(CMAKE_INCLUE_CURRENT_DIR ON)
#set(CMAKE_BUILD_TYPE RELEASE)
#set(CMAKE_CONFIGURATION_TYPES RELEASE)

include_directories( ${OpenCV_INCLUDE_DIRS} )

SET(GCC_COVERAGE_LINK_FLAGS "wiringPi m rt")

add_executable(tracker "src/stats.h" "src/utils.h" "src/tracker.cpp")
target_link_libraries(tracker ${OpenCV_LIBS})


#	add_executable(readBNO "src/BNO070.h" "src/test_BNO070.cpp")
#	target_link_libraries(readBNO ${OpenCV_LIBS} wiringPi m rt)

add_executable(vo "src/BNO070.h" "src/rotation.h" "src/vo_features.h" "src/visodo.cpp")
target_link_libraries(vo ${OpenCV_LIBS})

add_executable(cri "src/rotation.h" "src/vo_features.h" "src/captureRealImage.cpp")
target_link_libraries(cri ${OpenCV_LIBS})

add_executable(rectify "src/rotation.h" "src/vo_features.h" "src/test_rectify.cpp" "src/cam_stereo.yml")
target_link_libraries(rectify ${OpenCV_LIBS})

add_executable(singleCamera "src/BNO080.h" "src/rotation.h" "src/vo_features.h" "src/singleCamera.cpp" "src/cam_stereo.yml")
target_link_libraries(singleCamera ${OpenCV_LIBS})

add_executable(iRobot "src/test_iRobot.cpp" "src/iRobot.h" )
target_link_libraries(iRobot)

add_executable(bno080 "src/test_BNO080.cpp" "src/BNO080.h" )
target_link_libraries(bno080)

#add_executable(testImage ${ti} )
#target_link_libraries( testImage ${OpenCV_LIBS} )
