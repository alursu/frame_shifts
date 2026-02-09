cmake_minimum_required(VERSION 3.10.0)
project(frame_shifts VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )
message("OpenCV include dir ${OpenCV_INCLUDE_DIRS}")

set(CAMERA_SDK "${CMAKE_CURRENT_SOURCE_DIR}/camera_sdk")
set(CAMERA_INCLUDE "${CAMERA_SDK}/include")
set(CAMERA_LIB "${CAMERA_SDK}/lib")

file(GLOB src *.cpp *.hpp)
add_executable(frame_shifts ${src})
target_include_directories(frame_shifts PRIVATE ${OpenCV_INCLUDE_DIRS} 
                                                mavlink
                                                ${CAMERA_INCLUDE})

target_link_libraries(frame_shifts PUBLIC ${OpenCV_LIBS})

target_link_libraries(${PROJECT_NAME} PRIVATE pthread
                                              ${CAMERA_LIB}/libGuideSCUSB2.so
                                              usb-1.0
                                              udev
                                              m)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)