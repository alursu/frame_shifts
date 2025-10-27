cmake_minimum_required(VERSION 3.10.0)
project(frame_shifts VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
# find_package(CURL REQUIRED)
# find_package(nlohmann_json REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )
message("OpenCV include dir ${OpenCV_INCLUDE_DIRS}")


file(GLOB src *.cpp *.hpp)
add_executable(frame_shifts ${src})
target_include_directories(frame_shifts PRIVATE ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(frame_shifts PUBLIC ${OpenCV_LIBS})
    # CURL::libcurl
    # nlohmann_json::nlohmann_json)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


# # Find required packages
# find_package(CURL REQUIRED)
# find_package(nlohmann_json REQUIRED)

# target_link_libraries(optical_flow_mavlink PUBLIC
#     CURL::libcurl
#     nlohmann_json::nlohmann_json
# )