cmake_minimum_required(VERSION 2.8.3)
project(PandarSwiftSDK)
cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package( Boost REQUIRED  COMPONENTS thread)
find_package( PCL REQUIRED COMPONENTS common )
find_package(OpenSSL REQUIRED)

include_directories(
    .
    include
    src/taskflow
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OPENSSL_INCLUDE_DIR}
)

add_library( ${PROJECT_NAME} SHARED
    src/input.cc
    src/laser_ts.cpp
    src/pandarSwiftDriver.cc
    src/pandarSwiftSDK.cc
    src/platUtil.cc
    src/tcp_command_client.c
    src/util.c
    src/wrapper.cc
)

target_link_libraries(${PROJECT_NAME} 
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${OPENSSL_LIBRARIES}
    Boost::thread
    pcap
    zstd
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(PCL REQUIRED)

    add_executable(PandarCapture
        test/pandar_capture.cc
    )
    add_executable(PandarDownload
        test/download.cc
    )
    add_executable(PcapToPcd
        test/pcap_to_pcd.cc
    )

    target_link_libraries(PandarCapture
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${PCL_IO_LIBRARIES}
    )
    target_link_libraries(PandarDownload
        ${PROJECT_NAME}
    )
    target_link_libraries(PcapToPcd
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${PCL_IO_LIBRARIES}
    )

endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
