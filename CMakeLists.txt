cmake_minimum_required(VERSION 3.11.0)
project(TEST VERSION 1.0.0 LANGUAGES  CXX C)


set(CXX_STD "17" CACHE STRING "C++ standard")

if(CMAKE_TOOLCHAIN_FILE)
  get_filename_component(CMAKE_TOOLCHAIN_FILE_NAME ${CMAKE_TOOLCHAIN_FILE} NAME)
  find_file(CMAKE_TOOLCHAIN_FILE ${CMAKE_TOOLCHAIN_FILE_NAME} PATHS ${CMAKE_SOURCE_DIR} NO_DEFAULT_PATH)
  message(STATUS "CMAKE_TOOLCHAIN_FILE = ${CMAKE_TOOLCHAIN_FILE}")
endif()


set(COMMON_FLAGS " -Wl,--allow-shlib-undefined -Wno-class-memaccess -Wno-deprecated-declarations -Wno-sign-compare")
# 平台特定优化（仅 x86_64 启用）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(COMMON_FLAGS "${COMMON_FLAGS} -mpclmul -mavx2")
# 平台特定优化（仅 arm 启用）
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_options(-mcpu=native )
endif()
# 设置 C/C++ 编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS}")


set(CMAKE_CXX_STANDARD "${CXX_STD}")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# BUILD  TYPE
if(CMAKE_BUILD_TYPE AND(CMAKE_BUILD_TYPE STREQUAL "Debug"))
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -Wall -O0  -g")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -O0  -g")
  message(STATUS "Debug mode:")
  message(STATUS "CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")
  message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
  add_definitions(-DDEBUG)
elseif(CMAKE_BUILD_TYPE AND(CMAKE_BUILD_TYPE STREQUAL "Release"))
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -Wall -O3")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall -O3")
  message(STATUS "Release mode:")
  message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
  message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
  add_definitions(-DNDEBUG)
else()
  message("The CMAKE_BUILD_TYPE is not specified, defaulting to Debug mode.")
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -Wall -O0  -g")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -O0  -g")
  message(STATUS "Debug mode:")
  message(STATUS "CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")
  message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
  add_definitions(-DDEBUG)
endif()

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/build/test)
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/build/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_BUILD_RPATH "$ORIGIN/../lib")

set(CMAKE_INSTALL_RPATH "lib")

set(COMMON_HEADER_DIRS
  ${PROJECT_SOURCE_DIR}/include
)

# install target and libraries
if(NOT  CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
endif()
message(STATUS "CMAKE_INSTALL_PREFIX :${CMAKE_INSTALL_PREFIX}")

# ZeroMQ
find_package(ZeroMQ QUIET)
 if(NOT ZeroMQ_FOUND)
     # 如果find_package不能找到ZeroMQ，尝试使用pkg-config
     find_package(PkgConfig)
     if(PkgConfig_FOUND)
         pkg_check_modules(ZeroMQ libzmq)
     endif()
 endif()
 if(NOT ZeroMQ_FOUND)
     message(STATUS "ZeroMQ package not found, assuming ZeroMQ library is installed in system directories")
 endif()

# Eigen
include_directories("/usr/include/eigen3")

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# pthread
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# opencv
find_package(OpenCV REQUIRED COMPONENTS
    core 
    imgcodecs   
    imgproc     
    highgui   
    video   
)
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty) 
include_directories(
  ${COMMON_HEADER_DIRS}
)

aux_source_directory(${PROJECT_SOURCE_DIR}/src/modules DET_SRC_DIR)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/utils DET_SRC_DIR)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/framework DET_SRC_DIR)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/input DET_SRC_DIR)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/output DET_SRC_DIR)

add_subdirectory(test/)


add_executable(main_test main_test.cpp ${DET_SRC_DIR})
target_link_libraries(main_test PRIVATE ${OpenCV_LIBS} ${ZeroMQ_LIBRARIES} Threads::Threads yaml-cpp)
set_target_properties(main_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/bin)

add_executable(main main.cpp ${DET_SRC_DIR})
target_link_libraries(main PRIVATE ${OpenCV_LIBS} ${ZeroMQ_LIBRARIES} Threads::Threads yaml-cpp)
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/bin)