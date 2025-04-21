# add_executable(InputTest ${PROJECT_SOURCE_DIR}/test/InputTest.cpp ${DET_SRC_DIR} ${DET_SRC_DIR_INP})
# add_executable(ZeroMQTest ${PROJECT_SOURCE_DIR}/test/ZeroMQTest.cpp ${DET_SRC_DIR} ${DET_SRC_DIR_INP})
# add_executable(RaycastTest ${PROJECT_SOURCE_DIR}/test/RaycastTest.cpp ${DET_SRC_DIR})
# add_executable(SpatialTest ${PROJECT_SOURCE_DIR}/test/SpatialTest.cpp ${DET_SRC_DIR})
# add_executable(TimeTest ${PROJECT_SOURCE_DIR}/test/TimeTest.cpp ${DET_SRC_DIR})
add_executable(EstiPositionTest ${PROJECT_SOURCE_DIR}/test/EstiPositionTest.cpp ${DET_SRC_DIR})
# add_executable(ConvertpipeTest ${PROJECT_SOURCE_DIR}/test/ConvertpipeTest.cpp ${DET_SRC_DIR} ${DET_SRC_DIR_INP})
# add_executable(FusionTest ${PROJECT_SOURCE_DIR}/test/FusionTest.cpp ${DET_SRC_DIR})
# add_executable(KalmanTest ${PROJECT_SOURCE_DIR}/test/KalmanTest.cpp ${DET_SRC_DIR})
add_executable(LlhtoUtmTest ${PROJECT_SOURCE_DIR}/test/LlhtoUtmTest.cpp ${DET_SRC_DIR})

# target_link_libraries(InputTest PRIVATE ${OpenCV_LIBS} zmq Threads::Threads)
# target_link_libraries(ZeroMQTest PRIVATE ${OpenCV_LIBS} zmqThreads::Threads)
# target_link_libraries(RaycastTest PRIVATE ${OpenCV_LIBS} zmq Threads::Threads)
# target_link_libraries(SpatialTest PRIVATE ${OpenCV_LIBS} zmq Threads::Threads)
# target_link_libraries(TimeTest PRIVATE ${OpenCV_LIBS} zmq Threads::Threads)
target_link_libraries(EstiPositionTest PRIVATE ${OpenCV_LIBS} zmq Threads::Threads)
# target_link_libraries(ConvertpipeTest PRIVATE ${OpenCV_LIBS} zmq Threads::Threads)
# target_link_libraries(FusionTest PRIVATE ${OpenCV_LIBS} zmq Threads::Threads)
# target_link_libraries(KalmanTest PRIVATE ${OpenCV_LIBS} zmq Threads::Threads)
target_link_libraries(LlhtoUtmTest PRIVATE ${OpenCV_LIBS} zmq Threads::Threads)

# set_target_properties(InputTest PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
# set_target_properties(ZeroMQTest PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
# set_target_properties(RaycastTest PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
# set_target_properties(SpatialTest PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)    
# set_target_properties(TimeTest PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)   
set_target_properties(EstiPositionTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
# set_target_properties(ConvertpipeTest PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
# set_target_properties(FusionTest PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
# set_target_properties(KalmanTest PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
set_target_properties(LlhtoUtmTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
