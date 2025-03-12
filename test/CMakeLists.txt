add_executable(RaycastTest ${PROJECT_SOURCE_DIR}/test/RaycastTest.cpp ${DET_SRC_DIR})
add_executable(SpatialTest ${PROJECT_SOURCE_DIR}/test/SpatialTest.cpp ${DET_SRC_DIR})
add_executable(TimeTest ${PROJECT_SOURCE_DIR}/test/TimeTest.cpp ${DET_SRC_DIR})
add_executable(EstiPositionTest ${PROJECT_SOURCE_DIR}/test/EstiPositionTest.cpp ${DET_SRC_DIR})
add_executable(ConvertpipeTest ${PROJECT_SOURCE_DIR}/test/ConvertpipeTest.cpp ${DET_SRC_DIR})
add_executable(FusionTest ${PROJECT_SOURCE_DIR}/test/FusionTest.cpp ${DET_SRC_DIR})


target_link_libraries(RaycastTest PRIVATE Threads::Threads ${OpenCV_LIBS})
target_link_libraries(SpatialTest PRIVATE Threads::Threads ${OpenCV_LIBS})
target_link_libraries(TimeTest PRIVATE Threads::Threads ${OpenCV_LIBS})
target_link_libraries(EstiPositionTest PRIVATE Threads::Threads ${OpenCV_LIBS})
target_link_libraries(ConvertpipeTest PRIVATE Threads::Threads ${OpenCV_LIBS})
target_link_libraries(FusionTest PRIVATE Threads::Threads ${OpenCV_LIBS})

set_target_properties(RaycastTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
set_target_properties(SpatialTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)    
set_target_properties(TimeTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)   
set_target_properties(EstiPositionTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
set_target_properties(ConvertpipeTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
set_target_properties(FusionTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/test)
