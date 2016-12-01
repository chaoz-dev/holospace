add_executable(${PROJECT_NAME} main.cpp)

target_sources(${PROJECT_NAME} PRIVATE devices/kinect2.cpp)

target_link_libraries(${PROJECT_NAME}
                      ${freenect2_LIBRARIES}
                      ${PCL_COMMON_LIBRARIES}
                      ${PCL_VISUALIZATION_LIBRARIES}
                      ${VTK_LIBRARIES}
                      ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION . CONFIGURATIONS Debug)
install(TARGETS ${PROJECT_NAME} DESTINATION . CONFIGURATIONS Release)
