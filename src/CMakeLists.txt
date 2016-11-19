add_executable(${PROJECT_NAME} main.cpp)

#target_sources(${PROJECT_NAME} PRIVATE)

target_link_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION . CONFIGURATIONS Debug)
install(TARGETS ${PROJECT_NAME} DESTINATION . CONFIGURATIONS Release)
