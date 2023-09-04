project(dji_psdk_ros)

FILE(GLOB_RECURSE HEADERS
     ../../include/dji_psdk_ros/*.h*
     )

FILE(GLOB_RECURSE SOURCES
     *.cc *.c *.cpp
     )

FILE(GLOB EXAMPLES *.cc *.c *.cpp)

if(OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ)
add_definitions(-DOPEN_CV_INSTALLED)
endif()

ADD_LIBRARY(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      libpayloadsdk.a
                      )

FOREACH(EXAMPLE ${EXAMPLES})
  GET_FILENAME_COMPONENT(MY_TARGET "${EXAMPLE}" NAME_WE)
  ADD_EXECUTABLE(${MY_TARGET} ${EXAMPLE})

  ## Add cmake target dependencies of the executable/library
  ## as an example, message headers may need to be generated before nodes
  add_dependencies(${MY_TARGET}
          dji_osdk_ros_generate_messages_cpp)

  TARGET_LINK_LIBRARIES(${MY_TARGET}
                        ${PROJECT_NAME}
                        libpayloadsdk.a
                        )

  INSTALL(TARGETS ${MY_TARGET}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
          )
ENDFOREACH()
