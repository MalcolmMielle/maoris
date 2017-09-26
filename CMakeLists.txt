cmake_minimum_required(VERSION 2.6)
project(rsi)

find_package( OpenCV REQUIRED)
find_package( bettergraph REQUIRED)
find_package( vodigrex REQUIRED)
find_package( Boost REQUIRED )
find_package( editDistance REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()
# find_package(dlib REQUIRED)
# if ( NOT dlib_FOUND )
#   message(STATUS "This project requires the Boost library, and will not be compiled.")
#   return()  
# endif()

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

add_definitions(-std=c++11)

include_directories(include ${vodigrex_INCLUDE_DIRS} include/Evaluation)

add_subdirectory(src)

#add_library(RSILIB
#			src/FuzzyOpening.cpp src/ZoneExtractor.cpp src/Kmean.cpp src/GraphZone.cpp src/ZoneCompared.cpp src/Zone.cpp
#			)



add_subdirectory(include)

# add_executable(rsi main.cpp)
# target_link_libraries(rsi ${OpenCV_LIBS})



# install(TARGETS rsi RUNTIME DESTINATION bin)

#add_subdirectory(Test)


# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

## Installing the files

FILE(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")
FILE(GLOB files_hung "${CMAKE_CURRENT_SOURCE_DIR}/include/Evaluation/*.hpp")
INSTALL(FILES ${files} DESTINATION include/maoris)
INSTALL(FILES ${files_hung} DESTINATION include/maoris/Evaluation)
INSTALL(FILES maoris-config.cmake DESTINATION lib/maoris)




