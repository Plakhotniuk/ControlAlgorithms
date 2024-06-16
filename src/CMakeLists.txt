set(MODULE_NAME ControlAlgorithms)

message("Building " ${MODULE_NAME})

file(GLOB_RECURSE SOURCES *.cpp *.hpp *.h)
add_library(${MODULE_NAME} STATIC ${SOURCES})

target_include_directories(${MODULE_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
find_package(Boost REQUIRED)

target_link_libraries(${MODULE_NAME} PUBLIC eigen Boost::boost)
