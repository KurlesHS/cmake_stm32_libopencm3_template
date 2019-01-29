cmake_minimum_required(VERSION 2.8)

project(some_usb)

include_directories(${LIBOPENCM3_DIR}/include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/tinyprintf)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/include)
link_directories(${LIBOPENCM3_DIR}/lib)

add_definitions(-DSTM32F4)

set(CMAKE_INCLUDE_CURRENT_DIR 1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14")
# rdimon
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LIBOPENCM3_DIR}/lib/stm32f429i-discovery.ld")
set(TARGET ${PROJECT_NAME}.elf)


set(LIBS ${LIBS} -lopencm3_stm32f4)
add_executable(${PROJECT_NAME} "main.c")
target_link_libraries(${PROJECT_NAME} ${LIBS})

add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND arm-none-eabi-size --format=berkeley ${PROJECT_NAME})
