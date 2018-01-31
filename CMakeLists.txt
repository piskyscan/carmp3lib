cmake_minimum_required (VERSION 2.6)

project (carmp3lib)

add_executable(ir_receive carmp3lib.c main.c)

target_link_libraries(ir_receive  wiringPi rt m)

add_library(carmp3lib SHARED carmp3lib.c)
    
INSTALL(TARGETS carmp3lib        DESTINATION "/usr/local/lib" )
    
INSTALL(FILES carmp3lib.h         DESTINATION "/usr/local/include")
