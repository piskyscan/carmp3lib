cmake_minimum_required (VERSION 2.6)

project (carmp3lib)

# add_executable(carmp3lib carmp3lib.cpp)

add_library(carmp3lib SHARED carmp3lib.c)
    
INSTALL(TARGETS carmp3lib        DESTINATION "/usr/local/lib" )
    
INSTALL(FILES carmp3lib.h         DESTINATION "/usr/local/include")
