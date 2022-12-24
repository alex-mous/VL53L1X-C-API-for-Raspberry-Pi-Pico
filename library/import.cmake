# Specify build information for VL53L1X Pico library
# This is a static library, and you should include
# this file in your CMakeLists file to compile the library
# before linking to it.
#
# For example, if your project structure is:
# 
# root/ (for your project)
#     yourapp.h
#     yourapp.c
#     CMakeLists.txt
#     build/
#         ...
#     VL53L1X-C-API-Pico/
#         library/
#             import.cmake
#             ...
#         ...
#    
# Then, add this line to your CMakeLists.txt file before 
# defining/linking yourapp executables:
#     include(${PROJECT_SOURCE_DIR}/VL53L1X-C-API-Pico/library/import.cmake)
#
# Then, to link <your executable> with <your libraries> and this library:
#     target_link_libraries(<your executable> PUBLIC pico_stdlib hardware_i2c VL53L1X_pico_api <your libraries> )

message("Imported VL53L1X library")

# Define project structure
set(BIN_NAME VL53L1X_pico_api)
set(LIB_VERSION_STR 2.0)
set(LIB_VERSION_MAR 0)
set(PUBLIC_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/public/include")

# Find files for project
file(GLOB PUBLIC_SRCS "${CMAKE_CURRENT_LIST_DIR}/public/src/*")
file(GLOB PUBLIC_HEADERS "${PUBLIC_INCLUDE_DIR}/*")

# Dependencies
set(LIB_DEPS pico_stdlib hardware_i2c)

# Define library
add_library(${BIN_NAME} STATIC ${PUBLIC_SRCS})
set_target_properties(${BIN_NAME} PROPERTIES
        VERSION ${LIB_VERSION_STR}
        SOVERSION ${LIB_VERSION_MAJ}
        PUBLIC_HEADER ${PUBLIC_HEADERS})

# Specify information to link this static library
target_include_directories(${BIN_NAME} PUBLIC ${PUBLIC_INCLUDE_DIR})
target_link_libraries(${BIN_NAME} ${LIB_DEPS})