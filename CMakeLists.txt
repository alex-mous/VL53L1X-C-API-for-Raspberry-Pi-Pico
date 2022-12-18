cmake_minimum_required(VERSION 3.12)

# Pull in SDK by downloading the pico_sdk_import
set(PICO_SDK_IMPORT ${CMAKE_CURRENT_LIST_DIR}/external/pico_sdk_import.cmake)
if (NOT EXISTS ${MAKE_FS_DATA_SCRIPT})
        file(DOWNLOAD
                https://github.com/raspberrypi/pico-sdk/blob/2e6142b15b8a75c1227dd3edbe839193b2bf9041/external/pico_sdk_import.cmake
                ${MAKE_FS_DATA_SCRIPT}
                )
endif()

include(PICO_SDK_IMPORT)


project(vl53l1x_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format     
        -Wno-unused-function
        -Wno-maybe-uninitialized
        )


# Compile examples
add_subdirectory(example)