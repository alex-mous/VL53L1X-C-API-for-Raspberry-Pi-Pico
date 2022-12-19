# Define flag for Pico W
if(${PICO_BOARD} STREQUAL pico_w)
    add_compile_definitions(PICO_W_BOARD)
    message("Compiling examples for Pico W")
else()
    message("Compiling examples for Pico")
endif()

add_subdirectory(continuous_measurement)