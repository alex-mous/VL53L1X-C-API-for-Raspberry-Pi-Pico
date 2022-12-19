add_executable(continuous continuous.c)

# Link to pico libraries as well as the VL53L1X_api
target_link_libraries(continuous PUBLIC VL53L1X_api pico_stdlib)

# Add needed library if Pico W
if(${PICO_BOARD} STREQUAL pico_w)
    target_link_libraries(continuous PUBLIC pico_cyw43_arch_none)
endif()

# Generate executable, enabling STDIO over USB
pico_add_extra_outputs(continuous)
pico_enable_stdio_usb(continuous 1)