add_executable(example
        example.c
        )

target_link_libraries(example PUBLIC pico_stdlib VL53L1X_api pico_cyw43_arch_none)

# Generate Pico executable
pico_add_extra_outputs(example)
pico_enable_stdio_usb(example 1)