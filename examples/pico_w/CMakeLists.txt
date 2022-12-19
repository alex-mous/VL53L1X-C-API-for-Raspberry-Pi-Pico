add_executable(VL53L1X_example_pico_w
        VL53L1X_example_pico_w.c
        )

target_link_libraries(VL53L1X_example_pico_w PUBLIC pico_stdlib VL53L1X_api pico_cyw43_arch_none)

# Generate Pico executable
pico_add_extra_outputs(VL53L1X_example_pico_w)
pico_enable_stdio_usb(VL53L1X_example_pico_w 1)