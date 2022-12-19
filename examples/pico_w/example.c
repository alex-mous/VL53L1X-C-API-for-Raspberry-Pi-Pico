#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include <stdint.h>

#include "VL53L1X_api.h"


int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    for (int i=0; i<25; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(250);
        printf("Loading...\n");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(250);
    }
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
 
    uint16_t dev = 0x29;
    uint8_t state = 0;
    uint8_t status;

    /*while (state == 0) {
    *    status |= VL53L1X_BootState(dev, &state);
    }*/

    status |= VL53L1X_SensorInit(dev);
    status |= VL53L1X_SetInterMeasurementInMs(dev, 100);
    status |= VL53L1X_StartRanging(dev);
    printf("status %d\n", status);
    uint8_t dr, rs;
    uint16_t dist;
    while (1) {
        dr = 0;
        while (!dr) {
            status = VL53L1X_CheckForDataReady(dev, &dr);
        }
        status = VL53L1X_GetRangeStatus(dev, &rs);
        status = VL53L1X_GetDistance(dev, &dist);
        status = VL53L1X_ClearInterrupt(dev);
        printf("Distance %d\n", dist);

        sleep_ms(500);
    }

}