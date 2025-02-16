# 12-bit ADC ADS101x (ADS1013, ADS1014 and ADS1015) driver for ESP-IDF and STM32CubeIDE
Driver compatible with ESP-IDF and STM32CubeIDE to use ADS101x 12-bit ADC series

## Features
- Initialization function allows use a custom delay function to ensure compatibility with bare-metal and RTOS applications
- ADC conversion status by interrupt or polling
- Customizable gain and data rate 
- Multiple instances.

## How to use
### ESP-IDF
To use this component follow the next steps:

1. Configure the component in the project configuration menu (`idf.py menuconfig`)

`
Component config->ADS101x Configuration
`

2. Include the I2C driver component headers
```c
#include "driver/i2c_master.h"
#include "ads101x.h"
```
3. Declare a handle of I2C bus and an instance of the compoent
```c
i2c_master_bus_handle_t i2c_bus_handle; 
ads101x_t adc;
```

4. Define a custom delay function in miliseconds according your application
```c
/* Custom delay function */
void delay_ms(uint32_t time)
{
    vTaskDelay(pdMS_TO_TICKS(time));
}
```

5. In the main function initialize the I2C bus and the component instance
```c
/* Configure and create a new I2C bus */
i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_48,
    .sda_io_num = GPIO_NUM_47,
    .glitch_ignore_cnt = 7
};

ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle));

/* Initialize ADS101x instance for ADS1015 */
ESP_ERROR_CHECK(ads101x_init(&adc, ADS101X_MODEL_5, i2c_bus_handle, ADS101X_I2C_ADDRESS, delay_ms));
```

6. To use interrupts to check the ADC conversion status enable it after the initialization
```c
/* Enable ADS101x interrupt and configure the host pin */
ESP_ERROR_CHECK(ads101x_interrupt_enable(&adc, GPIO_NUM_21));
```

7. To perform an ADC read
```c
/* Declare a variable to store the ADC result and perform a single ended read */
int16_t adc_val = 0;
ads101x_read_single_ended(&adc, ADS101X_CHANNEL_0, &adc_val);
```

### STM32CubeIDE
TBD

## License
MIT License

Copyright (c) 2025 Mauricio Barroso Benavides

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
