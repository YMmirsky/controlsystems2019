#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include "driver/adc.h"
#include "driver/gpio.h"
#include "EEPROM.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "Arduino.h"
#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "RTOStasks.h"
#include "Source.h"
#include "constants.h"



// a lot of this code was inspired by https://github.com/espressif/esp-idf/blob/master/examples/peripherals/adc/main/adc1_example_main.c#L75

ParamsStruct params;
//Server used to listen for XHRs, and send SSEs.
AsyncWebServer server(80);

extern "C" void app_main() {
    
    Serial.begin(115200);
    initArduino();
    readESP32(&server, &params);

    //Init EEPROM (how we write data)
    if (!initEEPROM()) {
        for (int i = 10; i >= 0; i--) {
            printf("Restarting in %d seconds...\n", i);
            usleep(1000);
        }
        printf("Restarting now.\n");
        fflush(stdout);
        esp_restart();
    }
        vTaskDelay(10);    
}


