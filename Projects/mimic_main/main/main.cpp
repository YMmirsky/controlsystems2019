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

#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define SAMPLE_NUM 50   // number of samples taken for multisampling

constexpr uint32_t PWM_MIN_PERCENTAGE = 5;
constexpr uint32_t PWM_MAX_PERCENTAGE = 10;

// used to calculate ADC characteristics such as gain and offset factors
//  The ADC Characteristics structure stores the gain and offset factors of an ESP32 module's ADC. 
//  These factors are calculated using the reference voltage, and the Gain and Offset curves provided in the lookup tables.
static esp_adc_cal_characteristics_t adc_chars; // not entirely sure about what this does
//static const adc1_channel_t channel = ADC1_CHANNEL_0; // Corresponds to GPIO36
// attenuaton (ADC_ATTEN_DB_0) between 100 and 950mV        The input voltage of ADC will be reduced to about 1/1 
// attenuation (ADC_ATTEN_DB_2_5) between 100 and 1250mV    The input voltage of ADC will be reduced to about 1/1.34 
// attenuation (ADC_ATTEN_DB_6) between 150 to 1750mV       The input voltage of ADC will be reduced to about 1/2 
// attenuation (ADC_ATTEN_DB_11) between 150 to 2450mV      The input voltage of ADC will be reduced to about 1/3.6 
static const adc_atten_t atten = ADC_ATTEN_DB_11;  
static const adc_unit_t unit = ADC_UNIT_1; 
// width corresponds to capture width. Available bit widths are 9, 10, 11 and 12. 
static const adc_bits_width_t width = ADC_WIDTH_BIT_12; 

//static const adc1_channel_t channel = ADC1_CHANNEL_0;     //GPIO36
static const adc1_channel_t gpio_pin_rotunda = 	ADC1_CHANNEL_3;    // GPIO36
static const adc1_channel_t gpio_pin_shoulder = ADC1_CHANNEL_5;    // GPIO37
static const adc1_channel_t gpio_pin_elbow = 	ADC1_CHANNEL_6;    // GPIO38
static const adc1_channel_t gpio_pin_wrist = 	ADC1_CHANNEL_4;    // GPIO39

// a lot of this code was inspired by https://github.com/espressif/esp-idf/blob/master/examples/peripherals/adc/main/adc1_example_main.c#L75

ParamsStruct params;
//Server used to listen for XHRs, and send SSEs.
AsyncWebServer server(80);

extern "C" void app_main() {
    
    Serial.begin(115200);
    initArduino();
    initServer(&server, &params);

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

    // raw voltage readings coming from the potentiometers attached to their respective joints
    uint32_t raw_pin_val_rotunda = 0,
			raw_pin_val_shoulder = 0,
			raw_pin_val_elbow = 0,
			raw_pin_val_wrist = 0; 
	// the raw voltage reading needs to be converted to account for various factors
    uint32_t pot_voltage_rotunda = 0,
    		pot_voltage_shoulder = 0,
    		pot_voltage_elbow = 0,
    		pot_voltage_wrist = 0;


    //Characterize ADC
    //adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc1_config_width(ADC_WIDTH_BIT_12); // ADC capture width is 9 bit
    adc1_config_channel_atten(gpio_pin_rotunda, atten);
    adc1_config_channel_atten(gpio_pin_shoulder, atten);
    adc1_config_channel_atten(gpio_pin_elbow, atten);
    adc1_config_channel_atten(gpio_pin_wrist, atten);
    calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);

    while (1)
    {
    	raw_pin_val_rotunda = 0,
		raw_pin_val_shoulder = 0,
		raw_pin_val_elbow = 0,
		raw_pin_val_wrist = 0; 
		// the raw voltage reading needs to be converted to account for various factors
		pot_voltage_rotunda = 0,
		pot_voltage_shoulder = 0,
		pot_voltage_elbow = 0,
		pot_voltage_wrist = 0;

        // multisampling
        for (int i = 0; i < SAMPLE_NUM; i++) 
        {
        	// read raw voltage from the GPIOs
            raw_pin_val_rotunda += adc1_get_raw(gpio_pin_rotunda);
            raw_pin_val_shoulder += adc1_get_raw(gpio_pin_shoulder);
            raw_pin_val_elbow += adc1_get_raw(gpio_pin_elbow);
            raw_pin_val_wrist += adc1_get_raw(gpio_pin_wrist);
        }
        
        // take average of raw readings
        raw_pin_val_rotunda /= SAMPLE_NUM;
        raw_pin_val_shoulder /= SAMPLE_NUM;
        raw_pin_val_elbow /= SAMPLE_NUM;
        raw_pin_val_wrist /= SAMPLE_NUM;

        pot_voltage_rotunda = esp_adc_cal_raw_to_voltage(raw_pin_val_rotunda, &adc_chars);
        pot_voltage_shoulder = esp_adc_cal_raw_to_voltage(raw_pin_val_shoulder, &adc_chars);
        pot_voltage_elbow = esp_adc_cal_raw_to_voltage(raw_pin_val_elbow, &adc_chars);
        pot_voltage_wrist = esp_adc_cal_raw_to_voltage(raw_pin_val_wrist, &adc_chars);


        // write the value of the potentiometer
        printf("%s: %i \n", "Rotunda:	", pot_voltage_rotunda);
        printf("%s: %i \n", "Shoulder:	", pot_voltage_shoulder);
        printf("%s: %i \n", "Elbow:		", pot_voltage_elbow);
        printf("%s: %i \n", "Wrist:		", pot_voltage_wrist);
        printf("\n");	// print new line


        vTaskDelay(10);    
    }
    // told by adc.h comments to run call this before calling adc1_get_raw for the first time
}

