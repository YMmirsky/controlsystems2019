#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <iostream>
#include <WiFi.h>
#include "Arduino.h"
#include "constants.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "EEPROM.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "Source.h"

// TEST HEADERS: Taken from:
//  https://github.com/espressif/esp-idf/issues/1646
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// a lot of this code was inspired by 
//  https://github.com/espressif/esp-idf/blob/master/examples/peripherals/adc/main/adc1_example_main.c#L75
#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define SAMPLE_NUM 50   // number of samples taken for multisampling

constexpr uint32_t PWM_MIN_PERCENTAGE = 5;
constexpr uint32_t PWM_MAX_PERCENTAGE = 10;

/* used to calculate ADC characteristics such as gain and offset factors
    The ADC Characteristics structure stores the gain and offset factors of an ESP32 module's ADC. 
    These factors are calculated using the reference voltage, 
    and the Gain and Offset curves provided in the lookup tables.
*/
static esp_adc_cal_characteristics_t adc_chars; // not entirely sure about what this does
//static const adc1_channel_t channel = ADC1_CHANNEL_0; // Corresponds to GPIO36
/*  attenuaton (ADC_ATTEN_DB_0) between 100 and 950mV        The input voltage of ADC will be reduced to about 1/1 
    attenuation (ADC_ATTEN_DB_2_5) between 100 and 1250mV    The input voltage of ADC will be reduced to about 1/1.34 
    attenuation (ADC_ATTEN_DB_6) between 150 to 1750mV       The input voltage of ADC will be reduced to about 1/2 
    attenuation (ADC_ATTEN_DB_11) between 150 to 2450mV      The input voltage of ADC will be reduced to about 1/3.6 
*/
static const adc_atten_t atten = ADC_ATTEN_DB_11;  
static const adc_unit_t unit = ADC_UNIT_1; 
// width corresponds to capture width. Available bit widths are 9, 10, 11 and 12. 
static const adc_bits_width_t width = ADC_WIDTH_BIT_12; 

//static const adc1_channel_t channel = ADC1_CHANNEL_0;     //GPIO36
static const adc1_channel_t gpio_pin_rotunda =  ADC1_CHANNEL_3;    // GPIO36
static const adc1_channel_t gpio_pin_shoulder = ADC1_CHANNEL_5;    // GPIO37
static const adc1_channel_t gpio_pin_elbow =    ADC1_CHANNEL_6;    // GPIO38
static const adc1_channel_t gpio_pin_wrist =    ADC1_CHANNEL_4;    // GPIO39
//static const adc1_channel_t gpio_pin_wrist_roll = ADC1_CHANNEL_XXXXXXX

void readESP32(AsyncWebServer* server, ParamsStruct* params) {
    //Create Access Point
    WiFi.softAP("MyESP32AP", "testpassword");
    Serial.println();
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
    
    AsyncEventSource events("/events");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
 
    // raw voltage readings coming from the potentiometers attached to their respective joints
    uint32_t raw_pin_val_rotunda = 0,
            raw_pin_val_shoulder = 0,
            raw_pin_val_elbow = 0,
            raw_pin_val_wrist = 0,
            raw_pin_val_wrist_roll = 0;
    // the raw voltage reading needs to be converted to account for various factors
    uint32_t pot_voltage_rotunda = 0,
            pot_voltage_shoulder = 0,
            pot_voltage_elbow = 0,
            pot_voltage_wrist = 0,
            pot_voltage_wrist_roll = 0;

    double angle_rotunda = 0,
            angle_shoulder = 0,
            angle_elbow = 0,
            angle_wrist = 0,
            angle_wrist_roll = 0;

    std::string str_angle_rotunda = "rotunda: ",
            str_angle_shoulder = "shoulder: ",
            str_angle_elbow = "elbow: ",
            str_angle_wrist = "wrist: ",
            str_angle_wrist_roll = "wrist_roll: ";
    //Characterize ADC
    //adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc1_config_width(ADC_WIDTH_BIT_12); // ADC capture width is 9 bit
    adc1_config_channel_atten(gpio_pin_rotunda, atten);
    adc1_config_channel_atten(gpio_pin_shoulder, atten);
    adc1_config_channel_atten(gpio_pin_elbow, atten);
    adc1_config_channel_atten(gpio_pin_wrist, atten);
    //adc1_config_channel_atten(gpio_pin_wrist_roll, atten);
    calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);

    // Attach event source to the server.
    server->addHandler(&events);
    // Start server.
    server->begin();

    while (1)
    {
        raw_pin_val_rotunda = 0,
        raw_pin_val_shoulder = 0,
        raw_pin_val_elbow = 0,
        raw_pin_val_wrist = 0,
        raw_pin_val_wrist_roll = 0;
        // the raw voltage reading needs to be converted to account for various factors
        pot_voltage_rotunda = 0,
        pot_voltage_shoulder = 0,
        pot_voltage_elbow = 0,
        pot_voltage_wrist = 0,
        pot_voltage_wrist_roll = 0;

        // multisampling, take the average reading of a pin X times to reduce measurement errors
        for (int i = 0; i < SAMPLE_NUM; i++) 
        {
            // read raw voltage from the GPIOs
            raw_pin_val_rotunda += adc1_get_raw(gpio_pin_rotunda);
            raw_pin_val_shoulder += adc1_get_raw(gpio_pin_shoulder);
            raw_pin_val_elbow += adc1_get_raw(gpio_pin_elbow);
            raw_pin_val_wrist += adc1_get_raw(gpio_pin_wrist);
            //raw_pin_val_wrist_roll += adc1_get_raw(gpio_pin_wrist_roll);
        }
        raw_pin_val_rotunda /= SAMPLE_NUM;
        raw_pin_val_shoulder /= SAMPLE_NUM;
        raw_pin_val_elbow /= SAMPLE_NUM;
        raw_pin_val_wrist /= SAMPLE_NUM;
        //raw_pin_val_wrist_roll /= SAMPLE_NUM;

        pot_voltage_rotunda = esp_adc_cal_raw_to_voltage(raw_pin_val_rotunda, &adc_chars);
        pot_voltage_shoulder = esp_adc_cal_raw_to_voltage(raw_pin_val_shoulder, &adc_chars);
        pot_voltage_elbow = esp_adc_cal_raw_to_voltage(raw_pin_val_elbow, &adc_chars);
        pot_voltage_wrist = esp_adc_cal_raw_to_voltage(raw_pin_val_wrist, &adc_chars);
        //pot_voltage_wrist_roll = esp_adc_cal_raw_to_voltage(raw_pin_val_wrist_roll, &adc_chars);

        // do a thing here that converts the pot values to angles
        angle_rotunda = pot_voltage_rotunda;
        angle_shoulder = pot_voltage_shoulder;
        angle_elbow = pot_voltage_elbow;
        angle_wrist = pot_voltage_wrist;
        angle_wrist_roll = pot_voltage_wrist_roll;
        // convert all double values to raw strings in order to send via. 
        std::ostringstream strs;
        strs << angle_rotunda;
        str_angle_rotunda += strs.str();
        strs << angle_shoulder;
        str_angle_shoulder += strs.str();
        strs << angle_elbow;
        str_angle_elbow += strs.str();
        strs << angle_wrist;
        str_angle_wrist += strs.str();

        /* XHR Example.
        - Param "name" is sent from mission control.
        - "name" is copied to the params object
        - params object is then passed to vSayHelloTask - see main.cpp
        - vSayHello task then accesses name directly.

        Note: for ANY parameters you want to use, you must add them to
        the paramsStruct struct located in Source.h first. 
        */
        // the idea for this is taken from Nelson Wong's implementation of XHR for his ROSify project.
        // The server should send a request to the ESP32, which replies with data
        server->on("/update_name", HTTP_POST, [angle_rotunda, angle_shoulder, angle_elbow, angle_wrist](AsyncWebServerRequest *request){
            // b_t_r corresponds to the size of the package that will be sent via. XHR. 
            //  according to Nelson W., will need to adjust this precisely for the number 
            //  of chars that will be sent
            int bytes_to_write = 1000;
            char joint_buffer[bytes_to_write];
            snprintf(joint_buffer, bytes_to_write, 
            "rotunda: %f, shoulder: %f, elbow: %f, wrist pitch: %f, wrist roll: %f",
            angle_rotunda, angle_shoulder, angle_elbow, angle_wrist, angle_wrist_roll);
            request->send(200, "text/plain", joint_buffer);
        });
        delay(10);
        // TEST CODE, taken from:
        //  https://github.com/espressif/esp-idf/issues/1646
        TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
        TIMERG0.wdt_feed=1;
        TIMERG0.wdt_wprotect=0;

        /* SSE Example.
            - SSEs will be used to continuously send data that was
            not necessarily requested by mission control
            (e.g. temperature, something we should send periodically)

            - Once mission control declares the ESPs IP address at a certain 
            endpoint to be an EventSource, the ESP can trigger events on the web
            interface, which the web interface can attach event listeners to
            (similar to how we are attaching event listeners for when we recieve
            XHRs to /update_name above, allowing us to do things when we recieve an 
            XHR).
            - Below's example is an example of sending SSEs when mission control
            declares our ip address and endpoint (e.g. 192.168.4.1/events) to be
            an event source.
            - More info on this concept here: 
                https://developer.mozilla.org/en-US/docs/Web/API/EventSource
        */
        /*events.onConnect([str_angle_rotunda, str_angle_shoulder, str_angle_elbow, str_angle_wrist](AsyncEventSourceClient *client) 
        {
            if(client->lastId())
            {
                Serial.printf("Client reconnected! Last message ID that it gat is: %u\n", client->lastId());
            }
            // send event with message "hello!", id current millis and set reconnect delay to 1 second
            client->send("hello!", NULL, millis(), 1000);
            client->send(str_angle_rotunda.c_str(), NULL, millis(), 1000);
            client->send(str_angle_shoulder.c_str(), NULL, millis(), 1000);
            client->send(str_angle_elbow.c_str(), NULL, millis(), 1000);
            client->send(str_angle_wrist.c_str(), NULL, millis(), 1000);
        });
        */
        // write the value of the potentiometer
        //printf("%s: %i \n", "Rotunda: ", pot_voltage_rotunda);
        //printf("%s: %i \n", "Shoulder:    ", pot_voltage_shoulder);
        //printf("%s: %i \n", "Elbow:       ", pot_voltage_elbow);
        //printf("%s: %i \n", "Wrist:       ", pot_voltage_wrist);
        //delay(1000)
        //printf("\n"); // print new line
    }
    
}

bool initEEPROM() 
{
    bool status = EEPROM.begin(EEPROM_SIZE);
    switch(status)
    {
        case 0:
	    printf("ERROR: EEPROM initialization failure.\n");
	    usleep(1000);
	    break;
	case 1:
	    printf("Successfully initialized EEPROM, size = %d.\n", EEPROM_SIZE);
	    usleep(1000);
	    break;
	default:
	    break;
    }
    return status;    
}

int EEPROMCount(int addr)
{
    int data = EEPROM.read(addr);
    data++;
    EEPROM.write(addr, data);
    EEPROM.commit();
    return data;
}