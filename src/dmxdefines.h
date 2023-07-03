#include <Arduino.h>

#ifndef DMXDEFINES_H_
#define DMXDEFINES_H_

/**
 * Note for ESP32:
 * Pins have to match hardware-RX/TX pins, otherwise it will glitch
 * ESP32:
 * TX0: 1
 * RX0: 3
 * TX1: 10  //internal SPI
 * RX1: 9   //internal SPI
 * TX2: 17
 * RX2: 16
 * 
 * ESP32 C3:
 * RX: 20
 * TX: 21
 * 
*/

/*** UART2 (default) ***/
#ifdef CONFIG_IDF_TARGET_ESP32  //ESP32

#define DMX_SERIAL_INPUT_PIN    GPIO_NUM_16 // pin for dmx rx 20
#define DMX_SERIAL_OUTPUT_PIN   GPIO_NUM_NC // pin for dmx tx 21
#define DMX_SERIAL_IO_PIN       GPIO_NUM_NC  // pin for dmx rx/tx change

#define DMX_UART_NUM            UART_NUM_2  // dmx uart
#define HEALTHY_TIME            500         // timeout in ms 
#define BUF_SIZE                2048        //  buffer size for rx events
#define DMX_CORE                1           // select the core the rx/tx thread should run on
#define DMX_PRIORITY            3 
#define DMX_IGNORE_THREADSAFETY 0           // set to 1 to disable all threadsafe mechanisms

#elif CONFIG_IDF_TARGET_ESP32C3
#define DMX_SERIAL_INPUT_PIN    GPIO_NUM_20 // pin for dmx rx 20
#define DMX_SERIAL_OUTPUT_PIN   GPIO_NUM_21 // pin for dmx tx 21
#define DMX_SERIAL_IO_PIN       GPIO_NUM_1  // pin for dmx rx/tx change

#define DMX_UART_NUM            UART_NUM_1  // dmx uart
#define HEALTHY_TIME            500         // timeout in ms 
#define BUF_SIZE                1024        //  buffer size for rx events
#define DMX_CORE                0           // select the core the rx/tx thread should run on
#define DMX_PRIORITY            2 
#define DMX_IGNORE_THREADSAFETY 0           // set to 1 to disable all threadsafe mechanisms
#endif

/*** UART1 ***/
#ifdef CONFIG_IDF_TARGET_ESP32  //ESP32

#define DMX_SERIAL1_INPUT_PIN    GPIO_NUM_NC // pin for dmx rx 20
#define DMX_SERIAL1_OUTPUT_PIN   GPIO_NUM_14 // pin for dmx tx (X13)
#define DMX_SERIAL1_IO_PIN       GPIO_NUM_NC  // pin for dmx rx/tx change

#define DMX_UART1_NUM            UART_NUM_1  // dmx uart
#define HEALTHY1_TIME            500         // timeout in ms 
#define BUF1_SIZE                1024        //  buffer size for rx events
#define DMX1_CORE                1           // select the core the rx/tx thread should run on
#define DMX1_PRIORITY            1 
#define DMX1_IGNORE_THREADSAFETY 0           // set to 1 to disable all threadsafe mechanisms

#elif CONFIG_IDF_TARGET_ESP32C3
#define DMX_SERIAL1_INPUT_PIN    GPIO_NUM_20 // pin for dmx rx 20
#define DMX_SERIAL1_OUTPUT_PIN   GPIO_NUM_21 // pin for dmx tx 21
#define DMX_SERIAL1_IO_PIN       GPIO_NUM_1  // pin for dmx rx/tx change

#define DMX_UART1_NUM            UART_NUM_1  // dmx uart
#define HEALTHY1_TIME            500         // timeout in ms 
#define BUF1_SIZE                1024        //  buffer size for rx events
#define DMX1_CORE                0           // select the core the rx/tx thread should run on
#define DMX1_PRIORITY            2 
#define DMX1_IGNORE_THREADSAFETY 0           // set to 1 to disable all threadsafe mechanisms
#endif


enum DMXDirection { input, output };
enum DMXState { DMX_IDLE, DMX_BREAK, DMX_DATA, DMX_OUTPUT };



#endif  //defines end