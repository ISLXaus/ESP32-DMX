/* 
 * This file is part of the ESP32-DMX distribution (https://github.com/luksal/ESP32-DMX).
 * Copyright (c) 2021 Lukas Salomon.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"    //added gpio.h

#ifndef DMX_h
#define DMX_h
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
#define BUF_SIZE                1024        //  buffer size for rx events
#define DMX_CORE                0           // select the core the rx/tx thread should run on
#define DMX_PRIORITY            2 
#define DMX_IGNORE_THREADSAFETY 0           // set to 1 to disable all threadsafe mechanisms

/*** UART1 ***/
#define DMX_SERIAL1_INPUT_PIN    GPIO_NUM_NC // pin for dmx rx
#define DMX_SERIAL1_OUTPUT_PIN   GPIO_NUM_13 // pin for dmx tx
#define DMX_SERIAL1_IO_PIN       GPIO_NUM_NC  // pin for dmx rx/tx change

#define DMX_UART1_NUM            UART_NUM_1  // dmx uart
#define HEALTHY1_TIME            500         // timeout in ms 
#define BUF1_SIZE                1024        //  buffer size for rx events
#define DMX1_CORE                0           // select the core the rx/tx thread should run on
#define DMX1_PRIORITY            2 
#define DMX1_IGNORE_THREADSAFETY 0           // set to 1 to disable all threadsafe mechanisms


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




enum DMXDirection { input, output };
enum DMXState { DMX_IDLE, DMX_BREAK, DMX_DATA, DMX_OUTPUT };

class DMX
{
    public:
        static void Initialize(DMXDirection direction);     // initialize library
        static uint8_t Read(uint16_t channel);              // returns the dmx value for the givven address (values from 1 to 512)
        static void ReadAll(uint8_t * data, uint16_t start, size_t size);   // copies the defined channels from the read buffer
        static void Write(uint16_t channel, uint8_t value); // writes the dmx value to the buffer
        static void WriteAll(uint8_t * data, uint16_t start, size_t size);  // copies the defined channels into the write buffer
        static uint8_t IsHealthy();                            // returns true, when a valid DMX signal was received within the last 500ms
        static void changeDirection(DMXDirection direction);
        static long getLastPacket();
        
    private:
        DMX();                                              // hide constructor

        static QueueHandle_t dmx_rx_queue;                  // queue for uart rx events
        static SemaphoreHandle_t sync_dmx;                  // semaphore for syncronising access to dmx array
        static DMXState dmx_state;                           // status, in which recevied state we are
        static uint16_t current_rx_addr;                    // last received dmx channel
        static long last_dmx_packet;                        // timestamp for the last received packet
        static uint8_t dmx_data[513];                       // stores the received dmx data
        static bool initialized;
        static TaskHandle_t rxTaskHandle;                          //task handler for rx task
        static TaskHandle_t txTaskHandle;                          //task handler for tx task
        static void uart_event_task(void *pvParameters);    // event task
        static void uart_send_task(void*pvParameters);      // transmit task
};

#endif