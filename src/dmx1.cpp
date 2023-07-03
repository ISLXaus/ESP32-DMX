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

#include <dmx1.h>
#include <Arduino.h>

QueueHandle_t DMX1::dmx_rx_queue;
SemaphoreHandle_t DMX1::sync_dmx;
DMXState DMX1::dmx_state = DMX_IDLE;
uint16_t DMX1::current_rx_addr = 0;
long DMX1::last_dmx_packet = 0;
uint8_t DMX1::dmx_data[513];
bool DMX1::initialized = false;
TaskHandle_t DMX1::rxTaskHandle = NULL;
TaskHandle_t DMX1::txTaskHandle = NULL;

DMX1::DMX1()
{

}

void DMX1::Initialize(DMXDirection direction)
{
   if(initialized == false){
          // configure UART for DMX
      uart_config_t uart_config =
      {
         .baud_rate = 250000,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_2,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
      };

      uart_param_config(DMX_UART1_NUM, &uart_config);

      // Set pins for UART
      uart_set_pin(DMX_UART1_NUM, DMX_SERIAL1_OUTPUT_PIN, DMX_SERIAL1_INPUT_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

      // install queue
      uart_driver_install(DMX_UART1_NUM, BUF1_SIZE * 2, BUF1_SIZE * 2, 20, &dmx_rx_queue, 0);

      // create mutex for syncronisation
      sync_dmx = xSemaphoreCreateMutex();

      // set gpio for direction
      gpio_pad_select_gpio(DMX_SERIAL1_IO_PIN);
      gpio_set_direction(DMX_SERIAL1_IO_PIN, GPIO_MODE_OUTPUT);
   }
   //start rx/tx tasks
   changeDirection(direction);

   //now library is initialized
   initialized = true;
}

uint8_t DMX1::Read(uint16_t channel)
{
    // restrict acces to dmx array to valid values
    if(channel < 1 || channel > 512)
    {
        return 0;
    }

    // take data threadsafe from array and return
#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    uint8_t tmp_dmx = dmx_data[channel];
#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
    return tmp_dmx;
}

void DMX1::ReadAll(uint8_t * data, uint16_t start, size_t size)
{
    // restrict acces to dmx array to valid values
    if(start < 1 || start > 512 || start + size > 513)
    {
        return;
    }
#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    memcpy(data, (uint8_t *)dmx_data + start, size);
#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

void DMX1::Write(uint16_t channel, uint8_t value)
{
    // restrict acces to dmx array to valid values
    if(channel < 1 || channel > 512)
    {
        return;
    }

#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    dmx_data[channel] = value;
#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

void DMX1::WriteAll(uint8_t * data, uint16_t start, size_t size)
{
    // restrict acces to dmx array to valid values
    if(start < 1 || start > 512 || start + size > 513)
    {
        return;
    }
#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    memcpy((uint8_t *)dmx_data + start, data, size);
#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

uint8_t DMX1::IsHealthy()
{
    // get timestamp of last received packet
#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    long dmx_timeout = last_dmx_packet;
#ifndef DMX1_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
    // check if elapsed time < defined timeout
    if(xTaskGetTickCount() - dmx_timeout < HEALTHY1_TIME)
    {
        return 1;
    }
    return 0;
}

void DMX1::changeDirection(DMXDirection direction){
   if(initialized == true){
      if( txTaskHandle != NULL ){   //stop tx task
         vTaskDelete( txTaskHandle );
      }
      if( rxTaskHandle != NULL ){   //stop rx task
         vTaskDelete( rxTaskHandle );
      }
   }

   
   if(direction == output)
   {
      gpio_set_level(DMX_SERIAL1_IO_PIN, 1);
      dmx_state = DMX_OUTPUT;
      
      // create send task
      Serial.println("start uart_send_task1");
      xTaskCreatePinnedToCore(DMX1::uart_send_task1, "uart_send_task1", 1024, NULL, DMX1_PRIORITY, &DMX1::txTaskHandle, DMX1_CORE);
   }
   else
   {    
      gpio_set_level(DMX_SERIAL1_IO_PIN, 0);
      dmx_state = DMX_IDLE;

      // create receive task
      Serial.println("start uart_event_task1");
      xTaskCreatePinnedToCore(DMX1::uart_event_task1, "uart_event_task1", 2048, NULL, DMX1_PRIORITY, &DMX1::rxTaskHandle, DMX1_CORE);
   }
}

long DMX1::getLastPacket(){
   return last_dmx_packet;
}

void DMX1::uart_send_task1(void*pvParameters)
{
    uint8_t start_code = 0x00;
    for(;;)
    {
        // wait till uart is ready
        uart_wait_tx_done(DMX_UART1_NUM, 1000);
        // set line to inverse, creates break signal
        uart_set_line_inverse(DMX_UART1_NUM, UART_SIGNAL_TXD_INV);
        // wait break time
        ets_delay_us(184);
        // disable break signal
        uart_set_line_inverse(DMX_UART1_NUM,  0);
        // wait mark after break
        ets_delay_us(24);
        // write start code
        uart_write_bytes(DMX_UART1_NUM, (const char*) &start_code, 1);
#ifndef DMX1_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
        // transmit the dmx data
        
        uart_write_bytes(DMX_UART1_NUM, (const char*) dmx_data+1, 512);
#ifndef DMX1_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_dmx);
#endif
    }
}

void DMX1::uart_event_task1(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(BUF1_SIZE);
    for(;;)
    {
        // wait for data in the dmx_queue
        if(xQueueReceive(dmx_rx_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, BUF1_SIZE);
            switch(event.type)
            {
                case UART_DATA:
                    // read the received data
                    uart_read_bytes(DMX_UART1_NUM, dtmp, event.size, portMAX_DELAY);
                    // check if break detected
                    if(dmx_state == DMX_BREAK)
                    {
                        // if not 0, then RDM or custom protocol
                        if(dtmp[0] == 0)
                        {
                        dmx_state = DMX_DATA;
                        // reset dmx adress to 0
                        current_rx_addr = 0;
#ifndef DMX1_IGNORE_THREADSAFETY
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
                        // store received timestamp
                        last_dmx_packet = xTaskGetTickCount();
                        // last_dmx_packet = millis();
#ifndef DMX1_IGNORE_THREADSAFETY
                        xSemaphoreGive(sync_dmx);
#endif
                        }
                    }
                    // check if in data receive mode
                    if(dmx_state == DMX_DATA)
                    {
#ifndef DMX1_IGNORE_THREADSAFETY
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
                        // copy received bytes to dmx data array
                        for(int i = 0; i < event.size; i++)
                        {
                            if(current_rx_addr < 513)
                            {
                                dmx_data[current_rx_addr++] = dtmp[i];
                            }
                        }
#ifndef DMX1_IGNORE_THREADSAFETY
                        xSemaphoreGive(sync_dmx);
#endif
                    }
                    break;
                case UART_BREAK:
                    // break detected
                    // clear queue und flush received bytes                    
                    uart_flush_input(DMX_UART1_NUM);
                    xQueueReset(dmx_rx_queue);
                    dmx_state = DMX_BREAK;
                    break;
                case UART_FRAME_ERR:
                case UART_PARITY_ERR:
                case UART_BUFFER_FULL:
                case UART_FIFO_OVF:
                default:
                    // error recevied, going to idle mode
                    uart_flush_input(DMX_UART1_NUM);
                    xQueueReset(dmx_rx_queue);
                    dmx_state = DMX_IDLE;
                    break;
            }
        }
    }
}
