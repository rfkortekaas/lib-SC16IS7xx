#ifndef __SC16IS7xx_h
#define __SC16IS7xx_h

#include <Arduino.h>
#include <esp_err.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

class SC16IS7xx {
    public:
        SC16IS7xx(spi_host_device_t host, gpio_num_t cs, gpio_num_t reset);

        esp_err_t init(void);
        esp_err_t acquire_bus(void);
        void release_bus(void);
        int16_t set_baudrate(const uint32_t crystal_frequency, const uint32_t baudrate);
        void set_line(const uint8_t data_length, const uint8_t parity_select, const uint8_t stop_length);

        void write(const uint8_t val);
        int16_t read(void);
        

    protected:
        void reset_device(void);
        void enable_fifo(const bool fifo_enable);
        uint8_t read_register(const uint8_t register_address);
        void write_register(const uint8_t register_address, const uint8_t data);

        spi_device_handle_t _spi;
        uint8_t _acquire_bus_cnt = 0;
        gpio_num_t device_reset_pin;

    private:
        SC16IS7xx(const SC16IS7xx&);
        SC16IS7xx& operator=(const SC16IS7xx&);
};

#endif // __SC16IS7xx_h
