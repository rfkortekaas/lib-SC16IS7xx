#ifndef __SC16IS7xx_h
#define __SC16IS7xx_h

#include <Arduino.h>
#include <esp_err.h>

#include "SC16IS7xx_reg.h"
#include "Stream.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifndef SC16IS7XX_XTAL_FREQ
#define SC16IS7XX_XTAL_FREQ 20000000UL  // 20MHz
#endif

class SC16IS7xx : public Stream {
 public:
  SC16IS7xx(int uart_nr, spi_host_device_t host, gpio_num_t cs,
            gpio_num_t reset);
  SC16IS7xx(int uart_nr);

  void begin(const uint32_t baudrate,
             const uint32_t config = SC16IS7XX_SERIAL_8N1,
             unsigned long timeout_ms = 20000UL);
  void end();
  int read(void);
  size_t write(uint8_t);
  size_t write(const uint8_t* buffer, size_t size);
  int available(void);
  int peek(void);
  void flush(void);

  inline size_t write(const char* s) { return write((uint8_t*)s, strlen(s)); }

  inline size_t write(unsigned long n) { return write((uint8_t)n); }

  inline size_t write(long n) { return write((uint8_t)n); }

  inline size_t write(unsigned int n) { return write((uint8_t)n); }

  inline size_t write(int n) { return write((uint8_t)n); }

  uint32_t baudRate();
  operator bool() const;

 protected:
  void reset_device(void);
  void enable_fifo(const bool fifo_enable);
  uint8_t read_register(const uint8_t register_address);
  void write_register(const uint8_t register_address, const uint8_t data);

  static spi_device_handle_t _spi;
  uint8_t _acquire_bus_cnt = 0;
  gpio_num_t device_reset_pin;
  int _uart_nr = -1;
  uint32_t _actual_baudrate;
  int16_t _error;

 private:
  SC16IS7xx(const SC16IS7xx&);
  SC16IS7xx& operator=(const SC16IS7xx&);

  esp_err_t acquire_bus(void);
  void release_bus(void);
  int16_t set_baudrate(const uint32_t baudrate);
  void set_line(const uint8_t data_length, const uint8_t parity_select,
                const uint8_t stop_length);
};

#endif  // __SC16IS7xx_h
