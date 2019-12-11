#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>

#include "SC16IS7xx.h"
#include "SC16IS7xx_reg.h"

SC16IS7xx::SC16IS7xx(spi_host_device_t host, gpio_num_t cs, gpio_num_t reset) {
    spi_device_interface_config_t devcfg =
    {
        command_bits: 0,
        address_bits: 0,
        dummy_bits: 0,
        mode: 0,
        duty_cycle_pos: 0,
        cs_ena_pretrans: 0,
        cs_ena_posttrans: 0,
        clock_speed_hz: SPI_MASTER_FREQ_10M,
        input_delay_ns: 20,
        spics_io_num: cs,
        flags: 0,
        queue_size: 256,
        pre_cb: NULL,
        post_cb: NULL
    };
    device_reset_pin = reset;

    esp_err_t ret = spi_bus_add_device(host, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);
}

esp_err_t SC16IS7xx::init() {
    pinMode(device_reset_pin, OUTPUT);
    digitalWrite(device_reset_pin, LOW);
    delay(10);
    digitalWrite(device_reset_pin, HIGH);
    delay(10);

    acquire_bus();

    reset_device();
    enable_fifo(true);

    return ESP_OK;
}

esp_err_t SC16IS7xx::acquire_bus() {
    esp_err_t ret;
    if (_acquire_bus_cnt > 10)
    {
        ret = ESP_ERR_INVALID_ARG;
    } else  {   
        ret = spi_device_acquire_bus(_spi, portMAX_DELAY);
        ESP_ERROR_CHECK(ret);
        if (ret == ESP_OK)
            _acquire_bus_cnt++;
    }
    return ret;
}

void SC16IS7xx::release_bus() {
    if (_acquire_bus_cnt > 0) {
        _acquire_bus_cnt--;
        if (_acquire_bus_cnt == 0) {
            spi_device_release_bus(_spi);
        }
    }
}

int16_t SC16IS7xx::set_baudrate(const uint32_t crystal_frequency, const uint32_t baudrate) {
    uint16_t divisor;
    uint8_t prescaler;
    uint32_t actual_baudrate;
    int16_t error;
    uint8_t temp_lcr;
    
    if ((read_register(SC16IS7XX_REG_MCR) & 0x80) == 0) {
        prescaler = 1;
    } else {
        prescaler = 4;
    }

    divisor = (crystal_frequency/prescaler)/(baudrate*16) + 1;

    temp_lcr = read_register(SC16IS7XX_REG_LCR);
    temp_lcr |= 0x80;
    write_register(SC16IS7XX_REG_LCR, temp_lcr);
    // Write to DLL
    write_register(SC16IS7XX_REG_DLL, (uint8_t)divisor);
    // Write to DLH
    write_register(SC16IS7XX_REG_DLH, (uint8_t)(divisor>>8));
    temp_lcr &= 0x7F;
    write_register(SC16IS7XX_REG_LCR, temp_lcr);
    
    actual_baudrate = (crystal_frequency/prescaler)/(16*divisor);
    error = ((float)actual_baudrate-baudrate)*1000/baudrate;

    Serial.print("Crystal: ");
    Serial.println(crystal_frequency,DEC);
    Serial.print("Desired baudrate: ");
    Serial.println(baudrate,DEC);
    Serial.print("Prescaler: ");
    Serial.println(prescaler,DEC);
    Serial.print("Calculated divisor: ");
    Serial.println(divisor,DEC);
    Serial.print("Actual baudrate: ");
    Serial.println(actual_baudrate,DEC);
    Serial.print("Baudrate error: ");
    Serial.println(error,DEC);

    return error;
}

void SC16IS7xx::set_line(const uint8_t data_length, const uint8_t parity_select, uint8_t stop_length) {
    uint8_t temp_lcr;
    temp_lcr = read_register(SC16IS7XX_REG_LCR);
    temp_lcr &= 0xC0; // Clear the lower six bits of LCR (LCR[0..5])
    
    switch (data_length) {
    case 5:
        break;
    case 6:
        temp_lcr |= 0x01;
        break;
    case 7:
        temp_lcr |= 0x02;
        break;
    case 8:
    default:
        temp_lcr |= 0x03;
        break;
    }

    if (stop_length == 2) {
        temp_lcr |= 0x04;
    }

    switch (parity_select)
    {
    case 1:                 // Odd Parity
        temp_lcr |= 0x08;
        break;
    case 2:                 // Even Parity
        temp_lcr |= 0x18;
        break;
    case 3:                 // Force '1' Parity
        temp_lcr |= 0x03;
        break;
    case 0:                 // No Parity
    case 4:                 // Force '0' Parity
    default:
        break;
    }

    write_register(SC16IS7XX_REG_LCR, temp_lcr);
}

uint8_t SC16IS7xx::read_register(const uint8_t register_address) {
    esp_err_t ret;
    spi_transaction_t t;
    
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 16;
    t.rxlength = 8;
    t.tx_data[0] = 0x82|(register_address<<3);
    t.tx_data[1] = 0xFF;

    ret = spi_device_polling_transmit(_spi, &t);
    ESP_ERROR_CHECK(ret);
    return t.rx_data[1];
}

void SC16IS7xx::write_register(const uint8_t register_address, const uint8_t val) {
    esp_err_t ret;
    spi_transaction_t t;

    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 16;
    t.rxlength = 0;
    t.tx_data[0] = 0x2|register_address << 3;
    t.tx_data[1] = val;
    
    ret = spi_device_polling_transmit(_spi, &t);
    ESP_ERROR_CHECK(ret);
}

void SC16IS7xx::write(uint8_t val) {
    uint8_t tmp_lsr;

    do {
        tmp_lsr = read_register(SC16IS7XX_REG_LSR);
    } while ((tmp_lsr & 0x20) == 0);

    write_register(SC16IS7XX_REG_THR, val);
}

int16_t SC16IS7xx::read(void) {
    volatile uint8_t val;

    val = -1;
    if (read_register(SC16IS7XX_REG_RXLVL)) {
        val = read_register(SC16IS7XX_REG_RHR);
    }
    return val;
}

void SC16IS7xx::reset_device(void) {
    uint8_t reg;

    reg = read_register(SC16IS7XX_REG_IOCONTROL); // MOSI 0xF0.. MISO 0xFF00..
    reg |= 0x08;
    write_register(SC16IS7XX_REG_IOCONTROL, reg);
}

void SC16IS7xx::enable_fifo(const bool fifo_enable) {
    uint8_t temp_fcr;

    temp_fcr = read_register(SC16IS7XX_REG_FCR);

    if (fifo_enable == true) {
        temp_fcr &= 0xFE;
    } else {
        temp_fcr |= 0x01;
    }
    write_register(SC16IS7XX_REG_FCR, temp_fcr);
}