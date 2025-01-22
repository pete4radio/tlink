#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define COINES_I2C_BUS_MAX 2
#define COINES_SPI_BUS_MAX 2
#define I2C_TIMEOUT_MS 1000

#define I2C_SCL_PIN 5 // Replace with your actual SCL pin number
#define I2C_SDA_PIN 4 // Replace with your actual SDA pin number

enum coines_i2c_bus {
    COINES_I2C_BUS_0,
    COINES_I2C_BUS_1,
    COINES_I2C_BUS_MAX
};

enum coines_status {
    COINES_SUCCESS,
    COINES_E_COMM_IO_ERROR,
    COINES_E_FAILURE,
    COINES_E_I2C_BUS_NOT_ENABLED,
    COINES_E_I2C_INVALID_BUS_INTF
};

static i2c_inst_t *coines_i2c_instance[COINES_I2C_BUS_MAX] = { i2c0, i2c1 };

bool coines_is_i2c_enabled(enum coines_i2c_bus bus) {
    // Implement this function to check if the I2C bus is enabled
    return true;
}

void coines_yield() {
    // Implement this function if needed
}

uint32_t coines_get_millis() {
    return to_ms_since_boot(get_absolute_time());
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 */
int8_t coines_read_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) {
    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0)) {
        if (coines_is_i2c_enabled(bus)) {
            uint8_t buffer[1] = { reg_addr };
            absolute_time_t timeout = make_timeout_time_ms(I2C_TIMEOUT_MS);

            int result = i2c_write_blocking_until(coines_i2c_instance[bus], dev_addr, buffer, 1, true, timeout);
            if (result == PICO_ERROR_GENERIC) {
                return COINES_E_COMM_IO_ERROR;
            }

            result = i2c_read_blocking_until(coines_i2c_instance[bus], dev_addr, reg_data, count, false, timeout);
            if (result == PICO_ERROR_GENERIC) {
                return COINES_E_COMM_IO_ERROR;
            }
            elif(result == PICO_ERROR_TIMEOUT) {
                coines_i2c_bus_recover(bus);
                return COINES_E_COMM_IO_ERROR;
            }
            return COINES_SUCCESS;
        } else {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    } else {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 */
int8_t coines_i2c_set(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count) {
    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0)) {
        if (coines_is_i2c_enabled(bus)) {
            absolute_time_t timeout = make_timeout_time_ms(I2C_TIMEOUT_MS);

            int result = i2c_write_blocking_until(coines_i2c_instance[bus], dev_addr, data, count, false, timeout);
            if (result == PICO_ERROR_GENERIC) {
                return COINES_E_COMM_IO_ERROR;
            }

            return COINES_SUCCESS;
        } else {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    } else {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}

enum coines_i2c_bus {
    COINES_I2C_BUS_0,
    COINES_I2C_BUS_1,
    COINES_I2C_BUS_MAX
};

enum coines_spi_bus {
    COINES_SPI_BUS_0,
    COINES_SPI_BUS_1,
    COINES_SPI_BUS_MAX
};

enum coines_status {
    COINES_SUCCESS,
    COINES_E_COMM_IO_ERROR,
    COINES_E_FAILURE,
    COINES_E_I2C_BUS_NOT_ENABLED,
    COINES_E_I2C_INVALID_BUS_INTF
};

static i2c_inst_t *coines_i2c_instance[COINES_I2C_BUS_MAX] = { i2c0, i2c1 };
static spi_inst_t *coines_spi_instance[COINES_SPI_BUS_MAX] = { spi0, spi1 };

bool coines_is_i2c_enabled(enum coines_i2c_bus bus) {
    // Implement this function to check if the I2C bus is enabled
    return true;
}

bool coines_is_spi_enabled(enum coines_spi_bus bus) {
    // Implement this function to check if the SPI bus is enabled
    return true;
}

void coines_i2c_bus_recover(enum coines_i2c_bus bus) {
    // Implement this function to recover the I2C bus if needed
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 */
int8_t coines_i2c_get(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count) {
    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0)) {
        if (coines_is_i2c_enabled(bus)) {
            absolute_time_t timeout = make_timeout_time_ms(I2C_TIMEOUT_MS);

            int result = i2c_read_blocking_until(coines_i2c_instance[bus], dev_addr, data, count, false, timeout);
            if (result == PICO_ERROR_GENERIC) {
                coines_i2c_bus_recover(bus);
                return COINES_E_COMM_IO_ERROR;
            }

            return COINES_SUCCESS;
        } else {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    } else {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}


enum coines_i2c_bus {
    COINES_I2C_BUS_0,
    COINES_I2C_BUS_1,
    COINES_I2C_BUS_MAX
};

enum coines_spi_bus {
    COINES_SPI_BUS_0,
    COINES_SPI_BUS_1,
    COINES_SPI_BUS_MAX
};

enum coines_status {
    COINES_SUCCESS,
    COINES_E_COMM_IO_ERROR,
    COINES_E_FAILURE,
    COINES_E_I2C_BUS_NOT_ENABLED,
    COINES_E_I2C_INVALID_BUS_INTF
};

static i2c_inst_t *coines_i2c_instance[COINES_I2C_BUS_MAX] = { i2c0, i2c1 };
static spi_inst_t *coines_spi_instance[COINES_SPI_BUS_MAX] = { spi0, spi1 };

bool coines_is_i2c_enabled(enum coines_i2c_bus bus) {
    // Implement this function to check if the I2C bus is enabled
    return true;
}

bool coines_is_spi_enabled(enum coines_spi_bus bus) {
    // Implement this function to check if the SPI bus is enabled
    return true;
}

uint32_t coines_get_millis() {
    return to_ms_since_boot(get_absolute_time());
}

void coines_i2c_bus_recover(enum coines_i2c_bus bus) {
    uint scl_pin = (bus == COINES_I2C_BUS_0) ? I2C_SCL_PIN : I2C_SCL_PIN; // Adjust for your bus
    uint sda_pin = (bus == COINES_I2C_BUS_0) ? I2C_SDA_PIN : I2C_SDA_PIN; // Adjust for your bus

    // Set SCL and SDA as GPIO outputs
    gpio_init(scl_pin);
    gpio_set_dir(scl_pin, GPIO_OUT);
    gpio_init(sda_pin);
    gpio_set_dir(sda_pin, GPIO_OUT);

    // Generate clock pulses on SCL to release any stuck devices
    for (int i = 0; i < 9; i++) {
        gpio_put(scl_pin, 0);
        sleep_us(5);
        gpio_put(scl_pin, 1);
        sleep_us(5);
    }

    // Generate a STOP condition on the bus
    gpio_put(sda_pin, 0);
    sleep_us(5);
    gpio_put(scl_pin, 1);
    sleep_us(5);
    gpio_put(sda_pin, 1);
    sleep_us(5);

    // Reinitialize the I2C bus
    i2c_init(coines_i2c_instance[bus], 100 * 1000); // 100 kHz
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 */
int8_t coines_i2c_get(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count) {
    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0)) {
        if (coines_is_i2c_enabled(bus)) {
            absolute_time_t timeout = make_timeout_time_ms(I2C_TIMEOUT_MS);

            int result = i2c_read_blocking_until(coines_i2c_instance[bus], dev_addr, data, count, false, timeout);
            if (result == PICO_ERROR_GENERIC) {
                coines_i2c_bus_recover(bus);
                return COINES_E_COMM_IO_ERROR;
            }

            return COINES_SUCCESS;
        } else {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    } else {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}

/*!
 *  @brief This API is used to write the data in SPI communication.
 */
int8_t coines_write_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) {
    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0)) {
        if (coines_is_spi_enabled(bus)) {
            uint32_t pin_no = multi_io_map[dev_addr];
            if (pin_no == 0 || pin_no == 0xff) {
                return COINES_E_FAILURE;
            } else {
                gpio_init(pin_no);
                gpio_set_dir(pin_no, GPIO_OUT);
            }

            /* Activate CS pin */
            gpio_put(pin_no, 0);

            /* Write register address */
            absolute_time_t timeout = make_timeout_time_ms(SPI_TIMEOUT_MS);
            int result = spi_write_blocking_until(coines_spi_instance[bus], &reg_addr, 1, timeout);
            if (result == PICO_ERROR_GENERIC) {
                gpio_put(pin_no, 1);
                return COINES_E_COMM_IO_ERROR;
            }

            /* Write data */
            result = spi_write_blocking_until(coines_spi_instance[bus], reg_data, count, timeout);
            if (result == PICO_ERROR_GENERIC) {
                gpio_put(pin_no, 1);
                return COINES_E_COMM_IO_ERROR;
            }

            /* Deactivate CS pin */
            gpio_put(pin_no, 1);

            return COINES_SUCCESS;
        } else {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    } else {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}

#include "hardware/spi.h"
#include "pico/stdlib.h"

#define COINES_SPI_BUS_MAX 2
#define SPI_TIMEOUT_MS 1000

enum coines_spi_bus {
    COINES_SPI_BUS_0,
    COINES_SPI_BUS_1,
    COINES_SPI_BUS_MAX
};

enum coines_status {
    COINES_SUCCESS,
    COINES_E_COMM_IO_ERROR,
    COINES_E_FAILURE,
    COINES_E_I2C_BUS_NOT_ENABLED,
    COINES_E_I2C_INVALID_BUS_INTF
};

static spi_inst_t *coines_spi_instance[COINES_SPI_BUS_MAX] = { spi0, spi1 };

bool coines_is_spi_enabled(enum coines_spi_bus bus) {
    // Implement this function to check if the SPI bus is enabled
    return true;
}

uint32_t multi_io_map[256]; // Placeholder for the multi_io_map array

/*!
 *  @brief This API is used to write the data in SPI communication.
 */
int8_t coines_write_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) {
    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0)) {
        if (coines_is_spi_enabled(bus)) {
            uint32_t pin_no = multi_io_map[dev_addr];
            if (pin_no == 0 || pin_no == 0xff) {
                return COINES_E_FAILURE;
            } else {
                gpio_init(pin_no);
                gpio_set_dir(pin_no, GPIO_OUT);
            }

            /* Activate CS pin */
            gpio_put(pin_no, 0);

            /* Write register address */
            absolute_time_t timeout = make_timeout_time_ms(SPI_TIMEOUT_MS);
            int result = spi_write_blocking_until(coines_spi_instance[bus], &reg_addr, 1, timeout);
            if (result == PICO_ERROR_GENERIC) {
                gpio_put(pin_no, 1);
                return COINES_E_COMM_IO_ERROR;
            }

            /* Write data */
            result = spi_write_blocking_until(coines_spi_instance[bus], reg_data, count, timeout);
            if (result == PICO_ERROR_GENERIC) {
                gpio_put(pin_no, 1);
                return COINES_E_COMM_IO_ERROR;
            }

            /* Deactivate CS pin */
            gpio_put(pin_no, 1);

            return COINES_SUCCESS;
        } else {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    } else {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 */
int8_t coines_read_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) {
    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0)) {
        if (coines_is_spi_enabled(bus)) {
            uint32_t pin_no = multi_io_map[dev_addr];
            if (pin_no == 0 || pin_no == 0xff) {
                return COINES_E_FAILURE;
            } else {
                gpio_init(pin_no);
                gpio_set_dir(pin_no, GPIO_OUT);
            }

            /* Activate CS pin */
            gpio_put(pin_no, 0);

            /* Write register address */
            absolute_time_t timeout = make_timeout_time_ms(SPI_TIMEOUT_MS);
            int result = spi_write_blocking_until(coines_spi_instance[bus], &reg_addr, 1, timeout);
            if (result == PICO_ERROR_GENERIC) {
                gpio_put(pin_no, 1);
                return COINES_E_COMM_IO_ERROR;
            }

            /* Read data */
            result = spi_read_blocking_until(coines_spi_instance[bus], 0, reg_data, count, timeout);
            if (result == PICO_ERROR_GENERIC) {
                gpio_put(pin_no, 1);
                return COINES_E_COMM_IO_ERROR;
            }

            /* Deactivate CS pin */
            gpio_put(pin_no, 1);

            return COINES_SUCCESS;
        } else {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    } else {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}