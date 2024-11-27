#ifndef FIRMWARE_I2C_TOOLS_H
#define FIRMWARE_I2C_TOOLS_H

#include <stdio.h>
#include <stdint.h> // uint8_t, uint16_t, uint32_t, uint64_t
#include <linux/i2c-dev.h>  // I2C_SLAVE
#include <fcntl.h> // open(), O_RDONLY
#include <unistd.h> // read(), write(), usleep()
#include <sys/ioctl.h> // ioctl()

/* Descriptor for i2c device, must be initialised using open_i2c_device() */
typedef int i2c_descripter;

/* Open the i2c device file */
extern i2c_descripter open_i2c_device ( int i2c_bus_number , int i2c_device_number );

/* Write data to the i2c device at the specific register */
extern void i2c_write_register ( i2c_descripter file_descriptor , uint8_t register_to_write_to , uint8_t data_to_write_to_register );

/* Read data from the i2c device at the specific register */
extern uint8_t i2c_read_register ( i2c_descripter file_descriptor , uint8_t register_to_read_from );


#endif

