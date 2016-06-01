#ifndef _BCM2835_I2C_H
#define _BCM2835_I2C_H

#include <minix/chardriver.h>
#include <minix/i2c.h>
#include "bcm2835_i2c_registers.h"

int bcm2835_interface_setup(int (**process)(minix_i2c_ioctl_exec_t *ioctl_exec), int i2c_bus_id);

#endif /* _BCM2835_I2C_H */
