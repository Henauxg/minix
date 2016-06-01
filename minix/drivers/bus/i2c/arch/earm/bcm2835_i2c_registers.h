#ifndef _BCM2835_I2C_REGISTERS_H
#define _BCM2835_I2C_REGISTERS_H

/* Base Addresses */
#define BCM2835_BSC0_BASE 0x3f205000
#define BCM2835_BSC1_BASE 0x3f804000
#define BCM2835_BSC2_BASE 0x3f805000

/* Register Offsets */
#define BCM2835_I2C_CONTROL 	0x0
#define BCM2835_I2C_STATUS 		0x4
#define BCM2835_I2C_DLEN		0x8
#define BCM2835_I2C_SLADR		0xc
#define BCM2835_I2C_FIFO		0x10
#define BCM2835_I2C_CLKDIV		0x14
#define BCM2835_I2C_DEL			0x18
#define BCM2835_I2C_CLKT		0x1c

/* Size of I2C Register Address Range */
#define BCM2835_I2C0_SIZE 0x1000
#define BCM2835_I2C1_SIZE 0x1000
#define BCM2835_I2C2_SIZE 0x1000

#define I2C_EN 15 /* I2C_CONTROL */

#define CLKT	9 /* I2C_STAT */
#define ERR		8 /* I2C_STAT */
#define RXD 	5 /* I2C_STAT */
#define TXD 	4 /* I2C_STAT */
#define DONE	1 /* I2C_STAT */
#define TA 		0 /* I2C_STAT */

#endif /* _BCM2835_I2C_REGISTERS_H */

