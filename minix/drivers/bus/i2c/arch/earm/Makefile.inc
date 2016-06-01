# Makefile for arch-dependent i2c code
.include <bsd.own.mk>

HERE=${.CURDIR}/arch/${MACHINE_ARCH}
.PATH:  ${HERE}

SRCS += bcm2835_i2c.c bcm2835_i2c.h bcm2835_i2c_registers.h
SRCS += omap_i2c.c omap_i2c.h omap_i2c_registers.h
