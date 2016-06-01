/* kernel headers */
#include <minix/chardriver.h>
#include <minix/clkconf.h>
#include <minix/drivers.h>
#include <minix/ds.h>
#include <minix/log.h>
#include <minix/mmio.h>
#include <minix/padconf.h>
#include <minix/sysutil.h>
#include <minix/type.h>
#include <minix/board.h>
#include <minix/spin.h>

/* device headers */
#include <minix/i2c.h>

/* system headers */
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>

/* usr headers */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* local headers */
#include "bcm2835_i2c.h"


/*
 * defines the set of register
 */
typedef struct bcm2835_i2c_registers
{
	vir_bytes I2C_DATA;
	vir_bytes I2C_CON;
	vir_bytes I2C_STAT;
	vir_bytes I2C_DLEN;
	vir_bytes I2C_SLA;
} bcm2835_i2c_regs_t;

/* Define the registers for the chip */

static bcm2835_i2c_regs_t bcm2835_i2c_regs = {
	.I2C_CON = BCM2835_I2C_CONTROL,
	.I2C_DATA = BCM2835_I2C_FIFO,
	.I2C_STAT = BCM2835_I2C_STATUS,
	.I2C_DLEN = BCM2835_I2C_DLEN,
	.I2C_SLA = BCM2835_I2C_SLADR
};

/* generic definition an i2c bus */
typedef struct bcm2835_i2c_bus
{
	enum bus_types
	{ AM335X_I2C_BUS, DM37XX_I2C_BUS, BCM2835_I2C_BUS} bus_type;
	phys_bytes mr_base;
	phys_bytes mr_size;
	vir_bytes mapped_addr;
	bcm2835_i2c_regs_t *regs;
} bcm2835_i2c_bus_t;

/* Define the buses available on the chip */

static bcm2835_i2c_bus_t bcm2835_i2c_buses_array[] = {
	{BCM2835_I2C_BUS, BCM2835_BSC0_BASE, BCM2835_I2C0_SIZE, 0, &bcm2835_i2c_regs},
	{BCM2835_I2C_BUS, BCM2835_BSC1_BASE, BCM2835_I2C1_SIZE, 0, &bcm2835_i2c_regs},
	{BCM2835_I2C_BUS, BCM2835_BSC2_BASE, BCM2835_I2C2_SIZE, 0, &bcm2835_i2c_regs}
};

#define BCM2835_NBUSES (sizeof(bcm2835_i2c_buses_array) / sizeof(bcm2835_i2c_bus_t))

/* Globals */
static bcm2835_i2c_bus_t *bcm2835_i2c_buses;	/* all available buses for this SoC */
static bcm2835_i2c_bus_t *bcm2835_i2c_bus;	/* the bus selected at start-up */
static int bcm2835_i2c_nbuses;	/* number of buses supported by SoC */

/* logging - use with log_warn(), log_info(), log_debug(), log_trace() */
static struct log log = {
	.name = "i2c",
	.log_level = LEVEL_INFO,
	.log_func = default_log
};

/* Local Function Prototypes */

/* Implementation of Generic I2C Interface using Bus Specific Code */
static int bcm2835_i2c_process(minix_i2c_ioctl_exec_t * m);

/* Bus Specific Code */
static void bcm2835_i2c_flush(void);
static uint16_t bcm2835_i2c_poll(uint16_t mask);
static int bcm2835_i2c_bus_is_free(void);
static int bcm2835_i2c_soft_reset(void);
static void bcm2835_i2c_bus_init(void);
static void bcm2835_i2c_padconf(int i2c_bus_id);
static void bcm2835_i2c_clkconf(int i2c_bus_id);
static void bcm2835_i2c_intr_enable(void);
static uint16_t bcm2835_i2c_read_status(void);
static void bcm2835_i2c_write_status(uint16_t mask);
static int bcm2835_i2c_read(i2c_addr_t addr, uint8_t * buf, size_t buflen,
    int dostop);
static int bcm2835_i2c_write(i2c_addr_t addr, const uint8_t * buf, size_t buflen,
    int dostop);

/*
 * Performs the action in minix_i2c_ioctl_exec_t.
 */
static int
bcm2835_i2c_process(minix_i2c_ioctl_exec_t * ioctl_exec)
{
	int r;

	/*
	 * Zero data bytes transfers are not allowed. The controller treats
	 * I2C_CNT register value of 0x0 as 65536. This is true for both the
	 * am335x and dm37xx. Full details in the TRM on the I2C_CNT page.
	 */
	if (ioctl_exec->iie_buflen == 0) {
		return EINVAL;
	}

	bcm2835_i2c_flush();	/* clear any garbage in the fifo */

	/* Check bus busy flag before using the bus */
	r = bcm2835_i2c_bus_is_free();
	if (r == 0) {
		log_warn(&log, "Bus is busy\n");
		return EBUSY;
	}

	if (ioctl_exec->iie_cmdlen > 0) {
		r = bcm2835_i2c_write(ioctl_exec->iie_addr, ioctl_exec->iie_cmd,
		    ioctl_exec->iie_cmdlen,
		    !(I2C_OP_READ_P(ioctl_exec->iie_op)));
		if (r != OK) {
			bcm2835_i2c_soft_reset();
			bcm2835_i2c_bus_init();
			return r;
		}
	}

	if (I2C_OP_READ_P(ioctl_exec->iie_op)) {
		r = bcm2835_i2c_read(ioctl_exec->iie_addr, ioctl_exec->iie_buf,
		    ioctl_exec->iie_buflen, I2C_OP_STOP_P(ioctl_exec->iie_op));
	} else {
		r = bcm2835_i2c_write(ioctl_exec->iie_addr, ioctl_exec->iie_buf,
		    ioctl_exec->iie_buflen, I2C_OP_STOP_P(ioctl_exec->iie_op));
	}

	if (r != OK) {
		bcm2835_i2c_soft_reset();
		bcm2835_i2c_bus_init();
		return r;
	}

	return OK;
}

/*
 * Drain the incoming FIFO.
 *
 * Usually called to clear any garbage that may be in the buffer before
 * doing a read.
 */
static void
bcm2835_i2c_flush(void)
{
	int tries;
	int status;

	for (tries = 0; tries < 1000; tries++) {
		status = bcm2835_i2c_poll(1 << RXD);
		if ((status & (1 << RXD)) != 0) {	/* bytes available for reading */

			/* consume the byte and throw it away */
			(void) read16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_DATA);

			/* clear the read ready flag */
			//bcm2835_i2c_write_status(1 << RXD); No need on bcm

		} else {
			break;	/* buffer drained */
		}
	}
}

/*
 * Poll the status register checking the bits set in 'mask'.
 * Returns the status if any bits set or 0x0000 when the timeout is reached.
 */
static uint16_t
bcm2835_i2c_poll(uint16_t mask)
{
	spin_t spin;
	uint16_t status;

	/* poll for up to 1 s */
	spin_init(&spin, 1000000);
	do {
		status = bcm2835_i2c_read_status();
		if ((status & mask) != 0) {	/* any bits in mask set */
			return status;
		}

	} while (spin_check(&spin));

	return status;		/* timeout reached, abort */
}

/*
 * Poll Bus Busy Flag until the bus becomes free (return 1) or the timeout
 * expires (return 0).
 */
static int
bcm2835_i2c_bus_is_free(void)
{
	spin_t spin;
	uint16_t status;

	/* wait for up to 1 second for the bus to become free */
	spin_init(&spin, 1000000);
	do {

		status = bcm2835_i2c_read_status();
		if ((status & (1 << TA)) == 0) {
			return 1;	/* bus is free */
		}

	} while (spin_check(&spin));

	return 0;		/* timeout expired */
}

static void
bcm2835_i2c_clkconf(int i2c_bus_id)
{
	clkconf_init();

	if (bcm2835_i2c_bus->bus_type == BCM2835_I2C_BUS) {
		// No clkconf?
	}

	clkconf_release();
}

static void
bcm2835_i2c_padconf(int i2c_bus_id)
{
	// No padconf -> overlay
}

static int
bcm2835_i2c_soft_reset(void)
{
	spin_t spin;
	printf("soft_reset: enter soft reset\n");

	/* Disable to do soft reset */
	printf("soft_reset: disable i2c\n");
	write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_CON, 0);
	micro_delay(50000);

	/* Enable to do soft reset */
	printf("soft_reset: enable i2c\n");
	set16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_CON, (1<<I2C_EN), (1<<I2C_EN));
	micro_delay(50000);

	/* wait up to 3 seconds for reset to complete */
	printf("soft_reset: spin\n");
	spin_init(&spin, 3000000);
	do {
		if (read16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_CON) & (1 << I2C_EN)) {
			printf("soft_reset: finished\n");
			return OK;
		}

	} while (spin_check(&spin));

	log_warn(&log, "Tried soft reset, but bus never came back.\n");
	return EIO;
}

static void
bcm2835_i2c_intr_enable(void)
{
	// No intr
}

static void
bcm2835_i2c_bus_init(void)
{
	printf("bus_init: enter bus init\n");
	printf("bus_init: disable i2c\n");
	/* Ensure i2c module is disabled before setting prescalar & bus speed */
	write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_CON, 0);
	micro_delay(50000);
	printf("bus_init: enable i2c\n");
	/* Bring the i2c module out of reset */
	set16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_CON, (1<<I2C_EN), (1<<I2C_EN));
	micro_delay(50000);
	printf("bus_init: finished\n");
}

static uint16_t
bcm2835_i2c_read_status(void)
{
	uint16_t status = 0x0000;

	if (bcm2835_i2c_bus->bus_type == BCM2835_I2C_BUS) {
		status = read16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_STAT);
	} else {
		log_warn(&log, "Don't know how to read i2c bus status.\n");
	}

	return status;
}

static void
bcm2835_i2c_write_status(uint16_t mask)
{
	if (bcm2835_i2c_bus->bus_type == BCM2835_I2C_BUS) {
		write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_STAT, mask);
	} else {
		log_warn(&log, "Don't know how to clear i2c bus status.\n");
	}
}

static int
bcm2835_i2c_read(i2c_addr_t addr, uint8_t * buf, size_t buflen, int dostop)
{
	int i;
	uint16_t data_left = buflen;
	uint16_t status;

	/* Set address of slave device */
	addr &= 0x7f;	/* sanitize address (7-bit max) */
	write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_SLA, addr);

	// Set data length in DLEN register
	write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_DLEN, buflen);

	uint16_t control = read16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_CON);
	// Mask for clearing READ bit (= Write)
	control = control | (0x1);
	// Mask for setting START bit
	control = control | (0x80);
	set16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_CON, 0x7f, control);

	for (i = 0; i < buflen; i++) {

		// Wait for data in FIFO
		status = bcm2835_i2c_poll((1 << RXD));

		// Error flag
		if (status & (1 << ERR)) {
			// Clear all flags: ERR, CLKT, DONE
			status = status | (0x302);
			bcm2835_i2c_write_status(status);
			return -2;
		}

		// Timeout flag
		if (status & (1 << CLKT)) {
			// Clear CLKT and DONE flags
			status = status | (0x202);
			bcm2835_i2c_write_status(status);
			return -1;
		}

		// Check for data
		if (status & (1 << RXD)) {
			// Data length update
			data_left = read16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_DLEN);
			data_left --;
			write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_DLEN, data_left);
			// Data read
			buf[i] = read16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_DATA) & 0xff;
		} else {
			// Clear all flags: ERR, CLKT, DONE
			status = status | (0x302);
			bcm2835_i2c_write_status(status);
			// Timeout
			return -1;
		}
	}

	// Wait for DONE
	status = bcm2835_i2c_poll((1 << DONE));
	if (status & (1 << DONE)) {
		// Clear DONE flags
		status = status | (0x2);
		bcm2835_i2c_write_status(status);
		return 0;
	}
	return -1;
}

// Return: error status -2 NACK, -1 TIMEOUT, 0 NORMAL RETURN
static int
bcm2835_i2c_write(i2c_addr_t addr, const uint8_t * buf, size_t buflen, int dostop)
{
	int i;
	uint16_t data_left = buflen;
	uint16_t status;

	/* Set address of slave device */
	addr &= 0x7f;	/* sanitize address (7-bit max) */
	write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_SLA, addr);

	// Set data length in DLEN register
	write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_DLEN, buflen);

	uint16_t control = read16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_CON);
	// Mask for clearing READ bit (= Write)
	control = control & (0xfffe);
	// Mask for setting CLEAR bits and START bit
	control = control | (0xB0);
	set16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_CON, 0x7f, control);

	for (i = 0; i < buflen; i++) {

		// Wait for empty spot in FIFO
		status = bcm2835_i2c_poll((1 << TXD));

		// Error flag
		if (status & (1 << ERR)) {
			// Clear all flags: ERR, CLKT, DONE
			status = status | (0x302);
			bcm2835_i2c_write_status(status);
			return -2;
		}

		// Timeout flag
		if (status & (1 << CLKT)) {
			// Clear CLKT and DONE flags
			status = status | (0x202);
			bcm2835_i2c_write_status(status);
			return -1;
		}

		// Check for empty spot
		if (status & (1 << TXD)) {
			// Data length update
			data_left = read16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_DLEN);
			data_left --;
			write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_DLEN, data_left);
			// Data write
			write16(bcm2835_i2c_bus->mapped_addr + bcm2835_i2c_bus->regs->I2C_DATA, buf[i]);	
		} else {
			// Clear all flags: ERR, CLKT, DONE
			status = status | (0x302);
			bcm2835_i2c_write_status(status);
			// Timeout
			return -1;
		}
	}

	// Wait for DONE
	status = bcm2835_i2c_poll((1 << DONE));
	if (status & (1 << DONE)) {
		// Clear DONE flags
		status = status | (0x2);
		bcm2835_i2c_write_status(status);
		return 0;
	}
	return -1;
}

int
bcm2835_interface_setup(int (**process) (minix_i2c_ioctl_exec_t * ioctl_exec),
    int i2c_bus_id)
{
	printf("interface_setup: enter\n");
	int r;
	int i2c_rev, major, minor;
	struct minix_mem_range mr;
	struct machine machine;
	printf("interface_setup: calling sys_getmachine\n");
	sys_getmachine(&machine);
	printf("interface_setup: returned from sys_getmachine\n");

	/* Fill in the function pointer */

	*process = bcm2835_i2c_process;

	/* Select the correct i2c definition for this SoC */
	printf("interface_setup: select correct i2c definition\n");
	if (BOARD_IS_RPI_2_B(machine.board_id) || BOARD_IS_RPI_3_B(machine.board_id)){
		bcm2835_i2c_buses = bcm2835_i2c_buses_array;
		bcm2835_i2c_nbuses = BCM2835_NBUSES;
	} else {
		printf("interface_setup: return einval 1\n");
		return EINVAL;
	}
	printf("i2c_bus_id: %d \n", i2c_bus_id);
	if (i2c_bus_id < 0 || i2c_bus_id >= bcm2835_i2c_nbuses) {
		printf("interface_setup: return einval 2\n");
		return EINVAL;
	}

	/* select the bus to operate on */
	bcm2835_i2c_bus = &bcm2835_i2c_buses[i2c_bus_id];

	/* Configure Pins */
	bcm2835_i2c_padconf(i2c_bus_id);

	/*
	 * Map I2C Registers
	 */

	/* Configure memory access */
	mr.mr_base = bcm2835_i2c_bus->mr_base;	/* start addr */
	mr.mr_limit = mr.mr_base + bcm2835_i2c_bus->mr_size;	/* end addr */

	printf("interface_setup: ask for privileges\n");
	/* ask for privileges to access the I2C memory range */
	if (sys_privctl(SELF, SYS_PRIV_ADD_MEM, &mr) != OK) {
		panic("Unable to obtain i2c memory range privileges");
	}

	/* map the memory into this process */
	printf("interface_setup: mapping memory\n");
	bcm2835_i2c_bus->mapped_addr = (vir_bytes) vm_map_phys(SELF,
	    (void *) bcm2835_i2c_bus->mr_base, bcm2835_i2c_bus->mr_size);

	if (bcm2835_i2c_bus->mapped_addr == (vir_bytes) MAP_FAILED) {
		panic("Unable to map i2c registers");
	}

	/* Enable Clocks */
	bcm2835_i2c_clkconf(i2c_bus_id);

	printf("interface_setup: do a soft reset\n");
	/* Perform a soft reset of the I2C module to ensure a fresh start */
	r = bcm2835_i2c_soft_reset();
	if (r != OK) {
		/* module didn't come back up :( */
		return r;
	}

	printf("interface_setup: init the bus\n");
	/* Bring up I2C module */
	bcm2835_i2c_bus_init();
	printf("interface_setup: finished\n");
	return OK;
}
