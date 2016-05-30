/* Implements sys_padconf() for the Raspberry Pi. */

#include "kernel/kernel.h"
#include "arch_proto.h"
#include <sys/types.h>
#include <machine/cpu.h>
#include <minix/mmio.h>
#include <minix/padconf.h>
#include <minix/board.h>
#include <minix/com.h>
#include <assert.h>
#include <io.h>
#include <stdlib.h>
#include <stdio.h>

#include "bsp_padconf.h"

struct rpi3_padconf
{
	vir_bytes base;
	vir_bytes offset;
	vir_bytes size;
};

static struct rpi3_padconf rpi3_padconf;

static kern_phys_map padconf_phys_map;

int
bsp_padconf_set(u32_t padconf, u32_t mask, u32_t value)
{
	/* check that the value will be inside the padconf memory range */
	if (padconf >= (rpi3_padconf.size - rpi3_padconf.offset)) {
		return EINVAL;	/* outside of valid range */
	}

	set32(padconf + rpi3_padconf.base + rpi3_padconf.offset, mask,
	    value);

	return OK;
}

void
bsp_padconf_init(void)
{
	rpi3_padconf.base = PADCONF_RPI3_REGISTERS_BASE;
	rpi3_padconf.offset = PADCONF_RPI3_REGISTERS_OFFSET;
	rpi3_padconf.size = PADCONF_RPI3_REGISTERS_SIZE;

	kern_phys_map_ptr(rpi3_padconf.base, rpi3_padconf.size,
	    VMMF_UNCACHED | VMMF_WRITE,
	    &padconf_phys_map, (vir_bytes) & rpi3_padconf.base);

	return;
}
