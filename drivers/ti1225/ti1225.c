/*
ti1225.c

Created:	Dec 2005 by Philip Homburg
*/

#include <minix/drivers.h>
#include <minix/driver.h>
#include <machine/pci.h>
#include <machine/vm.h>
#include <sys/mman.h>

#include "ti1225.h"
#include "i82365.h"

/* The use of interrupts is not yet ready for prime time */
#define USE_INTS	0

#define NR_PORTS 2

PRIVATE struct port
{
	unsigned p_flags;
	int p_devind;
	u8_t p_cb_busnr;
	u16_t p_exca_port;
#if USE_INTS
	int p_irq;
	int p_hook;
#endif
	volatile struct csr *csr_ptr;
} ports[NR_PORTS];

#define PF_PRESENT	1

struct pcitab
{
	u16_t vid;
	u16_t did;
	int checkclass;
};

PRIVATE struct pcitab pcitab_ti[]=
{
	{ 0x104C, 0xAC1C, 0 },		/* TI PCI1225 */

	{ 0x0000, 0x0000, 0 }
};
PRIVATE int debug;

FORWARD _PROTOTYPE( void hw_init, (struct port *pp)			);
FORWARD _PROTOTYPE( void do_int, (struct port *pp)			);

/* SEF functions and variables. */
FORWARD _PROTOTYPE( void sef_local_startup, (void) );
FORWARD _PROTOTYPE( int sef_cb_init_fresh, (int type, sef_init_info_t *info) );

/*===========================================================================*
 *				main					     *
 *===========================================================================*/
int main(int argc, char *argv[])
{
	int r;
	message m;
	int ipc_status;

	/* SEF local startup. */
	env_setargs(argc, argv);
	sef_local_startup();

	for (;;)
	{
		r= driver_receive(ANY, &m, &ipc_status);
		if (r != OK)
			panic("driver_receive failed: %d", r);
		printf("ti1225: got message %u from %d\n",
			m.m_type, m.m_source);
	}
	return 0;
}

/*===========================================================================*
 *			       sef_local_startup			     *
 *===========================================================================*/
PRIVATE void sef_local_startup()
{
  /* Register init callbacks. */
  sef_setcb_init_fresh(sef_cb_init_fresh);
  sef_setcb_init_lu(sef_cb_init_fresh);
  sef_setcb_init_restart(sef_cb_init_fresh);

  /* Register live update callbacks. */
  sef_setcb_lu_prepare(sef_cb_lu_prepare_always_ready);
  sef_setcb_lu_state_isvalid(sef_cb_lu_state_isvalid_standard);

  /* Let SEF perform startup. */
  sef_startup();
}

/*===========================================================================*
 *		            sef_cb_init_fresh                                *
 *===========================================================================*/
PRIVATE int sef_cb_init_fresh(int UNUSED(type), sef_init_info_t *UNUSED(info))
{
/* Initialize the ti1225 driver. */
	int c, i, r, first, devind, port;
	u16_t vid, did;

	if((r=tsc_calibrate()) != OK)
		panic("tsc_calibrate failed: %d", r);

	debug= 0;
	while (c= getopt(env_argc, env_argv, "d?"), c != -1)
	{
		switch(c)
		{
		case '?': panic("Usage: ti1225 [-d]");
		case 'd': debug++; break;
		default: panic("getopt failed");
		}
	}

	pci_init();

	first= 1;
	port= 0;
	for (;;)
	{
		if (first)
		{
			first= 0;
			r= pci_first_dev(&devind, &vid, &did);
		}
		else
			r= pci_next_dev(&devind, &vid, &did);
		if (r != 1)
			break;

		for (i= 0; pcitab_ti[i].vid != 0; i++)
		{
			if (pcitab_ti[i].vid != vid)
				continue;
			if (pcitab_ti[i].did != did)
				continue;
			if (pcitab_ti[i].checkclass) {
				panic("fxp_probe: class check not implemented");
			}
			break;
		}
		if (pcitab_ti[i].vid == 0)
			continue;

		pci_reserve(devind);

		if (debug)
			printf("ti1225: found device %04x/%04x\n", vid, did);
		ports[port].p_devind= devind;
		ports[port].p_flags |= PF_PRESENT;
		port++;
		if (port >= NR_PORTS)
			break;
	}

	for (i= 0; i<NR_PORTS; i++)
	{
		if (!(ports[i].p_flags & PF_PRESENT))
			continue;
		hw_init(&ports[i]);
	}

	return(OK);
}

PRIVATE void hw_init(struct port *pp)
{
	int devind;
	u8_t v8;
	u16_t v16;
	u32_t v32;
#if USE_INTS
	int r, irq;
#endif

	devind= pp->p_devind;
	if (debug)
		printf("hw_init: devind = %d\n", devind);

	if (debug)
	{
		v16= pci_attr_r16(devind, PCI_CR);
		printf("ti1225: command register 0x%x\n", v16);
	}

	v32= pci_attr_r32(devind, TI_CB_BASEADDR);
	if (debug)
		printf("ti1225: Cardbus/ExCA base address 0x%x\n", v32);
	v32 &= ~(u32_t)0xF;	/* Clear low order bits in base */

	pp->csr_ptr=
		(struct csr *) vm_map_phys(SELF, (void *) v32, I386_PAGE_SIZE);
	if (pp->csr_ptr == MAP_FAILED)
		panic("hw_init: vm_map_phys failed");

	if (debug)
	{
		v8= pci_attr_r8(devind, TI_PCI_BUS_NR);
		printf("ti1225: PCI bus number %d\n", v8);
	}
	v8= pci_attr_r8(devind, TI_CB_BUS_NR);
	pp->p_cb_busnr= v8;
	if (debug)
	{
		printf("ti1225: CardBus bus number %d\n", v8);
		v8= pci_attr_r8(devind, TI_SO_BUS_NR);
		printf("ti1225: Subordinate bus number %d\n", v8);
	}

#if USE_INTS
	irq= pci_attr_r8(devind, PCI_ILR);
	pp->p_irq= irq;
	printf("ti1225 using IRQ %d\n", irq);
#endif

	v32= pci_attr_r32(devind, TI_LEGACY_BA);
	v32 &= ~1;
	if (debug)
	{
		printf("ti1225: PC Card 16-bit legacy-mode base address 0x%x\n",
			v32);
	}

	if (v32 == 0)
		panic("bad legacy-mode base address: %d", v32);
	pp->p_exca_port= v32;

	if (debug)
	{
		v32= pci_attr_r32(devind, TI_MF_ROUTE);
		printf("ti1225: Multifunction routing 0x%08x\n", v32);
	}

#if USE_INTS
 	pp->p_hook = pp->p_irq;
	r= sys_irqsetpolicy(pp->p_irq, 0, &pp->p_hook);
	if (r != OK)
		panic("sys_irqsetpolicy failed: %d", r);
#endif

	/* Clear CBB_BC_INTEXCA */
	v16= pci_attr_r16(devind, CBB_BRIDGECTRL);
	if (debug)
		printf("ti1225: Bridge control 0x%04x\n", v16);
	v16 &= ~CBB_BC_INTEXCA;
	pci_attr_w16(devind, CBB_BRIDGECTRL, v16);

	if (debug)
	{
		v32= pci_attr_r32(devind, TI_SYSCTRL);
		printf("ti1225: System Control Register 0x%08x\n", v32);

		v8= pci_attr_r8(devind, TI_CARD_CTRL);
		printf("ti1225: Card Control 0x%02x\n", v8);

		v8= pci_attr_r8(devind, TI_DEV_CTRL);
		printf("ti1225: Device Control 0x%02x\n", v8);
	}

	/* Enable socket interrupts */
	pp->csr_ptr->csr_mask |= CM_PWRMASK | CM_CDMASK | CM_CSTSMASK;

	do_int(pp);

#if USE_INTS
	r= sys_irqenable(&pp->p_hook);
	if (r != OK)
		panic("unable enable interrupts: %d", r);
#endif
}

PRIVATE void do_int(struct port *pp)
{
	int devind, vcc_5v, vcc_3v, vcc_Xv, vcc_Yv,
		socket_5v, socket_3v, socket_Xv, socket_Yv;
	spin_t spin;
	u32_t csr_event, csr_present, csr_control;
	u8_t v8;
	u16_t v16;
#if USE_INTS
	int r;
#endif

	devind= pp->p_devind;
	v8= pci_attr_r8(devind, TI_CARD_CTRL);
	if (v8 & TI_CCR_IFG)
	{
		printf("ti1225: got functional interrupt\n");
		pci_attr_w8(devind, TI_CARD_CTRL, v8);
	}

	if (debug)
	{
		printf("Socket event: 0x%x\n", pp->csr_ptr->csr_event);
		printf("Socket mask: 0x%x\n", pp->csr_ptr->csr_mask);
	}

	csr_present= pp->csr_ptr->csr_present;
	csr_control= pp->csr_ptr->csr_control;

	if ((csr_present & (CP_CDETECT1|CP_CDETECT2)) != 0)
	{
		if (debug)
			printf("do_int: no card present\n");
		return;
	}
	if (csr_present & CP_BADVCCREQ)
	{
		printf("do_int: Bad Vcc request\n");
		/* return; */
	}
	if (csr_present & CP_DATALOST)
	{
		/* Do we care? */
		if (debug)
			printf("do_int: Data lost\n");
		/* return; */
	}
	if (csr_present & CP_NOTACARD)
	{
		printf("do_int: Not a card\n");
		return;
	}
	if (debug)
	{
		if (csr_present & CP_CBCARD)
			printf("do_int: Cardbus card detected\n");
		if (csr_present & CP_16BITCARD)
			printf("do_int: 16-bit card detected\n");
	}
	if (csr_present & CP_PWRCYCLE)
	{
		if (debug)
			printf("do_int: powered up\n");
		return;
	}
	vcc_5v= !!(csr_present & CP_5VCARD);
	vcc_3v= !!(csr_present & CP_3VCARD);
	vcc_Xv= !!(csr_present & CP_XVCARD);
	vcc_Yv= !!(csr_present & CP_YVCARD);
	if (debug)
	{
		printf("do_int: card supports:%s%s%s%s\n",
			vcc_5v ? " 5V" : "", vcc_3v ? " 3V" : "",
			vcc_Xv ? " X.X V" : "", vcc_Yv ? " Y.Y V" : "");
	}
	socket_5v= !!(csr_present & CP_5VSOCKET);
	socket_3v= !!(csr_present & CP_3VSOCKET);
	socket_Xv= !!(csr_present & CP_XVSOCKET);
	socket_Yv= !!(csr_present & CP_YVSOCKET);
	if (debug)
	{
		printf("do_int: socket supports:%s%s%s%s\n",
			socket_5v ? " 5V" : "", socket_3v ? " 3V" : "",
			socket_Xv ? " X.X V" : "", socket_Yv ? " Y.Y V" : "");
	}
	if (vcc_5v && socket_5v)
	{
		csr_control= (csr_control & ~CC_VCCCTRL) | CC_VCC_5V;
		pp->csr_ptr->csr_control= csr_control;
		if (debug)
			printf("do_int: applying 5V\n");
	}
	else if (vcc_3v && socket_3v)
	{
		csr_control= (csr_control & ~CC_VCCCTRL) | CC_VCC_3V;
		pp->csr_ptr->csr_control= csr_control;
		if (debug)
			printf("do_int: applying 3V\n");
	}
	else if (vcc_Xv && socket_Xv)
	{
		csr_control= (csr_control & ~CC_VCCCTRL) | CC_VCC_XV;
		pp->csr_ptr->csr_control= csr_control;
		printf("do_int: applying X.X V\n");
	}
	else if (vcc_Yv && socket_Yv)
	{
		csr_control= (csr_control & ~CC_VCCCTRL) | CC_VCC_YV;
		pp->csr_ptr->csr_control= csr_control;
		printf("do_int: applying Y.Y V\n");
	}
	else
	{
		printf("do_int: socket and card are not compatible\n");
		return;
	}

	csr_event= pp->csr_ptr->csr_event;
	if (csr_event)
	{
		if (debug)
			printf("clearing socket event\n");
		pp->csr_ptr->csr_event= csr_event;
		if (debug)
		{
			printf("Socket event (cleared): 0x%x\n",
				pp->csr_ptr->csr_event);
		}
	}

	devind= pp->p_devind;
	v8= pci_attr_r8(devind, TI_CARD_CTRL);
	if (v8 & TI_CCR_IFG)
	{
		printf("ti1225: got functional interrupt\n");
		pci_attr_w8(devind, TI_CARD_CTRL, v8);
	}

	if (debug)
	{
		v8= pci_attr_r8(devind, TI_CARD_CTRL);
		printf("TI_CARD_CTRL: 0x%02x\n", v8);
	}

	spin_init(&spin, 100000);
	do {
		csr_present= pp->csr_ptr->csr_present;
		if (csr_present & CP_PWRCYCLE)
			break;
	} while (spin_check(&spin));

	if (!(csr_present & CP_PWRCYCLE))
	{
		printf("do_int: not powered up?\n");
		return;
	}

	/* Reset device */
	v16= pci_attr_r16(devind, CBB_BRIDGECTRL);
	v16 |= CBB_BC_CRST;
	pci_attr_w16(devind, CBB_BRIDGECTRL, v16);

	/* Wait one microsecond. Is this correct? What are the specs? */
	micro_delay(1);

	/* Clear CBB_BC_CRST */
	v16= pci_attr_r16(devind, CBB_BRIDGECTRL);
	v16 &= ~CBB_BC_CRST;
	pci_attr_w16(devind, CBB_BRIDGECTRL, v16);

	/* Wait one microsecond after clearing the reset line. Is this
	 * correct? What are the specs?
	 */
	micro_delay(1);

	pci_rescan_bus(pp->p_cb_busnr);

#if USE_INTS
	r= sys_irqenable(&pp->p_hook);
	if (r != OK)
		panic("unable enable interrupts: %d", r);
#endif

}

