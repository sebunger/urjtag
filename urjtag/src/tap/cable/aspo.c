/*
 * $Id$
 *
 * Aspo JTAG Cable Driver
 * Copyright (C) 2003 Ultra d.o.o.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * Written by Marcel Telka <marcel@telka.sk>, 2002, 2003.
 *
 * Documentation:
 * http://aspodata.se/git/openhw/boards_other/isp/ pp_jtag_arm20.{sch,pcb}
 *
 * Base on the code for the Macraigor WIGGLER code written by Marcel Telka.
 * Modified by Matej Kupljen <matej.kupljen@ultra.si> to support
 * the Modified WIGGLER JTAG cable. This has an additional pin, that is
 * used for CPU reset. The schematic is based on the source code for the
 * open source JTAG debugger for the PXA250 (255) processor, called Jelie
 * <www.jelie.org>.
 * Modified by Karl Hammar <karl@apodata.se> to support the JTAG cable at
 * http://aspodata.se/git/openhw/boards_other/isp/ pp_jtag_arm20.{sch,pcb}
 *
 */

#include <stdlib.h>
#include <sysdep.h>

#include <urjtag/cable.h>
#include <urjtag/parport.h>
#include <urjtag/chain.h>

#include "generic.h"
#include "generic_parport.h"

/* copied from <linux/parport.h>
   GNU Free Documentation License, Version 1.1 or any later version
 */
#define PARPORT_CONTROL_STROBE    0x1
#define PARPORT_CONTROL_AUTOFD    0x2
#define PARPORT_CONTROL_INIT      0x4
#define PARPORT_CONTROL_SELECT    0x8

#define PARPORT_STATUS_ERROR      0x8
#define PARPORT_STATUS_SELECT     0x10
#define PARPORT_STATUS_PAPEROUT   0x20
#define PARPORT_STATUS_ACK        0x40
#define PARPORT_STATUS_BUSY       0x80

/* copied from openocd src/jtag/drivers/parport.c
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
...
 */
/* parallel port cable description
 */
struct cable {
        const char *name;
        uint8_t TDO_MASK;       /* status port bit containing current TDO value */
        uint8_t TRST_MASK;      /* data port bit for TRST */
        uint8_t TMS_MASK;       /* data port bit for TMS */
        uint8_t TCK_MASK;       /* data port bit for TCK */
        uint8_t TDI_MASK;       /* data port bit for TDI */
        uint8_t SRST_MASK;      /* data port bit for SRST */
        uint8_t OUTPUT_INVERT;  /* data port bits that should be inverted */
        uint8_t INPUT_INVERT;   /* status port that should be inverted */
        uint8_t PORT_INIT;      /* initialize data port with this value */
        uint8_t PORT_EXIT;      /* de-initialize data port with this value */
        uint8_t LED_MASK;       /* data port bit for LED */
};

static const struct cable cables[] = {
        /* name                         tdo   trst  tms   tck   tdi   srst  o_inv i_inv init  exit  led */
        { "wiggler",                    0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x01, 0x80, 0x80, 0x80, 0x00 },
        { "wiggler2",                   0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x01, 0x80, 0x80, 0x00, 0x20 },
        { "wiggler_ntrst_inverted",     0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x11, 0x80, 0x80, 0x80, 0x00 },
        { "old_amt_wiggler",            0x80, 0x01, 0x02, 0x04, 0x08, 0x10, 0x11, 0x80, 0x80, 0x80, 0x00 },
        { "arm-jtag",                   0x80, 0x01, 0x02, 0x04, 0x08, 0x10, 0x01, 0x80, 0x80, 0x80, 0x00 },
        { "chameleon",                  0x80, 0x00, 0x04, 0x01, 0x02, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 },
        { "dlc5",                       0x10, 0x00, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x10, 0x10, 0x00 },
        { "triton",                     0x80, 0x08, 0x04, 0x01, 0x02, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 },
        { "lattice",                    0x40, 0x10, 0x04, 0x02, 0x01, 0x08, 0x00, 0x00, 0x18, 0x18, 0x00 },
        { "flashlink",                  0x20, 0x10, 0x02, 0x01, 0x04, 0x20, 0x30, 0x20, 0x00, 0x00, 0x00 },
/* Altium Universal JTAG cable. Set the cable to Xilinx Mode and wire to target as follows:
        HARD TCK - Target TCK
        HARD TMS - Target TMS
        HARD TDI - Target TDI
        HARD TDO - Target TDO
        SOFT TCK - Target TRST
        SOFT TDI - Target SRST
*/
        { "altium",                     0x10, 0x20, 0x04, 0x02, 0x01, 0x80, 0x00, 0x00, 0x10, 0x00, 0x08 },
        { "aspo",                       0x10, 0x01, 0x04, 0x08, 0x02, 0x10, 0x17, 0x00, 0x17, 0x17, 0x00 },
        { NULL,                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};
/* end copy */

/* Note about above:
   all values ^= OUTPUT_INVERT, even PORT_INIT and PORT_EXIT on write
   to hw register

   no support for dongles/cables that uses the control port for any of
   its output signals, nor for thoose that uses that data register for
   input

   urj_tap_cable_start() runs your drivers init function, where you
   can set any initial value, BUT it then runs urj_tap_trst_reset()
   which defeats the setting of an initial value
 */

/* some status and control pins are inverted in hw, see e.g. 
   https://en.wikipedia.org/wiki/Parallel_port#Port_addresses second table
   https://web.archive.org/web/20120301022928/http://retired.beyondlogic.org/spp/parallel.pdf p.3
 */
#define STATUS_PORT_INVERSION  ( PARPORT_STATUS_BUSY )
#define CONTROL_PORT_INVERSION ( PARPORT_CONTROL_STROBE | PARPORT_CONTROL_AUTOFD | PARPORT_CONTROL_SELECT )

/*
 * aspo Parallel port based JTAG dongle (#11 in above list)
 *
 * Design goal:
 *  pp idle at all zeroes (or not connected) => jtag idle at all high except TCK low
 *  target at 3.3V
 *
 * TDO -> select (status reg. 0x10, pin 13)
 * DATA0 (0x01 pin 2) -> nTRST  inverted
 * DATA1 (0x02 pin 3) -> TDI    inverted
 * DATA2 (0x04 pin 4) -> TMS    inverted
 * DATA3 (0x08 pin 5) -> TCK
 * DATA4 (0x10 pin 6) -> nSRST  inverted
 *
 * All lines except TCK and TDO are inverted
 *
 * One could argue for nTRST to be non-inverted (i.e. low) to keep
 * TAPs in the reset state when parport output is zero.
 */

struct pdata {
  struct cable wiring;
  unsigned char output;			/* signal level of jtag bus, but with
				   paralell port output pin order */
};

static int
cable_connect (urj_cable_t *cable, urj_cable_parport_devtype_t devtype,
	       const char *devname, const urj_param_t *params[])
{
    struct pdata *pdata;
    int st;

    /* Here we can handle possible params[] */

    if ((st = urj_tap_cable_generic_parport_connect (cable, devtype, devname, (const urj_param_t **)NULL)) != URJ_STATUS_OK)
      return st;

    /* allocate our private data containg config and output pin state */
    if (cable->params) { free(cable->params); }
    pdata = (struct pdata *) malloc(sizeof (struct pdata));
    if (!pdata) {
      urj_error_set (URJ_ERROR_OUT_OF_MEMORY, _("malloc(%zd) fails"), sizeof (struct pdata *));
      return URJ_STATUS_FAIL;
    }
    cable->params = pdata;

    /* possible search cables[] for matching driver */
    pdata->wiring = cables[11];
    pdata->output = 0; /* real value set at first data write */

    return URJ_STATUS_OK;
}

static int
cable_write(urj_cable_t *cable, unsigned char output)
{
    struct pdata *pdata = (struct pdata *) cable->params;
    output ^= pdata->wiring.OUTPUT_INVERT;

    return urj_tap_parport_set_data (cable->link.port, output);
}

static int
cable_init (urj_cable_t *cable)
{
    struct pdata *pdata = (struct pdata *) cable->params;
    int st;
    unsigned char output;
    unsigned char unused_bits;

    /* open/connect to device and check readability */
    if (urj_tap_parport_open (cable->link.port) != URJ_STATUS_OK)
        return URJ_STATUS_FAIL;

    if (urj_tap_parport_get_data (cable->link.port) < 0)
        return URJ_STATUS_FAIL;

    if (urj_tap_parport_get_status (cable->link.port) < 0)
        return URJ_STATUS_FAIL;

    /* init output
       only three cable drivers uses the control port
       we just set it to zero, but we could possible set it to 0xff
    */
    if (urj_tap_parport_set_control (cable->link.port, 0) != URJ_STATUS_OK)
        return URJ_STATUS_FAIL;

    output  = pdata->wiring.PORT_INIT;
    /* possible set unused pins high */
    unused_bits = ~(pdata->wiring.TDO_MASK & pdata->wiring.TRST_MASK & pdata->wiring.TMS_MASK  &
		    pdata->wiring.TCK_MASK & pdata->wiring.TDI_MASK  & pdata->wiring.SRST_MASK );
    (void) unused_bits;
    /*output |= unused_bits;*/

    if ((st=cable_write (cable, output)) != URJ_STATUS_OK) return st;
    pdata->output = output; /* we could possible move all thoose inte cable_write() */

    return URJ_STATUS_OK;
}

/*
  http://www.jtagtest.com/pdf/ssya002c.pdf page 3-5:
  The IEEE Std 1149.1 test bus uses both clock edges of TCK. TMS and
  TDI are sampled on the rising edge of TCK, while TDO changes on the
  falling edge of TCK.
 */

static void
cable_clock (urj_cable_t *cable, int tms, int tdi, int nn)
{
    struct pdata *pdata = (struct pdata *) cable->params;
    unsigned char output0;
    unsigned char output1;
    int ix;

    if (!pdata) {
      urj_error_set (URJ_ERROR_INVALID, _("pdata is null"));
      return;
    }

    output0 = pdata->output;
    if (tms) { output0 |=  pdata->wiring.TMS_MASK; }
    else {     output0 &= ~pdata->wiring.TMS_MASK; }

    if (tdi) { output0 |=  pdata->wiring.TDI_MASK; }
    else {     output0 &= ~pdata->wiring.TDI_MASK; }

    /* target TAP's reads TDI/TMS at rising edge of TCK  */
    output1 = output0;
    output0 &= ~pdata->wiring.TCK_MASK;
    output1 |=  pdata->wiring.TCK_MASK;

    for (ix = 0; ix < nn; ix++)
    {
        (void) cable_write (cable, output0);
        urj_tap_cable_wait (cable);

        (void) cable_write (cable, output1);
        urj_tap_cable_wait (cable);
    }

    pdata->output = output1;
}

static int
cable_get_tdo (urj_cable_t *cable)
{
    /* tested ok by cmd_pod_run() */
    struct pdata *pdata = (struct pdata *) cable->params;
    unsigned char output;
    int status;
    int tdo;
    int st;

    if (!pdata) {
      urj_error_set (URJ_ERROR_INVALID, _("pdata is null"));
      return -1;
    }

    /* TDO is set at the falling edge of TCK */
    output  = pdata->output & ~pdata->wiring.TCK_MASK;
    if ((st=cable_write (cable, output)) != URJ_STATUS_OK) return st;
    pdata->output = output;

    urj_tap_cable_wait (cable);

    if ((status = urj_tap_parport_get_status (cable->link.port)) < 0)
      return status;

    status ^= pdata->wiring.INPUT_INVERT;
    tdo = status & pdata->wiring.TDO_MASK ? 1 : 0;
    return tdo;
}

static int
cable_get_signal (urj_cable_t *cable, urj_pod_sigsel_t sig)
{
  /* TODO: Don't know how to test this one.
     used by urj_tap_chain_get_pod_signal() which only the python
     binding uses
  */
  /* 
     the default function (urj_tap_cable_generic_get_signal()) has:
     return (((PARAM_SIGNALS (cable)) & sig) != 0) ? 1 : 0;
   */

    struct pdata *pdata = (struct pdata *) cable->params;
    unsigned char output;
    int pod_signal;

    if (!pdata) {
      urj_error_set (URJ_ERROR_INVALID, _("pdata is null"));
      return -1;
    }

    output  = pdata->output;

    /* convert from output pin state to what cmd_pod wants to see, urj_pod_sigsel_t values or'ed together */
    pod_signal = 0;

    if (output & pdata->wiring.TRST_MASK) { pod_signal |= URJ_POD_CS_TRST;  }
    if (output & pdata->wiring.TMS_MASK ) { pod_signal |= URJ_POD_CS_TMS;   }
    if (output & pdata->wiring.TCK_MASK ) { pod_signal |= URJ_POD_CS_TCK;   }
    if (output & pdata->wiring.TDI_MASK ) { pod_signal |= URJ_POD_CS_TDI;   }
    if (output & pdata->wiring.SRST_MASK) { pod_signal |= URJ_POD_CS_RESET; }

    return (pod_signal & sig) ? 1 : 0;
}

static int
cable_set_signal (urj_cable_t *cable, int mask, int val)
{
    /* tested ok by cmd_pod_run() */
    /*
      return value: the old pod_signal

     */
    struct pdata *pdata = (struct pdata *) cable->params;
    unsigned char output;
    int pod_signal;
    int st;

    if (!pdata) {
      urj_error_set (URJ_ERROR_INVALID, _("pdata is null"));
      return -1;
    }

    /* convert from output pin state to what cmd_pod wants to see, urj_pod_sigsel_t values or'ed together */
    output  = pdata->output;
    pod_signal = 0;
    if (output & pdata->wiring.TRST_MASK) { pod_signal |= URJ_POD_CS_TRST;  }
    if (output & pdata->wiring.TMS_MASK ) { pod_signal |= URJ_POD_CS_TMS;   }
    if (output & pdata->wiring.TCK_MASK ) { pod_signal |= URJ_POD_CS_TCK;   }
    if (output & pdata->wiring.TDI_MASK ) { pod_signal |= URJ_POD_CS_TDI;   }
    if (output & pdata->wiring.SRST_MASK) { pod_signal |= URJ_POD_CS_RESET; }

    /* only these can be modified */
    /*mask &= (URJ_POD_CS_TDI | URJ_POD_CS_TCK | URJ_POD_CS_TMS | URJ_POD_CS_TRST);*/
    mask &= (URJ_POD_CS_TDI | URJ_POD_CS_TCK | URJ_POD_CS_TMS | URJ_POD_CS_TRST | URJ_POD_CS_RESET);


    if (mask != 0)
    {
        int sigs = (pod_signal & ~mask) | (val & mask);

	output &= ~( pdata->wiring.TRST_MASK | pdata->wiring.TMS_MASK | pdata->wiring.TCK_MASK |
		     pdata->wiring.TDI_MASK  | pdata->wiring.SRST_MASK );

	if (sigs & URJ_POD_CS_TDI)   { output |=  pdata->wiring.TDI_MASK;  }
	if (sigs & URJ_POD_CS_TCK)   { output |=  pdata->wiring.TCK_MASK;  }
	if (sigs & URJ_POD_CS_TMS)   { output |=  pdata->wiring.TMS_MASK;  }
	if (sigs & URJ_POD_CS_TRST)  { output |=  pdata->wiring.TRST_MASK; }
	if (sigs & URJ_POD_CS_RESET) { output |=  pdata->wiring.SRST_MASK; }

	if ((st=cable_write (cable, output)) != URJ_STATUS_OK) return st;

	pdata->output = output;
    }

    return pod_signal;
}

const urj_cable_driver_t urj_tap_cable_aspo_driver = {
    "ASPO",
    N_("Aspo JTAG Cable (experimental)"),
    URJ_CABLE_DEVICE_PARPORT,
    { .parport = cable_connect, },
    urj_tap_cable_generic_disconnect,
    urj_tap_cable_generic_parport_free,
    cable_init,
    urj_tap_cable_generic_parport_done,
    urj_tap_cable_generic_set_frequency,
    cable_clock,
    cable_get_tdo,
    urj_tap_cable_generic_transfer,
    cable_set_signal,
    cable_get_signal,
    urj_tap_cable_generic_flush_one_by_one,
    urj_tap_cable_generic_parport_help
};