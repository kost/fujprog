/*
 * FT-232R / FT-231X USB JTAG programmer
 *
 * Copyright (c) 2010 - 2018 Marko Zec, University of Zagreb
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * TODO:
 * 
 * - WIN32: check for USB device string description
 *
 * - JTAG scan / identify chain on entry.
 *
 * - verify SRAM / FLASH
 *
 * - RUN TEST delay downscaling.
 *
 * - disable resetting the TAP on entry / leave?
 *
 * - execute SVF commands provided as command line args?
 */

static const char *verstr = "ULX2S / ULX3S JTAG programmer v 3.0.92";


#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef __FreeBSD__
#define USE_PPI
#endif

#if defined(__linux__) || defined(WIN32)
#define isnumber(x) (x >= '0' && x <= '9')
#endif

#ifdef WIN32
#include <windows.h>
#include <ftd2xx.h>
#include <conio.h>
#else
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#ifdef USE_PPI
#include <dev/ppbus/ppi.h>
#include <dev/ppbus/ppbconf.h>
#endif
#include <ftdi.h>
#endif

#ifdef WIN32
#define	BITMODE_OFF		0x0
#define	BITMODE_BITBANG		0x1
#define	BITMODE_SYNCBB		0x4
#define	BITMODE_CBUS		0x20
#endif

/* Forward declarations */
static int commit(int);
static void set_state(int);
static int exec_svf_tokenized(int, char **);
static int send_dr(int, char *, char *, char *);
static int send_ir(int, char *, char *, char *);
static int exec_svf_mem(char *, int, int);
static int cmp_chip_ids(char *, char *);


enum svf_cmd {
	SVF_SDR, SVF_SIR, SVF_STATE, SVF_RUNTEST, SVF_HDR, SVF_HIR,
	SVF_TDR, SVF_TIR, SVF_ENDDR, SVF_ENDIR, SVF_FREQUENCY, SVF_UNKNOWN
};

static struct svf_cmdtable {
	enum svf_cmd cmd_id;
	char *cmd_str;
} svf_cmdtable[] = {
	{SVF_SDR,		"SDR"},
	{SVF_SIR,		"SIR"},
	{SVF_STATE,		"STATE"},
	{SVF_RUNTEST,		"RUNTEST"},
	{SVF_HDR,		"HDR"},
	{SVF_HIR,		"HIR"},
	{SVF_TDR,		"TDR"},
	{SVF_TIR,		"TIR"},
	{SVF_ENDDR,		"ENDDR"},
	{SVF_ENDIR,		"ENDIR"},
	{SVF_FREQUENCY,		"FREQUENCY"},
	{SVF_UNKNOWN,		NULL}
};


enum tap_state {
	RESET, IDLE,
	DRSELECT, DRCAPTURE, DRSHIFT, DREXIT1, DRPAUSE, DREXIT2, DRUPDATE,
	IRSELECT, IRCAPTURE, IRSHIFT, IREXIT1, IRPAUSE, IREXIT2, IRUPDATE,
	UNDEFINED, UNSUPPORTED,
};

static struct tap_statetable {
	enum tap_state state_id;
	char *state_str;
} tap_statetable[] = {
	{RESET,			"RESET"},
	{IDLE,			"IDLE"},
	{DRSELECT,		"DRSELECT"},
	{DRCAPTURE,		"DRCAPTURE"},
	{DRSHIFT,		"DRSHIFT"},
	{DREXIT1,		"DREXIT1"},
	{DRPAUSE,		"DRPAUSE"},
	{DREXIT2,		"DREXIT2"},
	{DRUPDATE,		"DRUPDATE"},
	{IRSELECT,		"IRSELECT"},
	{IRCAPTURE,		"IRCAPTURE"},
	{IRSHIFT,		"IRSHIFT"},
	{IREXIT1,		"IREXIT1"},
	{IRPAUSE,		"IRPAUSE"},
	{IREXIT2,		"IREXIT2"},
	{IRUPDATE,		"IRUPDATE"},
	{UNDEFINED,		"UNDEFINED"},
	{UNSUPPORTED,		NULL}
};

#define	STATE2STR(state)	(tap_statetable[state].state_str)


static enum port_mode {
	PORT_MODE_ASYNC, PORT_MODE_SYNC, PORT_MODE_UART, PORT_MODE_UNKNOWN
} port_mode = PORT_MODE_UNKNOWN;


static enum cable_hw {
	CABLE_HW_USB, CABLE_HW_PPI, CABLE_HW_COM, CABLE_HW_UNKNOWN
} cable_hw = CABLE_HW_UNKNOWN;


static struct cable_hw_map {
	int	cable_hw;
	int	usb_vid;
	int	usb_pid;
	char	*cable_path;
	char	tck, tms, tdi, tdo;
	char	cbus_led;
} cable_hw_map[] = {
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6001,
		.cable_path =	"FER ULXP2 board JTAG / UART",
		.tck =		0x20,
		.tms =		0x80,
		.tdi =		0x08,
		.tdo =		0x40,
		.cbus_led =	0x02
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6001,
		.cable_path =	"FER ULX2S board JTAG / UART",
		.tck =		0x20,
		.tms =		0x80,
		.tdi =		0x08,
		.tdo =		0x40,
		.cbus_led =	0x02
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA board",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA v1.7",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 25K v1.7",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 45K v1.7",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 12K v2.1.2",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 25K v2.1.2",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 45K v2.1.2",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 85K v2.1.2",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 12K v3.0.3",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 25K v3.0.3",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 45K v3.0.3",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 85K v3.0.3",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 12K v3.0.7",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 25K v3.0.7",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 45K v3.0.7",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6015,
		.cable_path =	"ULX3S FPGA 85K v3.0.7",
		.tck =		0x20,
		.tms =		0x40,
		.tdi =		0x80,
		.tdo =		0x08,
		.cbus_led =	0x00
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6010,
		.cable_path =	"Lattice ECP5 Evaluation Board",
		.tck =		0x01,
		.tms =		0x08,
		.tdi =		0x02,
		.tdo =		0x04,
		.cbus_led =	0x10
	},
	{
		.cable_hw = 	CABLE_HW_USB,
		.usb_vid = 	0x0403,
		.usb_pid =	0x6011,
		.cable_path =	"Digilent FFC",
		.tck =		0x01,
		.tms =		0x08,
		.tdi =		0x02,
		.tdo =		0x04,
		.cbus_led =	0x00
	},
	{
		.cable_hw =	CABLE_HW_UNKNOWN,
		.cable_path =	"UNKNOWN"
	}
};


#define	USB_BAUDS		1000000

#define	USB_TCK			(hmp->tck)
#define	USB_TMS			(hmp->tms)
#define	USB_TDI			(hmp->tdi)
#define	USB_TDO			(hmp->tdo)
#define	USB_CBUS_LED		(hmp->cbus_led)

#define	PPI_TCK			0x02
#define	PPI_TMS			0x04
#define	PPI_TDI			0x01
#define	PPI_TDO			0x40

#define	USB_BUFLEN_ASYNC	8192
#ifdef WIN32
#define	USB_BUFLEN_SYNC		4096
#else
#define	USB_BUFLEN_SYNC		384
#endif

#define	BUFLEN_MAX		USB_BUFLEN_ASYNC /* max(SYNC, ASYNC) */

#define	LED_BLINK_RATE		250

#define	BREAK_MS		250

#define	SPI_PAGE_SIZE		256
#define	SPI_SECTOR_SIZE		(256 * SPI_PAGE_SIZE)

static char *statc = "-\\|/";

/* Runtime globals */
static int cur_s = UNDEFINED;
static uint8_t txbuf[64 * BUFLEN_MAX];
static uint8_t rxbuf[64 * BUFLEN_MAX];
static int txpos;
static int need_led_blink;	/* Schedule CBUS led toggle */
static int last_ledblink_ms;	/* Last time we toggled the CBUS LED */
static int led_state;		/* CBUS LED indicator state */
static int blinker_phase;
static int progress_perc;
static int bauds = 115200;	/* async terminal emulation baudrate */
static int xbauds;		/* binary transfer baudrate */
static int port_index;
static int terminal;		/* terminal emulation mode */
static int reload;		/* send break to reset f32c */
static int quiet;		/* suppress standard messages */
char *svf_name;			/* SVF output name */
static int txfu_ms;		/* txt file upload character delay (ms) */
static int tx_binary;		/* send in raw (0) or binary (1) format */
static const char *txfname;	/* file to send */
static const char *com_name;	/* COM / TTY port name for -a or -t */
static int spi_addr;		/* Base address for -j flash programming */
static int global_debug;

static struct cable_hw_map *hmp; /* Selected cable hardware map */
#ifdef WIN32
static FT_HANDLE ftHandle;	/* USB port handle */
static HANDLE com_port;		/* COM port file */
static struct _DCB tty;		/* COM port TTY handle */
#else
static struct ftdi_context fc;	/* USB port handle */
static int com_port;		/* COM port file */
static struct termios tty;	/* COM port TTY handle */
#ifdef USE_PPI
static int ppi;			/* Parallel port handle */
#endif
#endif


/* ms_sleep() sleeps for at least the number of milliseconds given as arg */
#define	ms_sleep(delay_ms)	usleep((delay_ms) * 1000)


static long
ms_uptime(void)
{
	long ms;
#ifndef WIN32
	struct timeval tv;

	gettimeofday(&tv, 0);
	ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
#else
	ms = GetTickCount();
#endif
	return (ms);
}


static int
set_port_mode(int mode)
{
	int res = 0;

	/* No-op if already in requested mode, or not using USB */
	if (!need_led_blink &&
	    (port_mode == mode || cable_hw != CABLE_HW_USB)) {
		port_mode = mode;
		return (0);
	}

	/* Flush any stale TX buffers */
	commit(1);

	/* Blink status LED by deactivating CBUS pulldown pin */
	if (need_led_blink) {
		need_led_blink = 0;
		led_state ^= USB_CBUS_LED;
		if (!quiet && progress_perc < 100) {
			fprintf(stderr, "\rProgramming: %d%% %c ",
			    progress_perc, statc[blinker_phase]);
			fflush(stderr);
		}
		blinker_phase = (blinker_phase + 1) & 0x3;
	}

#ifdef WIN32
	if (mode == PORT_MODE_ASYNC)
		mode = PORT_MODE_SYNC;
#endif

	switch (mode) {
	case PORT_MODE_SYNC:
#ifdef WIN32
		/*
		 * If switching to SYNC mode, attempt to allow for TX
		 * buffers to drain first.
		 */
		if (port_mode != PORT_MODE_SYNC)
			ms_sleep(20);

		res = FT_SetBitMode(ftHandle,
#else
		res = ftdi_set_bitmode(&fc,
#endif
		    USB_TCK | USB_TMS | USB_TDI | led_state,
		    BITMODE_SYNCBB | (BITMODE_CBUS * (USB_CBUS_LED != 0)));

		if (port_mode == PORT_MODE_SYNC)
			break;

		/* Flush any stale RX buffers */
#ifdef WIN32
		for (res = 0; res < 2; res++) {
			do {
				ms_sleep(10);
			} while (FT_StopInTask(ftHandle) != FT_OK);
			FT_Purge(ftHandle, FT_PURGE_RX);
			do {} while (FT_RestartInTask(ftHandle) != FT_OK);
			ms_sleep(10);
		}
#else
		do {
			res = ftdi_read_data(&fc, &txbuf[0], sizeof(txbuf));
		} while (res == sizeof(txbuf));
#endif
		break;

	case PORT_MODE_ASYNC:
#ifdef WIN32
		res = FT_SetBitMode(ftHandle,
#else
		res = ftdi_set_bitmode(&fc,
#endif
		    USB_TCK | USB_TMS | USB_TDI | led_state,
		    BITMODE_BITBANG | (BITMODE_CBUS * (USB_CBUS_LED != 0)));
		break;

	case PORT_MODE_UART:
		res = 0;
		if (port_mode == PORT_MODE_UART)
			break;
		/* Pull TCK low so that we don't incidentally pulse it. */
		memset(txbuf, 0, 10);
#ifdef WIN32
		FT_Write(ftHandle, txbuf, 100, (DWORD *) &res);
		if (res < 0) {
			fprintf(stderr, "FT_Write() failed\n");
			return (res);
		}
		res = FT_SetBitMode(ftHandle, 0, BITMODE_OFF);
#else
		res = ftdi_write_data(&fc, &txbuf[0], 10);
		if (res < 0) {
			fprintf(stderr, "ftdi_write_data() failed\n");
			return (res);
		}
		res = ftdi_disable_bitbang(&fc);
#endif
		break;

	default:
		res = -1;
	}

	port_mode = mode;
	return (res);
}


#ifdef WIN32
static int
setup_usb(void)
{
	FT_STATUS res;
	FT_DEVICE ftDevice;
	DWORD deviceID;
	char SerialNumber[16];
	char Description[64];

	res = FT_Open(port_index, &ftHandle);
	if (res != FT_OK) {
		fprintf(stderr, "FT_Open() failed\n");
		return (res);
	}

	res = FT_GetDeviceInfo(ftHandle, &ftDevice, &deviceID, SerialNumber,
	    Description, NULL);
	if (res != FT_OK) {
		fprintf(stderr, "FT_GetDeviceInfo() failed\n");
		return (res);
	}
	for (hmp = cable_hw_map; hmp->cable_hw != CABLE_HW_UNKNOWN; hmp++) {
		if ((deviceID == hmp->usb_vid << 16 | hmp->usb_pid)
		    && strcmp(Description, hmp->cable_path) == 0)
			break;
	}
	if (hmp->cable_hw == CABLE_HW_UNKNOWN)
		return (-1);

	if (!quiet)
		printf("Using USB cable: %s\n", hmp->cable_path);

	res = FT_SetBaudRate(ftHandle, USB_BAUDS);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetBaudRate() failed\n");
		return (res);
	}

#ifdef NOTYET
	FT_setUSB_Parameters();
	res = ftdi_write_data_set_chunksize(&fc, BUFLEN_MAX);
	if (res < 0) {
		fprintf(stderr, "ftdi_write_data_set_chunksize() failed\n");
		return (res);
	}
#endif

	res = FT_SetLatencyTimer(ftHandle, 1);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetLatencyTimer() failed\n");
		return (res);
	}

	res = FT_SetFlowControl(ftHandle, FT_FLOW_NONE, 0, 0);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetFlowControl() failed\n");
		return (res);
	}

	res = FT_SetBitMode(ftHandle, 0, BITMODE_BITBANG);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetBitMode() failed\n");
		return (res);
	}

	res = FT_SetTimeouts(ftHandle, 1000, 1000);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetTimeouts() failed\n");
		return (res);
	}

	FT_Purge(ftHandle, FT_PURGE_TX);
	FT_Purge(ftHandle, FT_PURGE_RX);

	return (0);
}


static int
shutdown_usb(void)
{

	int res;

	/* Allow for the USB FIFO to drain, just in case. */
	ms_sleep(10);

	/* Clean up */
	res = set_port_mode(PORT_MODE_UART);
	if (res < 0) {
		fprintf(stderr, "set_port_mode() failed\n");
		return (res);
	}

	res = FT_SetLatencyTimer(ftHandle, 20);
	if (res < 0) {
		fprintf(stderr, "FT_SetLatencyTimer() failed\n");
		return (res);
	}

	res = FT_Close(ftHandle);
	if (res < 0) {
		fprintf(stderr, "FT_Close() failed\n");
		return (res);
	}

	return (0);
}
#endif /* WIN32 */


#ifndef WIN32
#ifdef USE_PPI
static int
setup_ppi(void)
{
	char c = 0;

	ppi = open("/dev/ppi0", O_RDWR);
	if (ppi < 0)
		return (errno);

	ioctl(ppi, PPISDATA, &c);
	ioctl(ppi, PPISSTATUS, &c);
	ioctl(ppi, PPIGSTATUS, &c);
	if ((c & 0xb6) != 0x06) {
		close (ppi);
		return (EINVAL);
	}

	return (0);
}


static void
shutdown_ppi(void)
{

	/* Pull TCK low so that we don't incidentally pulse it on next run. */
	txbuf[0] = 0;
	ioctl(ppi, PPISDATA, &txbuf[0]);

	close (ppi);
}
#endif


static int
setup_usb(void)
{
	int res;

#ifdef __APPLE__
	setuid(0);
	system("/sbin/kextunload"
	    " -bundle-id com.FTDI.driver.FTDIUSBSerialDriver");
	system("/sbin/kextunload"
	    " -bundle-id com.apple.driver.AppleUSBFTDI");
#endif

	res = ftdi_init(&fc);
	if (res < 0) {
		fprintf(stderr, "ftdi_init() failed\n");
		return (res);
	}

	for (hmp = cable_hw_map; hmp->cable_hw != CABLE_HW_UNKNOWN; hmp++) {
		res = ftdi_usb_open_desc_index(&fc, hmp->usb_vid, hmp->usb_pid,
		    hmp->cable_path, NULL, port_index);
		if (res == 0)
			break;
	}
	if (res < 0) {
		res = ftdi_usb_open_desc_index(&fc, 0x0403, 0x6001,
		    NULL, NULL, port_index);
		if (res < 0) {
#ifdef __APPLE__
			system("/sbin/kextload"
			    " -bundle-id com.FTDI.driver.FTDIUSBSerialDriver");
			system("/sbin/kextload"
			    "  -bundle-id com.apple.driver.AppleUSBFTDI");
#endif
			return (res);
		}
	}

	res = ftdi_set_baudrate(&fc, USB_BAUDS);
	if (res < 0) {
		fprintf(stderr, "ftdi_set_baudrate() failed\n");
		return (res);
	}

	res = ftdi_write_data_set_chunksize(&fc, BUFLEN_MAX);
	if (res < 0) {
		fprintf(stderr, "ftdi_write_data_set_chunksize() failed\n");
		return (res);
	}

	/* Reducing latency to 1 ms for BITMODE_SYNCBB is crucial! */
	res = ftdi_set_latency_timer(&fc, 1);
	if (res < 0) {
		fprintf(stderr, "ftdi_set_latency_timer() failed\n");
		return (res);
	}

	res = ftdi_set_bitmode(&fc, USB_TCK | USB_TMS | USB_TDI,
	    BITMODE_BITBANG);
	if (res < 0) {
		fprintf(stderr, "ftdi_set_bitmode() failed\n");
		return (EXIT_FAILURE);
	}

	return (0);
}


static int
shutdown_usb(void)
{
	int res;

	/* Clean up */
	res = set_port_mode(PORT_MODE_UART);
	if (res < 0) {
		fprintf(stderr, "ftdi_disable_bitbang() failed\n");
		return (res);
	}

	res = ftdi_set_latency_timer(&fc, 20);
	if (res < 0) {
		fprintf(stderr, "ftdi_set_latency_timer() failed\n");
		return (res);
	}

#ifdef __linux__
	usb_reset((void *) fc.usb_dev);
#else
	res = ftdi_usb_close(&fc);
	if (res < 0) {
		fprintf(stderr, "unable to close ftdi device: %d (%s)\n",
		    res, ftdi_get_error_string(&fc));
		return (res);
	}
#endif

	ftdi_deinit(&fc);

#ifdef __APPLE__
	system("/sbin/kextload"
	    " -bundle-id com.FTDI.driver.FTDIUSBSerialDriver");
	system("/sbin/kextload"
	    "  -bundle-id com.apple.driver.AppleUSBFTDI");
#endif

	return (0);
}
#endif /* !WIN32 */


static void
set_tms_tdi(int tms, int tdi)
{
	int val = 0;

	if (cable_hw == CABLE_HW_USB) {
		if (tms)
			val |= USB_TMS;
		if (tdi)
			val |= USB_TDI;
		txbuf[txpos++] = val;
		txbuf[txpos++] = val | USB_TCK;
	} else { /* PPI */
		if (tms)
			val |= PPI_TMS;
		if (tdi)
			val |= PPI_TDI;
		txbuf[txpos++] = val;
		txbuf[txpos++] = val | PPI_TCK;
	}

	if (txpos > sizeof(txbuf)) {
		fprintf(stderr, "txbuf overflow\n");
		if (cable_hw == CABLE_HW_USB)
			shutdown_usb();
		exit(EXIT_FAILURE);
	}
}


static int
send_generic(int bits, char *tdi, char *tdo, char *mask)
{
	int res, i, bitpos, tdomask, tdoval, maskval, val = 0, txval = 0;
	int rxpos, rxlen;

	if (cable_hw == CABLE_HW_USB)
		tdomask = USB_TDO;
	else
		tdomask = PPI_TDO;

	i = strlen(tdi);
	if (i != (bits + 3) / 4) {
		fprintf(stderr, "send_generic(): bitcount and tdi "
		    "data length do not match\n");
		return (EXIT_FAILURE);
	}
	if (tdo != NULL && strlen(tdo) != i) {
		if (mask != NULL && strlen(mask) != i) {
			fprintf(stderr, "send_generic(): tdi, tdo and mask "
			    "must be of same length\n");
			return (EXIT_FAILURE);
		}
		fprintf(stderr, "send_generic(): tdi and tdo "
		    "must be of same length\n");
		return (EXIT_FAILURE);
	}

	if (cur_s == DRPAUSE || cur_s == IRPAUSE ) {
		/* Move from *PAUSE to *EXIT2 state */
		set_tms_tdi(1, 0);
	}

	/* Move from *CAPTURE or *EXIT2 to *SHIFT state */
	set_tms_tdi(0, 0);

	/* Set up receive index / length */
	rxpos = txpos + 2;
	rxlen = bits;

	for (bitpos = 0; bits > 0; bits--) {
		if (bitpos == 0) {
			i--;
			val = tdi[i];
			if (val >= '0' && val <= '9')
				val = val - '0';
			else if (val >= 'A' && val <= 'F')
				val = val + 10 - 'A';
			else {
				fprintf(stderr, "send_generic():"
				    "TDI data not in hex format\n");
				return (EXIT_FAILURE);
			}
		}

		txval = val & 0x1;
		if (bits > 1)
			set_tms_tdi(0, txval);
		else
			set_tms_tdi(1, txval);

		val = val >> 1;
		bitpos = (bitpos + 1) & 0x3;
	}

	/* Move from *EXIT1 to *PAUSE state */
	set_tms_tdi(0, txval);

	/* Send / receive data on JTAG port */
	res = commit(0);

	/* Translate received bitstream into hex, apply mask, store in tdi */
	if (port_mode == PORT_MODE_SYNC) {
		if (mask != NULL)
			mask += strlen(tdi);
		if (tdo != NULL)
			tdo += strlen(tdi);
		tdi += strlen(tdi);
		val = 0;
		for (i = rxpos, bits = 0; bits < rxlen; i += 2) {
			val += (((txbuf[i] & tdomask) != 0) << (bits & 0x3));
			bits++;
			if ((bits & 0x3) == 0 || bits == rxlen) {
				if (mask != NULL) {
					/* Apply mask to received data */
					mask--;
					maskval = *mask;
					if (maskval >= '0' && maskval <= '9')
						maskval = maskval - '0';
					else if (maskval >= 'A' &&
					    maskval <= 'F')
						maskval = maskval + 10 - 'A';
					val &= maskval;
					/* Apply mask to expected TDO as well */
					if (tdo != NULL) {
						tdo--;
						tdoval = *tdo;
						if (tdoval >= '0' &&
						    tdoval <= '9')
							tdoval = tdoval - '0';
						else if (tdoval >= 'A' &&
						    tdoval <= 'F')
							tdoval =
							    tdoval + 10 - 'A';
						tdoval &= maskval;
						if (tdoval < 10)
							*tdo = tdoval + '0';
						else
							*tdo =
							    tdoval - 10 + 'A';
					}
				}
				tdi--;
				if (val < 10)
					*tdi = val + '0';
				else
					*tdi = val - 10 + 'A';
				val = 0;
			}
		}
	}

	return (res);
}


static int
send_dr(int bits, char *tdi, char *tdo, char *mask)
{
	int res;

	if (cur_s != DRPAUSE) {
		fprintf(stderr, "Must be in DRPAUSE on entry to send_dr()!\n");
		return (EXIT_FAILURE);
	}
	res = send_generic(bits, tdi, tdo, mask);
	cur_s = DRPAUSE;
	return (res);
}


static int
send_ir(int bits, char *tdi, char *tdo, char *mask)
{
	int res;

	if (cur_s != IRPAUSE) {
		fprintf(stderr, "Must be in IRPAUSE on entry to send_ir()!\n");
		return (EXIT_FAILURE);
	}
	res = send_generic(bits, tdi, tdo, mask);
	cur_s = IRPAUSE;
	return (res);
}


static int
commit_usb(void)
{
	int txchunklen, res, i;

	for (i = 0; i < txpos; i += txchunklen) {
		txchunklen = txpos - i;
		if (port_mode == PORT_MODE_SYNC && txchunklen > USB_BUFLEN_SYNC)
			txchunklen = USB_BUFLEN_SYNC;
#ifdef WIN32
		FT_Write(ftHandle, &txbuf[i], txchunklen, (DWORD *) &res);
#else
		res = ftdi_write_data(&fc, &txbuf[i], txchunklen);
#endif
		if (res != txchunklen) {
			fprintf(stderr, "ftdi_write_data() failed\n");
			return (EXIT_FAILURE);
		}

		if (port_mode == PORT_MODE_SYNC) {
#ifdef WIN32
			FT_Read(ftHandle, &txbuf[i], txchunklen,
			    (DWORD *) &res);
#else
			int rep = 0;
			for (res = 0; res < txchunklen && rep < 8;
			    rep++) {
				res += ftdi_read_data(&fc, &txbuf[i],
				    txchunklen - res);
			}
#endif
			if (res != txchunklen) {
#ifdef WIN32
				fprintf(stderr, "FT_Read() failed: "
				    "expected %d, received %d bytes\n",
				    txchunklen, res);
#else
				fprintf(stderr, "ftdi_read_data() failed\n");
#endif
				return (EXIT_FAILURE);
			}
		}
	}
	txpos = 0;

	/* Schedule CBUS LED blinking */
	i = ms_uptime();
	if (i - last_ledblink_ms >= LED_BLINK_RATE) {
		last_ledblink_ms += LED_BLINK_RATE;
		need_led_blink = 1;
	}

	return (0);
}


#ifdef USE_PPI
static int
commit_ppi(void)
{
	int i, val;

	for (i = 0; i < txpos; i++) {
		val = txbuf[i];
		if (port_mode == PORT_MODE_SYNC && !(i & 1))  {
			ioctl(ppi, PPIGSTATUS, &txbuf[i]);
		}
		ioctl(ppi, PPISDATA, &val);
	}

	txpos = 0;
	return (0);
}
#endif


static int
commit(int force)
{

	if (txpos == 0 || (!force && port_mode != PORT_MODE_SYNC &&
	    txpos < sizeof(txbuf) / 2))
		return (0);

#ifdef USE_PPI
	if (cable_hw == CABLE_HW_PPI)
		return (commit_ppi());
#endif
	if (cable_hw == CABLE_HW_USB)
		return (commit_usb());
	else
		return (EINVAL);
}


static int
str2tapstate(char *str)
{
	int i;

	for (i = 0; tap_statetable[i].state_str != NULL; i++) {
		if (strcmp(str, tap_statetable[i].state_str) == 0)
			break;
	}
	return (tap_statetable[i].state_id);
}


static void
set_state(int tgt_s) {
	int i, res = 0;

	switch (tgt_s) {
	case RESET:
		for (i = 0; i < 6; i++)
			set_tms_tdi(1, 0);
		break;

	case IDLE:
		switch (cur_s) {
		case RESET:
		case DRUPDATE:
		case IRUPDATE:
		case IDLE:
			set_tms_tdi(0, 0);
			break;

		case UNDEFINED:
			set_state(RESET);
			set_state(IDLE);
			break;

		case DRPAUSE:
			set_state(DREXIT2);
			set_state(DRUPDATE);
			set_state(IDLE);
			break;

		case IRPAUSE:
			set_state(IREXIT2);
			set_state(IRUPDATE);
			set_state(IDLE);
			break;

		default:
			res = -1;
		}
		break;

	case DRSELECT:
		switch (cur_s) {
		case IDLE:
		case DRUPDATE:
		case IRUPDATE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case DRCAPTURE:
		switch (cur_s) {
		case DRSELECT:
			set_tms_tdi(0, 0);
			break;

		case IDLE:
			set_state(DRSELECT);
			set_state(DRCAPTURE);
			break;

		case IRPAUSE:
			set_state(IDLE);
			set_state(DRSELECT);
			set_state(DRCAPTURE);
			break;

		default:
			res = -1;
		}
		break;

	case DREXIT1:
		switch (cur_s) {
		case DRCAPTURE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case DRPAUSE:
		switch (cur_s) {
		case DREXIT1:
			set_tms_tdi(0, 0);
			break;

		case IDLE:
			set_state(DRSELECT);
			set_state(DRCAPTURE);
			set_state(DREXIT1);
			set_state(DRPAUSE);
			break;

		case IRPAUSE:
			set_state(IREXIT2);
			set_state(IRUPDATE);
			set_state(DRSELECT);
			set_state(DRCAPTURE);
			set_state(DREXIT1);
			set_state(DRPAUSE);
			break;

		case DRPAUSE:
			break;

		default:
			res = -1;
		}
		break;

	case DREXIT2:
		switch (cur_s) {
		case DRPAUSE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case DRUPDATE:
		switch (cur_s) {
		case DREXIT2:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case IRSELECT:
		switch (cur_s) {
		case DRSELECT:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case IRCAPTURE:
		switch (cur_s) {
		case IRSELECT:
			set_tms_tdi(0, 0);
			break;

		case IDLE:
			set_state(DRSELECT);
			set_state(IRSELECT);
			set_state(IRCAPTURE);
			break;

		case DRPAUSE:
			set_state(DREXIT2);
			set_state(DRUPDATE);
			set_state(DRSELECT);
			set_state(IRSELECT);
			set_state(IRCAPTURE);
			break;

		default:
			res = -1;
		}
		break;

	case IREXIT1:
		switch (cur_s) {
		case IRCAPTURE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case IRPAUSE:
		switch (cur_s) {
		case IREXIT1:
			set_tms_tdi(0, 0);
			break;

		case IDLE:
			set_state(DRSELECT);
			set_state(IRSELECT);
			set_state(IRCAPTURE);
			set_state(IREXIT1);
			set_state(IRPAUSE);
			break;

		case DRPAUSE:
			set_state(DREXIT2);
			set_state(DRUPDATE);
			set_state(DRSELECT);
			set_state(IRSELECT);
			set_state(IRCAPTURE);
			set_state(IREXIT1);
			set_state(IRPAUSE);
			break;

		case IRPAUSE:
			break;

		default:
			res = -1;
		}
		break;

	case IREXIT2:
		switch (cur_s) {
		case IRPAUSE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case IRUPDATE:
		switch (cur_s) {
		case IREXIT2:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	default:
		res = -1;
	}

	if (res) {
		fprintf(stderr, "Don't know how to proceed: %s -> %s\n",
		    STATE2STR(cur_s), STATE2STR(tgt_s));
		if (cable_hw == CABLE_HW_USB)
			shutdown_usb();
		exit(EXIT_FAILURE);
	}

	cur_s = tgt_s;
}


static int
exec_svf_tokenized(int tokc, char *tokv[])
{
	static int last_sdr = PORT_MODE_UNKNOWN;
	int cmd, i, res = 0;
	int repeat = 1, delay_ms = 0;

	for (i = 0; svf_cmdtable[i].cmd_str != NULL; i++) {
		if (strcmp(tokv[0], svf_cmdtable[i].cmd_str) == 0)
			break;
	}

	cmd = svf_cmdtable[i].cmd_id;
	switch (cmd) {
	case SVF_SDR:
	case SVF_SIR:
		if (tokc == 4) {
			if (cmd == SVF_SDR && last_sdr == PORT_MODE_ASYNC)
				set_port_mode(PORT_MODE_ASYNC);
			tokv[5] = NULL;
			tokv[7] = NULL;
			if (cmd == SVF_SDR)
				last_sdr = PORT_MODE_ASYNC;
		} else if (tokc == 6 || tokc == 8) {
			set_port_mode(PORT_MODE_SYNC);
			if (tokc == 5)
				tokv[7] = NULL;
			if (cmd == SVF_SDR)
				last_sdr = PORT_MODE_SYNC;
		} else {
			res = EXIT_FAILURE;
			break;
		}
		if (cmd == SVF_SDR) {
			set_state(DRPAUSE);
			res = send_dr(atoi(tokv[1]), tokv[3], tokv[5], tokv[7]);
		} else {
			set_state(IRPAUSE);
			res = send_ir(atoi(tokv[1]), tokv[3], tokv[5], tokv[7]);
		}
		if (res)
			break;
		if ((tokc == 6 || tokc == 8) && strcmp(tokv[3], tokv[5]) != 0) {
			if (strlen(tokv[3]) == 8 && strlen(tokv[5]) == 8 &&
			    strcmp(tokv[7], "FFFFFFFF") == 0 &&
			    cmp_chip_ids(tokv[3], tokv[5]) == 0)
				return (ENODEV);
			fprintf(stderr, "Received and expected data "
			    "do not match!\n");
			if (tokc == 6)
				fprintf(stderr, "TDO: %s Expected: %s\n",
				    tokv[3], tokv[5]);
			if (tokc == 8)
				fprintf(stderr, "TDO: %s Expected: %s "
				    "mask: %s\n", tokv[3], tokv[5], tokv[7]);
			res = EXIT_FAILURE;
		}
		break;

	case SVF_STATE:
		set_state(str2tapstate(tokv[1]));
		res = commit(0);
		break;

	case SVF_RUNTEST:
		for (i = 2; i < tokc; i += 2) {
			if (strcmp(tokv[i + 1], "TCK") == 0) {
				repeat = atoi(tokv[i]);
				if (repeat < 1 || repeat > 1000) {
					fprintf(stderr,
					    "Unexpected token: %s\n",
					    tokv[i]);
					res = EXIT_FAILURE;
					break;
				}
			} else if (strcmp(tokv[i + 1], "SEC") == 0) {
				float f;
				sscanf(tokv[i], "%f", &f);
				delay_ms = (f + 0.0005) * 1000;
				if (delay_ms < 1 || delay_ms > 120000) {
					fprintf(stderr,
					    "Unexpected token: %s\n",
					    tokv[i]);
					res = EXIT_FAILURE;
					break;
				}
				/* Silently reduce insanely long waits */
				if (delay_ms > 3000)
					delay_ms = 3000;
			} else {
				fprintf(stderr, "Unexpected token: %s\n",
				    tokv[i + 1]);
				res = EXIT_FAILURE;
				break;
			}
		}
		set_state(str2tapstate(tokv[1]));
		i = delay_ms * (USB_BAUDS / 2000);
#ifdef USE_PPI
		/* libftdi is relatively slow in sync mode on FreeBSD */
		if (port_mode == PORT_MODE_SYNC && i > USB_BUFLEN_SYNC / 2)
			i /= 2;
#endif
		if (i > repeat)
			repeat = i;
		for (i = 0; i < repeat; i++) {
			txbuf[txpos++] = 0;
			txbuf[txpos++] = USB_TCK;
			if (txpos >= sizeof(txbuf) / 2) {
				commit(0);
				if (need_led_blink)
					set_port_mode(port_mode);
			}
		}
		break;

	case SVF_HDR:
	case SVF_HIR:
	case SVF_TDR:
	case SVF_TIR:
		if (tokc != 2 || strcmp(tokv[1], "0") != 0)
			res = EINVAL;
		break;

	case SVF_ENDDR:
		if (tokc != 2 ||
		    (strcmp(tokv[1], "DRPAUSE") != 0 &&
		    strcmp(tokv[1], "IDLE") != 0))
			res = EINVAL;
		break;

	case SVF_ENDIR:
		if (tokc != 2 ||
		    (strcmp(tokv[1], "IRPAUSE") != 0 &&
		    strcmp(tokv[1], "IDLE") != 0))
			res = EINVAL;
		break;

	case SVF_FREQUENCY:
		/* Silently ignored. */
		break;

	default:
		res = EOPNOTSUPP;
	}

	return (res);
}


enum jed_states {
	JED_INIT, JED_PACK_KNOWN, JED_SIZE_KNOWN, JED_PROG_INITIATED,
	JED_FUSES, JED_FUSES_DONE, JED_SED_CRC, JED_HAVE_SED_CRC, JED_USER_SIG
};

enum jed_target {
	JED_TGT_SRAM, JED_TGT_FLASH, JED_TGT_UNKNOWN
};

static struct jed_devices {
	char	*name;
	int	id;
	int	fuses;
	int	col_width;
	int	row_width;
} jed_devices[] = {
	{
		.name =		"LFXP2-5E",
		.id =		0x01299043,
		.fuses =	1236476,
		.col_width =	638,
		.row_width =	1938,
	},
	{
		.name =		"LFXP2-8E",
		.id =		0x0129A043,
		.fuses =	1954736,
		.col_width =	772,
		.row_width=	2532,
	},
	{
		.name =		"LFXP2-17E",
		.id =		0x0129B043,
		.fuses =	3627704,
		.col_width =	2188,
		.row_width =	1658,
	},
	{
		.name =		"LFXP2-30E",
		.id =		0x0129D043,
		.fuses =	5954320,
		.col_width =	2644,
		.row_width =	2532,
	},
	{
		.name =		"LFXP2-40E",
		.id =		0x0129E043,
		.fuses =	8304368,
		.col_width =	3384,
		.row_width =	2454,
	},
	{
		.name =		"LFE5U-12F",
		.id =		0x21111043,
		.fuses =	5681848,
		.col_width =	592,
		.row_width =	7562,
	},
	{
		.name =		"LFE5U-25F",
		.id =		0x41111043,
		.fuses =	5681848,
		.col_width =	592,
		.row_width =	7562,
	},
	{
		.name =		"LFE5U-45F",
		.id =		0x41112043,
		.fuses =	10208312,
		.col_width =	848,
		.row_width =	9470,
	},
	{
		.name =		"LFE5U-85F",
		.id =		0x41113043,
		.fuses =	19244856,
		.col_width =	1136,
		.row_width =	13294,
	},
	{
		.name =		"LFE5UM-25F",
		.id =		0x01111043,
		.fuses =	5681848,
		.col_width =	592,
		.row_width =	7562,
	},
	{
		.name =		"LFE5UM-45F",
		.id =		0x01112043,
		.fuses =	10208312,
		.col_width =	848,
		.row_width =	9470,
	},
	{
		.name =		"LFE5UM-85F",
		.id =		0x01113043,
		.fuses =	19244856,
		.col_width =	1136,
		.row_width =	13294,
	},
	{NULL, 0, 0}
};


static int
cmp_chip_ids(char *got, char *exp)
{
	int got_id, exp_id;
	struct jed_devices *got_jd, *exp_jd;

	sscanf(got, "%x", &got_id);
	sscanf(exp, "%x", &exp_id);

	for (got_jd = jed_devices; got_jd->name != NULL; got_jd++)
		if (got_jd->id == got_id)
			break;
	for (exp_jd = jed_devices; exp_jd->name != NULL; exp_jd++)
		if (exp_jd->id == exp_id)
			break;

	if (exp_jd->name == NULL && got_jd->name == NULL)
		return (EXIT_FAILURE);

	fprintf(stderr, "\nFound ");
	if (got_jd->name)
		fprintf(stderr, "%s", got_jd->name);
	else
		fprintf(stderr, "unknown (%s)", got);
	fprintf(stderr, " device, but the bitstream is for ");
	if (exp_jd->name)
		fprintf(stderr, "%s", exp_jd->name);
	else
		fprintf(stderr, "unknown (%s)", exp);
	fprintf(stderr, ".\n");
	return (0);
}

/*
 * Parse a Lattice XP2 JEDEC file and convert it into a SVF stream stored
 * in a contiguos chunk of memory.  If parsing is sucessfull proceed with
 * calling exec_svf_mem().
 */
static int
exec_jedec_file(char *path, int target, int debug)
{
	char *inbuf, *outbuf, *incp, *outcp;
	char tmpbuf[2048];
	FILE *fd;
	long flen;
	int jed_state = JED_INIT;
	int jed_dev = -1;
	int i, j, val, row, res;

	fd = fopen(path, "r");
	if (fd == NULL) {
		fprintf(stderr, "open(%s) failed\n", path);
		return (EXIT_FAILURE);
	}

	fseek(fd, 0, SEEK_END);
	flen = ftell(fd);
	fseek(fd, 0, SEEK_SET);

	inbuf = malloc(flen);
	outbuf = malloc(flen * 2); /* XXX rough estimate */
	if (inbuf == NULL || outbuf == NULL) {
		fprintf(stderr, "malloc(%ld) failed\n", flen);
		return (EXIT_FAILURE);
	}

	incp = inbuf;
	outcp = outbuf;
	while (!feof(fd) && fgets(incp, flen, fd) != NULL) {
		/* Trim CR / LF chars from the tail of the line */
		incp += strlen(incp) - 1;
		while (incp >= inbuf && (*incp == 10 || *incp == 13))
			incp--;
		incp[1] = 0;

		/* Is this the first line of an "L" command? */
		if (*inbuf == 'L') {
			if (jed_state < JED_PROG_INITIATED) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}
			if (jed_state == JED_PROG_INITIATED)
				jed_state = JED_FUSES;
			else
				jed_state = JED_SED_CRC;
			incp = inbuf;
			continue;
		}

		/* Does the command terminate on this line? */
		if (*incp != '*') {
			incp++;
			continue;
		} else
			*incp = 0;

		/* Is this the SED_CRC fuses string? */
		if (jed_state == JED_SED_CRC) {
			val = 0;
			for (i = 32, j = 0; i > 0; i--, val <<= 1) {
				val += (inbuf[i - 1] == '1');
				if ((i & 0x3) == 1) {
					if (val < 10)
						tmpbuf[j++] = '0' +
						    val;
					else
						tmpbuf[j++] = 'A' +
						    val - 10;
					val = 0;
				}
			}
			tmpbuf[j++] = 0;
			if (strlen(tmpbuf) != 8) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}
			jed_state = JED_HAVE_SED_CRC;
		}

		/* Is this the main fuses string? */
		if (jed_state == JED_FUSES) {

			outcp += sprintf(outcp, "\n\n! Program Fuse Map\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (21);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-002 SEC;\n");
			*outcp++ = 0;

			if (target == JED_TGT_SRAM) {
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (67);\n");
				*outcp++ = 0;
			}

			for (incp = inbuf, row = 1;
			    row <= jed_devices[jed_dev].row_width; row++) {
				if (target == JED_TGT_FLASH) {
					outcp += sprintf(outcp,
					    "SIR	8	TDI  (67);\n");
					*outcp++ = 0;
				}

				val = 0;
				for (i = jed_devices[jed_dev].col_width, j = 0;
				    i > 0; i--, val <<= 1) {
					val += (incp[i - 1] == '1');
					if ((i & 0x3) == 1) {
						if (val < 10)
							tmpbuf[j++] = '0' +
							    val;
						else
							tmpbuf[j++] = 'A' +
							    val - 10;
						val = 0;
					}
				}
				tmpbuf[j++] = 0;
				incp += jed_devices[jed_dev].col_width;

				outcp += sprintf(outcp,
				    "! Shift in Data Row = %d\n", row);
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	%d	TDI  (%s);\n",
				    jed_devices[jed_dev].col_width, tmpbuf);
				*outcp++ = 0;
				if (target == JED_TGT_FLASH) {
					outcp += sprintf(outcp,
					    "RUNTEST	IDLE"
					    "	3 TCK	1.00E-003 SEC;\n");
				} else {
					outcp += sprintf(outcp,
					    "RUNTEST	IDLE	3 TCK;\n");
				}
				*outcp++ = 0;

				if (target == JED_TGT_FLASH) {
					outcp += sprintf(outcp,
					    "SIR	8	TDI  (52);\n");
					*outcp++ = 0;

					outcp += sprintf(outcp,
					    "SDR	1	TDI  (0)\n");
					*outcp++ = 0;
					outcp += sprintf(outcp,
					    "		TDO  (1);\n");
					*outcp++ = 0;
				}
			}

			/* Check that we have consumed all fuse bits */
			if (strlen(incp) != 0) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}

			jed_state++;
		}
		
		/* Is this a comment line? */
		if (*inbuf == 'N') {
			if (jed_state == JED_INIT) {
				outcp += sprintf(outcp, "! %s\n", inbuf);
				*outcp++ = 0;
			}
			if (strncmp(inbuf, "NOTE DEVICE NAME:", 17) == 0) {
				incp = &inbuf[18];
				for (jed_dev = 0;
				    jed_devices[jed_dev].name != NULL;
				    jed_dev++) {
					if (strncmp(jed_devices[jed_dev].name,
					    incp, strlen(
					    jed_devices[jed_dev].name)) == 0)
						break; 
				}
				if (jed_devices[jed_dev].name == NULL) {
					fprintf(stderr, "Bitstream for "
					    "unsupported target: %s\n", incp);
					return (EXIT_FAILURE);
				}
			}
			incp = inbuf;
			continue;
		}

		/* Packaging line? */
		if (*inbuf == 'Q') {
			i = atoi(&inbuf[2]);
			if (inbuf[1] == 'P') {
				if (jed_dev < 0 || jed_state != JED_INIT) {
					fprintf(stderr,
					    "Invalid bitstream file\n");
					return (EXIT_FAILURE);
				}
				jed_state = JED_PACK_KNOWN;
			} else if (inbuf[1] == 'F') {
				if (jed_dev < 0 || jed_state != JED_PACK_KNOWN
				    || jed_devices[jed_dev].fuses != i) {
					fprintf(stderr,
					    "Invalid bitstream file\n");
					return (EXIT_FAILURE);
				}
				jed_state = JED_SIZE_KNOWN;
			} else {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}
		}

		/* "F" line? */
		if (*inbuf == 'F') {
			if (jed_state != JED_SIZE_KNOWN) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}
			jed_state = JED_PROG_INITIATED;

			outcp += sprintf(outcp, "\n\n! Check the IDCODE\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "STATE	RESET;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "STATE	IDLE;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (16);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "SDR	32	TDI  (FFFFFFFF)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		TDO  (%08X)\n",
			    jed_devices[jed_dev].id);
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "		MASK (FFFFFFFF);\n");
			*outcp++ = 0;

			if (target == JED_TGT_SRAM) {
				outcp += sprintf(outcp,
				    "\n\n! Program Bscan register\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (1C);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "STATE	DRPAUSE;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "STATE	IDLE;\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "\n\n! Enable SRAM programming mode\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (55);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "\n\n! Erase the device\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (03);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
			} else {
				outcp += sprintf(outcp,
				    "\n\n! Enable XPROGRAM mode\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (35);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "\n\n! Check the Key Protection fuses\n\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (10);\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (40);\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (04);\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "\n\n! Erase the device\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (03);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.20E+002 SEC;\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (52);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	1	TDI  (0)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (1);\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (01);\n");
				*outcp++ = 0;
			}
		}

		/* "U" line? */
		if (*inbuf == 'U') {
			if (inbuf[1] != 'H' || jed_state != JED_HAVE_SED_CRC) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}

			outcp += sprintf(outcp, "\n\n! Program USERCODE\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (1A);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "SDR	32	TDI  (%s);\n", &inbuf[2]);
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-002 SEC;\n");
			*outcp++ = 0;

			if (target == JED_TGT_FLASH) {
				outcp += sprintf(outcp,
				    "\n\n! Read the status bit;\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (01);\n");
				*outcp++ = 0;
			}

			outcp += sprintf(outcp,
			    "\n\n! Program and Verify 32 bits SED_CRC\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (45);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "SDR	32	TDI  (%s);\n", tmpbuf);
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-002 SEC;\n");
			*outcp++ = 0;

			outcp += sprintf(outcp, "SIR	8	TDI  (44);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-003 SEC;\n");
			*outcp++ = 0;

			outcp += sprintf(outcp,
			    "SDR	32	TDI  (00000000)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "		TDO  (%s);\n", tmpbuf);
			*outcp++ = 0;

			outcp += sprintf(outcp, "SIR	8	TDI  (B2);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-003 SEC;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SDR	8	TDI  (00)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		TDO  (00)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		MASK (01);\n");
			*outcp++ = 0;

			outcp += sprintf(outcp,
			    "\n\n! Program DONE bit\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (2F);\n");
			*outcp++ = 0;
			if (target == JED_TGT_FLASH) {
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	2.00E-001 SEC;\n");
			} else {
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK;\n");
			}
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (B2);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-003 SEC;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SDR	8	TDI  (00)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		TDO  (02)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		MASK (03);\n");
			*outcp++ = 0;

			if (target == JED_TGT_FLASH) {
				outcp += sprintf(outcp,
				    "\n\n! Verify DONE bit\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (FF)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (04);\n");
				*outcp++ = 0;
			}

			outcp += sprintf(outcp,
			    "\n\n! Exit the programming mode\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (1E);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	2.00E-003 SEC;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (FF);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-003 SEC;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "STATE	RESET;\n");
			*outcp++ = 0;
		}

		incp = inbuf;
	}
	fclose(fd);

	/* Count number of lines in outbuf, store in j */
	for (i = 0, j = 0; outcp > outbuf; outcp--)
		if (*outcp == 0)
			j++;

	if (svf_name) {
		int of;

		if (strncmp(svf_name, "-", 1) == 0)
			of = 0;
		else
			of = open(svf_name, O_RDWR | O_CREAT | O_TRUNC, 0644);
		if (of < 0) {
			res = errno;
			goto done;
		}
		res = 0;
		for (i = 0, outcp = outbuf; i < j; i++) {
			write(of, outcp, strlen(outcp));
			outcp += (strlen(outcp) + 1);
		}
		if (of)
			close(of);
	} else
		res = exec_svf_mem(outbuf, j, debug);

done:
	free(outbuf);
	free(inbuf);
	return (res);
}


#define	bitrev(a) ((a & 0x1)  << 7) | ((a & 0x2)  << 5) | ((a & 0x4)  << 3) | ((a & 0x8)  << 1) | ((a & 0x10) >> 1) | ((a & 0x20) >> 3) | ((a & 0x40) >> 5) | ((a & 0x80) >> 7)

#define	buf_sprintf(p, ...) do { \
	(p) += sprintf((p), ## __VA_ARGS__); \
	 *(p)++ = 0; \
} while (0)

/*
 * Parse a Lattice ECP5 bitstream file and convert it into a SVF stream,
 * stored in a contiguos chunk of memory.  If parsing is sucessfull proceed
 * with calling exec_svf_mem().
 */
static int
exec_bit_file(char *path, int jed_target, int debug)
{
	uint8_t *inbuf;
	char *outbuf, *op;
	char *outcp;
	FILE *fd;
	long flen, got;
	uint32_t idcode;
	int i, j, n, addr;
	int row_size = 64000 / 8;
	int hexlen = 50;
	int res;

	fd = fopen(path, "rb");
	if (fd == NULL) {
		fprintf(stderr, "open(%s) failed\n", path);
		return (EXIT_FAILURE);
	}

	fseek(fd, 0, SEEK_END);
	flen = ftell(fd);
	fseek(fd, 0, SEEK_SET);

	inbuf = malloc(flen);
	outbuf = malloc(flen * 4); /* XXX rough estimate */
	if (inbuf == NULL || outbuf == NULL) {
		fprintf(stderr, "malloc(%ld) failed\n", flen);
		return (EXIT_FAILURE);
	}
	op = outbuf;

	got = fread(inbuf, 1, flen, fd);
	if (got != flen) {
		fprintf(stderr, "short read: %ld instead of %ld\n",
		    got, flen);
		return (EXIT_FAILURE);
	}

	buf_sprintf(op, "STATE IDLE;\n");
	buf_sprintf(op, "STATE RESET;\n");
	buf_sprintf(op, "STATE IDLE;\n\n");

	if (strcasecmp(&path[strlen(path) - 4], ".img") != 0) {
		/* Search for bitstream preamble and IDCODE markers */
		for (i = 0, j = 0; i < flen - 32 && i < 2000; i++)
			if (inbuf[i] == 0xbd && inbuf[i + 1] == 0xb3
			    && inbuf[i + 10] == 0xe2 && inbuf[i + 11] == 0
			    && inbuf[i + 12] == 0 && inbuf[i + 13] == 0) {
				j = i;
				break;
			}
		if (j == 0) {
			fprintf(stderr,
			    "can't find IDCODE, invalid bitstream\n");
			return (EXIT_FAILURE);
		}
		idcode = inbuf[i + 14] << 24;
		idcode += inbuf[i + 15] << 16;
		idcode += inbuf[i + 16] << 8;
		idcode += inbuf[i + 17];

		/* IDCODE_PUB(0xE0): check IDCODE */
		buf_sprintf(op, "SIR	8	TDI	(E0);\n");
		buf_sprintf(op, "SDR	32	TDI	(00000000)\n");
		buf_sprintf(op, "	TDO	(%08X)\n", idcode);
		buf_sprintf(op, "	MASK	(FFFFFFFF);\n\n");
	}

	/* LSC_PRELOAD(0x1C): Program Bscan register */
	buf_sprintf(op, "SIR	8	TDI	(1C);\n");
	buf_sprintf(op, "SDR	510	TDI	(3FFFFFFFFFFFFFF"
	    "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
	    "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF);\n\n");

	/* ISC ENABLE(0xC6): Enable SRAM programming mode */
	buf_sprintf(op, "SIR	8	TDI	(C6);\n");
	buf_sprintf(op, "SDR	8	TDI	(00);\n");
	buf_sprintf(op, "RUNTEST IDLE    2 TCK;\n\n");

	/* ISC ERASE(0x0e): Erase the SRAM */
	buf_sprintf(op, "SIR	8	TDI	(0e);\n");
	buf_sprintf(op, "SDR	8	TDI	(01);\n");
	buf_sprintf(op, "RUNTEST IDLE	32 TCK 1.00E-01 SEC;\n\n");

	/* LSC_READ_STATUS(0x3c) */
	buf_sprintf(op, "SIR	8	TDI	(3C);\n");
	buf_sprintf(op, "SDR	32	TDI	(00000000)\n");
	buf_sprintf(op, "	TDO	(00000000)\n");
	buf_sprintf(op, "	MASK	(0000B000);\n\n");

	if (jed_target == JED_TGT_FLASH) {
		buf_sprintf(op, "STATE RESET;\n");
		buf_sprintf(op, "STATE IDLE;\n");

		/* BYPASS(0xFF) */
		buf_sprintf(op, "SIR	8	TDI(FF);\n");
		buf_sprintf(op, "RUNTEST IDLE	32 TCK;\n");

		/* LSC_PROG_SPI(0x3A) */
		buf_sprintf(op, "SIR	8	TDI(3A);\n");
		buf_sprintf(op, "SDR	16	TDI(68FE);\n");
		buf_sprintf(op, "RUNTEST IDLE	32 TCK;\n\n");

		/* Erase sectors */
		for (i = 0; i < flen; i += SPI_SECTOR_SIZE) {
			addr = i + spi_addr;

			/* SPI write enable */
			buf_sprintf(op, "SDR	8	TDI(60);\n");

			/* Read status register (some chips won't clear WIP without this) */
			buf_sprintf(op, "SDR	16	TDI(00A0)\n");
			buf_sprintf(op, "	TDO(40FF)\n");
			buf_sprintf(op, "	MASK(C100);\n\n");

			buf_sprintf(op, "SDR	32	TDI(0000%02x1B);\n",
			    bitrev(addr / SPI_SECTOR_SIZE));
			buf_sprintf(op, "RUNTEST DRPAUSE 5.50E-01 SEC;\n");

			/* Read status register */
			buf_sprintf(op, "SDR	16	TDI(00A0)\n");
			buf_sprintf(op, "	TDO(00FF)\n");
			buf_sprintf(op, "	MASK(C100);\n\n");
		}

		/* SPI write disable */
		buf_sprintf(op, "SDR	8	TDI(20);\n\n");

		row_size = SPI_PAGE_SIZE;
	} else {
		/* LSC_INIT_ADDRESS(0x46) */
		buf_sprintf(op, "SIR	8	TDI	(46);\n");
		buf_sprintf(op, "SDR	8	TDI	(01);\n");
		buf_sprintf(op, "RUNTEST IDLE    2 TCK;\n\n");

		/* LSC_BITSTREAM_BURST(0x7a) */
		buf_sprintf(op, "SIR	8	TDI	(7A);\n");
		buf_sprintf(op, "RUNTEST IDLE    2 TCK;\n\n");
	}

	for (i = 0; i < flen; i += row_size) {
		n = flen - i;
		if (n > row_size)
			n = row_size;
		if (jed_target == JED_TGT_FLASH) {
			/* Skip write if all bits set in a block */
			for (j = 0; j < n; j++)
				if (inbuf[i + j] != 0xff)
					break;
			if (j == n)
				continue;

			buf_sprintf(op, "SDR	8	TDI(60);\n");
			buf_sprintf(op, "SDR %d TDI (", n * 8 + 32);
		} else
			buf_sprintf(op, "SDR %d TDI (", n * 8);
		do {
			n--;
			op += sprintf(op, "%02X", bitrev(inbuf[i + n]));
			if (n % hexlen == 0 && n > 0)
				buf_sprintf(op, "\n\t");
		} while (n > 0);
		if (jed_target == JED_TGT_FLASH) {
			addr = i + spi_addr;

			buf_sprintf(op, "%02x%02x%02x40);\n\n",
			    bitrev(addr % 256), bitrev((addr / 256) % 256),
			    bitrev((addr / 65536) % 256));
			buf_sprintf(op, "RUNTEST DRPAUSE 2.00E-03 SEC;\n");
			buf_sprintf(op, "SDR	16	TDI(00A0)\n");
			buf_sprintf(op, "	TDO(00FF)\n");
			buf_sprintf(op, "	MASK(C100);\n\n");
		} else
			buf_sprintf(op, ");\n\n");
	}
	
	/* BYPASS(0xFF) */
	buf_sprintf(op, "SIR	8	TDI	(FF);\n");
	buf_sprintf(op, "RUNTEST IDLE    100 TCK;\n\n");

	/* ISC DISABLE(Ox26): exit the programming mode */
	buf_sprintf(op, "SIR	8	TDI	(26);\n");
	buf_sprintf(op, "RUNTEST IDLE    2 TCK   2.00E-03 SEC;\n\n");
	buf_sprintf(op, "SIR	8	TDI	(FF);\n");
	buf_sprintf(op, "RUNTEST IDLE    2 TCK   1.00E-03 SEC;\n\n");

	if (jed_target == JED_TGT_FLASH) {
		/* LSC_REFRESH(0x79) */
		buf_sprintf(op, "SIR	8	TDI	(79);\n");
		buf_sprintf(op, "SDR	24	TDI	(000000);\n");
		buf_sprintf(op, "RUNTEST IDLE    2 TCK   1.00E-01 SEC;\n\n");
	} else {
		/* LSC_READ_STATUS(0x3c): verify status register */
		buf_sprintf(op, "SIR	8	TDI	(3C);\n");
		buf_sprintf(op, "SDR	32	TDI	(00000000)\n");
		buf_sprintf(op, "	TDO	(00000100)\n");
		buf_sprintf(op, "	MASK	(00002100);\n\n");
	}
	op--;
	i = 0;
	do {
		if (*op == '\n')
			i++;
	} while (op-- != outbuf);

	if (svf_name) {
		int of;

		if (strncmp(svf_name, "-", 1) == 0)
			of = 0;
		else
			of = open(svf_name, O_RDWR | O_CREAT | O_TRUNC, 0644);
		if (of < 0) {
			res = errno;
			return(res);
		}
		res = 0;
		for (j = 0, outcp = outbuf; j < i; j++) {
			write(of, outcp, strlen(outcp));
			outcp += (strlen(outcp) + 1);
		}
		if (of)
			close(of);
	} else
		res = exec_svf_mem(outbuf, i, debug);

	free(outbuf);
	free(inbuf);
	return (res);
}


/*
 * Load a SVF file in a contiguos chunk of memory, count number of lines,
 * and then call exec_svf_mem().
 */
static int
exec_svf_file(char *path, int debug)
{
	char *linebuf, *fbuf;
	FILE *fd;
	long flen;
	int lines_tot = 1;
	int res;

	fd = fopen(path, "r");
	if (fd == NULL) {
		fprintf(stderr, "open(%s) failed\n", path);
		return (EXIT_FAILURE);
	}

	fseek(fd, 0, SEEK_END);
	flen = 2 * ftell(fd);
	fseek(fd, 0, SEEK_SET);

	fbuf = malloc(flen);
	if (fbuf == NULL) {
		fprintf(stderr, "malloc(%ld) failed\n", flen);
		return (EXIT_FAILURE);
	}

	for (linebuf = fbuf; !feof(fd); linebuf += strlen(linebuf) + 1) {
		if (fgets(linebuf, flen, fd) == NULL)
			break;
		lines_tot++;
		flen -= strlen(linebuf) + 1;
	}
	fclose(fd);
	*linebuf = 0;

	res = exec_svf_mem(fbuf, lines_tot, debug);
	free(fbuf);
	return (res);
}


/*
 * Parse SVF command lines stored in a contiguos chunk of memory and
 * execute appropriate JTAG actions, line by line, all in a single pass.
 */
static int
exec_svf_mem(char *fbuf, int lines_tot, int debug)
{
	char cmdbuf[128 * 1024];
	int lno, tokc, cmd_complete, parentheses_open;
	int res = 0;
	int llen = 0;
	char *cp, *c1;
	char *sep = " \t\n\r";
	char *linebuf, *item, *brkt;
	char *tokv[256];

	cp = cmdbuf;
	cmd_complete = 0;
	parentheses_open = 0;
	linebuf = fbuf;

	for (lno = 1; lno < lines_tot; lno++, linebuf += llen) {
		if (debug)
			printf("%d: %s", lno, linebuf);

		llen = strlen(linebuf) + 1;
		progress_perc = lno * 1005 / (lines_tot * 10);

		/* Pre-parse input, join multiple lines to a single command */
		for (item = strtok_r(linebuf, sep, &brkt); item;
		    item = strtok_r(NULL, sep, &brkt)) {
			/* Skip comments */
			if (item[0] == '!')
				break;
			if (item[0] == '/' && item[1] == '/')
				break;

			/* If command is complete we shouldn't end up here! */
			if (cmd_complete) {
				fprintf(stderr, "Line %d: multiple commands "
				    "on single line\n", lno);
				return (EXIT_FAILURE);
			}

			/* End of command? */
			c1 = item + strlen(item) - 1;
			if (*c1 == ';') {
				*c1-- = 0;
				cmd_complete = 1;
			}

			/* Check for parentheses */
			if (*item == '(') {
				item++;
				if (parentheses_open) {
					fprintf(stderr,
					    "Line %d: too many '('s\n", lno);
					return (EXIT_FAILURE);
				}
				parentheses_open = 1;
			}
			if (!parentheses_open)
				for (char *ct = item; ct < c1; ct++)
					if (*ct == '(') {
						*ct = ' ';
						parentheses_open = 1;
						break;
					}
			if (*c1 == ')') {
				*c1 = 0;
				if (!parentheses_open) {
					fprintf(stderr,
					    "Line %d: too many ')'s\n", lno);
					return (EXIT_FAILURE);
				}
				parentheses_open = 0;
			}

			/* Copy to command buffer */
			strcpy(cp, item);
			cp += strlen(item);
			if (!parentheses_open && !cmd_complete)
				*cp++ = ' ';
		}

		/* Proceed to next line if command is not complete yet */
		if (!cmd_complete)
			continue;

		/* Unmatched parentheses are not permitted */
		if (parentheses_open) {
			fprintf(stderr, "Line %d: missing ')'\n", lno);
			return (EXIT_FAILURE);
		}

		/* Normalize to all upper case letters, separate tokens */
		tokc = 0;
		tokv[0] = cmdbuf;
		for (cp = cmdbuf; *cp != 0; cp++) {
			if (*cp == ' ') {
				*cp++ = 0;
				tokc++;
				tokv[tokc] = cp;
			}
			*cp = toupper(*cp);
		}
		if (*tokv[tokc] != 0)
			tokc++;

		/* Execute command */
		res = exec_svf_tokenized(tokc, tokv);
		if (res) {
			if (res != ENODEV)
				fprintf(stderr, "Line %d: %s\n", lno,
				    strerror(res));
			return (res);
		}

		cp = cmdbuf;
		cmd_complete = 0;
	}

	/* Flush any buffered data */
	commit(1);

	return (res);
}


static void
terminal_help(void)
{

	printf(
	    "  ~>	send file\n"
	    "  ~b	change baudrate\n"
	    "  ~r	reprogram / reload "
		"the FPGA\n"
	    "  ~#	send a BREAK signal\n"
	    "  ~d	enter f32c debugger\n"
	    "  ~.	exit from ujprog\n"
	    "  ~?	get this summary\n"
	);
}


static void
usage(void)
{

	printf("Usage: ujprog [option(s)] [bitstream_file]\n\n");

	printf(" Valid options:\n");
#ifdef USE_PPI
	printf("  -c CABLE	Select USB (default) or PPI JTAG CABLE\n");
#endif
	printf("  -p PORT	Select USB JTAG / UART PORT (default is 0)\n");
#ifdef WIN32
	printf("  -P COM	Select COM port (valid only with -t or -a)\n");
#else
	printf("  -P TTY	Select TTY port (valid only with -t or -a)\n");
#endif
	printf("  -j TARGET	Select bitstream TARGET as SRAM (default)"
	    " or FLASH\n");
	printf("  -f ADDR	Start writing to SPI flash at ADDR, "
	    "optional with -j flash\n");
	printf("  -s FILE	Convert bitstream to SVF FILE and exit\n");
	printf("  -r		Reload FPGA configuration from"
	    " FLASH\n");
	printf("  -t		Enter terminal emulation mode after"
	    " completing JTAG operations\n");
	printf("  -b SPEED	Set baudrate to SPEED (300 to 3000000"
	    " bauds)\n");
	printf("  -e FILE	Send and execute a f32c (MIPS/RISCV) binary "
	    "FILE\n");
	printf("  -x SPEED	Set binary transfer speed, optional with -e\n");
	printf("  -a FILE	Send a raw FILE\n");
	printf("  -d 		debug (verbose)\n");
	printf("  -D DELAY	Delay transmission of each byte by"
	    " DELAY ms\n");
	printf("  -q 		Suppress messages\n");

	if (terminal) {
		printf("\n Terminal emulation mode commands:\n");
		terminal_help();
	}
#ifndef WIN32
	printf("\n");
#endif
}


static int
gets1(char *cp, int size)
{
	char *lp, *end;
	char c;
	int error = 0;

	lp = cp;
	end = cp + size - 1;
	for (;;) {
#ifdef WIN32
		c = getch() & 0177;
#else
		while (read(0, &c, 1) < 0)
			ms_sleep(10);
#endif
		switch (c) {
		case 3:	/* CTRL + C */
			error = -1;
		case '\n':
		case '\r':
			printf("\n");
			*lp = '\0';
			return (error);
		case '\b':
		case '\177':
			if (lp > cp) {
				printf("%c \b", c);
				fflush(stdout);
				lp--;
			}
			continue;
		case '\0':
			continue;
		default:
			if (lp < end) {
				printf("%c", c);
				fflush(stdout);
				*lp++ = c;
			}
		}
	}
}


static int
prog(char *fname, int target, int debug)
{
	int res, c, tstart, tend;

	tstart = ms_uptime();
	last_ledblink_ms = tstart;

	/* Move TAP into RESET state. */
	set_port_mode(PORT_MODE_ASYNC);
	set_state(RESET);

	commit(1);

	c = strlen(fname) - 4;
	if (c < 0) {
		usage();
		exit(EXIT_FAILURE);
	}
	if (strcasecmp(&fname[c], ".jed") == 0)
		res = exec_jedec_file(fname, target, debug);
	else if (strcasecmp(&fname[c], ".bit") == 0 ||
	    (strcasecmp(&fname[c], ".img") == 0 && target == JED_TGT_FLASH))
		res = exec_bit_file(fname, target, debug);
	else if (strcasecmp(&fname[c], ".svf") == 0)
		res = exec_svf_file(fname, debug);
	else
		res = -1;

	/* Leave TAP in RESET state. */
	set_port_mode(PORT_MODE_ASYNC);
	set_state(IDLE);
	set_state(RESET);
	commit(1);

	tend = ms_uptime();
	if (res == 0) {
		if (!quiet) {
			fprintf(stderr, "\rProgramming: 100%%  ");
			fprintf(stderr, "\nCompleted in %.2f seconds.\n",
			    (tend - tstart) / 1000.0);
		}
	} else
		fprintf(stderr, "\nFailed.\n");

	return (res);
}


#if 0
static void
reload_xp2_flash(int debug)
{
	char buf[128];
	char *c;

	if (!quiet)
		printf("Reconfiguring FPGA...\n");
	last_ledblink_ms = ms_uptime();
	need_led_blink = 0;

	/* Move TAP into RESET state. */
	set_port_mode(PORT_MODE_SYNC);
	set_state(IDLE);
	set_state(RESET);
	commit(1);

	/* Reset sequence */
	c = buf;
	c += sprintf(c, "RUNTEST IDLE 30 TCK;\n");
	*c++ = 0;
	c += sprintf(c, "SIR 8 TDI (1E);\n");
	*c++ = 0;
	c += sprintf(c, "SIR 8 TDI (23);\n");
	*c++ = 0;
	c += sprintf(c, "!\n");
	exec_svf_mem(buf, 4, debug);

	/* Leave TAP in RESET state. */
	set_state(IDLE);
	set_state(RESET);
	commit(1);
}
#endif


static int
async_read_block(int len)
{
	int res, got = 0, backoff = 0, backoff_lim = 5;
	int i;
#if defined(__FreeBSD__) || defined(__linux__)
	if (cable_hw == CABLE_HW_COM)
		backoff_lim = 10;
#endif
	do {
		if (cable_hw == CABLE_HW_USB) {
#ifdef WIN32
			DWORD ev_stat, avail;
			FT_GetStatus(ftHandle, &avail, &ev_stat, &ev_stat);
			if (avail > len - got)
				avail = len - got;
			if (avail)
				FT_Read(ftHandle, &rxbuf[got], avail,
				    (DWORD *) &res);
			else
				res = 0;
#else
			res = ftdi_read_data(&fc, &rxbuf[got], len - got);
#endif
		} else {
#ifdef WIN32
			DWORD n;
			n = len - got;
			if (n > 32)
				n = 32;
			ReadFile(com_port, &rxbuf[got], n,
			    (DWORD *) &res, NULL);
#else
			res = read(com_port, &rxbuf[got], len - got);
			if (res == -1)
				res = 0;
#endif
		}
		if (res > 0) {
			got += res;
			backoff = 0;
		} else {
			backoff++;
			ms_sleep(backoff * 4);
		}
	} while (got < len && backoff < backoff_lim);
        if(global_debug)
        {
		fprintf(stderr, "<");
		for(i = 0; i < got; i++)
			fprintf(stderr, " %02x", rxbuf[i]);
		fprintf(stderr, "\n");
	}
	return (got);
}


static int
async_send_block(int len)
{
	int sent;
	int i;

	if (cable_hw == CABLE_HW_USB) {
#ifdef WIN32
		FT_Write(ftHandle, txbuf, len, (DWORD *) &sent);
#else
		sent = ftdi_write_data(&fc, txbuf, len);
#endif
	} else {
#ifdef WIN32
		WriteFile(com_port, txbuf, len, (DWORD *) &sent, NULL);
#else
		fcntl(com_port, F_SETFL, 0);
		sent = write(com_port, txbuf, len);
		tcdrain(com_port); // flush data to hardware
		fcntl(com_port, F_SETFL, O_NONBLOCK);
#endif
	}
        if(global_debug)
        {
		fprintf(stderr, ">");
		for(i = 0; i < sent && i < 20; i++)
			fprintf(stderr, " %02x", txbuf[i]);
		if(sent >= 20)
			fprintf(stderr, "...");
		fprintf(stderr, "\n");
	}
	if (sent == len)
		return (0);
	else
		return (1);
}


static void
async_send_uint8(uint32_t data)
{

	txbuf[0] = data;
	async_send_block(1);
}


static void
async_send_uint32(uint32_t data)
{
	int i;

	for (i = 0; i < 4; i++) {
		txbuf[i] = (data >> 24);
		data <<= 8;
	}
	async_send_block(4);
}


static int
async_set_baudrate(int speed)
{

	if (cable_hw == CABLE_HW_USB) {
#ifdef WIN32
		FT_SetBaudRate(ftHandle, speed);
#else
		ftdi_set_baudrate(&fc, speed);
#endif
	} else {
#ifdef WIN32
		if (GetCommState(com_port, &tty) == 0) {
			fprintf(stderr, "%s is not a COM port\n", com_name);
			exit(EXIT_FAILURE);
		}
		tty.BaudRate = speed;
		tty.StopBits = 0;
		tty.Parity = 0;
		tty.ByteSize = 8;
		if (SetCommState(com_port, &tty) == 0) {
			fprintf(stderr, "Can't set baudrate to %d\n", speed);
			exit(EXIT_FAILURE);
		}
#else
		cfsetspeed(&tty, speed);
		if (tcsetattr(com_port, TCSAFLUSH, &tty) != 0) {
			fprintf(stderr, "Can't set baudrate to %d\n", speed);
			exit(EXIT_FAILURE);
		}
#endif
	}
	return (0);
}


static void
txfile(void)
{
	int tx_cnt, i, infile, res;
	int crc_retry;
	int tx_retry, tx_success;
	uint32_t rx_crc, local_crc, crc_i;
	uint32_t base, bootaddr;
	FILE *fd;
	uint8_t hdrbuf[16];

	if (tx_binary) {
		fd = fopen(txfname, "r");
		if (fd == NULL) {
			fprintf(stderr, "%s: cannot open\n", txfname);
			return;
		}
		i = fread(hdrbuf, 1, 16, fd);
		fseek(fd, 0, SEEK_END);
		// len = ftell(fd);
		fseek(fd, 0, SEEK_SET);
		fclose(fd);
		if (i != 16) {
			fprintf(stderr, "%s: short read\n", txfname);
			return;
		}
		if (hdrbuf[2] == 0x10 && hdrbuf[3] == 0x3c &&
		    hdrbuf[6] == 0x10 && hdrbuf[7] == 0x26 &&
		    hdrbuf[10] == 0x11 && hdrbuf[11] == 0x3c &&
		    hdrbuf[14] == 0x31 && hdrbuf[7] == 0x26) {
			/* MIPS, little-endian cookie found */
			base = (hdrbuf[1] << 24) + (hdrbuf[0] << 16)
			    + (hdrbuf[5] << 8) + hdrbuf[4];
			if (!quiet)
				printf("MIPS little-endian");
		} else if (hdrbuf[2] == 0x10 && hdrbuf[3] == 0x3c &&
		    hdrbuf[6] == 0x10 && hdrbuf[7] == 0x26 &&
		    hdrbuf[10] == 0x11 && hdrbuf[11] == 0x3c) {
			/* MIPS, big-endian cookie found */
			/* XXX fixme */
			fprintf(stderr, "%s: MIPS, big-endian UNSUPPORTED\n",
			    txfname);
			return;
		} else if ((hdrbuf[1] & 0xf) == 1 && hdrbuf[0] == 0x97 &&
		    hdrbuf[4] == 0x93 && hdrbuf[5] == 0x81) {
			/* RISC-V, little-endian cookie found */
			/* XXX hardcoded load address - fixme */
			base = 0x400;
			if (!quiet)
				printf("RISC-V (PIC)");
		} else {
			fprintf(stderr,
			    "invalid file type, missing header cookie\n");
			return;
		}
		bootaddr = base;
		if (!quiet)
			printf(" binary, loading at 0x%08x, "
			    "TX speed %d bauds\n", base, xbauds);
	}

	infile = open(txfname,
#ifdef WIN32
	    O_RDONLY | O_BINARY
#else
	    O_RDONLY
#endif
	    );
	if (infile < 0) {
		fprintf(stderr, "%s: cannot open\n", txfname);
		return;
	}

	async_set_baudrate(bauds);
	if (cable_hw == CABLE_HW_USB) {
		set_port_mode(PORT_MODE_UART);
#ifdef WIN32
		FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1,
		    FT_PARITY_NONE);
		FT_SetFlowControl(ftHandle, FT_FLOW_NONE, 0, 0);
		do {} while (FT_StopInTask(ftHandle) != FT_OK);
		ms_sleep(50);
		FT_Purge(ftHandle, FT_PURGE_RX);
		do {} while (FT_RestartInTask(ftHandle) != FT_OK);
#else
		ftdi_set_line_property(&fc, BITS_8, STOP_BIT_1, NONE);
		ftdi_setflowctrl(&fc, SIO_DISABLE_FLOW_CTRL);
		ftdi_usb_purge_buffers(&fc);
		ms_sleep(50);
#endif
	}

	/* Send a space mark to break into SIO loader prompt */
	async_send_uint8(' ');

	/* Wait for f32c ROM to catch up */
	if (tx_binary)
		ms_sleep(200);
	else
		ms_sleep(100);

	/* Prune any stale data from rx buffer */
	async_read_block(2048);

	if (tx_binary) {
		/* Start of binary transfer marker */
		async_send_uint8(255);

		async_send_uint8(0x80);	/* CMD: set base */
		async_send_uint32(xbauds);
		async_send_uint8(0xb0);	/* CMD: set baudrate */
		ms_sleep(50);
		async_set_baudrate(xbauds);
	}

	i = bauds / 300;
	if (bauds < 4800)
		i = 16;
	if (txfu_ms)
		i = 1;
	if (tx_binary)
		i = 8192;
	do {
		if (!quiet) {
			printf("%c ", statc[blinker_phase]);
			printf("\rSending %s: ", txfname);
			fflush(stdout);
			blinker_phase = (blinker_phase + 1) & 0x3;
		}
		res = read(infile, &txbuf[8192], i);
		if (!tx_binary && txfu_ms)
			ms_sleep(txfu_ms);
		if (res <= 0) {
			tx_cnt = 0;
		} else
			tx_cnt = res;

		if (tx_cnt == 0)
			break;

		if (tx_binary) {
		   tx_success = 0;
		   for(tx_retry = 0; tx_retry < 4 && tx_success == 0; tx_retry++)
		   {
			async_send_uint8(0x80);	/* CMD: set base */
			async_send_uint32(tx_cnt);
			async_send_uint8(0x90);	/* CMD: len = base */

			async_send_uint8(0x80);	/* CMD: set base */
			async_send_uint32(base);

			async_send_uint8(0xa0);	/* CMD: Write block */
			local_crc = 0;
			for (crc_i = 0; crc_i < tx_cnt; crc_i++) {
				local_crc =
				    (local_crc >> 31) | (local_crc << 1);
				txbuf[crc_i] = txbuf[crc_i + 8192];
				local_crc += txbuf[crc_i];
			}
			#if 0
			if(1) // intentionally damage tx packet to test CRC
			{
				// srandom(time(NULL)); // randomize seed, each run will be different
				if( (rand() % 256) > 100 ) // error probability 100/256
					txbuf[rand() % tx_cnt] = rand() % 0xFF; // error byte at random place
			}
			#endif
			if (async_send_block(tx_cnt)) {
				fprintf(stderr, "Block sending failed!\n");
				tx_cnt = -1;
				continue;
			}
			if(txfu_ms > 0)
				ms_sleep(txfu_ms);
			async_send_uint8(0x81); // read checksum
			res = 0;
			for(crc_retry = 4; crc_retry > 0 && res != 4; ms_sleep(10), crc_retry--)
			{
				res = async_read_block(4);
				if(crc_retry == 2 && res != 4)
					async_send_uint8(0x81); // try again to read checksum
			}
			if(res != 4)
			{
				fprintf(stderr, "Checksum not received: "
				    "got %d bytes, should be 4 (0x%08X)\n", res, local_crc);
				continue;
			}
			rx_crc = rxbuf[0] << 24;
			rx_crc += rxbuf[1] << 16;
			rx_crc += rxbuf[2] << 8;
			rx_crc += rxbuf[3];
			if (rx_crc != local_crc) {
				fprintf(stderr, "CRC error: "
				    "got 0x%08x, should be 0x%08x\n",
				    rx_crc, local_crc);
				continue;
			}
			else
			{
				tx_success = 1; // checksum ok
				tx_retry = 0; // reset number of retries
			}
		    }
		    if(tx_success == 0)
		    {
			tx_cnt = -1; // give up, no more retries retries
			break;
		    }
		} else {
			memcpy(txbuf, &txbuf[8192], tx_cnt);
			if (async_send_block(tx_cnt)) {
				fprintf(stderr, "Block sending failed!\n");
				tx_cnt = -1;
				break;
			}
			tx_success = 1;
		}
		if(tx_success)
			base += tx_cnt;
	} while (tx_cnt > 0);


	if (tx_cnt >= 0 && !quiet)
		printf("done.\n");
	close(infile);
	fflush(stdout);

	if (tx_success == 0)
		fprintf(stderr, "TX error at %08x\n", base);
	else if (tx_binary) {
		async_send_uint8(0x80);	/* CMD: set base */
		async_send_uint32(bauds);
		async_send_uint8(0xb0);	/* CMD: set baudrate */
		ms_sleep(50);
		async_set_baudrate(bauds);

		async_send_uint8(0x80);	/* CMD: set base */
		async_send_uint32(bootaddr);
		async_send_uint8(0xb1);	/* CMD: jump to base */
	}
}


static void
genbrk(void)
{

	if (cable_hw == CABLE_HW_USB) {
#ifdef WIN32
		FT_SetBreakOn(ftHandle);
		ms_sleep(BREAK_MS);
		FT_SetBreakOff(ftHandle);
#else
		ftdi_set_line_property2(&fc, BITS_8, STOP_BIT_1, NONE,
		    BREAK_ON);
		ms_sleep(BREAK_MS);
		ftdi_set_line_property2(&fc, BITS_8, STOP_BIT_1, NONE,
		    BREAK_OFF);
#endif
	} else {
#ifdef WIN32
		EscapeCommFunction(com_port, SETBREAK);
		ms_sleep(BREAK_MS);
		EscapeCommFunction(com_port, CLRBREAK);
#else
		ioctl(com_port, TIOCSBRK, NULL);
		ms_sleep(BREAK_MS);
		ioctl(com_port, TIOCCBRK, NULL);
#endif
	}
	ms_sleep(20);
}


static const char *riscv_reg_names[] = {
	"zr", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
	"s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
	"a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
	"s8", "s9", "sA", "sB", "t3", "t4", "t5", "t6"
};

static const char *mips_reg_names[] = {
	"zr", "at", "v0", "v1", "a0", "a1", "a2", "a3",
	"t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
	"s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
	"t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};


static int deb_seqn;
static int deb_big_endian;
static int deb_riscv;


static void
deb_print_reg(int off)
{
	int i;

	for (i = 0; i < 4; i++)
		printf("%02x", rxbuf[off * 4 + (3 - i)]);
}


static int
deb_get_seqn()
{
	int i;

	i = async_read_block(1);
	if (i == 0) {
		printf("Error: got no sequence number, "
		    "debugger disfunctional!\n");
		return (1);
	}
	if (rxbuf[0] != ((deb_seqn + 1) & 0xff)) {
		printf("Error: bad sequence number: "
		    "got %d, should have %d\n", rxbuf[0],
		    (deb_seqn + 1) & 0xff);
		return (1);
	}
	deb_seqn = rxbuf[0];
	return (0);
}


#define	BREAKPOINTS 2

static int
deb_print_breakpoints(void)
{
	int i, enabled, trapped;

	async_send_uint8(0xa1);		/* DEB_CMD_BREAKPOINT_RD */
	async_send_uint8(0);		/* start at breakpoint #0 */
	async_send_uint8(BREAKPOINTS - 1);/* fetch values */
	deb_get_seqn();
	i = async_read_block(BREAKPOINTS * 4);
	if (i != BREAKPOINTS * 4) {
		printf("\nError: short read "
		    "(%d instead of %d)\n", i, BREAKPOINTS * 4);
		return (-1);
	}

	for (i = 0; i < BREAKPOINTS; i++) {
		enabled = rxbuf[i * 4] & 1;
		trapped = rxbuf[i * 4] & 2;
		rxbuf[i * 4] &= ~3;
		printf("breakpoint #%d: ", i);
		if (enabled) {
			deb_print_reg(i);
			if (trapped)
				printf(" (trapped)");
			printf("\n");
		} else
			printf("disabled\n");
	}

	return (0);
}


static int
deb_print_registers(void)
{
	int r, c, i;

	async_send_uint8(0xa0);		/* DEB_CMD_REG_RD */
	async_send_uint8(0);		/* start at reg #0 */
	async_send_uint8(63);		/* fetch 64 values */
	deb_get_seqn();
	i = async_read_block(64 * 4);
	if (i != 64 * 4) {
		printf("\nError: short read "
		    "(%d instead of %d)\n", i, 64 * 4);
		return (-1);
	}

	for (r = 0; r < 8; r++) {
		for (c = 0; c < 4; c++) {
			if (r + 8 * c < 10)
				printf(" ");
			if (deb_riscv)
				printf("x%d (%s): ", r + 8 * c,
				    riscv_reg_names[r + 8 * c]);
			else
				printf("$%d (%s): ", r + 8 * c,
				    mips_reg_names[r + 8 * c]);
			deb_print_reg(r + 8 * c);
			if (c != 3)
				printf("  ");
		}
		printf("\n");
	}
	printf("\n");

	printf(" HI: ");
	deb_print_reg(32);
	printf(" LO: ");
	deb_print_reg(33);
	printf(" SR: ");
	deb_print_reg(34);
	printf(" CS: ");
	deb_print_reg(35);
	printf(" EPC: ");
	deb_print_reg(36);
	printf(" EB: ");
	deb_print_reg(37);
	printf("\n");

	printf("\n");
	printf(" IF A: ");
	deb_print_reg(40);
	printf("  ID A: ");
	deb_print_reg(42);
	printf("  EX A: ");
	deb_print_reg(44);
	printf("  MA A: ");
	deb_print_reg(46);
	printf("  WB A: ");
	deb_print_reg(48);
	printf("\n");

	printf(" IF I: ");
	deb_print_reg(41);
	printf("  ID I: ");
	deb_print_reg(43);
	printf("  EX I: ");
	deb_print_reg(45);
	printf("  MA I: ");
	deb_print_reg(47);
	printf("  WB I: ");
	deb_print_reg(49);
	printf("\n");

	printf("\n");
	printf(" Count: ");
	deb_print_reg(38);
	printf("     Exec: ");
	deb_print_reg(52);
	printf("     Branch: ");
	deb_print_reg(53);
	printf("     Mispred: ");
	deb_print_reg(54);
	printf("\n");

	return (0);
}


static void
debug_help(void)
{

	printf(
	    "  r	show registers\n"
	    "  R	show registers until a key is pressed\n"
	    "  s	execute a single clock cycle\n"
	    "  s N	execute at most N clock cycles\n"
	    "  c	continue execution\n"
	    "  b	show breakpoints\n"
	    "  b N	deactivate breakpoint N\n"
	    "  b N,A	set breakpoint N at address A\n"
	    "  .	exit from debugger\n"
	    "  h	get this summary\n"
	    "  ?	get this summary\n"
	);
}


static void
debug_cmd(void)
{
	char cmdbuf[256];
	int i, j, c, r;
#ifdef WIN32
	HANDLE cons_out = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_SCREEN_BUFFER_INFO screen_info;
#endif

	/* Enable debugger */
	printf("\n*** Entering debug mode ***\n");
	async_send_uint8(0x9d);
	async_send_uint8(0xed);

	/* Flush read buffer */
	async_read_block(BUFLEN_MAX);

	/* Fetch initial sequence number and config register */
	async_send_uint8(0xa0);		/* DEB_CMD_REG_RD */
	async_send_uint8(55);		/* start at reg #55 */
	async_send_uint8(0);		/* fetch 1 value */
	i = async_read_block(1);
	if (i == 0) {
		printf("Error: got no sequence number\n");
		printf("Debugger disfunctional, exiting.\n");
		return;
	}
	deb_seqn = rxbuf[0];
	i = async_read_block(4);
	if (i != 4) {
		printf("\nError: short read (%d instead of %d)\n", i, 4);
		printf("Debugger disfunctional, exiting.\n");
		return;
	}
	printf("Detected ");
	if (rxbuf[1] & 0x80) {
		printf("big-endian ");
		deb_big_endian = 1;
	} else {
		printf("little-endian ");
		deb_big_endian = 0;
	}
	if (rxbuf[1] & 0x40) {
		printf("f32c/riscv");
		deb_riscv = 1;
	} else {
		printf("f32c/mips");
		deb_riscv = 0;
	}
	printf(" core, clk ticks at %f MHz.\n", 1.0 *
	    (((rxbuf[3] & 0xf) << 8) + rxbuf[2]) / ((rxbuf[3] >> 5) + 1));

	do {
		printf("db> ");
		fflush(stdout);
		gets1(cmdbuf, sizeof(cmdbuf));
		for (i = 0; cmdbuf[i] != 0 ; i++) 
			if (cmdbuf[i] != ' ' && cmdbuf[i] != 9)
				break;
		switch (cmdbuf[i]) {
		case 's': /* single step */
			c = atoi(&cmdbuf[i + 1]);
			if (c < 1)
				c = 1;
			async_send_uint8(0xef);	/* DEB_CMD_CLK_STEP */
			async_send_uint32(0);	/* disable clk when done */
			async_send_uint32(c);	/* how many cycles */
			deb_get_seqn();
			printf("Single-stepping %d cycle(s)...\n", c);
			/* XXX ugly hack, wait for cycles to pass... */
			ms_sleep(c / 50000);
			deb_print_registers();
			break;
		case 'b': /* set / clear breakpoints */
			i++;
			while (cmdbuf[i] == ' ' || cmdbuf[i] == 8)
				i++;
			if (cmdbuf[i] == 0) {
				deb_print_breakpoints();
				break;
			}
			j = i;
			while (isnumber(cmdbuf[j]))
				j++;
			r = atoi(&cmdbuf[i]);
			i = j;
			while (cmdbuf[i] == ' ' || cmdbuf[i] == 8 ||
			    cmdbuf[i] == ',')
				i++;
			c = strtoul(&cmdbuf[i], NULL, 16);
			c &= ~3;	/* Word align */
			if (cmdbuf[i] != 0)
				c |= 1;	 /* Set breakpoint enable flag */
			async_send_uint8(0xe1); /* DEB_CMD_BREAKPOINT_WR */
			async_send_uint32(r);	/* dst reg */
			async_send_uint32(c);	/* value */
			deb_get_seqn();
			break;
		case 'c': /* continue */
			async_send_uint8(0xef); /* DEB_CMD_CLK_STEP */
			async_send_uint32(1);	/* enable clk */
			async_send_uint32(0);	/* don't wait */
			deb_get_seqn();
			break;
		case 'r': /* print registers */
			deb_print_registers();
			break;
		case 'R': /* continuously print registers */
			do {
				if (deb_print_registers() != 0)
					break;
#ifdef WIN32
				if (kbhit()) {
					c = getch();
#else
				if (read(0, &c, 1) > 0) {
#endif
						break;
				}
#ifdef WIN32
				GetConsoleScreenBufferInfo(cons_out,
				    &screen_info);
				screen_info.dwCursorPosition.X = 0;
				screen_info.dwCursorPosition.Y -= 15;
				SetConsoleCursorPosition(cons_out,
				    screen_info.dwCursorPosition);
#else
				printf("\r\033[15A");
#endif
			} while (1);
			break;
		case 'q':
			cmdbuf[i] = '.';
			break;
		case '.':
		case 0:
			break;
		case 'h':
		case '?':
			debug_help();
			break;
		default:
			printf("Unknown command\n");
			break;
		}
	} while (cmdbuf[i] != '.');

	/* Exit debugger */
	printf("*** Exiting debug mode ***\n");
	async_send_uint8(0x9d);
	async_send_uint8(0xdd);
	async_read_block(1);
}


static int
term_emul(void)
{
#ifdef WIN32
	DWORD saved_cons_mode;
	DWORD rx_cnt, tx_cnt, ev_stat, sent;
	HANDLE cons_in = GetStdHandle(STD_INPUT_HANDLE);
	HANDLE cons_out = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_CURSOR_INFO cursor_info;
	CONSOLE_SCREEN_BUFFER_INFO screen_info;
	COORD cursor_pos, saved_cursor_pos;
	int color0, cons_color;
	int rx_esc_seqn = 0;
	int tx_esc_seqn = 0;
	int esc_arg, esc_arg0;
	int prev_char = 0;
	char *keystr;
	char vt100buf[20];
#else
	int rx_cnt, tx_cnt, sent;
#endif
	int key_phase = 1; /* 0 .. normal; 1 .. CR; 2 .. CR + ~ */
	int c, res;
	int infile = -1;
	int sleep_t = 0;
	char argbuf[256];
	int i;
	
#ifdef WIN32
	if (cable_hw == CABLE_HW_USB) {
		FT_SetLatencyTimer(ftHandle, 20);
		FT_SetBaudRate(ftHandle, bauds);
		FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1,
		    FT_PARITY_NONE);
		FT_SetFlowControl(ftHandle, FT_FLOW_NONE, 0, 0);
		if (port_mode != PORT_MODE_UART) {
			set_port_mode(PORT_MODE_UART);
			do {} while (FT_StopInTask(ftHandle) != FT_OK);
			ms_sleep(50);
			FT_Purge(ftHandle, FT_PURGE_RX);
			do {} while (FT_RestartInTask(ftHandle) != FT_OK);
		}
	}

	/* Disable CTRL-C, XON/XOFF etc. processing on console input. */
	GetConsoleMode(cons_in, &saved_cons_mode);
	SetConsoleMode(cons_in, 0x40); /* ENABLE_QUICK_EDIT_MODE */
	GetConsoleScreenBufferInfo(cons_out, &screen_info);
	color0 = screen_info.wAttributes;
	color0 = FOREGROUND_GREEN | FOREGROUND_INTENSITY;
	cursor_info.bVisible = 1;
	cursor_info.dwSize = 100;
	SetConsoleCursorInfo(cons_out, &cursor_info);
	SetConsoleTextAttribute(cons_out, color0);
#else
	if (cable_hw == CABLE_HW_USB) {
		if (port_mode != PORT_MODE_UART) {
			set_port_mode(PORT_MODE_UART);
			ftdi_usb_purge_buffers(&fc);
		}
		ftdi_set_latency_timer(&fc, 20);
		ftdi_set_baudrate(&fc, bauds);
		ftdi_set_line_property(&fc, BITS_8, STOP_BIT_1, NONE);
		ftdi_setflowctrl(&fc, SIO_DISABLE_FLOW_CTRL);
	}

	/* Disable CTRL-C, XON/XOFF etc. processing on console input. */
	fcntl(0, F_SETFL, O_NONBLOCK);
	system("stty -echo -isig -icanon -iexten -ixon -ixoff -icrnl");
#endif

	printf("Terminal emulation mode, using %d bauds\n", bauds);
	printf("Press ENTER, ~, ? for help\n");

	do {
		tx_cnt = 0;
		if (infile > 0) {
			i = bauds / 300;
			if (bauds < 4800)
				i = 16;
			res = read(infile, txbuf, i);
			if (res <= 0) {
				close(infile);
				infile = -1;
				tx_cnt = 0;
			} else
				tx_cnt = res;
#ifdef WIN32
			if (kbhit()) {
				c = getch();
#else
			if (read(0, &c, 1) > 0) {
#endif
				if (c == 3) {
					close(infile);
					infile = -1;
					tx_cnt = 0;
					printf("\nTransfer interrupted!\n\n");
				}
			}
		} else
#ifdef WIN32
		while (kbhit()) {
			c = getch();
			txbuf[tx_cnt] = c;
#else
		while (read(0, &txbuf[tx_cnt], 1) > 0) {
			c = txbuf[tx_cnt];
#endif
#ifdef WIN32
			/*
			 * Translate cursor / function keys to vt100 sequences.
			 */
			if (c == 224) {
				tx_esc_seqn = 1; /* Cursor keys */
				continue;
			}
			if (c == 0) {
				tx_esc_seqn = 2; /* FN keys */
				continue;
			}
			if (tx_esc_seqn == 1) {
				keystr = "";
				switch (c) {
				case 75:	/* cursor left */
					keystr = "\x1b[D";
					break;
				case 77:	/* cursor right */
					keystr = "\x1b[C";
					break;
				case 72:	/* cursor up */
					keystr = "\x1b[A";
					break;
				case 80:	/* cursor down */
					keystr = "\x1b[B";
					break;
				case 82:	/* INS */
					keystr = "\x1b[2\x7e";
					break;
				case 83:	/* DEL */
					keystr = "\x1b[3\x7e";
					break;
				case 73:	/* PgUP */
					break;
				case 81:	/* PgDOWN */
					break;
				case 71:	/* Home */
					keystr = "\x1b[H";
					break;
				case 79:	/* End */
					keystr = "\x1b[F";
					break;
				default:
					break;
				}
				tx_cnt += sprintf(&txbuf[tx_cnt], "%s", keystr);
				tx_esc_seqn = 0;
				continue;
			}
			if (tx_esc_seqn == 2) {
				/* Ignore FN keys for now */
				tx_esc_seqn = 0;
				continue;
			}
#endif

			/*
			 * Catch and process ~ escape sequences.
			 */
			if (key_phase == 2) {
				switch (c) {
				case '?':
					printf("~?\n");
					terminal_help();
					key_phase = 0;
					continue;
				case '#':
					genbrk();
					key_phase = 0;
					continue;
				case 'r':
					reload = 1;
					res = 0;
					goto done;
				case '.':
					res = 1;
					goto done;
				case 'b':
					printf("~>New baudrate? ");
					fflush(stdout);
					gets1(argbuf, sizeof(argbuf));
					c = atoi(argbuf);
					if (c > 0 &&
					    cable_hw == CABLE_HW_USB) {
#ifdef WIN32
						res = FT_SetBaudRate(ftHandle,
						    c);
						if (res == FT_OK) {
#else
						res = ftdi_set_baudrate(&fc,
						    c);
						if (res == 0) {
#endif
							bauds = c;
							printf("new"
							    " baudrate: %d\n",
							    bauds);
						} else
							printf("%d: invalid"
							    " baudrate\n", c);
					}
					key_phase = 0;
					continue;
				case '>':
					printf("~>Local file name? ");
					fflush(stdout);
					gets1(argbuf, sizeof(argbuf));
					infile = open(argbuf,
#ifdef WIN32
					    O_RDONLY | O_BINARY
#else
					    O_RDONLY
#endif
					    );
					if (infile < 0)
						printf("%s: cannot open\n",
						    argbuf);
					key_phase = 0;
					continue;
				case 'd':
					debug_cmd();
					key_phase = 0;
					continue;
				default:
					if (c != '~') {
						txbuf[tx_cnt] = '~';
						tx_cnt++;
						txbuf[tx_cnt] = c;
					}
					key_phase = 0;
					break;
				}
			}
			if (key_phase == 1 && c == '~') {
				key_phase = 2;
				continue;
			}
			if (c == 13)
				key_phase = 1;
			else
				key_phase = 0;
			tx_cnt++;
			if (tx_cnt >= 16)
				break;
		}
		if (tx_cnt) {
			if (cable_hw == CABLE_HW_USB) {
#ifdef WIN32
				FT_Write(ftHandle, txbuf, tx_cnt, &sent);
#else
				sent = ftdi_write_data(&fc, txbuf, tx_cnt);
#endif
			} else {/* cable_hw == CABLE_HW_COM */
#ifdef WIN32
				WriteFile(com_port, txbuf, tx_cnt,
				    (DWORD *) &sent, NULL);
#else
				fcntl(com_port, F_SETFL, 0);
				sent = write(com_port, txbuf, tx_cnt);
				fcntl(com_port, F_SETFL, O_NONBLOCK);
#endif
			}
			if (sent != tx_cnt) {
				res = 1;
				goto done;
			}
		}

#ifdef WIN32
		if (cable_hw == CABLE_HW_USB) {
			FT_GetStatus(ftHandle, &rx_cnt, &ev_stat, &ev_stat);
			if (rx_cnt > BUFLEN_MAX)
				rx_cnt = BUFLEN_MAX;
			if (rx_cnt)
				FT_Read(ftHandle, txbuf, rx_cnt, &rx_cnt);
		} else {
			rx_cnt = 32;
			ReadFile(com_port, txbuf, rx_cnt,
			    (DWORD *) &rx_cnt, NULL);
		}
#else
		rx_cnt = 1;
		if (cable_hw == CABLE_HW_USB) {
			if (rx_cnt < fc.readbuffer_remaining)
				rx_cnt = fc.readbuffer_remaining;
			if (rx_cnt > BUFLEN_MAX)
				rx_cnt = BUFLEN_MAX;
			rx_cnt = ftdi_read_data(&fc, txbuf, rx_cnt);
		} else {
			rx_cnt = read(com_port, txbuf, rx_cnt);
			if (rx_cnt == -1)
				rx_cnt = 0;
		}
#endif
		if (rx_cnt) {
			if (rx_cnt < 0) {
				res = 1;
				goto done;
			}
#ifdef WIN32
			for (i = 0; i < rx_cnt; i++) {
				c = txbuf[i];
				/*
				 * Interpret selected VT-100 control sequences
				 */
				if (c == 27) {
					prev_char = 27;
					continue;
				}
				if (prev_char == 27 && c == '[') {
					rx_esc_seqn = 1;
					esc_arg = 0;
					esc_arg0 = 0;
					continue;
				}
				if (rx_esc_seqn) {
					if (c >= '0' && c <= '9') {
						esc_arg = esc_arg * 10 +
						    c - '0';
						continue;
					}
					if (c == ';') {
						esc_arg0 = esc_arg;
						esc_arg = 0;
						continue;
					}
					switch (c) {
					case 's': /* Save cursor position */
						break;
					case 'u': /* Restore cursor position */
						break;
					case 'n': /* Query cursor position */
						if (esc_arg != 6)
							break;
						break;
						GetConsoleScreenBufferInfo(
						    cons_out, &screen_info);
						c = sprintf(vt100buf,
						    "\x1b[%d;%dR",
						screen_info.dwCursorPosition.Y
						- screen_info.srWindow.Top,
						screen_info.dwCursorPosition.X
						- screen_info.srWindow.Left);
						FT_Write(ftHandle, vt100buf,
						    c, &sent);
						break;
					case 'C': /* Set cursor hpos */
						break;
					case 'H': /* Cursor home */
						break;
					case 'A': /* Cursor up */
						break;
					case 'K': /* Erase to end of line */
						break;
					case 'J': /* Clear screen */
						GetConsoleScreenBufferInfo(
						    cons_out, &screen_info);
						cursor_pos.X = 
						    screen_info.srWindow.Left;
						cursor_pos.Y = 
						    screen_info.srWindow.Top;
						FillConsoleOutputCharacter(
						    cons_out, ' ',
						(screen_info.srWindow.Bottom
						- screen_info.srWindow.Top) *
						(screen_info.srWindow.Right
						- screen_info.srWindow.Left),
						    cursor_pos, &sent);
						SetConsoleCursorPosition(
						    cons_out, cursor_pos);
						break;
					case 'm': /* Set char attribute */
						cons_color = color0;
						if (esc_arg == 1 ||
						    esc_arg == 4) {
							cons_color |=
							    FOREGROUND_RED;
						}
						SetConsoleTextAttribute(
						    cons_out, cons_color);
						break;
					default:
						break;
					}
					rx_esc_seqn = 0;
					continue;
				}
				rx_esc_seqn = 0;
				fwrite(&c, 1, 1, stdout);
			}
#else
			fwrite(txbuf, 1, rx_cnt, stdout);
#endif
			fflush(stdout);
		}
		if (tx_cnt == 0 && rx_cnt == 0) {
			ms_sleep(sleep_t);
			if (sleep_t < 5)
				sleep_t++;
		} else
			sleep_t = 0;
	} while (1);

done:
	printf("\n");

	/* Restore special key processing on console input. */
#ifdef WIN32
	SetConsoleMode(cons_in, saved_cons_mode);
	cursor_info.bVisible = 1;
	cursor_info.dwSize = 20;
	SetConsoleCursorInfo(cons_out, &cursor_info);
	FT_SetLatencyTimer(ftHandle, 1);
	FT_SetBaudRate(ftHandle, USB_BAUDS);
#else
	system("stty echo isig icanon iexten ixon ixoff icrnl");
	ftdi_set_latency_timer(&fc, 1);
	ftdi_set_baudrate(&fc, USB_BAUDS);
#endif

	return (res);
}


int
main(int argc, char *argv[])
{
	int res = EXIT_FAILURE;
	int jed_target = JED_TGT_SRAM;
	int debug = 0;
	int c;
#ifdef WIN32
	int had_terminal = 0;
	COMMTIMEOUTS com_to;
#endif

#ifndef USE_PPI
#define OPTS	"qtdj:b:p:x:p:P:a:e:f:D:rs:"
#else
#define OPTS	"qtdj:b:p:x:p:P:a:e:f:D:rs:c:"
#endif
	while ((c = getopt(argc, argv, OPTS)) != -1) {
		switch (c) {
		case 'a':
			txfname = optarg;
			tx_binary = 0;
			break;
		case 'b':
			bauds = atoi(optarg);
			break;
		case 'x':
			xbauds = atoi(optarg);
			break;
#ifdef USE_PPI
		case 'c':
			if (strcasecmp(optarg, "usb") == 0)
				cable_hw = CABLE_HW_USB;
			else if (strcasecmp(optarg, "ppi") == 0)
				cable_hw = CABLE_HW_PPI;
			else {
				usage();
				exit(EXIT_FAILURE);
			}
			break;
#endif
		case 'd':
			debug = 1;
			global_debug = 1;
			break;
		case 'D':
			txfu_ms = atoi(optarg);
			break;
		case 'e':
			txfname = optarg;
			tx_binary = 1;
			break;
		case 'j':
			if (strcasecmp(optarg, "sram") == 0)
				jed_target = JED_TGT_SRAM;
			else if (strcasecmp(optarg, "flash") == 0)
				jed_target = JED_TGT_FLASH;
			else {
				usage();
				exit(EXIT_FAILURE);
			}
			break;
		case 'f':
			if (optarg[0] == '0' && optarg[1] == 'x')
				sscanf(&optarg[2], "%x", &spi_addr);
			else if (isdigit(*optarg))
				spi_addr = atoi(optarg);
			else {
				printf("Invalid address format\n");
				exit(EXIT_FAILURE);
			}
			if ((spi_addr & (SPI_SECTOR_SIZE - 1)) != 0) {
				printf("SPI address must be a multiple of %d\n",
				    SPI_SECTOR_SIZE);
				exit(EXIT_FAILURE);
			}
			break;
		case 'p':
			port_index = atoi(optarg);
			break;
		case 'P':
			com_name = optarg;
			cable_hw = CABLE_HW_COM;
			break;
		case 'q':
			quiet = 1;
			break;
		case 'r':
			reload = 1;
			break;
		case 's':
			svf_name = optarg;
			break;
		case 't':
			terminal = 1;
#ifdef WIN32
			had_terminal = 1;
#endif
			break;
		case '?':
		default:
			usage();
			exit(EXIT_FAILURE);
		}
	}
	argc -= optind;
	argv += optind;

#ifdef WIN32
	if (terminal) {
		system("color 0a");
		system("cls");
	}
#endif

	if (!quiet)
		printf("%s (built %s %s)\n", verstr, __DATE__, __TIME__);

	if (svf_name) {
		if (terminal || reload || txfname || com_name || argc == 0) {
			usage();
			exit(EXIT_FAILURE);
		}
		res = exec_bit_file(argv[0], jed_target, debug);
		return(res);
	}

	if (argc == 0 && terminal == 0 && txfname == NULL && reload == 0) {
		usage();
		exit(EXIT_FAILURE);
	}

	if (com_name && terminal == 0 && txfname == NULL && reload == 0) {
		fprintf(stderr, "error: "
		    "option -P must be used with -r, -t or -a\n");
		exit(EXIT_FAILURE);
	}

	if (com_name && cable_hw != CABLE_HW_COM) {
		fprintf(stderr, "error: "
		    "options -P and -c are mutualy exclusive\n");
		exit(EXIT_FAILURE);
	}

	if (spi_addr && jed_target != JED_TGT_FLASH) {
		fprintf(stderr, "error: "
		    "-f may be specified only with -j flash\n");
		exit(EXIT_FAILURE);
	}

	switch (cable_hw) {
	case CABLE_HW_UNKNOWN:
	case CABLE_HW_USB:
		res = setup_usb();
		if (res == 0)
			cable_hw = CABLE_HW_USB;
		if (cable_hw == CABLE_HW_USB) {
			if (xbauds == 0)
				xbauds = 3000000;
			break;
		}
#ifdef USE_PPI
	case CABLE_HW_PPI:
		res = setup_ppi();
#endif
		break;
	case CABLE_HW_COM:
		if (xbauds == 0)
			xbauds = bauds;
#ifdef WIN32
		sprintf(txbuf, "\\\\.\\%s", com_name);
		com_port = CreateFile(txbuf,  GENERIC_READ | GENERIC_WRITE, 
		    0, NULL, OPEN_EXISTING, 0, NULL);
		if (com_port == INVALID_HANDLE_VALUE) {
			fprintf(stderr, "Can't open %s\n", com_name);
			exit(EXIT_FAILURE);
		}
		async_set_baudrate(bauds);
		if (GetCommTimeouts(com_port, &com_to) == 0) {
			fprintf(stderr, "Can't configure %s\n", com_port);
			exit(EXIT_FAILURE);
		}
		com_to.ReadIntervalTimeout = 1;
		com_to.ReadTotalTimeoutConstant = 1;
		com_to.ReadTotalTimeoutMultiplier = 1;
		SetCommTimeouts(com_port, &com_to);
		res = 0;
#else
		com_port = open(com_name, O_RDWR);
		if (com_port < 0 || tcgetattr(com_port, &tty)) {
			fprintf(stderr, "Can't open %s\n", com_name);
			exit(EXIT_FAILURE);
		}
		tty.c_cflag &= ~(CSIZE|PARENB);
		tty.c_cflag |= CS8;
		tty.c_cflag |= CLOCAL;
		tty.c_cflag &= ~CRTSCTS;
		tty.c_iflag &= ~(ISTRIP|ICRNL);
		tty.c_iflag &= ~(IXON|IXOFF);
		tty.c_oflag &= ~OPOST;
		tty.c_lflag &= ~(ICANON|ISIG|IEXTEN|ECHO);
		tty.c_cc[VMIN] = 1;
		tty.c_cc[VTIME] = 0;
		async_set_baudrate(bauds);
		res = fcntl(com_port, F_SETFL, O_NONBLOCK);
#if defined(__FreeBSD__) || defined(__linux__)
		/* XXX w/o this a BREAK won't be sent properly on FreeBSD ?!?*/
		ms_sleep(300);
#endif
#endif /* !WIN32 */
	default:
		/* can't happen, shut up gcc warnings */
		break;
	}

	if (res) {
		fprintf(stderr, "Cannot find JTAG cable.\n");
		exit(EXIT_FAILURE);
	}

	if (!quiet && cable_hw != CABLE_HW_COM) {
#ifndef WIN32
		if (cable_hw == CABLE_HW_USB)
			printf("Using USB cable: %s\n", hmp->cable_path);
		else
#ifdef USE_PPI
			printf("Using parallel port JTAG cable.\n");
#else
			printf("Parallel port JTAG cable not supported!\n");
#endif
#endif /* !WIN32 */
	}

	do {
		if (reload) {
			genbrk();
			reload = 0;
		}
		if (argc)
			prog(argv[0], jed_target, debug);
		jed_target = JED_TGT_SRAM; /* for subsequent prog() calls */
		if (txfname)
			txfile();
	} while (terminal && term_emul() == 0);

#ifdef WIN32
	if (had_terminal)
		system("color");
#endif

	if (cable_hw == CABLE_HW_COM) {
#ifdef WIN32
		CloseHandle(com_port);
#else
		close(com_port);
#endif
	} else if (cable_hw == CABLE_HW_USB) {
		ms_sleep(1); // small delay for f32c to start
		shutdown_usb();
	}
#ifdef USE_PPI
	else
		shutdown_ppi();
#endif

	return (res);
}
