// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * X-Box gamepad driver
 *
 * Copyright (c) 2002 Marko Friedemann <mfr@bmx-chemnitz.de>
 *               2004 Oliver Schwartz <Oliver.Schwartz@gmx.de>,
 *                    Steven Toth <steve@toth.demon.co.uk>,
 *                    Franz Lehner <franz@caos.at>,
 *                    Ivan Hawkes <blackhawk@ivanhawkes.com>
 *               2005 Dominic Cerquetti <binary1230@yahoo.com>
 *               2006 Adam Buchbinder <adam.buchbinder@gmail.com>
 *               2007 Jan Kratochvil <honza@jikos.cz>
 *               2010 Christoph Fritz <chf.fritz@googlemail.com>
 *
 * This driver is based on:
 *  - information from     http://euc.jp/periphs/xbox-controller.ja.html
 *  - the iForce driver    drivers/char/joystick/iforce.c
 *  - the skeleton-driver  drivers/usb/usb-skeleton.c
 *  - Xbox 360 information http://www.free60.org/wiki/Gamepad
 *  - Xbox One information https://github.com/quantus/xbox-one-controller-protocol
 *
 * Thanks to:
 *  - ITO Takayuki for providing essential xpad information on his website
 *  - Vojtech Pavlik     - iforce driver / input subsystem
 *  - Greg Kroah-Hartman - usb-skeleton driver
 *  - XBOX Linux project - extra USB id's
 *  - Pekka Pöyry (quantus) - Xbox One controller reverse engineering
 *
 * TODO:
 *  - fine tune axes (especially trigger axes)
 *  - fix "analog" buttons (reported as digital now)
 *  - get rumble working
 *  - need USB IDs for other dance pads
 *
 * History:
 *
 * 2002-06-27 - 0.0.1 : first version, just said "XBOX HID controller"
 *
 * 2002-07-02 - 0.0.2 : basic working version
 *  - all axes and 9 of the 10 buttons work (german InterAct device)
 *  - the black button does not work
 *
 * 2002-07-14 - 0.0.3 : rework by Vojtech Pavlik
 *  - indentation fixes
 *  - usb + input init sequence fixes
 *
 * 2002-07-16 - 0.0.4 : minor changes, merge with Vojtech's v0.0.3
 *  - verified the lack of HID and report descriptors
 *  - verified that ALL buttons WORK
 *  - fixed d-pad to axes mapping
 *
 * 2002-07-17 - 0.0.5 : simplified d-pad handling
 *
 * 2004-10-02 - 0.0.6 : DDR pad support
 *  - borrowed from the XBOX linux kernel
 *  - USB id's for commonly used dance pads are present
 *  - dance pads will map D-PAD to buttons, not axes
 *  - pass the module paramater 'dpad_to_buttons' to force
 *    the D-PAD to map to buttons if your pad is not detected
 *
 * Later changes can be tracked in SCM.
 */
// #define DEBUG
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/usb/input.h>
#include <linux/usb/quirks.h>
#include <linux/timer.h>

#define XPAD_PKT_LEN 64

/* The Guitar Hero Live (GHL) Xbox One dongles require a poke 
 * every 8 seconds.
 */
#define GHL_GUITAR_POKE_INTERVAL 8 /* In seconds */

/*
 * xbox d-pads should map to buttons, as is required for DDR pads
 * but we map them to axes when possible to simplify things
 */
#define MAP_DPAD_TO_BUTTONS		(1 << 0)
#define MAP_TRIGGERS_TO_BUTTONS		(1 << 1)
#define MAP_STICKS_TO_NULL		(1 << 2)
#define MAP_SELECT_BUTTON		(1 << 3)
#define MAP_PADDLES				(1 << 4)
#define DANCEPAD_MAP_CONFIG	(MAP_DPAD_TO_BUTTONS |			\
				MAP_TRIGGERS_TO_BUTTONS | MAP_STICKS_TO_NULL)

#define XTYPE_XBOX        0
#define XTYPE_XBOX360     1
#define XTYPE_XBOX360W    2
#define XTYPE_XBOXONE     3
#define XTYPE_UNKNOWN     4

/* Send power-off packet to xpad360w after holding the mode button for this many
 * seconds
 */
#define XPAD360W_POWEROFF_TIMEOUT 5

#define PKT_XB              0
#define PKT_XBE1            1
#define PKT_XBE2_FW_OLD     2
#define PKT_XBE2_FW_5_EARLY 3
#define PKT_XBE2_FW_5_11    4

#define QUIRK_360_START_PKT_1	(1 << 0)
#define QUIRK_360_START_PKT_2	(1 << 1)
#define QUIRK_360_START_PKT_3	(1 << 2)
#define QUIRK_GHL_XBOXONE	(1 << 3)
#define QUIRK_360_START (QUIRK_360_START_PKT_1 |			\
				QUIRK_360_START_PKT_2 | QUIRK_360_START_PKT_3)

static bool dpad_to_buttons;
module_param(dpad_to_buttons, bool, S_IRUGO);
MODULE_PARM_DESC(dpad_to_buttons, "Map D-PAD to buttons rather than axes for unknown pads");

static bool triggers_to_buttons;
module_param(triggers_to_buttons, bool, S_IRUGO);
MODULE_PARM_DESC(triggers_to_buttons, "Map triggers to buttons rather than axes for unknown pads");

static bool sticks_to_null;
module_param(sticks_to_null, bool, S_IRUGO);
MODULE_PARM_DESC(sticks_to_null, "Do not map sticks at all for unknown pads");

static bool auto_poweroff = true;
module_param(auto_poweroff, bool, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(auto_poweroff, "Power off wireless controllers on suspend");

static const struct xpad_device {
	u16 idVendor;
	u16 idProduct;
	char *name;
	u8 mapping;
	u8 xtype;
	u8 quirks;
} xpad_device[] = {
	{ 0x045e, 0x028e, "Microsoft X-Box 360 pad", 0, XTYPE_XBOX360 },
};

/* buttons shared with xbox and xbox360 */
static const signed short xpad_common_btn[] = {
	BTN_A, BTN_B, BTN_X, BTN_Y,			/* "analog" buttons */
	BTN_START, BTN_SELECT, BTN_THUMBL, BTN_THUMBR,	/* start/back/sticks */
	-1						/* terminating entry */
};

/* original xbox controllers only */
static const signed short xpad_btn[] = {
	BTN_C, BTN_Z,		/* "analog" buttons */
	-1			/* terminating entry */
};

/* used when dpad is mapped to buttons */
static const signed short xpad_btn_pad[] = {
	BTN_TRIGGER_HAPPY1, BTN_TRIGGER_HAPPY2,		/* d-pad left, right */
	BTN_TRIGGER_HAPPY3, BTN_TRIGGER_HAPPY4,		/* d-pad up, down */
	-1				/* terminating entry */
};

/* used when triggers are mapped to buttons */
static const signed short xpad_btn_triggers[] = {
	BTN_TL2, BTN_TR2,		/* triggers left/right */
	-1
};

static const signed short xpad360_btn[] = {  /* buttons for x360 controller */
	BTN_TL, BTN_TR,		/* Button LB/RB */
	BTN_MODE,		/* The big X button */
	-1
};

static const signed short xpad_abs[] = {
	ABS_X, ABS_Y,		/* left stick */
	ABS_RX, ABS_RY,		/* right stick */
	-1			/* terminating entry */
};

/* used when dpad is mapped to axes */
static const signed short xpad_abs_pad[] = {
	ABS_HAT0X, ABS_HAT0Y,	/* d-pad axes */
	-1			/* terminating entry */
};

/* used when triggers are mapped to axes */
static const signed short xpad_abs_triggers[] = {
	ABS_Z, ABS_RZ,		/* triggers left/right */
	-1
};

/* used when the controller has extra paddle buttons */
static const signed short xpad_btn_paddles[] = {
	BTN_TRIGGER_HAPPY5, BTN_TRIGGER_HAPPY6, /* paddle upper right, lower right */
	BTN_TRIGGER_HAPPY7, BTN_TRIGGER_HAPPY8, /* paddle upper left, lower left */
	-1						/* terminating entry */
};

/* used for GHL dpad mapping */
static const struct {int x; int y; } dpad_mapping[] = {
	{0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1},
	{0, 0}
};

/*
 * Xbox 360 has a vendor-specific class, so we cannot match it with only
 * USB_INTERFACE_INFO (also specifically refused by USB subsystem), so we
 * match against vendor id as well. Wired Xbox 360 devices have protocol 1,
 * wireless controllers have protocol 129.
 */
#define XPAD_XBOX360_VENDOR_PROTOCOL(vend, pr) \
	.match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_INT_INFO, \
	.idVendor = (vend), \
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC, \
	.bInterfaceSubClass = 93, \
	.bInterfaceProtocol = (pr)
#define XPAD_XBOX360_VENDOR(vend) \
	{ XPAD_XBOX360_VENDOR_PROTOCOL((vend), 1) }, \
	{ XPAD_XBOX360_VENDOR_PROTOCOL((vend), 129) }


//----------------X BOX ONE STUFF HERE - NOT USED-----------------------------

/* The Xbox One controller uses subclass 71 and protocol 208. */
// #define XPAD_XBOXONE_VENDOR_PROTOCOL(vend, pr) \
// 	.match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_INT_INFO, \
// 	.idVendor = (vend), \
// 	.bInterfaceClass = USB_CLASS_VENDOR_SPEC, \
// 	.bInterfaceSubClass = 71, \
// 	.bInterfaceProtocol = (pr)
// #define XPAD_XBOXONE_VENDOR(vend) \
// 	{ XPAD_XBOXONE_VENDOR_PROTOCOL((vend), 208) }

static const struct usb_device_id xpad_table[] = {
	{ USB_INTERFACE_INFO('X', 'B', 0) },	/* X-Box USB-IF not approved class */
	XPAD_XBOX360_VENDOR(0x045e),		/* Microsoft X-Box 360 controllers */
	{ }
};

MODULE_DEVICE_TABLE(usb, xpad_table);


//----------------X BOX ONE STUFF HERE - NOT USED-----------------------------

// struct xboxone_init_packet {
// 	u16 idVendor;
// 	u16 idProduct;
// 	const u8 *data;
// 	u8 len;
// };

// #define XBOXONE_INIT_PKT(_vid, _pid, _data)		\
// 	{						\
// 		.idVendor	= (_vid),		\
// 		.idProduct	= (_pid),		\
// 		.data		= (_data),		\
// 		.len		= ARRAY_SIZE(_data),	\
// 	}

/*
 * starting with xbox one, the game input protocol is used
 * magic numbers are taken from
 * - https://github.com/xpadneo/gip-dissector/blob/main/src/gip-dissector.lua
 * - https://github.com/medusalix/xone/blob/master/bus/protocol.c
 */
#define GIP_CMD_ACK      0x01
#define GIP_CMD_IDENTIFY 0x04
#define GIP_CMD_POWER    0x05
#define GIP_CMD_AUTHENTICATE 0x06
#define GIP_CMD_VIRTUAL_KEY  0x07
#define GIP_CMD_RUMBLE   0x09
#define GIP_CMD_LED      0x0a
#define GIP_CMD_FIRMWARE 0x0c
#define GIP_CMD_INPUT    0x20

#define GIP_SEQ0 0x00

#define GIP_OPT_ACK      0x10
#define GIP_OPT_INTERNAL 0x20

/*
 * length of the command payload encoded with
 * https://en.wikipedia.org/wiki/LEB128
 * which is a no-op for N < 128
 */
#define GIP_PL_LEN(N) (N)

/*
 * payload specific defines
 */
#define GIP_PWR_ON 0x00
#define GIP_LED_ON 0x01

#define GIP_MOTOR_R  BIT(0)
#define GIP_MOTOR_L  BIT(1)
#define GIP_MOTOR_RT BIT(2)
#define GIP_MOTOR_LT BIT(3)
#define GIP_MOTOR_ALL (GIP_MOTOR_R | GIP_MOTOR_L | GIP_MOTOR_RT | GIP_MOTOR_LT)

/*
 * This packet is required for all Xbox One pads with 2015
 * or later firmware installed (or present from the factory).
 */
static const u8 xboxone_power_on[] = {
	GIP_CMD_POWER, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(1), GIP_PWR_ON
};

/*
 * This packet is required for Xbox One S (0x045e:0x02ea)
 * and Xbox One Elite Series 2 (0x045e:0x0b00) pads to
 * initialize the controller that was previously used in
 * Bluetooth mode.
 */
static const u8 xboxone_s_init[] = {
	GIP_CMD_POWER, GIP_OPT_INTERNAL, GIP_SEQ0, 0x0f, 0x06
};

/*
 * This packet is required to get additional input data
 * from Xbox One Elite Series 2 (0x045e:0x0b00) pads.
 * We mostly do this right now to get paddle data
 */
static const u8 extra_input_packet_init[] = {
	0x4d, 0x10, 0x01, 0x02, 0x07, 0x00
};

/*
 * This packet is required for the Titanfall 2 Xbox One pads
 * (0x0e6f:0x0165) to finish initialization and for Hori pads
 * (0x0f0d:0x0067) to make the analog sticks work.
 */
static const u8 xboxone_hori_ack_id[] = {
	GIP_CMD_ACK, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(9),
	0x00, GIP_CMD_IDENTIFY, GIP_OPT_INTERNAL, 0x3a, 0x00, 0x00, 0x00, 0x80, 0x00
};

/*
 * This packet is required for most (all?) of the PDP pads to start
 * sending input reports. These pads include: (0x0e6f:0x02ab),
 * (0x0e6f:0x02a4), (0x0e6f:0x02a6).
 */
static const u8 xboxone_pdp_led_on[] = {
	GIP_CMD_LED, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(3), 0x00, GIP_LED_ON, 0x14
};

/*
 * This packet is required for most (all?) of the PDP pads to start
 * sending input reports. These pads include: (0x0e6f:0x02ab),
 * (0x0e6f:0x02a4), (0x0e6f:0x02a6).
 */
static const u8 xboxone_pdp_auth[] = {
	GIP_CMD_AUTHENTICATE, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(2), 0x01, 0x00
};

/*
 * A specific rumble packet is required for some PowerA pads to start
 * sending input reports. One of those pads is (0x24c6:0x543a).
 */
static const u8 xboxone_rumblebegin_init[] = {
	GIP_CMD_RUMBLE, 0x00, GIP_SEQ0, GIP_PL_LEN(9),
	0x00, GIP_MOTOR_ALL, 0x00, 0x00, 0x1D, 0x1D, 0xFF, 0x00, 0x00
};

struct xpad_output_packet {
	u8 data[XPAD_PKT_LEN];
	u8 len;
	bool pending;
};

#define XPAD_OUT_CMD_IDX	0
#define XPAD_OUT_FF_IDX		1
#define XPAD_OUT_LED_IDX	(1 + IS_ENABLED(CONFIG_JOYSTICK_XPAD_FF))
#define XPAD_NUM_OUT_PACKETS	(1 + \
				 IS_ENABLED(CONFIG_JOYSTICK_XPAD_FF) + \
				 IS_ENABLED(CONFIG_JOYSTICK_XPAD_LEDS))

struct usb_xpad {
	struct input_dev *dev;		/* input device interface */
	struct input_dev __rcu *x360w_dev;
	struct usb_device *udev;	/* usb device */
	struct usb_interface *intf;	/* usb interface */

	bool pad_present;
	bool input_created;

	struct urb *irq_in;		/* urb for interrupt in report */
	unsigned char *idata;		/* input data */
	dma_addr_t idata_dma;

	struct urb *irq_out;		/* urb for interrupt out report */
	struct usb_anchor irq_out_anchor;
	bool irq_out_active;		/* we must not use an active URB */
	u8 odata_serial;		/* serial number for xbox one protocol */
	unsigned char *odata;		/* output data */
	dma_addr_t odata_dma;
	spinlock_t odata_lock;

	struct xpad_output_packet out_packets[XPAD_NUM_OUT_PACKETS];
	int last_out_packet;
	int init_seq;

#if defined(CONFIG_JOYSTICK_XPAD_LEDS)
	struct xpad_led *led;
#endif

	char phys[64];			/* physical device path */

	int mapping;			/* map d-pad to buttons or to axes */
	int xtype;			/* type of xbox device */
	int packet_type;		/* type of the extended packet */
	int pad_nr;			/* the order x360 pads were attached */
	int quirks;
	const char *name;		/* name of the device */
	struct work_struct work;	/* init/remove device from callback */
	struct delayed_work poweroff_work; /* work struct for poweroff on mode long press */
	time64_t mode_btn_down_ts;
	struct urb *ghl_urb;		/* URB for GHL Xbox One magic data */
	struct timer_list ghl_poke_timer;	/* Timer for periodic poke of GHL magic data */
};

static int xpad_init_input(struct usb_xpad *xpad);
static void xpad_deinit_input(struct usb_xpad *xpad);

/*
 *	xpad_process_packet
 *
 *	Completes a request by converting the data into events for the
 *	input subsystem.
 *
 *	The used report descriptor was taken from ITO Takayukis website:
 *	 http://euc.jp/periphs/xbox-controller.ja.html
 */
static void xpad_process_packet(struct usb_xpad *xpad, u16 cmd, unsigned char *data)
{
	struct input_dev *dev = xpad->dev;

	if (!(xpad->mapping & MAP_STICKS_TO_NULL)) {
		/* left stick */
		input_report_abs(dev, ABS_X,
				 (__s16) le16_to_cpup((__le16 *)(data + 12)));
		input_report_abs(dev, ABS_Y,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 14)));

		/* right stick */
		input_report_abs(dev, ABS_RX,
				 (__s16) le16_to_cpup((__le16 *)(data + 16)));
		input_report_abs(dev, ABS_RY,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 18)));
	}

	/* triggers left/right */
	if (xpad->mapping & MAP_TRIGGERS_TO_BUTTONS) {
		input_report_key(dev, BTN_TL2, data[10]);
		input_report_key(dev, BTN_TR2, data[11]);
	} else {
		input_report_abs(dev, ABS_Z, data[10]);
		input_report_abs(dev, ABS_RZ, data[11]);
	}

	/* digital pad */
	if (xpad->mapping & MAP_DPAD_TO_BUTTONS) {
		/* dpad as buttons (left, right, up, down) */
		input_report_key(dev, BTN_TRIGGER_HAPPY1, data[2] & BIT(2));
		input_report_key(dev, BTN_TRIGGER_HAPPY2, data[2] & BIT(3));
		input_report_key(dev, BTN_TRIGGER_HAPPY3, data[2] & BIT(0));
		input_report_key(dev, BTN_TRIGGER_HAPPY4, data[2] & BIT(1));
	} else {
		input_report_abs(dev, ABS_HAT0X,
				 !!(data[2] & 0x08) - !!(data[2] & 0x04));
		input_report_abs(dev, ABS_HAT0Y,
				 !!(data[2] & 0x02) - !!(data[2] & 0x01));
	}

	/* start/back buttons and stick press left/right */
	input_report_key(dev, BTN_START,  data[2] & BIT(4));
	input_report_key(dev, BTN_SELECT, data[2] & BIT(5));
	input_report_key(dev, BTN_THUMBL, data[2] & BIT(6));
	input_report_key(dev, BTN_THUMBR, data[2] & BIT(7));

	/* "analog" buttons A, B, X, Y */
	input_report_key(dev, BTN_A, data[4]);
	input_report_key(dev, BTN_B, data[5]);
	input_report_key(dev, BTN_X, data[6]);
	input_report_key(dev, BTN_Y, data[7]);

	/* "analog" buttons black, white */
	input_report_key(dev, BTN_C, data[8]);
	input_report_key(dev, BTN_Z, data[9]);

	input_sync(dev);
}

/*
 *	xpad360_process_packet
 *
 *	Completes a request by converting the data into events for the
 *	input subsystem. It is version for xbox 360 controller
 *
 *	The used report descriptor was taken from:
 *		http://www.free60.org/wiki/Gamepad
 */

static void xpad360_process_packet(struct usb_xpad *xpad, struct input_dev *dev,
				   u16 cmd, unsigned char *data)
{
	/* valid pad data */
	if (data[0] != 0x00)
		return;

	/* digital pad */
	if (xpad->mapping & MAP_DPAD_TO_BUTTONS) {
		/* dpad as buttons (left, right, up, down) */
		input_report_key(dev, BTN_TRIGGER_HAPPY1, data[2] & BIT(2));
		input_report_key(dev, BTN_TRIGGER_HAPPY2, data[2] & BIT(3));
		input_report_key(dev, BTN_TRIGGER_HAPPY3, data[2] & BIT(0));
		input_report_key(dev, BTN_TRIGGER_HAPPY4, data[2] & BIT(1));
	}

	/* start/back buttons */
	input_report_key(dev, BTN_START,  data[2] & BIT(4));
	input_report_key(dev, BTN_SELECT, data[2] & BIT(5));

	/* stick press left/right */
	input_report_key(dev, BTN_THUMBL, data[2] & BIT(6));
	input_report_key(dev, BTN_THUMBR, data[2] & BIT(7));

	/* buttons A,B,X,Y,TL,TR and MODE */
	input_report_key(dev, BTN_A,	data[3] & BIT(4));
	input_report_key(dev, BTN_B,	data[3] & BIT(5));
	input_report_key(dev, BTN_X,	data[3] & BIT(6));
	input_report_key(dev, BTN_Y,	data[3] & BIT(7));
	input_report_key(dev, BTN_TL,	data[3] & BIT(0));
	input_report_key(dev, BTN_TR,	data[3] & BIT(1));
	input_report_key(dev, BTN_MODE,	data[3] & BIT(2));

	if (!(xpad->mapping & MAP_STICKS_TO_NULL)) {
		/* left stick */
		input_report_abs(dev, ABS_X,
				 (__s16) le16_to_cpup((__le16 *)(data + 6)));
		input_report_abs(dev, ABS_Y,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 8)));

		/* right stick */
		input_report_abs(dev, ABS_RX,
				 (__s16) le16_to_cpup((__le16 *)(data + 10)));
		input_report_abs(dev, ABS_RY,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 12)));
	}

	/* triggers left/right */
	if (xpad->mapping & MAP_TRIGGERS_TO_BUTTONS) {
		input_report_key(dev, BTN_TL2, data[4]);
		input_report_key(dev, BTN_TR2, data[5]);
	} else {
		input_report_abs(dev, ABS_Z, data[4]);
		input_report_abs(dev, ABS_RZ, data[5]);
	}

	input_sync(dev);

}

static void xpad_presence_work(struct work_struct *work)
{
	struct usb_xpad *xpad = container_of(work, struct usb_xpad, work);
	int error;

	if (xpad->pad_present) {
		error = xpad_init_input(xpad);
		if (error) {
			/* complain only, not much else we can do here */
			dev_err(&xpad->dev->dev,
				"unable to init device: %d\n", error);
		} else {
			rcu_assign_pointer(xpad->x360w_dev, xpad->dev);
		}
	} else {
		RCU_INIT_POINTER(xpad->x360w_dev, NULL);
		synchronize_rcu();
		/*
		 * Now that we are sure xpad360w_process_packet is not
		 * using input device we can get rid of it.
		 */
		xpad_deinit_input(xpad);
	}
}

static void xpad_irq_in(struct urb *urb)
{
	struct usb_xpad *xpad = urb->context;
	struct device *dev = &xpad->intf->dev;
	int retval, status;

	status = urb->status;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(dev, "%s - urb shutting down with status: %d\n",
			__func__, status);
		return;
	default:
		dev_dbg(dev, "%s - nonzero urb status received: %d\n",
			__func__, status);
		goto exit;
	}

#if defined(DEBUG_VERBOSE)
	/* If you set rowsize to larger than 32 it defaults to 16?
	 * Otherwise I would set it to XPAD_PKT_LEN                  V
	 */
	print_hex_dump(KERN_DEBUG, "xpad-dbg: ", DUMP_PREFIX_OFFSET, 32, 1, xpad->idata, XPAD_PKT_LEN, 0);
#endif

	switch (xpad->xtype) {
	case XTYPE_XBOX360:
		xpad360_process_packet(xpad, xpad->dev, 0, xpad->idata);
		break;
	default:
		xpad_process_packet(xpad, 0, xpad->idata);
	}

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(dev, "%s - usb_submit_urb failed with result %d\n",
			__func__, retval);
}


/* Callers must hold xpad->odata_lock spinlock */
static bool xpad_prepare_next_out_packet(struct usb_xpad *xpad)
{
	struct xpad_output_packet *pkt, *packet = NULL;
	int i;

	/* We may have init packets to send before we can send user commands */
	// if (xpad_prepare_next_init_packet(xpad))
	// 	return true;

	for (i = 0; i < XPAD_NUM_OUT_PACKETS; i++) {
		if (++xpad->last_out_packet >= XPAD_NUM_OUT_PACKETS)
			xpad->last_out_packet = 0;

		pkt = &xpad->out_packets[xpad->last_out_packet];
		if (pkt->pending) {
			dev_dbg(&xpad->intf->dev,
				"%s - found pending output packet %d\n",
				__func__, xpad->last_out_packet);
			packet = pkt;
			break;
		}
	}

	if (packet) {
		memcpy(xpad->odata, packet->data, packet->len);
		xpad->irq_out->transfer_buffer_length = packet->len;
		packet->pending = false;
		return true;
	}

	return false;
}

/* Callers must hold xpad->odata_lock spinlock */
static int xpad_try_sending_next_out_packet(struct usb_xpad *xpad)
{
	int error;

	if (!xpad->irq_out_active && xpad_prepare_next_out_packet(xpad)) {
		usb_anchor_urb(xpad->irq_out, &xpad->irq_out_anchor);
		error = usb_submit_urb(xpad->irq_out, GFP_ATOMIC);
		if (error) {
			dev_err(&xpad->intf->dev,
				"%s - usb_submit_urb failed with result %d\n",
				__func__, error);
			usb_unanchor_urb(xpad->irq_out);
			return -EIO;
		}

		xpad->irq_out_active = true;
	}

	return 0;
}

static void xpad_irq_out(struct urb *urb)
{
	struct usb_xpad *xpad = urb->context;
	struct device *dev = &xpad->intf->dev;
	int status = urb->status;
	int error;
	unsigned long flags;

	spin_lock_irqsave(&xpad->odata_lock, flags);

	switch (status) {
	case 0:
		/* success */
		xpad->irq_out_active = xpad_prepare_next_out_packet(xpad);
		break;

	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(dev, "%s - urb shutting down with status: %d\n",
			__func__, status);
		xpad->irq_out_active = false;
		break;

	default:
		dev_dbg(dev, "%s - nonzero urb status received: %d\n",
			__func__, status);
		break;
	}

	if (xpad->irq_out_active) {
		usb_anchor_urb(urb, &xpad->irq_out_anchor);
		error = usb_submit_urb(urb, GFP_ATOMIC);
		if (error) {
			dev_err(dev,
				"%s - usb_submit_urb failed with result %d\n",
				__func__, error);
			usb_unanchor_urb(urb);
			xpad->irq_out_active = false;
		}
	}

	spin_unlock_irqrestore(&xpad->odata_lock, flags);
}

// Analisar essa função
static int xpad_init_output(struct usb_interface *intf, struct usb_xpad *xpad,
			struct usb_endpoint_descriptor *ep_irq_out)
{
	int error;

	if (xpad->xtype == XTYPE_UNKNOWN)
		return 0;


	init_usb_anchor(&xpad->irq_out_anchor);

	xpad->odata = usb_alloc_coherent(xpad->udev, XPAD_PKT_LEN,
					 GFP_KERNEL, &xpad->odata_dma);
	if (!xpad->odata)
		return -ENOMEM;

	spin_lock_init(&xpad->odata_lock);

	xpad->irq_out = usb_alloc_urb(0, GFP_KERNEL);
	if (!xpad->irq_out) {
		error = -ENOMEM;
		goto err_free_coherent;
	}

	usb_fill_int_urb(xpad->irq_out, xpad->udev,
			 usb_sndintpipe(xpad->udev, ep_irq_out->bEndpointAddress),
			 xpad->odata, XPAD_PKT_LEN,
			 xpad_irq_out, xpad, ep_irq_out->bInterval);
	xpad->irq_out->transfer_dma = xpad->odata_dma;
	xpad->irq_out->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	return 0;

err_free_coherent:
	usb_free_coherent(xpad->udev, XPAD_PKT_LEN, xpad->odata, xpad->odata_dma);
	return error;
}

static void xpad_stop_output(struct usb_xpad *xpad)
{
	if (xpad->xtype != XTYPE_UNKNOWN) {
		if (!usb_wait_anchor_empty_timeout(&xpad->irq_out_anchor,
						   5000)) {
			dev_warn(&xpad->intf->dev,
				 "timed out waiting for output URB to complete, killing\n");
			usb_kill_anchored_urbs(&xpad->irq_out_anchor);
		}
	}
}

static void xpad_deinit_output(struct usb_xpad *xpad)
{
	if (xpad->xtype != XTYPE_UNKNOWN) {
		usb_free_urb(xpad->irq_out);
		usb_free_coherent(xpad->udev, XPAD_PKT_LEN,
				xpad->odata, xpad->odata_dma);
	}
}

static int xpad_inquiry_pad_presence(struct usb_xpad *xpad)
{
	struct xpad_output_packet *packet =
			&xpad->out_packets[XPAD_OUT_CMD_IDX];
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&xpad->odata_lock, flags);

	packet->data[0] = 0x08;
	packet->data[1] = 0x00;
	packet->data[2] = 0x0F;
	packet->data[3] = 0xC0;
	packet->data[4] = 0x00;
	packet->data[5] = 0x00;
	packet->data[6] = 0x00;
	packet->data[7] = 0x00;
	packet->data[8] = 0x00;
	packet->data[9] = 0x00;
	packet->data[10] = 0x00;
	packet->data[11] = 0x00;
	packet->len = 12;
	packet->pending = true;

	/* Reset the sequence so we send out presence first */
	xpad->last_out_packet = -1;
	retval = xpad_try_sending_next_out_packet(xpad);

	spin_unlock_irqrestore(&xpad->odata_lock, flags);

	return retval;
}


static int xpad_start_xbox_360(struct usb_xpad *xpad)
{
	int status;

	char *data = kzalloc(20, GFP_KERNEL);

	int TIMEOUT = 100;

	/*
	this init sequence is needed for the gamesir g3w controller
	and for shanwan controllers in xpad mode.
	Unfortunately, in this mode they identify as 0x045e, 0x028e, so we
	have to inspect the manufacturer string.
	Sending this sequence to other controllers will break initialization.
	*/
	bool is_shanwan = xpad->udev->manufacturer && strcasecmp("shanwan", xpad->udev->manufacturer) == 0;
	if (!(xpad->quirks & QUIRK_360_START) && !is_shanwan) {
		status = 0;
		goto err_free_ctrl_data;
	}

	if ((xpad->quirks & QUIRK_360_START_PKT_1) || is_shanwan) {
	    status = usb_control_msg(xpad->udev,
		    usb_rcvctrlpipe(xpad->udev, 0),
		    0x1, 0xc1,
		    cpu_to_le16(0x100), cpu_to_le16(0x0), data, cpu_to_le16(20),
		    TIMEOUT);

#ifdef DEBUG
	    dev_dbg(&xpad->intf->dev,
		    "%s - control message 1 returned %d\n", __func__, status);
#endif

	    if (status < 0) {
		    goto err_free_ctrl_data;
	    }
#ifdef DEBUG
	    else {
		    print_hex_dump(KERN_DEBUG, "xpad-dbg: ", DUMP_PREFIX_OFFSET, 32, 1, data, 20, 0);
	    }
#endif
	}

	if ((xpad->quirks & QUIRK_360_START_PKT_2) || is_shanwan) {
	    status = usb_control_msg(xpad->udev,
		    usb_rcvctrlpipe(xpad->udev, 0),
		    0x1, 0xc1,
		    cpu_to_le16(0x0), cpu_to_le16(0x0), data, cpu_to_le16(8),
		    TIMEOUT);
#ifdef DEBUG
	    dev_dbg(&xpad->intf->dev,
		    "%s - control message 2 returned %d\n", __func__, status);
#endif

	    if (status < 0) {
		    goto err_free_ctrl_data;
	    }
#ifdef DEBUG
	    else {
		    print_hex_dump(KERN_DEBUG, "xpad-dbg: ", DUMP_PREFIX_OFFSET, 32, 1, data, 8, 0);
	    }
#endif
	}

	if ((xpad->quirks & QUIRK_360_START_PKT_3) || is_shanwan) {
	    status = usb_control_msg(xpad->udev,
		    usb_rcvctrlpipe(xpad->udev, 0),
		    0x1, 0xc0,
		    cpu_to_le16(0x0), cpu_to_le16(0x0), data, cpu_to_le16(4),
		    TIMEOUT);
#ifdef DEBUG
	    dev_dbg(&xpad->intf->dev,
		    "%s - control message 3 returned %d\n", __func__, status);
#endif

	    if (status < 0) {
		    goto err_free_ctrl_data;
	    }
#ifdef DEBUG
	    else {
		    print_hex_dump(KERN_DEBUG, "xpad-dbg: ", DUMP_PREFIX_OFFSET, 32, 1, data, 4, 0);
	    }
#endif
	}

	status = 0;

err_free_ctrl_data:
	kfree(data);
	return status;
}


static int xpad_init_ff(struct usb_xpad *xpad) { return 0; }

#if defined(CONFIG_JOYSTICK_XPAD_LEDS)
#include <linux/leds.h>
#include <linux/idr.h>

static DEFINE_IDA(xpad_pad_seq);

struct xpad_led {
	char name[16];
	struct led_classdev led_cdev;
	struct usb_xpad *xpad;
};

/*
 * set the LEDs on Xbox360 / Wireless Controllers
 * @param command
 *  0: off
 *  1: all blink, then previous setting
 *  2: 1/top-left blink, then on
 *  3: 2/top-right blink, then on
 *  4: 3/bottom-left blink, then on
 *  5: 4/bottom-right blink, then on
 *  6: 1/top-left on
 *  7: 2/top-right on
 *  8: 3/bottom-left on
 *  9: 4/bottom-right on
 * 10: rotate
 * 11: blink, based on previous setting
 * 12: slow blink, based on previous setting
 * 13: rotate with two lights
 * 14: persistent slow all blink
 * 15: blink once, then previous setting
 */
static void xpad_send_led_command(struct usb_xpad *xpad, int command)
{
	struct xpad_output_packet *packet =
			&xpad->out_packets[XPAD_OUT_LED_IDX];
	unsigned long flags;

	command %= 16;

	spin_lock_irqsave(&xpad->odata_lock, flags);

	switch (xpad->xtype) {
	case XTYPE_XBOX360:
		packet->data[0] = 0x01;
		packet->data[1] = 0x03;
		packet->data[2] = command;
		packet->len = 3;
		packet->pending = true;
		break;

	case XTYPE_XBOX360W:
		packet->data[0] = 0x00;
		packet->data[1] = 0x00;
		packet->data[2] = 0x08;
		packet->data[3] = 0x40 + command;
		packet->data[4] = 0x00;
		packet->data[5] = 0x00;
		packet->data[6] = 0x00;
		packet->data[7] = 0x00;
		packet->data[8] = 0x00;
		packet->data[9] = 0x00;
		packet->data[10] = 0x00;
		packet->data[11] = 0x00;
		packet->len = 12;
		packet->pending = true;
		break;
	}

	xpad_try_sending_next_out_packet(xpad);

	spin_unlock_irqrestore(&xpad->odata_lock, flags);
}

/*
 * Light up the segment corresponding to the pad number on
 * Xbox 360 Controllers.
 */
static void xpad_identify_controller(struct usb_xpad *xpad)
{
	led_set_brightness(&xpad->led->led_cdev, (xpad->pad_nr % 4) + 2);
}

static void xpad_led_set(struct led_classdev *led_cdev,
			 enum led_brightness value)
{
	struct xpad_led *xpad_led = container_of(led_cdev,
						 struct xpad_led, led_cdev);

	xpad_send_led_command(xpad_led->xpad, value);
}

static int xpad_led_probe(struct usb_xpad *xpad)
{
	struct xpad_led *led;
	struct led_classdev *led_cdev;
	int error;

	if (xpad->xtype != XTYPE_XBOX360 && xpad->xtype != XTYPE_XBOX360W)
		return 0;

	xpad->led = led = kzalloc(sizeof(struct xpad_led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	xpad->pad_nr = ida_simple_get(&xpad_pad_seq, 0, 0, GFP_KERNEL);
	if (xpad->pad_nr < 0) {
		error = xpad->pad_nr;
		goto err_free_mem;
	}

	snprintf(led->name, sizeof(led->name), "xpad%d", xpad->pad_nr);
	led->xpad = xpad;

	led_cdev = &led->led_cdev;
	led_cdev->name = led->name;
	led_cdev->brightness_set = xpad_led_set;
	led_cdev->flags = LED_CORE_SUSPENDRESUME;

	error = led_classdev_register(&xpad->udev->dev, led_cdev);
	if (error)
		goto err_free_id;

	xpad_identify_controller(xpad);

	return 0;

err_free_id:
	ida_simple_remove(&xpad_pad_seq, xpad->pad_nr);
err_free_mem:
	kfree(led);
	xpad->led = NULL;
	return error;
}

static void xpad_led_disconnect(struct usb_xpad *xpad)
{
	struct xpad_led *xpad_led = xpad->led;

	if (xpad_led) {
		led_classdev_unregister(&xpad_led->led_cdev);
		ida_simple_remove(&xpad_pad_seq, xpad->pad_nr);
		kfree(xpad_led);
	}
}
#else
static int xpad_led_probe(struct usb_xpad *xpad) { return 0; }
static void xpad_led_disconnect(struct usb_xpad *xpad) { }
#endif

static int xpad_start_input(struct usb_xpad *xpad)
{
	int error;

	if (xpad->xtype == XTYPE_XBOX360) {
		error = xpad_start_xbox_360(xpad);
		if (error)
			return error;
	}

	if (usb_submit_urb(xpad->irq_in, GFP_KERNEL))
		return -EIO;


	return 0;
}

static void xpad_stop_input(struct usb_xpad *xpad)
{
	usb_kill_urb(xpad->irq_in);
}

static int xpad_open(struct input_dev *dev)
{
	struct usb_xpad *xpad = input_get_drvdata(dev);

	return xpad_start_input(xpad);
}

static void xpad_close(struct input_dev *dev)
{
	struct usb_xpad *xpad = input_get_drvdata(dev);

	xpad_stop_input(xpad);
}

static void xpad_set_up_abs(struct input_dev *input_dev, signed short abs)
{
	struct usb_xpad *xpad = input_get_drvdata(input_dev);

	switch (abs) {
	case ABS_X:
	case ABS_Y:
		break;
	case ABS_RX:
	case ABS_RY:	/* the two sticks */
		input_set_abs_params(input_dev, abs, -32768, 32767, 16, 128);
		break;
	case ABS_Z:
		//faca um ptint pra deixar de log
		printk("ABS_Z\n");
		break;
	case ABS_RZ:	/* the triggers (if mapped to axes) */
		input_set_abs_params(input_dev, abs, 0, 255, 0, 0);
		break;
	case ABS_HAT0X:
	case ABS_HAT0Y:	/* the d-pad (only if dpad is mapped to axes */
		input_set_abs_params(input_dev, abs, -1, 1, 0, 0);
		break;
	default:
		input_set_abs_params(input_dev, abs, 0, 0, 0, 0);
		break;
	}
}

static void xpad_deinit_input(struct usb_xpad *xpad)
{
	if (xpad->input_created) {
		xpad->input_created = false;
		xpad_led_disconnect(xpad);
		input_unregister_device(xpad->dev);
	}
}

static int xpad_init_input(struct usb_xpad *xpad)
{
	struct input_dev *input_dev;
	int i, error;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	xpad->dev = input_dev;
	input_dev->name = xpad->name;
	input_dev->phys = xpad->phys;
	usb_to_input_id(xpad->udev, &input_dev->id);

	input_dev->dev.parent = &xpad->intf->dev;

	input_set_drvdata(input_dev, xpad);

	if (!(xpad->mapping & MAP_STICKS_TO_NULL)) {
		/* set up axes */
		for (i = 0; xpad_abs[i] >= 0; i++)
			xpad_set_up_abs(input_dev, xpad_abs[i]);
	}

	/* set up standard buttons */
	for (i = 0; xpad_common_btn[i] >= 0; i++)
		input_set_capability(input_dev, EV_KEY, xpad_common_btn[i]);

	/* set up model-specific ones */
	if (xpad->xtype == XTYPE_XBOX360) {
		for (i = 0; xpad360_btn[i] >= 0; i++)
			input_set_capability(input_dev, EV_KEY, xpad360_btn[i]);
		if (xpad->mapping & MAP_SELECT_BUTTON)
			input_set_capability(input_dev, EV_KEY, KEY_RECORD);
	} else {
		for (i = 0; xpad_btn[i] >= 0; i++)
			input_set_capability(input_dev, EV_KEY, xpad_btn[i]);
	}

	if (xpad->mapping & MAP_DPAD_TO_BUTTONS) {
		for (i = 0; xpad_btn_pad[i] >= 0; i++)
			input_set_capability(input_dev, EV_KEY,
					     xpad_btn_pad[i]);
	}

	/* set up paddles if the controller has them */
	if (xpad->mapping & MAP_PADDLES) {
		for (i = 0; xpad_btn_paddles[i] >= 0; i++)
			input_set_capability(input_dev, EV_KEY, xpad_btn_paddles[i]);
	}

	/* set up triggers */
	if (xpad->mapping & MAP_TRIGGERS_TO_BUTTONS) {
		for (i = 0; xpad_btn_triggers[i] >= 0; i++)
			input_set_capability(input_dev, EV_KEY,
					     xpad_btn_triggers[i]);
	} else {
		for (i = 0; xpad_abs_triggers[i] >= 0; i++)
			xpad_set_up_abs(input_dev, xpad_abs_triggers[i]);
	}

	error = xpad_init_ff(xpad);
	if (error)
		goto err_free_input;

	error = xpad_led_probe(xpad);
	if (error)
		goto err_destroy_ff;

	error = input_register_device(xpad->dev);
	if (error)
		goto err_disconnect_led;

	xpad->input_created = true;
	return 0;

err_disconnect_led:
	xpad_led_disconnect(xpad);
err_destroy_ff:
	input_ff_destroy(input_dev);
err_free_input:
	input_free_device(input_dev);
	return error;
}

static int xpad_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_xpad *xpad;
	struct usb_endpoint_descriptor *ep_irq_in, *ep_irq_out;
	int i, error;

	if (intf->cur_altsetting->desc.bNumEndpoints != 2)
		return -ENODEV;

	//Ele itera na lista de controles para encntrar o que está conectado
	for (i = 0; xpad_device[i].idVendor; i++) {
		if ((le16_to_cpu(udev->descriptor.idVendor) == xpad_device[i].idVendor) &&
		    (le16_to_cpu(udev->descriptor.idProduct) == xpad_device[i].idProduct))
			break;
	}

	xpad = kzalloc(sizeof(struct usb_xpad), GFP_KERNEL);
	if (!xpad)
		return -ENOMEM;

	//retorna o endereço físico do controle na usb	tree
	usb_make_path(udev, xpad->phys, sizeof(xpad->phys));
	strlcat(xpad->phys, "/input0", sizeof(xpad->phys));


	//aloca memória para o controle e verifica se foi alocado apropriadamente e 
	xpad->idata = usb_alloc_coherent(udev, XPAD_PKT_LEN,
					 GFP_KERNEL, &xpad->idata_dma);
	if (!xpad->idata) {
		error = -ENOMEM;
		goto err_free_mem;
	}


	xpad->irq_in = usb_alloc_urb(0, GFP_KERNEL);
	if (!xpad->irq_in) {
		error = -ENOMEM;
		goto err_free_idata;
	}

	xpad->udev = udev;
	xpad->intf = intf;
	xpad->mapping = xpad_device[i].mapping;
	xpad->xtype = xpad_device[i].xtype;
	xpad->name = xpad_device[i].name;
	xpad->quirks = xpad_device[i].quirks;
	xpad->packet_type = PKT_XB;
	INIT_WORK(&xpad->work, xpad_presence_work);


	//
	if (xpad->xtype == XTYPE_UNKNOWN) {
		if (intf->cur_altsetting->desc.bInterfaceClass == USB_CLASS_VENDOR_SPEC) {
			if (intf->cur_altsetting->desc.bInterfaceProtocol == 129)
				xpad->xtype = XTYPE_XBOX360W;
			else
				xpad->xtype = XTYPE_XBOX360;
		} else {
			xpad->xtype = XTYPE_XBOX;
		}

		if (dpad_to_buttons)
			xpad->mapping |= MAP_DPAD_TO_BUTTONS;
		if (triggers_to_buttons)
			xpad->mapping |= MAP_TRIGGERS_TO_BUTTONS;
		if (sticks_to_null)
			xpad->mapping |= MAP_STICKS_TO_NULL;
	}


	ep_irq_in = ep_irq_out = NULL;

	for (i = 0; i < 2; i++) {
		//usb_endpoint_descriptor buscar definição -> specifies the transfer type, direction, polling interval, and maximum packet size for each endpoint
		struct usb_endpoint_descriptor *ep =
				&intf->cur_altsetting->endpoint[i].desc;

		if (usb_endpoint_xfer_int(ep)) {
			if (usb_endpoint_dir_in(ep))
				ep_irq_in = ep;
			else
				ep_irq_out = ep;
		}
	}

	if (!ep_irq_in || !ep_irq_out) {
		error = -ENODEV;
		goto err_free_in_urb;
	}

	error = xpad_init_output(intf, xpad, ep_irq_out);
	if (error)
		goto err_free_in_urb;

	usb_fill_int_urb(xpad->irq_in, udev,
			 usb_rcvintpipe(udev, ep_irq_in->bEndpointAddress),
			 xpad->idata, XPAD_PKT_LEN, xpad_irq_in,
			 xpad, ep_irq_in->bInterval);
	xpad->irq_in->transfer_dma = xpad->idata_dma;
	xpad->irq_in->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	usb_set_intfdata(intf, xpad);

	// Deletar pq nao 
	error = xpad_init_input(xpad);
	if (error)
		goto err_deinit_output;	
	return 0;

err_deinit_output:
	xpad_deinit_output(xpad);
err_free_in_urb:
	usb_free_urb(xpad->irq_in);
err_free_idata:
	usb_free_coherent(udev, XPAD_PKT_LEN, xpad->idata, xpad->idata_dma);
err_free_mem:
	kfree(xpad);
	return error;
}

static void xpad_disconnect(struct usb_interface *intf)
{
	struct usb_xpad *xpad = usb_get_intfdata(intf);

	xpad_deinit_input(xpad);

	/*
	 * Now that both input device and LED device are gone we can
	 * stop output URB.
	 */
	xpad_stop_output(xpad);

	xpad_deinit_output(xpad);

	usb_free_urb(xpad->irq_in);

	if (xpad->quirks & QUIRK_GHL_XBOXONE) {
		usb_free_urb(xpad->ghl_urb);
		del_timer_sync(&xpad->ghl_poke_timer);
	}

	usb_free_coherent(xpad->udev, XPAD_PKT_LEN,
			xpad->idata, xpad->idata_dma);

	kfree(xpad);

	usb_set_intfdata(intf, NULL);
}

static int xpad_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_xpad *xpad = usb_get_intfdata(intf);
	struct input_dev *input = xpad->dev;

	mutex_lock(&input->mutex);
	if (input->users)
		xpad_stop_input(xpad);
	mutex_unlock(&input->mutex);

	xpad_stop_output(xpad);

	return 0;
}

static int xpad_resume(struct usb_interface *intf)
{
	struct usb_xpad *xpad = usb_get_intfdata(intf);
	struct input_dev *input = xpad->dev;
	int retval = 0;

	mutex_lock(&input->mutex);
	if (input->users) {
		retval = xpad_start_input(xpad);
	} 
	mutex_unlock(&input->mutex);

	return retval;
}

static struct usb_driver xpad_driver = {
	.name		= "xpad",
	.probe		= xpad_probe,
	.disconnect	= xpad_disconnect,
	.suspend	= xpad_suspend,
	.resume		= xpad_resume,
	.id_table	= xpad_table,
};

module_usb_driver(xpad_driver);

MODULE_AUTHOR("Marko Friedemann <mfr@bmx-chemnitz.de>");
MODULE_DESCRIPTION("X-Box pad driver");
MODULE_LICENSE("GPL");