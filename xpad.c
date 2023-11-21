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

/* The Xbox One controller uses subclass 71 and protocol 208. */
#define XPAD_XBOXONE_VENDOR_PROTOCOL(vend, pr) \
	.match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_INT_INFO, \
	.idVendor = (vend), \
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC, \
	.bInterfaceSubClass = 71, \
	.bInterfaceProtocol = (pr)
#define XPAD_XBOXONE_VENDOR(vend) \
	{ XPAD_XBOXONE_VENDOR_PROTOCOL((vend), 208) }

static const struct usb_device_id xpad_table[] = {
	{ USB_INTERFACE_INFO('X', 'B', 0) },	/* X-Box USB-IF not approved class */
	XPAD_XBOX360_VENDOR(0x045e),		/* Microsoft X-Box 360 controllers */
	{ }
};

MODULE_DEVICE_TABLE(usb, xpad_table);

struct xboxone_init_packet {
	u16 idVendor;
	u16 idProduct;
	const u8 *data;
	u8 len;
};

#define XBOXONE_INIT_PKT(_vid, _pid, _data)		\
	{						\
		.idVendor	= (_vid),		\
		.idProduct	= (_pid),		\
		.data		= (_data),		\
		.len		= ARRAY_SIZE(_data),	\
	}

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

/*
 * A rumble packet with zero FF intensity will immediately
 * terminate the rumbling required to init PowerA pads.
 * This should happen fast enough that the motors don't
 * spin up to enough speed to actually vibrate the gamepad.
 */
static const u8 xboxone_rumbleend_init[] = {
	GIP_CMD_RUMBLE, 0x00, GIP_SEQ0, GIP_PL_LEN(9),
	0x00, GIP_MOTOR_ALL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* GHL Xbox One magic data */
static const char ghl_xboxone_magic_data[] = {
	0x22, 0x00, 0x00, 0x08, 0x02, 0x08, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*
 * This specifies the selection of init packets that a gamepad
 * will be sent on init *and* the order in which they will be
 * sent. The correct sequence number will be added when the
 * packet is going to be sent.
 */
static const struct xboxone_init_packet xboxone_init_packets[] = {
	XBOXONE_INIT_PKT(0x0e6f, 0x0165, xboxone_hori_ack_id),
	XBOXONE_INIT_PKT(0x0f0d, 0x0067, xboxone_hori_ack_id),
	XBOXONE_INIT_PKT(0x1430, 0x079b, xboxone_hori_ack_id),
	XBOXONE_INIT_PKT(0x0000, 0x0000, xboxone_power_on),
	XBOXONE_INIT_PKT(0x045e, 0x02ea, xboxone_s_init),
	XBOXONE_INIT_PKT(0x045e, 0x0b00, xboxone_s_init),
	XBOXONE_INIT_PKT(0x045e, 0x0b00, extra_input_packet_init),
	XBOXONE_INIT_PKT(0x0e6f, 0x0000, xboxone_pdp_led_on),
	XBOXONE_INIT_PKT(0x1430, 0x079b, xboxone_pdp_led_on),
	XBOXONE_INIT_PKT(0x0e6f, 0x0000, xboxone_pdp_auth),
	XBOXONE_INIT_PKT(0x1430, 0x079b, xboxone_pdp_auth),
	XBOXONE_INIT_PKT(0x24c6, 0x541a, xboxone_rumblebegin_init),
	XBOXONE_INIT_PKT(0x24c6, 0x542a, xboxone_rumblebegin_init),
	XBOXONE_INIT_PKT(0x24c6, 0x543a, xboxone_rumblebegin_init),
	XBOXONE_INIT_PKT(0x24c6, 0x541a, xboxone_rumbleend_init),
	XBOXONE_INIT_PKT(0x24c6, 0x542a, xboxone_rumbleend_init),
	XBOXONE_INIT_PKT(0x24c6, 0x543a, xboxone_rumbleend_init),
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
static void xpadone_ack_mode_report(struct usb_xpad *xpad, u8 seq_num);
static void xpad360w_poweroff_controller(struct usb_xpad *xpad);

/*
 *	ghl_magic_poke_cb
 *
 *	Call back function that resets the timer for the next magic data poke.
 */
static void ghl_magic_poke_cb(struct urb *urb)
{
	struct usb_xpad *xpad = urb->context;

	if (urb->status < 0)
		pr_warn("URB transfer failed.\n");

	mod_timer(&xpad->ghl_poke_timer, jiffies + GHL_GUITAR_POKE_INTERVAL*HZ);
}

/*
 *	ghl_magic_poke
 *
 *	Submits the GHL magic_data URB.
 */
static void ghl_magic_poke(struct timer_list *t)
{
	int ret;
	struct usb_xpad *xpad = from_timer(xpad, t, ghl_poke_timer);

	ret = usb_submit_urb(xpad->ghl_urb, GFP_ATOMIC);
	if (ret < 0)
		pr_warn("URB transfer failed.\n");
}

/*
 *	ghl_init_urb
 *
 *	Prepares the interrupt URB for GHL magic_data.
 */
static int ghl_init_urb(struct usb_xpad *xpad, struct usb_device *usbdev,
		const char ghl_magic_data[], u16 poke_size, struct usb_endpoint_descriptor *ep_irq_out)
{
	u8 *databuf;
	unsigned int pipe;

	pipe = usb_sndintpipe(usbdev, ep_irq_out->bEndpointAddress);

	databuf = devm_kzalloc(&xpad->udev->dev, poke_size, GFP_ATOMIC);
	if (databuf == NULL)
		return -ENOMEM;

	memcpy(databuf, ghl_magic_data, poke_size);

	usb_fill_int_urb(
		xpad->ghl_urb, usbdev, pipe,
		databuf, poke_size,
		ghl_magic_poke_cb, xpad, ep_irq_out->bInterval);

	return 0;
}

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

	/*
	 * This should be a simple else block. However historically
	 * xbox360w has mapped DPAD to buttons while xbox360 did not. This
	 * made no sense, but now we can not just switch back and have to
	 * support both behaviors.
	 */
	if (!(xpad->mapping & MAP_DPAD_TO_BUTTONS) ||
	    xpad->xtype == XTYPE_XBOX360W) {
		input_report_abs(dev, ABS_HAT0X,
				 !!(data[2] & 0x08) - !!(data[2] & 0x04));
		input_report_abs(dev, ABS_HAT0Y,
				 !!(data[2] & 0x02) - !!(data[2] & 0x01));
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

	/* XBOX360W controllers can't be turned off without driver assistance */
	if (xpad->xtype == XTYPE_XBOX360W) {
		if (data[3] & BIT(2)) {
			if (xpad->mode_btn_down_ts == 0)
				xpad->mode_btn_down_ts = ktime_get_seconds();
			schedule_delayed_work(&xpad->poweroff_work, msecs_to_jiffies(0));
		} else {
				xpad->mode_btn_down_ts = 0;
		}
	}
}

static void xpad360w_poweroff_work(struct work_struct *work) {
	struct usb_xpad *xpad = container_of(to_delayed_work(work), struct usb_xpad, poweroff_work);

	if (xpad->mode_btn_down_ts == 0)
		return;

	if ((ktime_get_seconds() - xpad->mode_btn_down_ts) >= XPAD360W_POWEROFF_TIMEOUT) {
		xpad360w_poweroff_controller(xpad);
		xpad->mode_btn_down_ts = 0;
		return;
	}

	schedule_delayed_work(&xpad->poweroff_work, msecs_to_jiffies(200));
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

/*
 * xpad360w_process_packet
 *
 * Completes a request by converting the data into events for the
 * input subsystem. It is version for xbox 360 wireless controller.
 *
 * Byte.Bit
 * 00.1 - Status change: The controller or headset has connected/disconnected
 *                       Bits 01.7 and 01.6 are valid
 * 01.7 - Controller present
 * 01.6 - Headset present
 * 01.1 - Pad state (Bytes 4+) valid
 *
 */
static void xpad360w_process_packet(struct usb_xpad *xpad, u16 cmd, unsigned char *data)
{
	struct input_dev *dev;
	bool present;

	/* Presence change */
	if (data[0] & 0x08) {
		present = (data[1] & 0x80) != 0;

		if (xpad->pad_present != present) {
			xpad->pad_present = present;
			schedule_work(&xpad->work);
		}
	}

	/* Valid pad data */
	if (data[1] != 0x1)
		return;

	rcu_read_lock();
	dev = rcu_dereference(xpad->x360w_dev);
	if (dev)
		xpad360_process_packet(xpad, dev, cmd, &data[4]);
	rcu_read_unlock();
}

/*
 *	xpadone_process_packet
 *
 *	Completes a request by converting the data into events for the
 *	input subsystem. This version is for the Xbox One controller.
 *
 *	The report format was gleaned from
 *	https://github.com/kylelemons/xbox/blob/master/xbox.go
 */
static void xpadone_process_packet(struct usb_xpad *xpad, u16 cmd, unsigned char *data)
{
	struct input_dev *dev = xpad->dev;
	bool do_sync = false;
	int dpad_value;

	/* the xbox button has its own special report */
	if (data[0] == GIP_CMD_VIRTUAL_KEY) {
		/*
		 * The Xbox One S controller requires these reports to be
		 * acked otherwise it continues sending them forever and
		 * won't report further mode button events.
		 */
		if (data[1] == (GIP_OPT_ACK | GIP_OPT_INTERNAL))
			xpadone_ack_mode_report(xpad, data[2]);

		input_report_key(dev, BTN_MODE, data[4] & BIT(0));

		do_sync = true;
	} else if (data[0] == GIP_CMD_FIRMWARE) {
		/* Some packet formats force us to use this separate to poll paddle inputs */
		if (xpad->packet_type == PKT_XBE2_FW_5_11) {
			/* Mute paddles if controller is in a custom profile slot
			 * Checked by looking at the active profile slot to
			 * verify it's the default slot
			 */
			if (data[19] != 0)
				data[18] = 0;

			/* Elite Series 2 split packet paddle bits */
			input_report_key(dev, BTN_TRIGGER_HAPPY5, data[18] & BIT(0));
			input_report_key(dev, BTN_TRIGGER_HAPPY6, data[18] & BIT(1));
			input_report_key(dev, BTN_TRIGGER_HAPPY7, data[18] & BIT(2));
			input_report_key(dev, BTN_TRIGGER_HAPPY8, data[18] & BIT(3));

			do_sync = true;
		}
	} else if (data[0] == GIP_CMD_INPUT) { /* The main valid packet type for inputs */
		/* menu/view buttons */
		input_report_key(dev, BTN_START,  data[4] & BIT(2));
		input_report_key(dev, BTN_SELECT, data[4] & BIT(3));
		if (xpad->mapping & MAP_SELECT_BUTTON)
			input_report_key(dev, KEY_RECORD, data[22] & BIT(0) || /* 8BitDo: */ data[18] & BIT(0));

		/* buttons A,B,X,Y */
		input_report_key(dev, BTN_A,	data[4] & BIT(4));
		input_report_key(dev, BTN_B,	data[4] & BIT(5));
		input_report_key(dev, BTN_X,	data[4] & BIT(6));
		input_report_key(dev, BTN_Y,	data[4] & BIT(7));

		/* digital pad */
		if (xpad->mapping & MAP_DPAD_TO_BUTTONS) {
			/* dpad as buttons (left, right, up, down) */
			input_report_key(dev, BTN_TRIGGER_HAPPY1, data[5] & BIT(2));
			input_report_key(dev, BTN_TRIGGER_HAPPY2, data[5] & BIT(3));
			input_report_key(dev, BTN_TRIGGER_HAPPY3, data[5] & BIT(0));
			input_report_key(dev, BTN_TRIGGER_HAPPY4, data[5] & BIT(1));
		} else {
			input_report_abs(dev, ABS_HAT0X,
					!!(data[5] & 0x08) - !!(data[5] & 0x04));
			input_report_abs(dev, ABS_HAT0Y,
					!!(data[5] & 0x02) - !!(data[5] & 0x01));
		}

		/* TL/TR */
		input_report_key(dev, BTN_TL,	data[5] & BIT(4));
		input_report_key(dev, BTN_TR,	data[5] & BIT(5));

		/* stick press left/right */
		input_report_key(dev, BTN_THUMBL, data[5] & BIT(6));
		input_report_key(dev, BTN_THUMBR, data[5] & BIT(7));

		if (!(xpad->mapping & MAP_STICKS_TO_NULL)) {
			/* left stick */
			input_report_abs(dev, ABS_X,
					(__s16) le16_to_cpup((__le16 *)(data + 10)));
			input_report_abs(dev, ABS_Y,
					~(__s16) le16_to_cpup((__le16 *)(data + 12)));

			/* right stick */
			input_report_abs(dev, ABS_RX,
					(__s16) le16_to_cpup((__le16 *)(data + 14)));
			input_report_abs(dev, ABS_RY,
					~(__s16) le16_to_cpup((__le16 *)(data + 16)));
		}

		/* triggers left/right */
		if (xpad->mapping & MAP_TRIGGERS_TO_BUTTONS) {
			input_report_key(dev, BTN_TL2,
					(__u16) le16_to_cpup((__le16 *)(data + 6)));
			input_report_key(dev, BTN_TR2,
					(__u16) le16_to_cpup((__le16 *)(data + 8)));
		} else {
			input_report_abs(dev, ABS_Z,
					(__u16) le16_to_cpup((__le16 *)(data + 6)));
			input_report_abs(dev, ABS_RZ,
					(__u16) le16_to_cpup((__le16 *)(data + 8)));
		}

		/* paddle handling */
		/* based on SDL's SDL_hidapi_xboxone.c */
		if (xpad->mapping & MAP_PADDLES) {
			if (xpad->packet_type == PKT_XBE1) {
				/* Mute paddles if controller has a custom mapping applied.
				 * Checked by comparing the current mapping
				 * config against the factory mapping config
				 */
				if (memcmp(&data[4], &data[18], 2) != 0)
					data[32] = 0;

				/* OG Elite Series Controller paddle bits */
				input_report_key(dev, BTN_TRIGGER_HAPPY5, data[32] & BIT(1));
				input_report_key(dev, BTN_TRIGGER_HAPPY6, data[32] & BIT(3));
				input_report_key(dev, BTN_TRIGGER_HAPPY7, data[32] & BIT(0));
				input_report_key(dev, BTN_TRIGGER_HAPPY8, data[32] & BIT(2));
			} else if (xpad->packet_type == PKT_XBE2_FW_OLD) {
				/* Mute paddles if controller has a custom mapping applied.
				 * Checked by comparing the current mapping
				 * config against the factory mapping config
				 */
				if (data[19] != 0)
					data[18] = 0;

				/* Elite Series 2 4.x firmware paddle bits */
				input_report_key(dev, BTN_TRIGGER_HAPPY5, data[18] & BIT(0));
				input_report_key(dev, BTN_TRIGGER_HAPPY6, data[18] & BIT(1));
				input_report_key(dev, BTN_TRIGGER_HAPPY7, data[18] & BIT(2));
				input_report_key(dev, BTN_TRIGGER_HAPPY8, data[18] & BIT(3));
			} else if (xpad->packet_type == PKT_XBE2_FW_5_EARLY) {
				/* Mute paddles if controller has a custom mapping applied.
				 * Checked by comparing the current mapping
				 * config against the factory mapping config
				 */
				if (data[23] != 0)
					data[22] = 0;

				/* Elite Series 2 5.x firmware paddle bits
				 * (before the packet was split)
				 */
				input_report_key(dev, BTN_TRIGGER_HAPPY5, data[22] & BIT(0));
				input_report_key(dev, BTN_TRIGGER_HAPPY6, data[22] & BIT(1));
				input_report_key(dev, BTN_TRIGGER_HAPPY7, data[22] & BIT(2));
				input_report_key(dev, BTN_TRIGGER_HAPPY8, data[22] & BIT(3));
			}
		}

		do_sync = true;

	} else if (data[0] == 0X21) { /* The main valid packet type for GHL inputs */
		/* Mapping chosen to be coherent with GHL dongles of other consoles */

		/* The 6 fret buttons */
		input_report_key(dev, BTN_B, data[4] & BIT(1));
		input_report_key(dev, BTN_X, data[4] & BIT(2));
		input_report_key(dev, BTN_Y, data[4] & BIT(3));
		input_report_key(dev, BTN_A, data[4] & BIT(0));
		input_report_key(dev, BTN_TL, data[4] & BIT(4));
		input_report_key(dev, BTN_TR, data[4] & BIT(5));

		/* D-pad */
		dpad_value = data[6] & 0xF;
		if (dpad_value > 7)
			dpad_value = 8;

		input_report_abs(dev, ABS_HAT0X, dpad_mapping[dpad_value].x);
		input_report_abs(dev, ABS_HAT0Y, dpad_mapping[dpad_value].y);

		/* Strum bar */
		input_report_abs(dev, ABS_Y, ((data[8] - 0x80) << 9));

		/* Tilt Sensor */
		input_report_abs(dev, ABS_Z, ((data[9] - 0x80) << 9));

		/* Whammy bar */
		input_report_abs(dev, ABS_RZ, ((data[10] - 0x80) << 9));

		/* Power Button */
		input_report_key(dev, BTN_THUMBR, data[5] & BIT(4));

		/* GHTV button */
		input_report_key(dev, BTN_START, data[5] & BIT(2));

		/* Hero Power button */
		input_report_key(dev, BTN_MODE,	data[5] & BIT(0));

		/* Pause button */
		input_report_key(dev, BTN_THUMBL, data[5] & BIT(1));

		do_sync = true;
	}

	if (do_sync)
		input_sync(dev);
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
	case XTYPE_XBOX360W:
		xpad360w_process_packet(xpad, 0, xpad->idata);
		break;
	case XTYPE_XBOXONE:
		xpadone_process_packet(xpad, 0, xpad->idata);
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
static bool xpad_prepare_next_init_packet(struct usb_xpad *xpad)
{
	const struct xboxone_init_packet *init_packet;

	if (xpad->xtype != XTYPE_XBOXONE)
		return false;

	/* Perform initialization sequence for Xbox One pads that require it */
	while (xpad->init_seq < ARRAY_SIZE(xboxone_init_packets)) {
		init_packet = &xboxone_init_packets[xpad->init_seq++];

		if (init_packet->idVendor != 0 &&
		    init_packet->idVendor != xpad->dev->id.vendor)
			continue;

		if (init_packet->idProduct != 0 &&
		    init_packet->idProduct != xpad->dev->id.product)
			continue;

		/* This packet applies to our device, so prepare to send it */
		memcpy(xpad->odata, init_packet->data, init_packet->len);
		xpad->irq_out->transfer_buffer_length = init_packet->len;

		/* Update packet with current sequence number */
		xpad->odata[2] = xpad->odata_serial++;
		return true;
	}

	return false;
}

/* Callers must hold xpad->odata_lock spinlock */
static bool xpad_prepare_next_out_packet(struct usb_xpad *xpad)
{
	struct xpad_output_packet *pkt, *packet = NULL;
	int i;

	/* We may have init packets to send before we can send user commands */
	if (xpad_prepare_next_init_packet(xpad))
		return true;

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

static int xpad_start_xbox_one(struct usb_xpad *xpad)
{
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&xpad->odata_lock, flags);

	/*
	 * Begin the init sequence by attempting to send a packet.
	 * We will cycle through the init packet sequence before
	 * sending any packets from the output ring.
	 */
	xpad->init_seq = 0;
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

static void xpadone_ack_mode_report(struct usb_xpad *xpad, u8 seq_num)
{
	unsigned long flags;
	struct xpad_output_packet *packet =
			&xpad->out_packets[XPAD_OUT_CMD_IDX];
	static const u8 mode_report_ack[] = {
		GIP_CMD_ACK, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(9),
		0x00, GIP_CMD_VIRTUAL_KEY, GIP_OPT_INTERNAL, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	spin_lock_irqsave(&xpad->odata_lock, flags);

	packet->len = sizeof(mode_report_ack);
	memcpy(packet->data, mode_report_ack, packet->len);
	packet->data[2] = seq_num;
	packet->pending = true;

	/* Reset the sequence so we send out the ack now */
	xpad->last_out_packet = -1;
	xpad_try_sending_next_out_packet(xpad);

	spin_unlock_irqrestore(&xpad->odata_lock, flags);
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

	if (xpad->xtype == XTYPE_XBOXONE) {
		error = xpad_start_xbox_one(xpad);
		if (error) {
			usb_kill_urb(xpad->irq_in);
			return error;
		}
	}

	return 0;
}

static void xpad_stop_input(struct usb_xpad *xpad)
{
	usb_kill_urb(xpad->irq_in);
}

static void xpad360w_poweroff_controller(struct usb_xpad *xpad)
{
	unsigned long flags;
	struct xpad_output_packet *packet =
			&xpad->out_packets[XPAD_OUT_CMD_IDX];

	spin_lock_irqsave(&xpad->odata_lock, flags);

	packet->data[0] = 0x00;
	packet->data[1] = 0x00;
	packet->data[2] = 0x08;
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

	/* Reset the sequence so we send out poweroff now */
	xpad->last_out_packet = -1;
	xpad_try_sending_next_out_packet(xpad);

	spin_unlock_irqrestore(&xpad->odata_lock, flags);
}

static int xpad360w_start_input(struct usb_xpad *xpad)
{
	int error;

	error = usb_submit_urb(xpad->irq_in, GFP_KERNEL);
	if (error)
		return -EIO;

	/*
	 * Send presence packet.
	 * This will force the controller to resend connection packets.
	 * This is useful in the case we activate the module after the
	 * adapter has been plugged in, as it won't automatically
	 * send us info about the controllers.
	 */
	error = xpad_inquiry_pad_presence(xpad);
	if (error) {
		usb_kill_urb(xpad->irq_in);
		return error;
	}

	INIT_DELAYED_WORK(&xpad->poweroff_work, xpad360w_poweroff_work);

	return 0;
}

static void xpad360w_stop_input(struct usb_xpad *xpad)
{
	usb_kill_urb(xpad->irq_in);

	/* Make sure we are done with presence work if it was scheduled */
	flush_work(&xpad->work);
	flush_delayed_work(&xpad->poweroff_work);
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
		/* GHL Strum bar */
		if ((xpad->xtype == XTYPE_XBOXONE) && (xpad->quirks & QUIRK_GHL_XBOXONE)) {
			input_set_abs_params(input_dev, abs, -32767, 32767, 0, 0);
			break;
		}
	case ABS_RX:
	case ABS_RY:	/* the two sticks */
		input_set_abs_params(input_dev, abs, -32768, 32767, 16, 128);
		break;
	case ABS_Z:
		/* GHL Tilt sensor */
		if ((xpad->xtype == XTYPE_XBOXONE) && (xpad->quirks & QUIRK_GHL_XBOXONE)) {
			input_set_abs_params(input_dev, abs, -32767, 32767, 0, 0);
			break;
		}
	case ABS_RZ:	/* the triggers (if mapped to axes) */
		if (xpad->xtype == XTYPE_XBOXONE) {
			/* GHL Whammy bar */
			if (xpad->quirks & QUIRK_GHL_XBOXONE)
				input_set_abs_params(input_dev, abs, -32767, 32767, 0, 0);
			else
				input_set_abs_params(input_dev, abs, 0, 1023, 0, 0);
		} else
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

	if (xpad->xtype == XTYPE_XBOX360W) {
		/* x360w controllers and the receiver have different ids */
		input_dev->id.product = 0x02a1;
	}

	input_dev->dev.parent = &xpad->intf->dev;

	input_set_drvdata(input_dev, xpad);

	if (xpad->xtype != XTYPE_XBOX360W) {
		input_dev->open = xpad_open;
		input_dev->close = xpad_close;
	}

	if (!(xpad->mapping & MAP_STICKS_TO_NULL)) {
		/* set up axes */
		for (i = 0; xpad_abs[i] >= 0; i++)
			xpad_set_up_abs(input_dev, xpad_abs[i]);
	}

	/* set up standard buttons */
	for (i = 0; xpad_common_btn[i] >= 0; i++)
		input_set_capability(input_dev, EV_KEY, xpad_common_btn[i]);

	/* set up model-specific ones */
	if (xpad->xtype == XTYPE_XBOX360 || xpad->xtype == XTYPE_XBOX360W ||
	    xpad->xtype == XTYPE_XBOXONE) {
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

	/*
	 * This should be a simple else block. However historically
	 * xbox360w has mapped DPAD to buttons while xbox360 did not. This
	 * made no sense, but now we can not just switch back and have to
	 * support both behaviors.
	 */
	if (!(xpad->mapping & MAP_DPAD_TO_BUTTONS) ||
	    xpad->xtype == XTYPE_XBOX360W) {
		for (i = 0; xpad_abs_pad[i] >= 0; i++)
			xpad_set_up_abs(input_dev, xpad_abs_pad[i]);
	}

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

	// Comentando - achar que eh apenas relacionado a detecção de controles da microsoft elite
	
	/* Packet type detection  -> o que eh esse packet type detection*/ 
	// if (le16_to_cpu(udev->descriptor.idVendor) == 0x045e) { /*  faz a detecnao se eh um Microsoft controllers */
	// 	if (le16_to_cpu(udev->descriptor.idProduct) == 0x02e3) {
	// 		/* The original elite controller always uses the oldest
	// 		 * type of extended packet
	// 		 */
	// 		xpad->packet_type = PKT_XBE1;
	// 	} else if (le16_to_cpu(udev->descriptor.idProduct) == 0x0b00) {
	// 		/* The elite 2 controller has seen multiple packet
	// 		 * revisions. These are tied to specific firmware
	// 		 * versions
	// 		 */
	// 		if (le16_to_cpu(udev->descriptor.bcdDevice) < 0x0500) {
	// 			/* This is the format that the Elite 2 used
	// 			 * prior to the BLE update
	// 			 */
	// 			xpad->packet_type = PKT_XBE2_FW_OLD;
	// 		} else if (le16_to_cpu(udev->descriptor.bcdDevice) <
	// 			   0x050b) {
	// 			/* This is the format that the Elite 2 used
	// 			 * prior to the update that split the packet
	// 			 */
	// 			xpad->packet_type = PKT_XBE2_FW_5_EARLY;
	// 		} else {
	// 			/* The split packet format that was introduced
	// 			 * in firmware v5.11
	// 			 */
	// 			xpad->packet_type = PKT_XBE2_FW_5_11;
	// 		}
	// 	}
	// }

	// Deletar pq nao 
	if (xpad->xtype == XTYPE_XBOX360W) {
		/*
		 * Submit the int URB immediately rather than waiting for open
		 * because we get status messages from the device whether
		 * or not any controllers are attached.  In fact, it's
		 * exactly the message that a controller has arrived that
		 * we're waiting for.
		 */
		error = xpad360w_start_input(xpad);
		if (error)
			goto err_deinit_output;
		/*
		 * Wireless controllers require RESET_RESUME to work properly
		 * after suspend. Ideally this quirk should be in usb core
		 * quirk list, but we have too many vendors producing these
		 * controllers and we'd need to maintain 2 identical lists
		 * here in this driver and in usb core.
		 */
		udev->quirks |= USB_QUIRK_RESET_RESUME;
	} else {
		error = xpad_init_input(xpad);
		if (error)
			goto err_deinit_output;
	}
	
   // pequisar o que é quirk usb
	if (xpad->quirks & QUIRK_GHL_XBOXONE) {

		xpad->ghl_urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!xpad->ghl_urb)
			return -ENOMEM;

		error = ghl_init_urb(xpad, udev, ghl_xboxone_magic_data, ARRAY_SIZE(ghl_xboxone_magic_data), ep_irq_out);

		if (error)
			return error;

		timer_setup(&xpad->ghl_poke_timer, ghl_magic_poke, 0);
		mod_timer(&xpad->ghl_poke_timer, jiffies + GHL_GUITAR_POKE_INTERVAL*HZ);
	}
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

	if (xpad->xtype == XTYPE_XBOX360W)
		xpad360w_stop_input(xpad);

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

	if (xpad->xtype == XTYPE_XBOX360W) {
		/*
		 * Wireless controllers always listen to input so
		 * they are notified when controller shows up
		 * or goes away.
		 */
		xpad360w_stop_input(xpad);

		/*
		 * The wireless adapter is going off now, so the
		 * gamepads are going to become disconnected.
		 * Unless explicitly disabled, power them down
		 * so they don't just sit there flashing.
		 */
		if (auto_poweroff && xpad->pad_present)
			xpad360w_poweroff_controller(xpad);
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			xpad_stop_input(xpad);
		mutex_unlock(&input->mutex);
	}

	xpad_stop_output(xpad);

	return 0;
}

static int xpad_resume(struct usb_interface *intf)
{
	struct usb_xpad *xpad = usb_get_intfdata(intf);
	struct input_dev *input = xpad->dev;
	int retval = 0;

	if (xpad->xtype == XTYPE_XBOX360W) {
		retval = xpad360w_start_input(xpad);
	} else {
		mutex_lock(&input->mutex);
		if (input->users) {
			retval = xpad_start_input(xpad);
		} else if (xpad->xtype == XTYPE_XBOXONE) {
			/*
			 * Even if there are no users, we'll send Xbox One pads
			 * the startup sequence so they don't sit there and
			 * blink until somebody opens the input device again.
			 */
			retval = xpad_start_xbox_one(xpad);
		}
		mutex_unlock(&input->mutex);
	}

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
