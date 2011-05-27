/*
	LPCUSB, an USB device driver for LPC microcontrollers	
	Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright
	   notice, this list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright
	   notice, this list of conditions and the following disclaimer in the
	   documentation and/or other materials provided with the distribution.
	3. The name of the author may not be used to endorse or promote products
	   derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
	OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
	IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
	THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	Modified by Sivan Toledo.
*/

#define	TRUE	1					/**< TRUE */
#define FALSE	0					/**< FALSE */

#ifndef NULL
#define NULL	((void*)0)			/**< NULL pointer */
#endif

/* some other useful macros */
#define MIN(x,y)	((x)<(y)?(x):(y))	/**< MIN */
#define MAX(x,y)	((x)>(y)?(x):(y))	/**< MAX */

#define LE_WORD(x)    ((x)&0xFF),((x)>>8)

/******************************************************/
/* USB Hardware Layer                                 */
/******************************************************/

/* attributes in interface descriptor */
#define USB_ATTR_BUS_POWERED  (1<<7)
#define USB_ATTR_SELF_POWERED (1<<6)
#define USB_ATTR_REMOTE_WAKE  (1<<5)

#define USB_EP_IN  0x80
#define USB_EP_OUT 0x00

#define USB_EP_ATTR_CONTROL     0
#define USB_EP_ATTR_ISOCHRONOUS 1
#define USB_EP_ATTR_BULK        2
#define USB_EP_ATTR_INTERRUPT   3

// endpoint status sent through callback
#define EP_STATUS_DATA    (1<<0)    /**< EP has data */
#define EP_STATUS_STALLED (1<<1)    /**< EP is stalled */
#define EP_STATUS_SETUP   (1<<2)    /**< EP received setup packet */
#define EP_STATUS_ERROR   (1<<3)    /**< EP data was overwritten by setup packet */
#define EP_STATUS_NACKED  (1<<4)    /**< EP sent NAK */

// device status sent through callback
#define DEV_STATUS_CONNECT (1<<0) /**< device just got connected */
#define DEV_STATUS_SUSPEND (1<<2) /**< device entered suspend state */
#define DEV_STATUS_RESET   (1<<4) /**< device just got reset */

// interrupt bits for NACK events in USBHwNakIntEnable
// (these bits conveniently coincide with the LPC214x USB controller bit)
#define INACK_CI    (1<<1)      /**< interrupt on NACK for control in */
#define INACK_CO    (1<<2)      /**< interrupt on NACK for control out */
#define INACK_II    (1<<3)      /**< interrupt on NACK for interrupt in */
#define INACK_IO    (1<<4)      /**< interrupt on NACK for interrupt out */
#define INACK_BI    (1<<5)      /**< interrupt on NACK for bulk in */
#define INACK_BO    (1<<6)      /**< interrupt on NACK for bulk out */

typedef void (usb_ep_handler_t)(uint8_t bEP, uint8_t bEPStatus);

static void usbEnableNAKInterrupts(uint8_t bIntBits);

static void usbWrite(uint8_t ep, uint8_t *buffer, int len);
static int usbRead(uint8_t ep, uint8_t *buffer, int buffer_len);
//static inline void usbConfigureDevice();
//static inline void usbUnconfigureDevice();

#define REQTYPE_GET_DIR(x)		(((x)>>7)&0x01)
#define REQTYPE_GET_TYPE(x)		(((x)>>5)&0x03)
#define REQTYPE_GET_RECIP(x)	((x)&0x1F)

#define REQTYPE_DIR_TO_DEVICE	0
#define REQTYPE_DIR_TO_HOST		1

#define REQTYPE_TYPE_STANDARD	0
#define REQTYPE_TYPE_CLASS		1
#define REQTYPE_TYPE_VENDOR		2
#define REQTYPE_TYPE_RESERVED	3

#define REQTYPE_RECIP_DEVICE    0
#define REQTYPE_RECIP_INTERFACE	1
#define REQTYPE_RECIP_ENDPOINT	2
#define REQTYPE_RECIP_OTHER     3

/* standard requests */
#define	REQ_GET_STATUS        0x00
#define REQ_CLEAR_FEATURE     0x01
#define REQ_SET_FEATURE       0x03
#define REQ_SET_ADDRESS       0x05
#define REQ_GET_DESCRIPTOR		0x06
#define REQ_SET_DESCRIPTOR		0x07
#define REQ_GET_CONFIGURATION	0x08
#define REQ_SET_CONFIGURATION	0x09
#define REQ_GET_INTERFACE     0x0A
#define REQ_SET_INTERFACE     0x0B
#define REQ_SYNCH_FRAME       0x0C

/* class requests HID */
#define HID_GET_REPORT	 0x01
#define HID_GET_IDLE		 0x02
#define HID_GET_PROTOCOL 0x03
#define HID_SET_REPORT	 0x09
#define HID_SET_IDLE		 0x0A
#define HID_SET_PROTOCOL 0x0B

/* feature selectors */
#define FEA_ENDPOINT_HALT	0x00
#define FEA_REMOTE_WAKEUP	0x01
#define FEA_TEST_MODE			0x02

#define MAX_PACKET_SIZE0 64     /**< maximum packet size for EP 0 */


/*
	USB descriptors
*/

/** USB descriptor header */
//typedef struct {
//	uint8_t	bLength;			/**< descriptor length */
//	uint8_t	bDescriptorType;	/**< descriptor type */
//} TUSBDescHeader;

#define DESC_DEVICE           1
#define DESC_CONFIGURATION    2
#define DESC_STRING           3
#define DESC_INTERFACE        4
#define DESC_ENDPOINT         5
#define DESC_DEVICE_QUALIFIER	6
#define DESC_OTHER_SPEED      7
#define DESC_INTERFACE_POWER	8

#define DESC_HID_HID      0x21
#define DESC_HID_REPORT		0x22
#define DESC_HID_PHYSICAL	0x23

#define GET_DESC_TYPE(x)	(((x)>>8)&0xFF)
#define GET_DESC_INDEX(x)	((x)&0xFF)

/** setup packet definitions */
static struct {
  uint8_t  type;
  uint8_t  request;     
  uint16_t data;       
  uint16_t index;       
  uint16_t length;      
} usbRequest;


static uint8_t* usbControlTransferPtr; /**< pointer to data buffer */
static int      usbControlTransferLen;   /**< total length of control transfer */

/* general descriptor field offsets */
#define DESC_bLength				 0	/**< length offset */
#define DESC_bDescriptorType 1	/**< descriptor type offset */	

/* config descriptor field offsets */
#define CONF_DESC_wTotalLength        2 /**< total length offset */
#define CONF_DESC_bConfigurationValue	5 /**< configuration value offset */	
#define CONF_DESC_bmAttributes        7 /**< configuration characteristics */

/* interface descriptor field offsets */
#define INTF_DESC_bAlternateSetting	3 /**< alternate setting offset */

/* endpoint descriptor field offsets */
#define ENDP_DESC_bEndpointAddress 2 /**< endpoint address offset */
#define ENDP_DESC_wMaxPacketSize	 4 /**< maximum packet size offset */

static void usbEP0OutHandler(uint8_t bEP, uint8_t bEPStat);
static void usbEP0InHandler(uint8_t bEP, uint8_t bEPStat);

static void usbInit(void);

static bool usbGetDescriptor(uint16_t wTypeIndex, uint16_t wLangID);
