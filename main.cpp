#include <stdint.h>
#include <functional>
#include <string.h>

#include "chip.h"
#include "adc_11xx.h"
#include "gpio_11xx_1.h"
#include "gpiogroup_11xx.h"
#include "ssp_11xx.h"
#include "printf.h"
#include "usbd_rom_api.h"


#include "duck_font.h"

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000
#define I2C_DEFAULT_SPEED    SPEED_400KHZ
#define I2C_FASTPLUS_BIT     0

#if (I2C_DEFAULT_SPEED > SPEED_400KHZ)
#undef  I2C_FASTPLUS_BIT
#define I2C_FASTPLUS_BIT IOCON_FASTI2C_EN
#endif

#define NO_FLASH

#ifdef __cplusplus
extern "C"
{
#endif

const uint8_t USB_DeviceDescriptor[] = {
	USB_DEVICE_DESC_SIZE,				/* bLength */
	USB_DEVICE_DESCRIPTOR_TYPE,			/* bDescriptorType */
	WBVAL(0x0200),						/* bcdUSB */
	0xEF,								/* bDeviceClass */
	0x02,								/* bDeviceSubClass */
	0x01,								/* bDeviceProtocol */
	USB_MAX_PACKET0,					/* bMaxPacketSize0 */
	WBVAL(0x1FC9),						/* idVendor */
	WBVAL(0x0083),						/* idProduct */
	WBVAL(0x0100),						/* bcdDevice */
	0x01,								/* iManufacturer */
	0x02,								/* iProduct */
	0x03,								/* iSerialNumber */
	0x01								/* bNumConfigurations */
};

uint8_t USB_FsConfigDescriptor[] = {
	/* Configuration 1 */
	USB_CONFIGURATION_DESC_SIZE,			/* bLength */
	USB_CONFIGURATION_DESCRIPTOR_TYPE,		/* bDescriptorType */
	WBVAL(									/* wTotalLength */
		USB_CONFIGURATION_DESC_SIZE     +
		USB_INTERFACE_ASSOC_DESC_SIZE   +	/* interface association descriptor */
		USB_INTERFACE_DESC_SIZE         +	/* communication control interface */
		0x0013                          +	/* CDC functions */
		1 * USB_ENDPOINT_DESC_SIZE      +	/* interrupt endpoint */
		USB_INTERFACE_DESC_SIZE         +	/* communication data interface */
		2 * USB_ENDPOINT_DESC_SIZE      +	/* bulk endpoints */
		0
		),
	0x02,									/* bNumInterfaces */
	0x01,									/* bConfigurationValue */
	0x00,									/* iConfiguration */
	USB_CONFIG_SELF_POWERED,				/* bmAttributes  */
	USB_CONFIG_POWER_MA(500),				/* bMaxPower */

	/* Interface association descriptor IAD*/
	USB_INTERFACE_ASSOC_DESC_SIZE,		/* bLength */
	USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,	/* bDescriptorType */
	USB_CDC_CIF_NUM,					/* bFirstInterface */
	0x02,								/* bInterfaceCount */
	CDC_COMMUNICATION_INTERFACE_CLASS,	/* bFunctionClass */
	CDC_ABSTRACT_CONTROL_MODEL,			/* bFunctionSubClass */
	0x00,								/* bFunctionProtocol */
	0x04,								/* iFunction */

	/* Interface 0, Alternate Setting 0, Communication class interface descriptor */
	USB_INTERFACE_DESC_SIZE,			/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_CIF_NUM,					/* bInterfaceNumber: Number of Interface */
	0x00,								/* bAlternateSetting: Alternate setting */
	0x01,								/* bNumEndpoints: One endpoint used */
	CDC_COMMUNICATION_INTERFACE_CLASS,	/* bInterfaceClass: Communication Interface Class */
	CDC_ABSTRACT_CONTROL_MODEL,			/* bInterfaceSubClass: Abstract Control Model */
	0x00,								/* bInterfaceProtocol: no protocol used */
	0x04,								/* iInterface: */
	/* Header Functional Descriptor*/
	0x05,								/* bLength: CDC header Descriptor size */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_HEADER,							/* bDescriptorSubtype: Header Func Desc */
	WBVAL(CDC_V1_10),					/* bcdCDC 1.10 */
	/* Call Management Functional Descriptor*/
	0x05,								/* bFunctionLength */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_CALL_MANAGEMENT,				/* bDescriptorSubtype: Call Management Func Desc */
	0x01,								/* bmCapabilities: device handles call management */
	USB_CDC_DIF_NUM,					/* bDataInterface: CDC data IF ID */
	/* Abstract Control Management Functional Descriptor*/
	0x04,								/* bFunctionLength */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_ABSTRACT_CONTROL_MANAGEMENT,	/* bDescriptorSubtype: Abstract Control Management desc */
	0x02,								/* bmCapabilities: SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported */
	/* Union Functional Descriptor*/
	0x05,								/* bFunctionLength */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_UNION,							/* bDescriptorSubtype: Union func desc */
	USB_CDC_CIF_NUM,					/* bMasterInterface: Communication class interface is master */
	USB_CDC_DIF_NUM,					/* bSlaveInterface0: Data class interface is slave 0 */
	/* Endpoint 1 Descriptor*/
	USB_ENDPOINT_DESC_SIZE,				/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_INT_EP,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_INTERRUPT,		/* bmAttributes */
	WBVAL(0x0010),						/* wMaxPacketSize */
	0x02,			/* 2ms */           /* bInterval */

	/* Interface 1, Alternate Setting 0, Data class interface descriptor*/
	USB_INTERFACE_DESC_SIZE,			/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_DIF_NUM,					/* bInterfaceNumber: Number of Interface */
	0x00,								/* bAlternateSetting: no alternate setting */
	0x02,								/* bNumEndpoints: two endpoints used */
	CDC_DATA_INTERFACE_CLASS,			/* bInterfaceClass: Data Interface Class */
	0x00,								/* bInterfaceSubClass: no subclass available */
	0x00,								/* bInterfaceProtocol: no protocol used */
	0x04,								/* iInterface: */
	/* Endpoint, EP Bulk Out */
	USB_ENDPOINT_DESC_SIZE,				/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_OUT_EP,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_BULK,				/* bmAttributes */
	WBVAL(USB_FS_MAX_BULK_PACKET),		/* wMaxPacketSize */
	0x00,								/* bInterval: ignore for Bulk transfer */
	/* Endpoint, EP Bulk In */
	USB_ENDPOINT_DESC_SIZE,				/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_IN_EP,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_BULK,				/* bmAttributes */
	WBVAL(64),							/* wMaxPacketSize */
	0x00,								/* bInterval: ignore for Bulk transfer */
	/* Terminator */
	0									/* bLength */
};

/**
 * USB String Descriptor (optional)
 */
const uint8_t USB_StringDescriptor[] = {
	/* Index 0x00: LANGID Codes */
	0x04,								/* bLength */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	WBVAL(0x0409),	/* US English */    /* wLANGID */
	/* Index 0x01: Manufacturer */
	(3 * 2 + 2),						/* bLength (13 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	'N', 0,
	'X', 0,
	'P', 0,
	/* Index 0x02: Product */
	(9 * 2 + 2),						/* bLength */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	'V', 0,
	'C', 0,
	'O', 0,
	'M', 0,
	' ', 0,
	'P', 0,
	'o', 0,
	'r', 0,
	't', 0,
	/* Index 0x03: Serial Number */
	(6 * 2 + 2),						/* bLength (8 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	'N', 0,
	'X', 0,
	'P', 0,
	'-', 0,
	'7', 0,
	'7', 0,
	/* Index 0x04: Interface 1, Alternate Setting 0 */
	( 4 * 2 + 2),						/* bLength (4 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	'V', 0,
	'C', 0,
	'O', 0,
	'M', 0,
};


#ifdef __cplusplus
}
#endif


#define VCOM_RX_BUF_SZ      512
#define VCOM_TX_CONNECTED   _BIT(8)		/* connection state is for both RX/Tx */
#define VCOM_TX_BUSY        _BIT(0)
#define VCOM_RX_DONE        _BIT(0)
#define VCOM_RX_BUF_FULL    _BIT(1)
#define VCOM_RX_BUF_QUEUED  _BIT(2)
#define VCOM_RX_DB_QUEUED   _BIT(3)

typedef struct VCOM_DATA {
	USBD_HANDLE_T hUsb;
	USBD_HANDLE_T hCdc;
	uint8_t *rx_buff;
	uint16_t rx_rd_count;
	uint16_t rx_count;
	volatile uint16_t tx_flags;
	volatile uint16_t rx_flags;
} VCOM_DATA_T;

VCOM_DATA_T g_vCOM;

/* VCOM bulk EP_IN endpoint handler */
static ErrorCode_t VCOM_bulk_in_hdlr(USBD_HANDLE_T hUsb, void *data, uint32_t event)
{
	VCOM_DATA_T *pVcom = (VCOM_DATA_T *) data;

	if (event == USB_EVT_IN) {
		pVcom->tx_flags &= ~VCOM_TX_BUSY;
	}
	return LPC_OK;
}

/* VCOM bulk EP_OUT endpoint handler */
static ErrorCode_t VCOM_bulk_out_hdlr(USBD_HANDLE_T hUsb, void *data, uint32_t event)
{
	VCOM_DATA_T *pVcom = (VCOM_DATA_T *) data;

	switch (event) {
	case USB_EVT_OUT:
		pVcom->rx_count = USBD_API->hw->ReadEP(hUsb, USB_CDC_OUT_EP, pVcom->rx_buff);
		if (pVcom->rx_flags & VCOM_RX_BUF_QUEUED) {
			pVcom->rx_flags &= ~VCOM_RX_BUF_QUEUED;
			if (pVcom->rx_count != 0) {
				pVcom->rx_flags |= VCOM_RX_BUF_FULL;
			}

		}
		else if (pVcom->rx_flags & VCOM_RX_DB_QUEUED) {
			pVcom->rx_flags &= ~VCOM_RX_DB_QUEUED;
			pVcom->rx_flags |= VCOM_RX_DONE;
		}
		break;

	case USB_EVT_OUT_NAK:
		/* queue free buffer for RX */
		if ((pVcom->rx_flags & (VCOM_RX_BUF_FULL | VCOM_RX_BUF_QUEUED)) == 0) {
			USBD_API->hw->ReadReqEP(hUsb, USB_CDC_OUT_EP, pVcom->rx_buff, VCOM_RX_BUF_SZ);
			pVcom->rx_flags |= VCOM_RX_BUF_QUEUED;
		}
		break;

	default:
		break;
	}

	return LPC_OK;
}

/* Set line coding call back routine */
static ErrorCode_t VCOM_SetLineCode(USBD_HANDLE_T hCDC, CDC_LINE_CODING *line_coding)
{
	VCOM_DATA_T *pVcom = &g_vCOM;

	/* Called when baud rate is changed/set. Using it to know host connection state */
	pVcom->tx_flags = VCOM_TX_CONNECTED;	/* reset other flags */

	return LPC_OK;
}

/* Virtual com port init routine */
ErrorCode_t vcom_init(USBD_HANDLE_T hUsb, USB_CORE_DESCS_T *pDesc, USBD_API_INIT_PARAM_T *pUsbParam)
{
	USBD_CDC_INIT_PARAM_T cdc_param;
	ErrorCode_t ret = LPC_OK;
	uint32_t ep_indx;

	g_vCOM.hUsb = hUsb;
	memset((void *) &cdc_param, 0, sizeof(USBD_CDC_INIT_PARAM_T));
	cdc_param.mem_base = pUsbParam->mem_base;
	cdc_param.mem_size = pUsbParam->mem_size;
	cdc_param.cif_intf_desc = (uint8_t *) find_IntfDesc(pDesc->high_speed_desc, CDC_COMMUNICATION_INTERFACE_CLASS);
	cdc_param.dif_intf_desc = (uint8_t *) find_IntfDesc(pDesc->high_speed_desc, CDC_DATA_INTERFACE_CLASS);
	cdc_param.SetLineCode = VCOM_SetLineCode;

	ret = USBD_API->cdc->init(hUsb, &cdc_param, &g_vCOM.hCdc);

	if (ret == LPC_OK) {
		/* allocate transfer buffers */
		g_vCOM.rx_buff = (uint8_t *) cdc_param.mem_base;
		cdc_param.mem_base += VCOM_RX_BUF_SZ;
		cdc_param.mem_size -= VCOM_RX_BUF_SZ;

		/* register endpoint interrupt handler */
		ep_indx = (((USB_CDC_IN_EP & 0x0F) << 1) + 1);
		ret = USBD_API->core->RegisterEpHandler(hUsb, ep_indx, VCOM_bulk_in_hdlr, &g_vCOM);
		if (ret == LPC_OK) {
			/* register endpoint interrupt handler */
			ep_indx = ((USB_CDC_OUT_EP & 0x0F) << 1);
			ret = USBD_API->core->RegisterEpHandler(hUsb, ep_indx, VCOM_bulk_out_hdlr, &g_vCOM);

		}
		/* update mem_base and size variables for cascading calls. */
		pUsbParam->mem_base = cdc_param.mem_base;
		pUsbParam->mem_size = cdc_param.mem_size;
	}

	return ret;
}

static INLINE uint32_t vcom_connected(void) {
	return g_vCOM.tx_flags & VCOM_TX_CONNECTED;
}

/* Virtual com port buffered read routine */
uint32_t vcom_bread(uint8_t *pBuf, uint32_t buf_len)
{
	VCOM_DATA_T *pVcom = &g_vCOM;
	uint16_t cnt = 0;
	/* read from the default buffer if any data present */
	if (pVcom->rx_count) {
		cnt = (pVcom->rx_count < buf_len) ? pVcom->rx_count : buf_len;
		memcpy(pBuf, pVcom->rx_buff, cnt);
		pVcom->rx_rd_count += cnt;

		/* enter critical section */
		NVIC_DisableIRQ(USB0_IRQn);
		if (pVcom->rx_rd_count >= pVcom->rx_count) {
			pVcom->rx_flags &= ~VCOM_RX_BUF_FULL;
			pVcom->rx_rd_count = pVcom->rx_count = 0;
		}
		/* exit critical section */
		NVIC_EnableIRQ(USB0_IRQn);
	}
	return cnt;

}

/* Virtual com port read routine */
ErrorCode_t vcom_read_req(uint8_t *pBuf, uint32_t len)
{
	VCOM_DATA_T *pVcom = &g_vCOM;

	/* check if we queued Rx buffer */
	if (pVcom->rx_flags & (VCOM_RX_BUF_QUEUED | VCOM_RX_DB_QUEUED)) {
		return ERR_BUSY;
	}
	/* enter critical section */
	NVIC_DisableIRQ(USB0_IRQn);
	/* if not queue the request and return 0 bytes */
	USBD_API->hw->ReadReqEP(pVcom->hUsb, USB_CDC_OUT_EP, pBuf, len);
	/* exit critical section */
	NVIC_EnableIRQ(USB0_IRQn);
	pVcom->rx_flags |= VCOM_RX_DB_QUEUED;

	return LPC_OK;
}

/* Gets current read count. */
uint32_t vcom_read_cnt(void)
{
	VCOM_DATA_T *pVcom = &g_vCOM;
	uint32_t ret = 0;

	if (pVcom->rx_flags & VCOM_RX_DONE) {
		ret = pVcom->rx_count;
		pVcom->rx_count = 0;
	}

	return ret;
}

/* Virtual com port write routine*/
uint32_t vcom_write(const uint8_t *pBuf, uint32_t len)
{
	VCOM_DATA_T *pVcom = &g_vCOM;
	uint32_t ret = 0;

	if ( (pVcom->tx_flags & VCOM_TX_CONNECTED) && ((pVcom->tx_flags & VCOM_TX_BUSY) == 0) ) {
		pVcom->tx_flags |= VCOM_TX_BUSY;

		/* enter critical section */
		NVIC_DisableIRQ(USB0_IRQn);
		ret = USBD_API->hw->WriteEP(pVcom->hUsb, USB_CDC_IN_EP, pBuf, len);
		/* exit critical section */
		NVIC_EnableIRQ(USB0_IRQn);
	}

	return ret;
}

static USBD_HANDLE_T g_hUsb;
static uint8_t g_rxBuff[256];
const  USBD_API_T *g_pUsbApi;

/* Initialize pin and clocks for USB0/USB1 port */
static void usb_pin_clk_init(void)
{
	/* enable USB main clock */
	Chip_Clock_SetUSBClockSource(SYSCTL_USBCLKSRC_PLLOUT, 1);
	/* Enable AHB clock to the USB block and USB RAM. */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USB);
//	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USBRAM);
	/* power UP USB Phy */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_USBPAD_PD);
}

/* Find the address of interface descriptor for given class type. */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass)
{
	USB_COMMON_DESCRIPTOR *pD;
	USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
	uint32_t next_desc_adr;

	pD = (USB_COMMON_DESCRIPTOR *) pDesc;
	next_desc_adr = (uint32_t) pDesc;

	while (pD->bLength) {
		/* is it interface descriptor */
		if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE) {

			pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) pD;
			/* did we find the right interface descriptor */
			if (pIntfDesc->bInterfaceClass == intfClass) {
				break;
			}
		}
		pIntfDesc = 0;
		next_desc_adr = (uint32_t) pD + pD->bLength;
		pD = (USB_COMMON_DESCRIPTOR *) next_desc_adr;
	}

	return pIntfDesc;
}


static int usb_init()
{
	USBD_API_INIT_PARAM_T usb_param;
	USB_CORE_DESCS_T desc;
	ErrorCode_t ret = LPC_OK;
	
	/* enable clocks and pinmux */
	usb_pin_clk_init();

	/* initialize USBD ROM API pointer. */
	g_pUsbApi = (const USBD_API_T *) LPC_ROM_API->usbdApiBase;
	
	/* initialize call back structures */
	memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	usb_param.usb_reg_base = LPC_USB0_BASE;
	/*	WORKAROUND for artf44835 ROM driver BUG:
	    Code clearing STALL bits in endpoint reset routine corrupts memory area
	    next to the endpoint control data. For example When EP0, EP1_IN, EP1_OUT,
	    EP2_IN are used we need to specify 3 here. But as a workaround for this
	    issue specify 4. So that extra EPs control structure acts as padding buffer
	    to avoid data corruption. Corruption of padding memory doesn’t affect the
	    stack/program behaviour.
	 */
	usb_param.max_num_ep = 3 + 1;
	usb_param.mem_base = USB_STACK_MEM_BASE;
	usb_param.mem_size = USB_STACK_MEM_SIZE;

	/* Set the USB descriptors */
	desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
	/* Note, to pass USBCV test full-speed only devices should have both
	   descriptor arrays point to same location and device_qualifier set to 0.
	 */
	desc.high_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
	desc.device_qualifier = 0;

	/* USB Initialization */
	ret = USBD_API->hw->Init(&g_hUsb, &desc, &usb_param);

	if (ret == LPC_OK) {
		/*	WORKAROUND for artf32219 ROM driver BUG:
		    The mem_base parameter part of USB_param structure returned
		    by Init() routine is not accurate causing memory allocation issues for
		    further components.
		 */
		usb_param.mem_base = USB_STACK_MEM_BASE + (USB_STACK_MEM_SIZE - usb_param.mem_size);
		/* Init VCOM interface */
		ret = vcom_init(g_hUsb, &desc, &usb_param);
		if (ret == LPC_OK) 
		{
			/*  enable USB interrupts */
			NVIC_EnableIRQ(USB0_IRQn);
			/* now connect */
			USBD_API->hw->Connect(g_hUsb, 1);
		}
	}
}

namespace std {
    void __throw_bad_function_call() { for(;;) {} }
}

namespace {
	

template<class T> const T& max(const T &a, const T &b) { return (a>b) ? a : b; }
template<class T> const T& min(const T &a, const T &b) { return (a<b) ? a : b; }
template<class T> const T& abs(const T &a) { return (a<0) ? -a : a; }
template<class T> const T& constrain(const T& x, const T &a, const T &b) { return (x>b)?b:((x<a)?a:x); }

static volatile uint32_t system_clock_ms = 0;

#include "duck_font.h"

struct rgba {
	
	rgba() { rgbp = 0; }

	rgba(const rgba &in) { rgbp = in.rgbp; }
	
	rgba(uint32_t _rgbp) { rgbp = _rgbp; }
	
	rgba(uint8_t _r, 
		 uint8_t _g, 
		 uint8_t _b, 
		 uint8_t _a = 0) { 
		 rgbp = (uint32_t(_r)<<16)|
		  	    (uint32_t(_g)<< 8)|
				(uint32_t(_b)<< 0)|
				(uint32_t(_a)<<24); 
	}
	
	operator uint32_t() const { return rgbp; }
	
	rgba operator/(const uint32_t div) const {
		return rgba(ru()/div, gu()/div, bu()/div);
	}

	rgba operator*(const uint32_t mul) const {
		return rgba(ru()*mul, gu()*mul, bu()*mul);
	}

	uint8_t r() const { return ((rgbp>>16)&0xFF); };
	uint8_t g() const { return ((rgbp>> 8)&0xFF); };
	uint8_t b() const { return ((rgbp>> 0)&0xFF); };
	
	uint32_t ru() const { return uint32_t((rgbp>>16)&0xFF); };
	uint32_t gu() const { return uint32_t((rgbp>> 8)&0xFF); };
	uint32_t bu() const { return uint32_t((rgbp>> 0)&0xFF); };

	static rgba hsvToRgb(uint16_t h, uint8_t s, uint8_t v) {
		uint8_t f = (h % 60) * 255 / 60;
		uint8_t p = (255 - s) * (uint16_t)v / 255;
		uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
		uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
		uint8_t r = 0, g = 0, b = 0;
		switch ((h / 60) % 6) {
			case 0: r = v; g = t; b = p; break;
			case 1: r = q; g = v; b = p; break;
			case 2: r = p; g = v; b = t; break;
			case 3: r = p; g = q; b = v; break;
			case 4: r = t; g = p; b = v; break;
			case 5: r = v; g = p; b = q; break;
		}
		return rgba(r, g, b);
	}

private:

	uint32_t rgbp;
};

static const uint8_t rev_bits[] = 
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

static const uint8_t sine_wave[256] = {
	0x80, 0x83, 0x86, 0x89, 0x8C, 0x90, 0x93, 0x96,
	0x99, 0x9C, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAE,
	0xB1, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF, 0xC1, 0xC4,
	0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8,
	0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8,
	0xEA, 0xEB, 0xED, 0xEF, 0xF0, 0xF1, 0xF3, 0xF4,
	0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB, 0xFC,
	0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFD,
	0xFD, 0xFC, 0xFB, 0xFA, 0xFA, 0xF9, 0xF8, 0xF6,
	0xF5, 0xF4, 0xF3, 0xF1, 0xF0, 0xEF, 0xED, 0xEB,
	0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC, 
	0xDA, 0xD8, 0xD5, 0xD3, 0xD1, 0xCE, 0xCC, 0xC9,
	0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9, 0xB6, 0xB3,
	0xB1, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C,
	0x99, 0x96, 0x93, 0x90, 0x8C, 0x89, 0x86, 0x83,
	0x80, 0x7D, 0x7A, 0x77, 0x74, 0x70, 0x6D, 0x6A,
	0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52,
	0x4F, 0x4D, 0x4A, 0x47, 0x44, 0x41, 0x3F, 0x3C,
	0x39, 0x37, 0x34, 0x32, 0x2F, 0x2D, 0x2B, 0x28,
	0x26, 0x24, 0x22, 0x20, 0x1E, 0x1C, 0x1A, 0x18,
	0x16, 0x15, 0x13, 0x11, 0x10, 0x0F, 0x0D, 0x0C,
	0x0B, 0x0A, 0x08, 0x07, 0x06, 0x06, 0x05, 0x04,
	0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03,
	0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A,
	0x0B, 0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x15,
	0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x24,
	0x26, 0x28, 0x2B, 0x2D, 0x2F, 0x32, 0x34, 0x37,
	0x39, 0x3C, 0x3F, 0x41, 0x44, 0x47, 0x4A, 0x4D,
	0x4F, 0x52, 0x55, 0x58, 0x5B, 0x5E, 0x61, 0x64,
	0x67, 0x6A, 0x6D, 0x70, 0x74, 0x77, 0x7A, 0x7D
};

static const uint32_t bird_colors[] = {
	0x404000UL,
	0x104020UL,
	0x005010UL,
	0x004040UL,
	0x083040UL,
	0x001050UL,
	0x300050UL,
	0x401040UL,
	0x501000UL,
	0x401818UL,
	0x400020UL,
	0x453000UL,
	0x452000UL,
	0x204000UL,
	0x201024UL,
	0x102014UL,
	0x201810UL,
	0x101820UL,
	0x303030UL,
	0x202020UL
};

static const uint32_t ring_colors[] = {
	0x404000UL,
	0x104020UL,
	0x005010UL,
	0x004040UL,
	0x083040UL,
	0x001050UL,
	0x300050UL,
	0x401040UL,
	0x501000UL,
	0x401818UL,
	0x400020UL,
	0x453000UL,
	0x402000UL,
	0x204000UL,
	0x201024UL,
	0x102014UL,
	0x201810UL,
	0x101820UL,
	0x303030UL,
	0x202020UL
};

static void delay(uint32_t ms) {
    for (volatile uint32_t j = 0; j < ms; j++) {
        Chip_WWDT_Feed(LPC_WWDT);
        for (volatile uint32_t i = 0; i < 3000; i++) {
        }
    }
}

class Random {
public:
	Random(uint32_t seed) {
		uint32_t i;
		a = 0xf1ea5eed, b = c = d = seed;
		for (i=0; i<20; ++i) {
		    (void)get();
		}
	}

	#define rot(x,k) (((x)<<(k))|((x)>>(32-(k))))
	uint32_t get() {
		uint32_t e = a - rot(b, 27);
		a = b ^ rot(c, 17);
		b = c + d;
		c = d + e;
		d = e + a;
		return d;
	}

	uint32_t get(uint32_t lower, uint32_t upper) {
		return (get() % (upper-lower)) + lower;
	}

private:
	uint32_t a; 
	uint32_t b; 
	uint32_t c; 
	uint32_t d; 

};

#ifndef NO_FLASH
class Flash {

public:
	const uint32_t FLASH_MOSI0_PIN = 0x0006; // 0_6
	const uint32_t FLASH_MISO0_PIN = 0x0009; // 0_9
	const uint32_t FLASH_SCK0_PIN = 0x000A; // 0_10
	const uint32_t FLASH_CSEL_PIN = 0x000A; // 0_10

	const uint32_t ALT_SCK0_PIN = 0x0009; // 0_9

	void Flash() {
		// CSEL
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), IOCON_FUNC2);
		// MISO0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MISO0_PIN>>8), (FLASH_MISO0_PIN&0xFF), IOCON_FUNC2);
		// SCK0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_SCK0_PIN>>8), (FLASH_SCK0_PIN&0xFF), IOCON_FUNC0);
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC1);
		// Set to GPIO, shared with leds
		Chip_IOCON_PinMuxSet(LPC_IOCON, (ALT_SCK0_PIN>>8), (ALT_SCK0_PIN&0xFF), IOCON_FUNC0);
	}
	
	void read_data(uint32_t address, uint8_t *ptr, uint32_t size) {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC1);
		// Set to GPIO, shared with leds
		Chip_IOCON_PinMuxSet(LPC_IOCON, (ALT_SCK0_PIN>>8), (ALT_SCK0_PIN&0xFF), IOCON_FUNC0);

		// CSEL to low
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), false);

		push_byte(0x03);
		push_byte((address>>16)&0xFF);
		push_byte((address>> 8)&0xFF);
		push_byte((address>> 0)&0xFF);
		
		for (uint32_t c=0; c<size; c++) {
			*ptr++ = read_byte();
		}

		// CSEL to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), true);
	}

	void write_data(uint32_t address, uint8_t *ptr, uint32_t size) {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC1);
		// Set to GPIO, shared with leds
		Chip_IOCON_PinMuxSet(LPC_IOCON, (ALT_SCK0_PIN>>8), (ALT_SCK0_PIN&0xFF), IOCON_FUNC0);

		// CSEL to low
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), false);

		push_byte(0x02);
		push_byte((address>>16)&0xFF);
		push_byte((address>> 8)&0xFF);
		push_byte((address>> 0)&0xFF);
		
		for (uint32_t c=0; c<size; c++) {
			push_byte(*ptr++);
		}

		// CSEL to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), true);
	}

private:

	void push_byte(uint8_t byte) {
		while(!Chip_SSP_GetStatus(LPC_SSP0, SSP_STAT_TNF));
		Chip_SSP_SendFrame(LPC_SSP0, byte);
	}

	uint8_t read_byte() {
		while(!Chip_SSP_GetStatus(LPC_SSP0, SSP_STAT_RNE));
		return Chip_SSP_ReceiveFrame(LPC_SSP0);
	}
};
#endif  // #ifndef NO_FLASH

class EEPROM {

public:
	static bool loaded;

	EEPROM() {
		program_count = 27;
		program_curr = 2;
		program_change_count = 0;
		brightness = 1;
		bird_color_index = 0;
		bird_color = rgba(bird_colors[bird_color_index]);
		ring_color_index = 5;
		ring_color = rgba(ring_colors[ring_color_index]);
	}

	void Load() {
		unsigned int param[5] = { 0 };
		param[0] = 62; // Read EEPROM
		param[1] = 0; // EEPROM address
		param[2] = (uintptr_t)this; // RAM address
		param[3] = sizeof(EEPROM); // Number of bytes
		param[4] = SystemCoreClock / 1000; // CCLK
		unsigned int result[4] = { 0 };
		iap_entry(param, result);

		if (program_count != 27 ||
			bird_color == 0UL ||
			bird_color_index > 16 ||
			ring_color == 0UL ||
			ring_color_index > 16 ||
			brightness == 0 ||
			brightness >= 8 ) {
				
			bird_color_index = 0;
			bird_color = bird_colors[bird_color_index];
			ring_color_index = 5;
			ring_color = ring_colors[ring_color_index];
			program_count = 27;
			program_curr = 0;
			program_change_count = 0;
			brightness = 1;
			Save();
		}
		
		loaded = true;
	}

	void Save() {
		if (loaded) {
			unsigned int param[5] = { 0 };
			param[0] = 61; // Write EEPROM
			param[1] = 0; // EEPROM address
			param[2] = (uintptr_t)this; // RAM address
			param[3] = sizeof(EEPROM); // Number of bytes
			param[4] = SystemCoreClock / 1000; // CCLK
			unsigned int result[4] = { 0 };
			iap_entry(param, result);
		}
	}

	void NextEffect() {
		if (loaded) {
			program_curr++;
			program_change_count++;
			if (program_curr >= program_count) {
				program_curr = 0;
			}
			Save();
		}
	}
	
	void NextBrightness() {
		if (loaded) {
			brightness ++;
			brightness &= 0x7;
			Save();
		}
	}
	
	uint32_t program_count;
	uint32_t program_curr;
	uint32_t program_change_count;
	rgba bird_color;
	uint32_t bird_color_index;
	rgba ring_color;
	uint32_t ring_color_index;
	uint32_t brightness;
};

bool EEPROM::loaded = 0;
 
class SPI;

class LEDs {

#define HALF_LEDS 			12

	const uint8_t frnt_ring_indecies[8] = { 0x04*3, 0x05*3, 0x06*3, 0x07*3, 0x08*3, 0x09*3, 0x0A*3, 0x0B*3 };
	const uint8_t back_ring_indecies[8] = { 0x04*3, 0x05*3, 0x06*3, 0x07*3, 0x08*3, 0x09*3, 0x0A*3, 0x0B*3 };
	const uint8_t frnt_bird_indecies[4] = { 0x00*3, 0x01*3, 0x02*3, 0x03*3 };
	const uint8_t back_bird_indecies[4] = { 0x00*3, 0x01*3, 0x02*3, 0x03*3 };

public:

	LEDs() {
		memset(led_data, 0, sizeof(led_data));
	}

	void set_ring(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = r;
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = g;
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = b;
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+1] = r;
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+0] = g;
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+2] = b;
	}

	void set_ring(uint32_t index, const rgba &color) {
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = color.b();
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+2] = color.b();
	}

	void set_ring_synced(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = r;
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = g;
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = b;
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+1] = r;
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+0] = g;
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+2] = b;
	}

	void set_ring_synced(uint32_t index, const rgba &color) {
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = color.b();
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+1] = color.r();
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+0] = color.g();
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+2] = color.b();
	}

	void set_ring_all(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		if(index < 8) { 
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = r;
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = g;
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = b;
		} else if (index < 16) {
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+1] = r;
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+0] = g;
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+2] = b;
		}
	}

	void set_ring_all(uint32_t index, const rgba &color) {
		if(index < 8) { 
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = color.r();
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = color.g();
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = color.b();
		} else if (index < 16) {
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+1] = color.r();
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+0] = color.g();
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+2] = color.b();
		}
	}

	void set_bird(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+1] = r;
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+0] = g;
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+2] = b;
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+1] = r;
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+0] = g;
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+2] = b;
	}

	void set_bird(uint32_t index, const rgba &color) {
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+2] = color.b();
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+2] = color.b();
	}
	
private:
	friend class SPI;

	uint8_t led_data[2*HALF_LEDS*3];

};

class SPI {
public:
	const uint32_t BOTTOM_LED_MOSI0_PIN = 0x0009; // 0_9
	const uint32_t BOTTOM_LED_SCK0_PIN = 0x011D; // 1_29

	const uint32_t ALT_SCK0_PIN = 0x0006; // 0_6

	const uint32_t TOP_LED_MOSI1_PIN = 0x0116; // 1_22
	const uint32_t TOP_LED_SCK1_PIN = 0x010F; // 1_15

	SPI() {
		// Set to GPIO, shared with flash chip
		Chip_IOCON_PinMuxSet(LPC_IOCON, (ALT_SCK0_PIN>>8), (ALT_SCK0_PIN&0xFF), IOCON_FUNC0);

		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_MOSI0_PIN>>8), (BOTTOM_LED_MOSI0_PIN&0xFF), IOCON_FUNC1);
		// SCK0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_SCK0_PIN>>8), (BOTTOM_LED_SCK0_PIN&0xFF), IOCON_FUNC1);
		
		// MOSI1
		Chip_IOCON_PinMuxSet(LPC_IOCON, (TOP_LED_MOSI1_PIN>>8), (TOP_LED_MOSI1_PIN&0xFF), IOCON_FUNC2);
		// SCK1
		Chip_IOCON_PinMuxSet(LPC_IOCON, (TOP_LED_SCK1_PIN>>8), (TOP_LED_SCK1_PIN&0xFF), IOCON_FUNC3);

		Chip_SSP_Init(LPC_SSP0);
	    Chip_SSP_SetMaster(LPC_SSP0, 1);
		Chip_SSP_SetClockRate(LPC_SSP0, 0, 2);
		Chip_SSP_SetFormat(LPC_SSP0, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
		Chip_SSP_Enable(LPC_SSP0);

	    Chip_SSP_SetMaster(LPC_SSP1, 1);
		Chip_SSP_Init(LPC_SSP1);
		Chip_SSP_SetClockRate(LPC_SSP1, 0, 2);
		Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
		Chip_SSP_Enable(LPC_SSP1);
	}

	void push_frame(LEDs &leds, uint32_t brightness = 0x01)  {
	
		// Set to GPIO, shared with flash chip
		Chip_IOCON_PinMuxSet(LPC_IOCON, (ALT_SCK0_PIN>>8), (ALT_SCK0_PIN&0xFF), IOCON_FUNC0);
		// SCK0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_SCK0_PIN>>8), (BOTTOM_LED_SCK0_PIN&0xFF), IOCON_FUNC1);

		// Set format again
		Chip_SSP_SetClockRate(LPC_SSP0, 0, 2);
		Chip_SSP_SetFormat(LPC_SSP0, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);

		// Start frame
		push_byte_top(0);
		push_byte_btm(0);
		push_byte_top(0);
		push_byte_btm(0);
		push_byte_top(0);
		push_byte_btm(0);
		push_byte_top(0);
		push_byte_btm(0);

		// Frame data
		for (int32_t c=0; c<HALF_LEDS; c++) {
			push_byte_top(0xE0 | brightness);
			push_byte_btm(0xE0 | brightness);
			push_byte_top(leds.led_data[HALF_LEDS*0*3 + c*3+2]);
			push_byte_btm(leds.led_data[HALF_LEDS*1*3 + c*3+2]);
			push_byte_top(leds.led_data[HALF_LEDS*0*3 + c*3+0]);
			push_byte_btm(leds.led_data[HALF_LEDS*1*3 + c*3+0]);
			push_byte_top(leds.led_data[HALF_LEDS*0*3 + c*3+1]);
			push_byte_btm(leds.led_data[HALF_LEDS*1*3 + c*3+1]);
		}

		// End frame
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
	}

private:
	
	void push_byte_top(uint8_t byte) {
		while(!Chip_SSP_GetStatus(LPC_SSP1, SSP_STAT_TNF));
		Chip_SSP_SendFrame(LPC_SSP1, byte);
	}
	
	void push_byte_btm(uint8_t byte) {
		while(!Chip_SSP_GetStatus(LPC_SSP0, SSP_STAT_TNF));
		Chip_SSP_SendFrame(LPC_SSP0, byte);
	}
};

class BQ24295 {
	public:
			static const uint32_t i2caddr = 0x6B;
	
			BQ24295() {
				devicePresent = false;
			}
			
			void SetBoostVoltage (uint32_t voltageMV) {
				uint8_t reg = getRegister(0x06);
				if ((voltageMV >= 4550) && (voltageMV <= 5510)) {
					uint32_t codedValue = voltageMV;
					codedValue = (codedValue - 4550) / 64;
					if ((voltageMV - 4550) % 64 != 0) {
						codedValue++;
					}
					reg &= ~(0x0f << 4);
					reg |= (uint8_t) ((codedValue & 0x0f) << 4);
					setRegister (0x06, reg);
				}
			}
			
			uint32_t GetBoostVoltage () {
				uint8_t reg = getRegister(0x06);
				reg = (reg >> 4) & 0x0f;
				return 4550 + ((uint32_t) reg) * 64;
			}

			void SetBoostUpperTemperatureLimit (uint32_t temperatureC) {
				uint8_t reg = getRegister(0x06);
				uint8_t codedValue = 0;
				if (temperatureC < 60) {
					codedValue = 0;
				} else if (temperatureC < 65) {
					codedValue = 1;
				} else {
					codedValue = 2;
				}
				reg &= ~(0x03 << 2);
				reg |= (uint8_t) (codedValue << 2);
				setRegister (0x06, reg);
			}

			uint32_t GetBoostUpperTemperatureLimit () {
				uint8_t reg = getRegister(0x06);
				if (((reg >> 2) & 0x03) != 0x03) {
					switch ((reg >> 2) & 0x03) {
						case 0:
							return 55;
						break;
						case 1:
							return 60;
						break;
						case 2:
							return 65;
						break;
					}
				}
				return 0;
			}

			void SetInputCurrentLimit(uint32_t currentMA) {
				uint8_t reg = 0;
				if ((reg = getRegister(0x00)) != 0) {
					// Input current limit is in bits 0 to 2, coded
					// such that the smallest limit is applied for
					// a range (e.g. 120 mA ends up as 100 mA rather
					// than 150 mA)
					if ((currentMA >= 100) && (currentMA <= 3000)) {
						uint8_t codedValue = 0;
						if (currentMA < 150) {
							codedValue = 0;
						} else if (currentMA < 500) {
							codedValue = 1;
						} else if (currentMA < 900) {
							codedValue = 2;
						} else if (currentMA < 1000) {
							codedValue = 3;
						} else if (currentMA < 1500) {
							codedValue = 4;
						} else if (currentMA < 2000) {
							codedValue = 5;
						} else if (currentMA < 3000) {
							codedValue = 6;
						} else {
							codedValue = 7;
						}                
						reg &= ~(0x07 << 0);
						reg |= codedValue;
						setRegister (0x00, reg);
					}
				}
			}
			
			uint32_t GetInputCurrentLimit() {
				uint8_t reg = getRegister(0x00);
				switch (reg & 0x07) {
					case 0:
						return 100;
					break;
					case 1:
						return 150;
					break;
					case 2:
						return 500;
					break;
					case 3:
						return 900;
					break;
					case 4:
						return 1000;
					break;
					case 5:
						return 1500;
					break;
					case 6:
						return 2000;
					break;
					case 7:
						return 3000;
					break;
				}
				return 0;
			}

			void EnableInputLimits() {
				setRegisterBits(0x00, (1 << 7));
			}
			
			void DisableInputLimits() {
				clearRegisterBits(0x00, (1 << 7));
			}

			void SetChipThermalRegulationThreshold(uint32_t temperatureC) {
				uint8_t reg = getRegister(0x06);
				uint8_t codedValue = 0;
				if (temperatureC < 80) {
					codedValue = 0;
				} else if (temperatureC < 100) {
					codedValue = 1;
				} else if (temperatureC < 120) {
					codedValue = 2;
				} else {
					codedValue = 3;
				}
				reg &= ~0x03;
				reg |= (uint8_t) codedValue;
				setRegister (0x06, reg);
			}
			
			uint32_t GetChipThermalRegulationThreshold() {
				uint8_t reg = getRegister(0x06);
				switch (reg & 0x03) {
					case 0:
						return 60;
					break;
					case 1:
						return 80;
					break;
					case 2:
						return 100;
					break;
					case 3:
						return 120;
					break;
				}
				return 0;
			}
			
			bool DevicePresent() const { return devicePresent; }

	private:

			uint8_t getRegister(uint8_t address) {
				uint8_t value = 0;
				Chip_I2C_MasterSend(I2C0, i2caddr, &address, 1);
				Chip_I2C_MasterRead(I2C0, i2caddr, &value, 1);
				return value;
			}

			void setRegister(uint8_t address, uint8_t value) {
				uint8_t set[2];
				set[0] = address;
				set[1] = value;
				Chip_I2C_MasterSend(I2C0, i2caddr, &set[0], 2);
			}

			void setRegisterBits(uint8_t address, uint8_t mask) {
				uint8_t value = getRegister(address);
				value |= mask;
				setRegister(address, value);
			}

			void clearRegisterBits(uint8_t address, uint8_t mask) {
				uint8_t value = getRegister(address);
				value &= ~mask;
				setRegister(address, value);
			}
			
			friend class Setup;

			bool devicePresent;
};


class SDD1306 {

	public:
 			static const uint32_t i2caddr = 0x3C;

			SDD1306() {
				devicePresent = false;
			}
			
			void Clear() {
				memset(text_buffer_cache, 0, sizeof(text_buffer_cache));
				memset(text_buffer_screen, 0, sizeof(text_buffer_screen));
				memset(text_attr_cache, 0, sizeof(text_attr_cache));
				memset(text_attr_screen, 0, sizeof(text_attr_screen));
				center_flip_screen = 0;
				center_flip_cache = 0;
			}
			
			void DisplayBootScreen() {
				for (uint32_t c=0; c<8*4; c++) {
					text_buffer_cache[c] = 0x80 + c;
				}
			}
			
			void SetCenterFlip(int8_t progression) {
				center_flip_cache = progression;
			}

			void PlaceAsciiStr(uint32_t x, uint32_t y, const char *str) {
				if (y>3 || x>7) return;
				size_t len = strlen(str);
				if (x+len > 8) len = 7-x;
				for (size_t c=0; c<len; c++) {
					text_buffer_cache[y*8+x+c] = uint8_t(str[c]) - 0x20;
				}
			}

			void PlaceCustomChar(uint32_t x, uint32_t y, uint8_t code) {
				if (y>3 || x>7) return;
				text_buffer_cache[y*8+x] = code;
			}
			
			void Invert() {
				for (uint32_t c=0; c<8*4; c++) {
					text_attr_cache[c] ^= 1;
				}
			}
			
			void SetAttr(uint32_t x, uint32_t y, uint8_t attr) {
				if (y>3 || x>7) return;
				text_attr_cache[y*8+x] = attr;
			}

			void Display() {
				bool display_center_flip = false;
				if (center_flip_cache || center_flip_screen) {
					center_flip_screen = center_flip_cache;
					display_center_flip = true;
				}
				for (uint32_t y=0; y<4; y++) {
					for (uint32_t x=0; x<8; x++) {
						if (text_buffer_cache[y*8+x] != text_buffer_screen[y*8+x] ||
						    text_attr_cache[y*8+x] != text_attr_screen[y*8+x]) {
							text_buffer_screen[y*8+x] = text_buffer_cache[y*8+x];
							text_attr_screen[y*8+x] = text_attr_cache[y*8+x];
							if (!display_center_flip) {
								DisplayChar(x,y,text_buffer_screen[y*8+x],text_attr_screen[y*8+x]);
							}
						}
					}
				}
				if (display_center_flip) {
					DisplayCenterFlip();
				}
			}
			
			void SetVerticalShift(int8_t val) {
				WriteCommand(0xD3);
				if (val < 0) {
					val = 64+val;
					WriteCommand(val&0x3F);
				} else {
					WriteCommand(val&0x3F);
				}
			}

			void DisplayOn() {
				WriteCommand(0xAF);
			}

			void DisplayOff() {
				WriteCommand(0xAE);
			}

			void DisplayUID() {
				unsigned int param[1] = { 0 };
				param[0] = 58; // Read UID
				unsigned int result[4] = { 0 };
				iap_entry(param, result);

				char str[32];
				sprintf(str,"%08x",result[0]);
				PlaceAsciiStr(0,0,str);
				
				sprintf(str,"%08x",result[1]);
				PlaceAsciiStr(0,1,str);
				
				sprintf(str,"%08x",result[2]);
				PlaceAsciiStr(0,2,str);
				
				sprintf(str,"%08x",result[3]);
				PlaceAsciiStr(0,3,str);
				
				Display();
			}
			
			void Init() const {

				// Toggle RESET line
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 8);
				Chip_GPIO_SetPinState(LPC_GPIO, 0, 8, true);
				delay(1);
				Chip_GPIO_SetPinState(LPC_GPIO, 0, 8, false);
				delay(10);
				Chip_GPIO_SetPinState(LPC_GPIO, 0, 8, true);


				static uint8_t startup_sequence[] = {
					0xAE,			// Display off

					0xD5, 0x80,		// Set Display Clock Divide Ratio
					
					0xA8, 0x1F,		// Set Multiplex Ratio
					
					0xD3, 0x00,		// Set Display Offset

					0x8D, 0x14,		// Enable Charge Pump

					0x40,			// Set Display RAM start

					0xA6,			// Set to normal display (0xA7 == inverse)
					
					0xA4,			// Force Display From RAM On

					0xA1,			// Set Segment Re-map

					0xC8,			// Set COM Output Scan Direction (flipped)
					
					0xDA, 0x12, 	// Set Pins configuration
				
					0x81, 0x00,		// Set Contrast (0x00-0xFF)
					
					0xD9, 0xF1,		// Set Pre-Charge period

					0xDB, 0x40,		// Adjust Vcomm regulator output

					0xAF			// Display on
				};

				for (size_t c = 0; c < sizeof(startup_sequence); c++) {
					WriteCommand(startup_sequence[c]);
				}

			}

			bool DevicePresent() const { return devicePresent; }

	private:

			void DisplayCenterFlip() {
				uint8_t buf[65];
				buf[0] = 0x40;
				for (uint32_t y=0; y<4; y++) {
					WriteCommand(0xB0+y);
					uint32_t sx = 32;
					WriteCommand(0x0f&(sx   )); // 0x20 offset
					WriteCommand(0x10|(sx>>4)); // 0x20 offset
					for (uint32_t x = 0; x < 64; x++) {
						if (center_flip_screen == 32) {
							buf[x+1] = 0x00;
						} else {
							int32_t rx = ( ( ( int32_t(x) - 32 ) * 32 ) / int32_t(32 - center_flip_screen) ) + 32;
							if (rx < 0 || rx > 63) { 
								buf[x+1] = 0x00;
							} else {
                                uint8_t a = text_attr_screen[y*8+rx/8];
                                uint8_t r = (a & 4) ? (7-(rx&7)) : (rx&7);
                                uint8_t v = duck_font_raw[text_buffer_screen[y*8+rx/8]*8+r];
								if (a & 1) {
									v = ~v;
								}
                                if (a & 2) {
									v = rev_bits[v];
								}
                                buf[x+1] = v;
							}
						}
					}
					Chip_I2C_MasterSend(I2C0, i2caddr, buf, 0x41);
				} 
			}

			void DisplayChar(uint32_t x, uint32_t y, char ch, uint8_t attr) {
				WriteCommand(0xB0+y);
				x=x*8+32;
				WriteCommand(0x0f&(x   )); // 0x20 offset
				WriteCommand(0x10|(x>>4)); // 0x20 offset
				
				uint8_t buf[9];
				buf[0] = 0x40;
                if ((attr & 4)) {
                    if ((attr & 1)) {
                        if ((attr & 2)) {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] = ~rev_bits[duck_font_raw[ch*8+7-c]];
                            }
                        } else {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] = ~duck_font_raw[ch*8+7-c];
                            }
                        }
                    } else {
                        if ((attr & 2)) {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] =  rev_bits[duck_font_raw[ch*8+7-c]];
                            }
                        } else {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] =  duck_font_raw[ch*8+7-c];
                            }
                        }
                    }
                } else {
                    if ((attr & 1)) {
                        if ((attr & 2)) {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] = ~rev_bits[duck_font_raw[ch*8+c]];
                            }
                        } else {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] = ~duck_font_raw[ch*8+c];
                            }
                        }
                    } else {
                        if ((attr & 2)) {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] =  rev_bits[duck_font_raw[ch*8+c]];
                            }
                        } else {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] =  duck_font_raw[ch*8+c];
                            }
                        }
                    }
                }
				Chip_I2C_MasterSend(I2C0, i2caddr, buf, 0x9);
			}

			void WriteCommand(uint8_t v) const {
				uint8_t control[2];
				control[0] = 0;
				control[1] = v;
				Chip_I2C_MasterSend(I2C0, i2caddr, control, 2);
			}


			friend class Setup;

			bool devicePresent;

			int8_t center_flip_screen;
			int8_t center_flip_cache;
			uint8_t text_buffer_cache[8*4];
			uint8_t text_buffer_screen[8*4];
			uint8_t text_attr_cache[8*4];
			uint8_t text_attr_screen[8*4];
};  // class SDD1306

class Setup {

	public:
            Setup(SDD1306 &sdd1306, BQ24295 &bq24295) {
                InitWWDT();
				InitGPIO();
				InitPININT();
				InitADC();
				InitI2C(sdd1306, bq24295);
			}

	private:
            void InitWWDT() {
                Chip_WWDT_Init(LPC_WWDT);

                Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_WDTOSC_PD);
                Chip_Clock_SetWDTOSC(WDTLFO_OSC_1_05, 20);
                
                // 1s watchdog timer
                Chip_WWDT_SelClockSource(LPC_WWDT, WWDT_CLKSRC_WATCHDOG_WDOSC);
                Chip_WWDT_SetTimeOut(LPC_WWDT, 4 * Chip_Clock_GetWDTOSCRate() / 4);
                Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);
                Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);

                NVIC_ClearPendingIRQ(WDT_IRQn);
                NVIC_EnableIRQ(WDT_IRQn);
                
                Chip_WWDT_Start(LPC_WWDT);
            }
    
			void InitGPIO() {
				Chip_GPIO_Init(LPC_GPIO);
			}
			
			void InitADC() {
				ADC_CLOCK_SETUP_T setup;
				Chip_ADC_Init(LPC_ADC, &setup);
				Chip_ADC_EnableChannel(LPC_ADC, ADC_CH5, ENABLE);
				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, IOCON_FUNC1 | IOCON_ADMODE_EN);
			}
			
			void InitPININT() {
				Chip_PININT_Init(LPC_PININT);
				Chip_PININT_EnableIntHigh(LPC_PININT, 0);
				Chip_PININT_EnableIntLow(LPC_PININT, 0);
			}
			
			void InitI2C(SDD1306 &sdd1306, BQ24295 &bq24295) {
				Chip_SYSCTL_PeriphReset(RESET_I2C0);

				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 4, IOCON_FUNC1 | I2C_FASTPLUS_BIT);
				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5, IOCON_FUNC1 | I2C_FASTPLUS_BIT);

				Chip_I2C_Init(I2C0);
				Chip_I2C_SetClockRate(I2C0, I2C_DEFAULT_SPEED);

				NVIC_DisableIRQ(I2C0_IRQn);
				Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling);

				ProbeI2CSlaves(sdd1306, bq24295);
			}

			void ProbeI2CSlaves(SDD1306 &sdd1306, BQ24295 &bq24295) {
				int i;
				uint8_t ch[2];

				for (i = 0; i <= 0x7F; i++) {
					if (!(i & 0x0F)) {
					}
					if ((i <= 7) || (i > 0x78)) {
						continue;
					}
					if (Chip_I2C_MasterRead(I2C0, i, ch, 1 + (i == 0x48)) > 0) {
						switch(i) {
							case	sdd1306.i2caddr:
									sdd1306.devicePresent = true;
									break;
							case	bq24295.i2caddr:
									bq24295.devicePresent = true;
									break;
						}
					}
				}
			}
};  // class Setup {


class SX1280 {
	public:
			enum {
				REG_LR_FIRMWARE_VERSION_MSB             = 0x0153,
				REG_LR_PACKETPARAMS                     = 0x0903,
				REG_LR_PAYLOADLENGTH                    = 0x0901,
				REG_LR_CRCSEEDBASEADDR                  = 0x09C8,
				REG_LR_CRCPOLYBASEADDR                  = 0x09C6,
				REG_LR_WHITSEEDBASEADDR                 = 0x09C5,
				REG_LR_RANGINGIDCHECKLENGTH             = 0x0931,
				REG_LR_DEVICERANGINGADDR                = 0x0916,
				REG_LR_REQUESTRANGINGADDR               = 0x0912,
				REG_LR_RANGINGRESULTCONFIG              = 0x0924,
				REG_LR_RANGINGRESULTBASEADDR            = 0x0961,
				REG_LR_RANGINGRESULTSFREEZE             = 0x097F,
				REG_LR_RANGINGRERXTXDELAYCAL            = 0x092C,
				REG_LR_RANGINGFILTERWINDOWSIZE          = 0x091E,
				REG_LR_RANGINGRESULTCLEARREG            = 0x0923,
				REG_LR_SYNCWORDBASEADDRESS1             = 0x09CE,
				REG_LR_SYNCWORDBASEADDRESS2             = 0x09D3,
				REG_LR_SYNCWORDBASEADDRESS3             = 0x09D8,
				REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB    = 0x0954,
				REG_LR_SYNCWORDTOLERANCE                = 0x09CD,
				REG_LR_PREAMBLELENGTH                   = 0x09C1,
				REG_LR_BLE_ACCESS_ADDRESS               = 0x09CF,
			};
			
			enum {
				REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK   = 0x0FFFFF,
			};
	
			enum RadioCommand {
				RADIO_GET_STATUS                        = 0xC0,
				RADIO_WRITE_REGISTER                    = 0x18,
				RADIO_READ_REGISTER                     = 0x19,
				RADIO_WRITE_BUFFER                      = 0x1A,
				RADIO_READ_BUFFER                       = 0x1B,
				RADIO_SET_SLEEP                         = 0x84,
				RADIO_SET_STANDBY                       = 0x80,
				RADIO_SET_FS                            = 0xC1,
				RADIO_SET_TX                            = 0x83,
				RADIO_SET_RX                            = 0x82,
				RADIO_SET_RXDUTYCYCLE                   = 0x94,
				RADIO_SET_CAD                           = 0xC5,
				RADIO_SET_TXCONTINUOUSWAVE              = 0xD1,
				RADIO_SET_TXCONTINUOUSPREAMBLE          = 0xD2,
				RADIO_SET_PACKETTYPE                    = 0x8A,
				RADIO_GET_PACKETTYPE                    = 0x03,
				RADIO_SET_RFFREQUENCY                   = 0x86,
				RADIO_SET_TXPARAMS                      = 0x8E,
				RADIO_SET_CADPARAMS                     = 0x88,
				RADIO_SET_BUFFERBASEADDRESS             = 0x8F,
				RADIO_SET_MODULATIONPARAMS              = 0x8B,
				RADIO_SET_PACKETPARAMS                  = 0x8C,
				RADIO_GET_RXBUFFERSTATUS                = 0x17,
				RADIO_GET_PACKETSTATUS                  = 0x1D,
				RADIO_GET_RSSIINST                      = 0x1F,
				RADIO_SET_DIOIRQPARAMS                  = 0x8D,
				RADIO_GET_IRQSTATUS                     = 0x15,
				RADIO_CLR_IRQSTATUS                     = 0x97,
				RADIO_CALIBRATE                         = 0x89,
				RADIO_SET_REGULATORMODE                 = 0x96,
				RADIO_SET_SAVECONTEXT                   = 0xD5,
				RADIO_SET_AUTOTX                        = 0x98,
				RADIO_SET_AUTOFS                        = 0x9E,
				RADIO_SET_LONGPREAMBLE                  = 0x9B,
				RADIO_SET_UARTSPEED                     = 0x9D,
				RADIO_SET_RANGING_ROLE                  = 0xA3,
			};

			enum IrqRangingCode
			{
				IRQ_RANGING_SLAVE_ERROR_CODE            = 0x00,
				IRQ_RANGING_SLAVE_VALID_CODE,
				IRQ_RANGING_MASTER_ERROR_CODE,
				IRQ_RANGING_MASTER_VALID_CODE,
			};

			enum IrqErrorCode
			{
				IRQ_HEADER_ERROR_CODE                   = 0x00,
				IRQ_SYNCWORD_ERROR_CODE,
				IRQ_CRC_ERROR_CODE,
				IRQ_RANGING_ON_LORA_ERROR_CODE,
			};

			enum IrqValidCode
			{
				IRQ_HEADER_VALID_CODE                   = 0x00,
				IRQ_SYNCWORD_VALID_CODE,
			};

			enum RadioStates
			{
				RF_IDLE                                 = 0x00,         //!< The radio is idle
				RF_RX_RUNNING,                                          //!< The radio is in reception state
				RF_TX_RUNNING,                                          //!< The radio is in transmission state
				RF_CAD,                                                 //!< The radio is doing channel activity detection
			};
			
			enum RadioCommandStatus
			{
				CMD_SUCCESS								= 0x01,
				CMD_DATA_AVAILABLE						= 0x02,
				CMD_TIMEOUT								= 0x03,
				CMD_ERROR								= 0x04,
				CMD_FAIL								= 0x05,
				CMD_TX_DONE								= 0x06,
			};

			enum RadioOperatingModes
			{
				MODE_SLEEP                              = 0x00,         //! The radio is in sleep mode
				MODE_CALIBRATION,                                       //! The radio is in calibration mode
				MODE_STDBY_RC,                                          //! The radio is in standby mode with RC oscillator
				MODE_STDBY_XOSC,                                        //! The radio is in standby mode with XOSC oscillator
				MODE_FS,                                                //! The radio is in frequency synthesis mode
				MODE_RX,                                                //! The radio is in receive mode
				MODE_TX,                                                //! The radio is in transmit mode
				MODE_CAD                                                //! The radio is in channel activity detection mode
			};

			enum RadioStandbyModes
			{
				STDBY_RC                                = 0x00,
				STDBY_XOSC                              = 0x01,
			};

			enum RadioRegulatorModes
			{
				USE_LDO                                 = 0x00,         //! Use LDO (default value)
				USE_DCDC                                = 0x01,         //! Use DCDC
			};

			enum  RadioPacketTypes
			{
				PACKET_TYPE_GFSK                        = 0x00,
				PACKET_TYPE_LORA,
				PACKET_TYPE_RANGING,
				PACKET_TYPE_FLRC,
				PACKET_TYPE_BLE,
				PACKET_TYPE_NONE                        = 0x0F,
			};

			enum  RadioRampTimes
			{
				RADIO_RAMP_02_US                        = 0x00,
				RADIO_RAMP_04_US                        = 0x20,
				RADIO_RAMP_06_US                        = 0x40,
				RADIO_RAMP_08_US                        = 0x60,
				RADIO_RAMP_10_US                        = 0x80,
				RADIO_RAMP_12_US                        = 0xA0,
				RADIO_RAMP_16_US                        = 0xC0,
				RADIO_RAMP_20_US                        = 0xE0,
			};

			enum  RadioLoRaCadSymbols
			{
				LORA_CAD_01_SYMBOL                      = 0x00,
				LORA_CAD_02_SYMBOLS                     = 0x20,
				LORA_CAD_04_SYMBOLS                     = 0x40,
				LORA_CAD_08_SYMBOLS                     = 0x60,
				LORA_CAD_16_SYMBOLS                     = 0x80,
			};

			enum RadioGfskBleBitrates
			{
				GFSK_BLE_BR_2_000_BW_2_4                = 0x04,
				GFSK_BLE_BR_1_600_BW_2_4                = 0x28,
				GFSK_BLE_BR_1_000_BW_2_4                = 0x4C,
				GFSK_BLE_BR_1_000_BW_1_2                = 0x45,
				GFSK_BLE_BR_0_800_BW_2_4                = 0x70,
				GFSK_BLE_BR_0_800_BW_1_2                = 0x69,
				GFSK_BLE_BR_0_500_BW_1_2                = 0x8D,
				GFSK_BLE_BR_0_500_BW_0_6                = 0x86,
				GFSK_BLE_BR_0_400_BW_1_2                = 0xB1,
				GFSK_BLE_BR_0_400_BW_0_6                = 0xAA,
				GFSK_BLE_BR_0_250_BW_0_6                = 0xCE,
				GFSK_BLE_BR_0_250_BW_0_3                = 0xC7,
				GFSK_BLE_BR_0_125_BW_0_3                = 0xEF,
			};

			enum RadioGfskBleModIndexes
			{
				GFSK_BLE_MOD_IND_0_35                   =  0,
				GFSK_BLE_MOD_IND_0_50                   =  1,
				GFSK_BLE_MOD_IND_0_75                   =  2,
				GFSK_BLE_MOD_IND_1_00                   =  3,
				GFSK_BLE_MOD_IND_1_25                   =  4,
				GFSK_BLE_MOD_IND_1_50                   =  5,
				GFSK_BLE_MOD_IND_1_75                   =  6,
				GFSK_BLE_MOD_IND_2_00                   =  7,
				GFSK_BLE_MOD_IND_2_25                   =  8,
				GFSK_BLE_MOD_IND_2_50                   =  9,
				GFSK_BLE_MOD_IND_2_75                   = 10,
				GFSK_BLE_MOD_IND_3_00                   = 11,
				GFSK_BLE_MOD_IND_3_25                   = 12,
				GFSK_BLE_MOD_IND_3_50                   = 13,
				GFSK_BLE_MOD_IND_3_75                   = 14,
				GFSK_BLE_MOD_IND_4_00                   = 15,
			};

			enum RadioFlrcBitrates
			{
				FLRC_BR_1_300_BW_1_2                    = 0x45,
				FLRC_BR_1_040_BW_1_2                    = 0x69,
				FLRC_BR_0_650_BW_0_6                    = 0x86,
				FLRC_BR_0_520_BW_0_6                    = 0xAA,
				FLRC_BR_0_325_BW_0_3                    = 0xC7,
				FLRC_BR_0_260_BW_0_3                    = 0xEB,
			};

			enum RadioFlrcCodingRates
			{
				FLRC_CR_1_2                             = 0x00,
				FLRC_CR_3_4                             = 0x02,
				FLRC_CR_1_0                             = 0x04,
			};

			enum RadioModShapings
			{
				RADIO_MOD_SHAPING_BT_OFF                = 0x00,         //! No filtering
				RADIO_MOD_SHAPING_BT_1_0                = 0x10,
				RADIO_MOD_SHAPING_BT_0_5                = 0x20,
			};

			enum RadioLoRaSpreadingFactors
			{
				LORA_SF5                                = 0x50,
				LORA_SF6                                = 0x60,
				LORA_SF7                                = 0x70,
				LORA_SF8                                = 0x80,
				LORA_SF9                                = 0x90,
				LORA_SF10                               = 0xA0,
				LORA_SF11                               = 0xB0,
				LORA_SF12                               = 0xC0,
			};

			enum RadioLoRaBandwidths
			{
				LORA_BW_0200                            = 0x34,
				LORA_BW_0400                            = 0x26,
				LORA_BW_0800                            = 0x18,
				LORA_BW_1600                            = 0x0A,
			};

			enum RadioLoRaCodingRates
			{
				LORA_CR_4_5                             = 0x01,
				LORA_CR_4_6                             = 0x02,
				LORA_CR_4_7                             = 0x03,
				LORA_CR_4_8                             = 0x04,
				LORA_CR_LI_4_5                          = 0x05,
				LORA_CR_LI_4_6                          = 0x06,
				LORA_CR_LI_4_7                          = 0x07,
			};

			enum RadioPreambleLengths
			{
				PREAMBLE_LENGTH_04_BITS                 = 0x00,         //!< Preamble length: 04 bits
				PREAMBLE_LENGTH_08_BITS                 = 0x10,         //!< Preamble length: 08 bits
				PREAMBLE_LENGTH_12_BITS                 = 0x20,         //!< Preamble length: 12 bits
				PREAMBLE_LENGTH_16_BITS                 = 0x30,         //!< Preamble length: 16 bits
				PREAMBLE_LENGTH_20_BITS                 = 0x40,         //!< Preamble length: 20 bits
				PREAMBLE_LENGTH_24_BITS                 = 0x50,         //!< Preamble length: 24 bits
				PREAMBLE_LENGTH_28_BITS                 = 0x60,         //!< Preamble length: 28 bits
				PREAMBLE_LENGTH_32_BITS                 = 0x70,         //!< Preamble length: 32 bits
			};

			enum RadioFlrcSyncWordLengths
			{
				FLRC_NO_SYNCWORD                        = 0x00,
				FLRC_SYNCWORD_LENGTH_4_BYTE             = 0x04,
			};

			enum RadioSyncWordLengths
			{
				GFSK_SYNCWORD_LENGTH_1_BYTE             = 0x00,         //!< Sync word length: 1 byte
				GFSK_SYNCWORD_LENGTH_2_BYTE             = 0x02,         //!< Sync word length: 2 bytes
				GFSK_SYNCWORD_LENGTH_3_BYTE             = 0x04,         //!< Sync word length: 3 bytes
				GFSK_SYNCWORD_LENGTH_4_BYTE             = 0x06,         //!< Sync word length: 4 bytes
				GFSK_SYNCWORD_LENGTH_5_BYTE             = 0x08,         //!< Sync word length: 5 bytes
			};

			enum RadioSyncWordRxMatchs
			{
				RADIO_RX_MATCH_SYNCWORD_OFF             = 0x00,         //!< No correlator turned on, i.e. do not search for SyncWord
				RADIO_RX_MATCH_SYNCWORD_1               = 0x10,
				RADIO_RX_MATCH_SYNCWORD_2               = 0x20,
				RADIO_RX_MATCH_SYNCWORD_1_2             = 0x30,
				RADIO_RX_MATCH_SYNCWORD_3               = 0x40,
				RADIO_RX_MATCH_SYNCWORD_1_3             = 0x50,
				RADIO_RX_MATCH_SYNCWORD_2_3             = 0x60,
				RADIO_RX_MATCH_SYNCWORD_1_2_3           = 0x70,
			};

			enum RadioPacketLengthModes
			{
				RADIO_PACKET_FIXED_LENGTH               = 0x00,         //!< The packet is known on both sides, no header included in the packet
				RADIO_PACKET_VARIABLE_LENGTH            = 0x20,         //!< The packet is on variable size, header included
			};

			enum RadioCrcTypes
			{
				RADIO_CRC_OFF                           = 0x00,         //!< No CRC in use
				RADIO_CRC_1_BYTES                       = 0x10,
				RADIO_CRC_2_BYTES                       = 0x20,
				RADIO_CRC_3_BYTES                       = 0x30,
			};

			enum RadioWhiteningModes
			{
				RADIO_WHITENING_ON                      = 0x00,
				RADIO_WHITENING_OFF                     = 0x08,
			};

			enum RadioLoRaPacketLengthsModes
			{
				LORA_PACKET_VARIABLE_LENGTH             = 0x00,         //!< The packet is on variable size, header included
				LORA_PACKET_FIXED_LENGTH                = 0x80,         //!< The packet is known on both sides, no header included in the packet
				LORA_PACKET_EXPLICIT                    = LORA_PACKET_VARIABLE_LENGTH,
				LORA_PACKET_IMPLICIT                    = LORA_PACKET_FIXED_LENGTH,
			};

			enum RadioLoRaCrcModes
			{
				LORA_CRC_ON                             = 0x20,         //!< CRC activated
				LORA_CRC_OFF                            = 0x00,         //!< CRC not used
			};

			enum RadioLoRaIQModes
			{
				LORA_IQ_NORMAL                          = 0x40,
				LORA_IQ_INVERTED                        = 0x00,
			};

			enum RadioRangingIdCheckLengths
			{
				RANGING_IDCHECK_LENGTH_08_BITS          = 0x00,
				RANGING_IDCHECK_LENGTH_16_BITS,
				RANGING_IDCHECK_LENGTH_24_BITS,
				RANGING_IDCHECK_LENGTH_32_BITS,
			};

			enum RadioRangingResultTypes
			{
				RANGING_RESULT_RAW                      = 0x00,
				RANGING_RESULT_AVERAGED                 = 0x01,
				RANGING_RESULT_DEBIASED                 = 0x02,
				RANGING_RESULT_FILTERED                 = 0x03,
			};

			enum RadioBleConnectionStates
			{
				BLE_MASTER_SLAVE                        = 0x00,
				BLE_ADVERTISER                          = 0x20,
				BLE_TX_TEST_MODE                        = 0x40,
				BLE_RX_TEST_MODE                        = 0x60,
				BLE_RXTX_TEST_MODE                      = 0x80,
			};

			enum RadioBleCrcTypes
			{
				BLE_CRC_OFF                             = 0x00,
				BLE_CRC_3B                              = 0x10,
			};

			enum RadioBleTestPayloads
			{
				BLE_PRBS_9                              = 0x00,         //!< Pseudo Random Binary Sequence based on 9th degree polynomial
				BLE_PRBS_15                             = 0x0C,         //!< Pseudo Random Binary Sequence based on 15th degree polynomial
				BLE_EYELONG_1_0                         = 0x04,         //!< Repeated '11110000' sequence
				BLE_EYELONG_0_1                         = 0x18,         //!< Repeated '00001111' sequence
				BLE_EYESHORT_1_0                        = 0x08,         //!< Repeated '10101010' sequence
				BLE_EYESHORT_0_1                        = 0x1C,         //!< Repeated '01010101' sequence
				BLE_ALL_1                               = 0x10,         //!< Repeated '11111111' sequence
				BLE_ALL_0                               = 0x14,         //!< Repeated '00000000' sequence
			};

			enum RadioIrqMasks
			{
				IRQ_RADIO_NONE                          = 0x0000,
				IRQ_TX_DONE                             = 0x0001,
				IRQ_RX_DONE                             = 0x0002,
				IRQ_SYNCWORD_VALID                      = 0x0004,
				IRQ_SYNCWORD_ERROR                      = 0x0008,
				IRQ_HEADER_VALID                        = 0x0010,
				IRQ_HEADER_ERROR                        = 0x0020,
				IRQ_CRC_ERROR                           = 0x0040,
				IRQ_RANGING_SLAVE_RESPONSE_DONE         = 0x0080,
				IRQ_RANGING_SLAVE_REQUEST_DISCARDED     = 0x0100,
				IRQ_RANGING_MASTER_RESULT_VALID         = 0x0200,
				IRQ_RANGING_MASTER_TIMEOUT              = 0x0400,
				IRQ_RANGING_SLAVE_REQUEST_VALID         = 0x0800,
				IRQ_CAD_DONE                            = 0x1000,
				IRQ_CAD_DETECTED                        = 0x2000,
				IRQ_RX_TX_TIMEOUT                       = 0x4000,
				IRQ_PREAMBLE_DETECTED                   = 0x8000,
				IRQ_RADIO_ALL                           = 0xFFFF,
			};

			enum RadioDios
			{
				RADIO_DIO1                              = 0x02,
				RADIO_DIO2                              = 0x04,
				RADIO_DIO3                              = 0x08,
			};
			
			enum RadioTickSizes
			{
				RADIO_TICK_SIZE_0015_US                 = 0x00,
				RADIO_TICK_SIZE_0062_US                 = 0x01,
				RADIO_TICK_SIZE_1000_US                 = 0x02,
				RADIO_TICK_SIZE_4000_US                 = 0x03,
			};

			enum RadioRangingRoles
			{
				RADIO_RANGING_ROLE_SLAVE                = 0x00,
				RADIO_RANGING_ROLE_MASTER               = 0x01,
			};

			struct RadioStatus
			{
				struct
				{
					uint8_t CpuBusy   : 1;  //!< Flag for CPU radio busy
					uint8_t DmaBusy   : 1;  //!< Flag for DMA busy
					uint8_t CmdStatus : 3;  //!< Command status
					uint8_t ChipMode  : 3;  //!< Chip mode
				} Fields;
				uint8_t Value;
			};

			typedef struct
			{
				uint8_t WakeUpRTC               : 1;                    //!< Get out of sleep mode if wakeup signal received from RTC
				uint8_t InstructionRamRetention : 1;                    //!< InstructionRam is conserved during sleep
				uint8_t DataBufferRetention     : 1;                    //!< Data buffer is conserved during sleep
				uint8_t DataRamRetention        : 1;                    //!< Data ram is conserved during sleep
			} SleepParams;

			typedef struct
			{
				uint8_t RC64KEnable    : 1;                             //!< Calibrate RC64K clock
				uint8_t RC13MEnable    : 1;                             //!< Calibrate RC13M clock
				uint8_t PLLEnable      : 1;                             //!< Calibrate PLL
				uint8_t ADCPulseEnable : 1;                             //!< Calibrate ADC Pulse
				uint8_t ADCBulkNEnable : 1;                             //!< Calibrate ADC bulkN
				uint8_t ADCBulkPEnable : 1;                             //!< Calibrate ADC bulkP
			} CalibrationParams;

			typedef struct
			{
				RadioPacketTypes                    packetType;       //!< Packet to which the packet status are referring to.
				union
				{
					struct
					{
						uint16_t PacketReceived;                        //!< Number of received packets
						uint16_t CrcError;                              //!< Number of CRC errors
						uint16_t LengthError;                           //!< Number of length errors
						uint16_t SyncwordError;                         //!< Number of sync-word errors
					}Gfsk;
					struct
					{
						uint16_t PacketReceived;                        //!< Number of received packets
						uint16_t CrcError;                              //!< Number of CRC errors
						uint16_t HeaderValid;                           //!< Number of valid headers
					}LoRa;
				};
			} RxCounter;


			typedef struct
			{
				RadioPacketTypes                    packetType;        //!< Packet to which the packet status are referring to.
				union
				{
					struct
					{
						int8_t RssiSync;                                //!< The RSSI measured on last packet
						struct
						{
							bool SyncError :1;                          //!< SyncWord error on last packet
							bool LengthError :1;                        //!< Length error on last packet
							bool CrcError :1;                           //!< CRC error on last packet
							bool AbortError :1;                         //!< Abort error on last packet
							bool HeaderReceived :1;                     //!< Header received on last packet
							bool PacketReceived :1;                     //!< Packet received
							bool PacketControlerBusy :1;                //!< Packet controller busy
						}ErrorStatus;                                   //!< The error status Byte
						struct
						{
							bool RxNoAck :1;                            //!< No acknowledgment received for Rx with variable length packets
							bool PacketSent :1;                         //!< Packet sent, only relevant in Tx mode
						}TxRxStatus;                                    //!< The Tx/Rx status Byte
						uint8_t SyncAddrStatus :3;                      //!< The id of the correlator who found the packet
					}Gfsk;
					struct
					{
						int8_t RssiPkt;                                 //!< The RSSI of the last packet
						int8_t SnrPkt;                                  //!< The SNR of the last packet
					}LoRa;
					struct
					{
						int8_t RssiSync;                                //!< The RSSI of the last packet
						struct
						{
							bool SyncError :1;                          //!< SyncWord error on last packet
							bool LengthError :1;                        //!< Length error on last packet
							bool CrcError :1;                           //!< CRC error on last packet
							bool AbortError :1;                         //!< Abort error on last packet
							bool HeaderReceived :1;                     //!< Header received on last packet
							bool PacketReceived :1;                     //!< Packet received
							bool PacketControlerBusy :1;                //!< Packet controller busy
						}ErrorStatus;                                   //!< The error status Byte
						struct
						{
							uint8_t RxPid :2;                           //!< PID of the Rx
							bool RxNoAck :1;                            //!< No acknowledgment received for Rx with variable length packets
							bool RxPidErr :1;                           //!< Received PID error
							bool PacketSent :1;                         //!< Packet sent, only relevant in Tx mode
						}TxRxStatus;                                    //!< The Tx/Rx status Byte
						uint8_t SyncAddrStatus :3;                      //!< The id of the correlator who found the packet
					}Flrc;
					struct
					{
						int8_t RssiSync;                                //!< The RSSI of the last packet
						struct
						{
							bool SyncError :1;                          //!< SyncWord error on last packet
							bool LengthError :1;                        //!< Length error on last packet
							bool CrcError :1;                           //!< CRC error on last packet
							bool AbortError :1;                         //!< Abort error on last packet
							bool HeaderReceived :1;                     //!< Header received on last packet
							bool PacketReceived :1;                     //!< Packet received
							bool PacketControlerBusy :1;                //!< Packet controller busy
						}ErrorStatus;                                   //!< The error status Byte
						struct
						{
							bool PacketSent :1;                         //!< Packet sent, only relevant in Tx mode
						}TxRxStatus;                                    //!< The Tx/Rx status Byte
						uint8_t SyncAddrStatus :3;                      //!< The id of the correlator who found the packet
					}Ble;
				};
			} PacketStatus;

			typedef struct
			{
				RadioPacketTypes                    PacketType;       //!< Packet to which the packet parameters are referring to.
				struct
				{
					/*!
					 * \brief Holds the GFSK packet parameters
					 */
					struct
					{
						RadioPreambleLengths       PreambleLength;    //!< The preamble length for GFSK packet type
						RadioSyncWordLengths       SyncWordLength;    //!< The synchronization word length for GFSK packet type
						RadioSyncWordRxMatchs      SyncWordMatch;     //!< The synchronization correlator to use to check synchronization word
						RadioPacketLengthModes     HeaderType;        //!< If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
						uint8_t                      PayloadLength;     //!< Size of the payload in the GFSK packet
						RadioCrcTypes              CrcLength;         //!< Size of the CRC block in the GFSK packet
						RadioWhiteningModes        Whitening;         //!< Usage of whitening on payload and CRC blocks plus header block if header type is variable
					}Gfsk;
					/*!
					 * \brief Holds the LORA packet parameters
					 */
					struct
					{
						uint8_t                       PreambleLength;   //!< The preamble length is the number of LORA symbols in the preamble. To set it, use the following formula @code Number of symbols = PreambleLength[3:0] * ( 2^PreambleLength[7:4] ) @endcode
						RadioLoRaPacketLengthsModes HeaderType;       //!< If the header is explicit, it will be transmitted in the LORA packet. If the header is implicit, it will not be transmitted
						uint8_t                       PayloadLength;    //!< Size of the payload in the LORA packet
						RadioLoRaCrcModes           Crc;              //!< Size of CRC block in LORA packet
						RadioLoRaIQModes            InvertIQ;         //!< Allows to swap IQ for LORA packet
					}LoRa;
					/*!
					 * \brief Holds the FLRC packet parameters
					 */
					struct
					{
						RadioPreambleLengths       PreambleLength;    //!< The preamble length for FLRC packet type
						RadioFlrcSyncWordLengths   SyncWordLength;    //!< The synchronization word length for FLRC packet type
						RadioSyncWordRxMatchs      SyncWordMatch;     //!< The synchronization correlator to use to check synchronization word
						RadioPacketLengthModes     HeaderType;        //!< If the header is explicit, it will be transmitted in the FLRC packet. If the header is implicit, it will not be transmitted.
						uint8_t                      PayloadLength;     //!< Size of the payload in the FLRC packet
						RadioCrcTypes              CrcLength;         //!< Size of the CRC block in the FLRC packet
						RadioWhiteningModes        Whitening;         //!< Usage of whitening on payload and CRC blocks plus header block if header type is variable
					}Flrc;
					/*!
					 * \brief Holds the BLE packet parameters
					 */
					struct
					{
						RadioBleConnectionStates    ConnectionState;   //!< The BLE state
						RadioBleCrcTypes            CrcLength;         //!< Size of the CRC block in the BLE packet
						RadioBleTestPayloads        BleTestPayload;    //!< Special BLE payload for test purpose
						RadioWhiteningModes         Whitening;         //!< Usage of whitening on PDU and CRC blocks of BLE packet
					}Ble;
				}Params;                                                 //!< Holds the packet parameters structure
			} PacketParams;

			typedef struct
			{
				RadioPacketTypes                    PacketType;       //!< Packet to which the modulation parameters are referring to.
				struct
				{
					/*!
					 * \brief Holds the GFSK modulation parameters
					 *
					 * In GFSK modulation, the bit-rate and bandwidth are linked together. In this structure, its values are set using the same token.
					 */
					struct
					{
						RadioGfskBleBitrates    BitrateBandwidth;     //!< The bandwidth and bit-rate values for BLE and GFSK modulations
						RadioGfskBleModIndexes  ModulationIndex;      //!< The coding rate for BLE and GFSK modulations
						RadioModShapings        ModulationShaping;    //!< The modulation shaping for BLE and GFSK modulations
					}Gfsk;
					/*!
					 * \brief Holds the LORA modulation parameters
					 *
					 * LORA modulation is defined by Spreading Factor (SF), Bandwidth and Coding Rate
					 */
					struct
					{
						RadioLoRaSpreadingFactors  SpreadingFactor;   //!< Spreading Factor for the LORA modulation
						RadioLoRaBandwidths        Bandwidth;         //!< Bandwidth for the LORA modulation
						RadioLoRaCodingRates       CodingRate;        //!< Coding rate for the LORA modulation
					}LoRa;
					/*!
					 * \brief Holds the FLRC modulation parameters
					 *
					 * In FLRC modulation, the bit-rate and bandwidth are linked together. In this structure, its values are set using the same token.
					 */
					struct
					{
						RadioFlrcBitrates          BitrateBandwidth;  //!< The bandwidth and bit-rate values for FLRC modulation
						RadioFlrcCodingRates       CodingRate;        //!< The coding rate for FLRC modulation
						RadioModShapings           ModulationShaping; //!< The modulation shaping for FLRC modulation
					}Flrc;
					/*!
					 * \brief Holds the BLE modulation parameters
					 *
					 * In BLE modulation, the bit-rate and bandwidth are linked together. In this structure, its values are set using the same token.
					 */
					struct
					{
						RadioGfskBleBitrates       BitrateBandwidth;  //!< The bandwidth and bit-rate values for BLE and GFSK modulations
						RadioGfskBleModIndexes     ModulationIndex;   //!< The coding rate for BLE and GFSK modulations
						RadioModShapings           ModulationShaping; //!< The modulation shaping for BLE and GFSK modulations
					}Ble;
				}Params;                                                //!< Holds the modulation parameters structure
			} ModulationParams;
			
			typedef struct TickTime_s
			{
				RadioTickSizes PeriodBase;                            //!< The base time of ticktime
				/*!
				 * \brief The number of periodBase for ticktime
				 * Special values are:
				 *     - 0x0000 for single mode
				 *     - 0xFFFF for continuous mode
				 */
				uint16_t PeriodBaseCount;
			} TickTime;

			// Customization
			//#define GFSK_SUPPORT
			//#define FLRC_SUPPORT
			//#define BLE_SUPPORT
			#define RANGING_SUPPORT
			#define LORA_SUPPORT
			
			const uint32_t BUSY_PIN = 0x0118; // 1_24
			const uint32_t DIO1_PIN = 0x0006; // 0_6
			const uint32_t RESET_PIN = 0x011C; // 1_28
			
			const uint32_t SPI_MOSI = 0x010D; // 1_13
			const uint32_t SPI_MISO = 0x010E; // 1_14
			const uint32_t SPI_SCLK = 0x0007; // 0_7
			const uint32_t SPI_CSEL = 0x0011; // 0_17
			
			const uint32_t LORA_BUFFER_SIZE = 16;
			uint8_t txBuffer[16] = { 0 };
			uint8_t rxBuffer[16] = { 0 };
			
			const uint32_t RF_FREQUENCY = 2425000000UL;
			const uint32_t TX_OUTPUT_POWER = 13;

			const uint16_t TX_TIMEOUT_VALUE = 1000; // ms
			const uint16_t RX_TIMEOUT_VALUE = 0xffff; // ms
			const RadioTickSizes RX_TIMEOUT_TICK_SIZE = RADIO_TICK_SIZE_1000_US;

			const uint16_t IrqMask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;

			// Standard values

			const uint32_t AUTO_TX_OFFSET = 0x21;
			const uint32_t MASK_RANGINGMUXSEL = 0xCF;
			const uint32_t DEFAULT_RANGING_FILTER_SIZE = 0x7F;
			const uint32_t MASK_FORCE_PREAMBLELENGTH = 0x8F;
			const uint32_t BLE_ADVERTIZER_ACCESS_ADDRESS = 0x8E89BED6;
			
			SDD1306 *sdd1306;

			SX1280() { }
			
			void Init(SDD1306 &_sdd1306, bool pollMode) {
				// Configure control pins
				Chip_IOCON_PinMuxSet(LPC_IOCON, (RESET_PIN>>8), (RESET_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (BUSY_PIN>>8), (BUSY_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (BUSY_PIN>>8), (BUSY_PIN&0xFF));
				
				Chip_IOCON_PinMuxSet(LPC_IOCON, (DIO1_PIN>>8), (DIO1_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (DIO1_PIN>>8), (DIO1_PIN&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (SPI_MISO>>8), (SPI_MISO&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (SPI_MISO>>8), (SPI_MISO&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (SPI_MOSI>>8), (SPI_MOSI&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (SPI_MOSI>>8), (SPI_MOSI&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (SPI_CSEL>>8), (SPI_CSEL&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (SPI_SCLK>>8), (SPI_SCLK&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));

				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));
				
				Reset();

				if (!pollMode) {
					Chip_IOCON_PinMuxSet(LPC_IOCON, uint8_t(DIO1_PIN>>8), uint8_t(DIO1_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
					Chip_GPIO_SetPinDIRInput(LPC_GPIO, uint8_t(DIO1_PIN>>8), uint8_t(DIO1_PIN&0xFF));

					Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH0);
					Chip_PININT_EnableIntLow(LPC_PININT, Chip_PININT_GetHighEnabled(LPC_PININT) | PININTCH0);
					Chip_PININT_EnableIntHigh(LPC_PININT, Chip_PININT_GetLowEnabled(LPC_PININT) | PININTCH0);
					Chip_SYSCTL_SetPinInterrupt(0, uint8_t(DIO1_PIN>>8), uint8_t(DIO1_PIN&0xFF));  
					
					NVIC_ClearPendingIRQ(PIN_INT0_IRQn);  
					Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
					NVIC_EnableIRQ(PIN_INT0_IRQn);  
				}

				Wakeup();

				SetStandby( STDBY_RC );

				ModulationParams modulationParams;
				modulationParams.PacketType                  = PACKET_TYPE_LORA;
				modulationParams.Params.LoRa.SpreadingFactor = LORA_SF5;
				modulationParams.Params.LoRa.Bandwidth       = LORA_BW_0200;
				modulationParams.Params.LoRa.CodingRate      = LORA_CR_LI_4_7;
				SetPacketType( modulationParams.PacketType );
				SetModulationParams( &modulationParams );

				PacketParams PacketParams;
				PacketParams.PacketType                 	 = PACKET_TYPE_LORA;
				PacketParams.Params.LoRa.PreambleLength      = 0x0C;
				PacketParams.Params.LoRa.HeaderType          = LORA_PACKET_VARIABLE_LENGTH;
				PacketParams.Params.LoRa.PayloadLength       = LORA_BUFFER_SIZE;
				PacketParams.Params.LoRa.Crc                 = LORA_CRC_ON;
				PacketParams.Params.LoRa.InvertIQ            = LORA_IQ_NORMAL;
				SetPacketParams( &PacketParams );
				
				SetRfFrequency( RF_FREQUENCY );
				SetBufferBaseAddresses( 0x00, 0x00 );
				SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_20_US );
				SetDioIrqParams( SX1280::IrqMask, SX1280::IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				
		    	SetRx( TickTime { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );

				sdd1306 = &_sdd1306;
			}
			
			void SendBuffer() {
				SendPayload( txBuffer, LORA_BUFFER_SIZE, TickTime { RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } ); 
			}
			
			void Reset() {
				disableIRQ();
				
				delay( 50 );
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				Chip_GPIO_SetPinOutLow(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				delay( 50 );
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				delay( 50 );
				enableIRQ();

				WaitOnBusy();
				
				delay( 10 );
			}
			
			void Wakeup() {
				disableIRQ();

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite(RADIO_GET_STATUS);
				spiwrite(0);
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );

				enableIRQ( );
			}

			uint16_t GetFirmwareVersion( void )
			{
				return( ( ( ReadRegister( REG_LR_FIRMWARE_VERSION_MSB ) ) << 8 ) | ( ReadRegister( REG_LR_FIRMWARE_VERSION_MSB + 1 ) ) );
			}

			RadioStatus GetStatus( void )
			{
				uint8_t stat = 0;
				RadioStatus status;

				ReadCommand( RADIO_GET_STATUS, ( uint8_t * )&stat, 1 );
				status.Value = stat;
				return( status );
			}

			void WaitOnBusy() {
				while ( Chip_GPIO_GetPinState(LPC_GPIO, uint8_t(BUSY_PIN>>8), uint8_t(BUSY_PIN&0xFF)) == 1) { };
			}
			
			void WriteCommand(RadioCommand command, uint8_t *buffer, uint32_t size) {
				WaitOnBusy();

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( ( uint8_t )command );
				for( uint16_t i = 0; i < size; i++ ) {
					spiwrite( buffer[i] );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
			
				if( command != RADIO_SET_SLEEP ) {
					WaitOnBusy( );
				}
			}

			void ReadCommand(RadioCommand command, uint8_t *buffer, uint32_t size ) {
				WaitOnBusy();

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				if( command == RADIO_GET_STATUS ) {
					buffer[0] = spiwrite( ( uint8_t )command );
					spiwrite( 0 );
					spiwrite( 0 );
				} else {
					spiwrite( ( uint8_t )command );
					spiwrite( 0 );
					for( uint16_t i = 0; i < size; i++ )
					{
						 buffer[i] = spiwrite( 0 );
					}
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}
			
			void WriteRegister( uint32_t address, uint8_t *buffer, uint32_t size ) {
				WaitOnBusy( );

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( RADIO_WRITE_REGISTER );
				spiwrite( ( address & 0xFF00 ) >> 8 );
				spiwrite( address & 0x00FF );
				for( uint16_t i = 0; i < size; i++ ) {
					spiwrite( buffer[i] );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}

			void WriteRegister( uint32_t address, uint8_t value ) {
				WriteRegister( address, &value, 1 );
			}
			
			void ReadRegister( uint32_t address, uint8_t *buffer, uint32_t size ) {
				WaitOnBusy( );

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( RADIO_READ_REGISTER );
				spiwrite( ( address & 0xFF00 ) >> 8 );
				spiwrite( address & 0x00FF );
				spiwrite( 0 );
				for( uint16_t i = 0; i < size; i++ ) {
					buffer[i] = spiwrite( 0 );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}

			uint8_t ReadRegister( uint32_t address ) {
				uint8_t data;
				ReadRegister( address, &data, 1 );
				return data;
			}

			void WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size ) {
			    WaitOnBusy( );

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( RADIO_WRITE_BUFFER );
				spiwrite( offset );
				for( uint16_t i = 0; i < size; i++ ) {
					spiwrite( buffer[i] );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}

			void ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size ) {
			    WaitOnBusy( );

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( RADIO_READ_BUFFER );
				spiwrite( offset );
				spiwrite( 0 );
				for( uint16_t i = 0; i < size; i++ ) {
					buffer[i] = spiwrite( 0 );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}


			void SetSleep( SleepParams sleepConfig ) {
				uint8_t sleep = ( sleepConfig.WakeUpRTC << 3 ) |
								( sleepConfig.InstructionRamRetention << 2 ) |
								( sleepConfig.DataBufferRetention << 1 ) |
								( sleepConfig.DataRamRetention );

				OperatingMode = MODE_SLEEP;
				WriteCommand( RADIO_SET_SLEEP, &sleep, 1 );
			}

			void SetStandby( RadioStandbyModes standbyConfig ) {
				WriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
				if( standbyConfig == STDBY_RC )
				{
					OperatingMode = MODE_STDBY_RC;
				}
				else
				{
					OperatingMode = MODE_STDBY_XOSC;
				}
			}

			void SetFs( void ) {
				WriteCommand( RADIO_SET_FS, 0, 0 );
				OperatingMode = MODE_FS;
			}

			void SetTx( TickTime timeout ) {
				uint8_t buf[3];
				buf[0] = timeout.PeriodBase;
				buf[1] = ( uint8_t )( ( timeout.PeriodBaseCount >> 8 ) & 0x00FF );
				buf[2] = ( uint8_t )( timeout.PeriodBaseCount & 0x00FF );

				ClearIrqStatus( IRQ_RADIO_ALL );

				// If the radio is doing ranging operations, then apply the specific calls
				// prior to SetTx
				#ifdef RANGING_SUPPORT
				if( GetPacketType( true ) == PACKET_TYPE_RANGING )
				{
					SetRangingRole( RADIO_RANGING_ROLE_MASTER );
				}
				#endif  // #ifdef RANGING_SUPPORT
				WriteCommand( RADIO_SET_TX, buf, 3 );
				OperatingMode = MODE_TX;
			}

			void SetRx( TickTime timeout ) {
				uint8_t buf[3];
				buf[0] = timeout.PeriodBase;
				buf[1] = ( uint8_t )( ( timeout.PeriodBaseCount >> 8 ) & 0x00FF );
				buf[2] = ( uint8_t )( timeout.PeriodBaseCount & 0x00FF );

				ClearIrqStatus( IRQ_RADIO_ALL );

				// If the radio is doing ranging operations, then apply the specific calls
				// prior to SetRx
				#ifdef RANGING_SUPPORT
				if( GetPacketType( true ) == PACKET_TYPE_RANGING )
				{
					SetRangingRole( RADIO_RANGING_ROLE_SLAVE );
				}
				#endif  // #ifdef RANGING_SUPPORT
				WriteCommand( RADIO_SET_RX, buf, 3 );
				OperatingMode = MODE_RX;
			}

			void SetRxDutyCycle( RadioTickSizes periodBase, uint16_t periodBaseCountRx, uint16_t periodBaseCountSleep ) {
				uint8_t buf[5];

				buf[0] = periodBase;
				buf[1] = ( uint8_t )( ( periodBaseCountRx >> 8 ) & 0x00FF );
				buf[2] = ( uint8_t )( periodBaseCountRx & 0x00FF );
				buf[3] = ( uint8_t )( ( periodBaseCountSleep >> 8 ) & 0x00FF );
				buf[4] = ( uint8_t )( periodBaseCountSleep & 0x00FF );
				WriteCommand( RADIO_SET_RXDUTYCYCLE, buf, 5 );
				OperatingMode = MODE_RX;
			}

			void SetCad( void ) {
				WriteCommand( RADIO_SET_CAD, 0, 0 );
				OperatingMode = MODE_CAD;
			}

			void SetTxContinuousWave( void ) {
				WriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
			}

			void SetTxContinuousPreamble( void ) {
				WriteCommand( RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
			}

			void SetPacketType( RadioPacketTypes packetType ) {
				// Save packet type internally to avoid questioning the radio
				PacketType = packetType;

				WriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
			}

			RadioPacketTypes GetPacketType( bool returnLocalCopy ) {
				RadioPacketTypes packetType = PACKET_TYPE_NONE;
				if( returnLocalCopy == false )
				{
					ReadCommand( RADIO_GET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
					if( PacketType != packetType )
					{
						PacketType = packetType;
					}
				}
				else
				{
					packetType = PacketType;
				}
				return packetType;
			}
			
			void SetRfFrequency( uint32_t rfFrequency ) {
				uint8_t buf[3];
				uint32_t freq = 0;

				const uint64_t XTAL_FREQ = 52000000;
				freq = uint32_t( (uint64_t(rfFrequency) * uint64_t(262144)) / uint64_t(XTAL_FREQ) );

				buf[0] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
				buf[1] = ( uint8_t )( ( freq >> 8  ) & 0xFF );
				buf[2] = ( uint8_t )( ( freq       ) & 0xFF );
				WriteCommand( RADIO_SET_RFFREQUENCY, buf, 3 );
			}

			void SetTxParams( int8_t power, RadioRampTimes rampTime ) {
				uint8_t buf[2];

				// The power value to send on SPI/UART is in the range [0..31] and the
				// physical output power is in the range [-18..13]dBm
				buf[0] = power + 18;
				buf[1] = ( uint8_t )rampTime;
				WriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
			}

			void SetCadParams( RadioLoRaCadSymbols cadSymbolNum ) {
				WriteCommand( RADIO_SET_CADPARAMS, ( uint8_t* )&cadSymbolNum, 1 );
				OperatingMode = MODE_CAD;
			}

			void SetBufferBaseAddresses( uint8_t txBaseAddress, uint8_t rxBaseAddress ) {
				uint8_t buf[2];

				buf[0] = txBaseAddress;
				buf[1] = rxBaseAddress;
				WriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
			}

			void SetModulationParams( ModulationParams *modParams ) {
				uint8_t buf[3];

				// Check if required configuration corresponds to the stored packet type
				// If not, silently update radio packet type
				if( PacketType != modParams->PacketType )
				{
					SetPacketType( modParams->PacketType );
				}

				switch( modParams->PacketType )
				{
				#ifdef GFSK_SUPPORT
					case PACKET_TYPE_GFSK:
						buf[0] = modParams->Params.Gfsk.BitrateBandwidth;
						buf[1] = modParams->Params.Gfsk.ModulationIndex;
						buf[2] = modParams->Params.Gfsk.ModulationShaping;
						break;
				#endif  // #ifdef GFSK_SUPPORT
				#if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						buf[0] = modParams->Params.LoRa.SpreadingFactor;
						buf[1] = modParams->Params.LoRa.Bandwidth;
						buf[2] = modParams->Params.LoRa.CodingRate;
						LoRaBandwidth = modParams->Params.LoRa.Bandwidth;
						break;
				#endif  // #if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
				#ifdef FLRC_SUPPORT
					case PACKET_TYPE_FLRC:
						buf[0] = modParams->Params.Flrc.BitrateBandwidth;
						buf[1] = modParams->Params.Flrc.CodingRate;
						buf[2] = modParams->Params.Flrc.ModulationShaping;
						break;
				#endif  // #ifdef FLRC_SUPPORT
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						buf[0] = modParams->Params.Ble.BitrateBandwidth;
						buf[1] = modParams->Params.Ble.ModulationIndex;
						buf[2] = modParams->Params.Ble.ModulationShaping;
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
					case PACKET_TYPE_NONE:
						buf[0] = 0;
						buf[1] = 0;
						buf[2] = 0;
						break;
				}
				WriteCommand( RADIO_SET_MODULATIONPARAMS, buf, 3 );
			}

			void SetPacketParams( PacketParams *packetParams ) {
				uint8_t buf[7];
				// Check if required configuration corresponds to the stored packet type
				// If not, silently update radio packet type
				if( PacketType != packetParams->PacketType )
				{
					SetPacketType( packetParams->PacketType );
				}

				switch( packetParams->PacketType )
				{
				#ifdef GFSK_SUPPORT
					case PACKET_TYPE_GFSK:
						buf[0] = packetParams->Params.Gfsk.PreambleLength;
						buf[1] = packetParams->Params.Gfsk.SyncWordLength;
						buf[2] = packetParams->Params.Gfsk.SyncWordMatch;
						buf[3] = packetParams->Params.Gfsk.HeaderType;
						buf[4] = packetParams->Params.Gfsk.PayloadLength;
						buf[5] = packetParams->Params.Gfsk.CrcLength;
						buf[6] = packetParams->Params.Gfsk.Whitening;
						break;
				#endif  // #ifdef GFSK_SUPPORT
				#if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						buf[0] = packetParams->Params.LoRa.PreambleLength;
						buf[1] = packetParams->Params.LoRa.HeaderType;
						buf[2] = packetParams->Params.LoRa.PayloadLength;
						buf[3] = packetParams->Params.LoRa.Crc;
						buf[4] = packetParams->Params.LoRa.InvertIQ;
						buf[5] = 0;
						buf[6] = 0;
						break;
				#endif  // #if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
				#ifdef FLRC_SUPPORT
					case PACKET_TYPE_FLRC:
						buf[0] = packetParams->Params.Flrc.PreambleLength;
						buf[1] = packetParams->Params.Flrc.SyncWordLength;
						buf[2] = packetParams->Params.Flrc.SyncWordMatch;
						buf[3] = packetParams->Params.Flrc.HeaderType;
						buf[4] = packetParams->Params.Flrc.PayloadLength;
						buf[5] = packetParams->Params.Flrc.CrcLength;
						buf[6] = packetParams->Params.Flrc.Whitening;
						break;
				#endif  // #ifdef FLRC_SUPPORT
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						buf[0] = packetParams->Params.Ble.ConnectionState;
						buf[1] = packetParams->Params.Ble.CrcLength;
						buf[2] = packetParams->Params.Ble.BleTestPayload;
						buf[3] = packetParams->Params.Ble.Whitening;
						buf[4] = 0;
						buf[5] = 0;
						buf[6] = 0;
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
					case PACKET_TYPE_NONE:
						buf[0] = 0;
						buf[1] = 0;
						buf[2] = 0;
						buf[3] = 0;
						buf[4] = 0;
						buf[5] = 0;
						buf[6] = 0;
						break;
				}
				WriteCommand( RADIO_SET_PACKETPARAMS, buf, 7 );
			}

			void ForcePreambleLength( RadioPreambleLengths preambleLength ) {
				WriteRegister( REG_LR_PREAMBLELENGTH, ( ReadRegister( REG_LR_PREAMBLELENGTH ) & MASK_FORCE_PREAMBLELENGTH ) | preambleLength );
			}

			void GetRxBufferStatus( uint8_t *rxPayloadLength, uint8_t *rxStartBufferPointer ) {
				uint8_t status[2];

				ReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

				// In case of LORA fixed header, the rxPayloadLength is obtained by reading
				// the register REG_LR_PAYLOADLENGTH
				if( ( this -> GetPacketType( true ) == PACKET_TYPE_LORA ) && ( ReadRegister( REG_LR_PACKETPARAMS ) >> 7 == 1 ) )
				{
					*rxPayloadLength = ReadRegister( REG_LR_PAYLOADLENGTH );
				}
				else if( this -> GetPacketType( true ) == PACKET_TYPE_BLE )
				{
					// In the case of BLE, the size returned in status[0] do not include the 2-byte length PDU header
					// so it is added there
					*rxPayloadLength = status[0] + 2;
				}
				else
				{
					*rxPayloadLength = status[0];
				}

				*rxStartBufferPointer = status[1];
			}

			void GetPacketStatus( PacketStatus *packetStatus ) {
				uint8_t status[5];

				ReadCommand( RADIO_GET_PACKETSTATUS, status, 5 );

				packetStatus->packetType = this -> GetPacketType( true );
				switch( packetStatus->packetType )
				{
				#ifdef GFSK_SUPPORT
					case PACKET_TYPE_GFSK:
						packetStatus->Gfsk.RssiSync = -( status[1] / 2 );

						packetStatus->Gfsk.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

						packetStatus->Gfsk.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
						packetStatus->Gfsk.TxRxStatus.PacketSent = status[3] & 0x01;

						packetStatus->Gfsk.SyncAddrStatus = status[4] & 0x07;
						break;
				#endif  // #ifdef GFSK_SUPPORT
				#if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						packetStatus->LoRa.RssiPkt = -( status[0] / 2 );
						( status[1] < 128 ) ? ( packetStatus->LoRa.SnrPkt = status[1] / 4 ) : ( packetStatus->LoRa.SnrPkt = ( ( status[1] - 256 ) /4 ) );
						break;
				#endif  // #if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
				#ifdef FLRC_SUPPORT
					case PACKET_TYPE_FLRC:
						packetStatus->Flrc.RssiSync = -( status[1] / 2 );

						packetStatus->Flrc.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

						packetStatus->Flrc.TxRxStatus.RxPid = ( status[3] >> 6 ) & 0x03;
						packetStatus->Flrc.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
						packetStatus->Flrc.TxRxStatus.RxPidErr = ( status[3] >> 4 ) & 0x01;
						packetStatus->Flrc.TxRxStatus.PacketSent = status[3] & 0x01;

						packetStatus->Flrc.SyncAddrStatus = status[4] & 0x07;
						break;
				#endif  // #ifdef FLRC_SUPPORT
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						packetStatus->Ble.RssiSync =  -( status[1] / 2 );

						packetStatus->Ble.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
						packetStatus->Ble.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
						packetStatus->Ble.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
						packetStatus->Ble.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
						packetStatus->Ble.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
						packetStatus->Ble.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
						packetStatus->Ble.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

						packetStatus->Ble.TxRxStatus.PacketSent = status[3] & 0x01;

						packetStatus->Ble.SyncAddrStatus = status[4] & 0x07;
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
					case PACKET_TYPE_NONE:
						// In that specific case, we set everything in the packetStatus to zeros
						// and reset the packet type accordingly
						for (uint32_t c=0; c<sizeof(PacketStatus); c++) { ((uint8_t*)packetStatus)[c] = 0; } 
						packetStatus->packetType = PACKET_TYPE_NONE;
						break;
				}
			}

			int8_t GetRssiInst( void ) {
				uint8_t raw = 0;

				ReadCommand( RADIO_GET_RSSIINST, &raw, 1 );

				return ( int8_t ) ( -raw / 2 );
			}

			void SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask ) {
				uint8_t buf[8];

				buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
				buf[1] = ( uint8_t )( irqMask & 0x00FF );
				buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
				buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
				buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
				buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
				buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
				buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
				WriteCommand( RADIO_SET_DIOIRQPARAMS, buf, 8 );
			}

			uint16_t GetIrqStatus( void ) {
				uint8_t irqStatus[2];
				ReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
				return ( irqStatus[0] << 8 ) | irqStatus[1];
			}

			void ClearIrqStatus( uint16_t irqMask ) {
				uint8_t buf[2];

				buf[0] = ( uint8_t )( ( ( uint16_t )irqMask >> 8 ) & 0x00FF );
				buf[1] = ( uint8_t )( ( uint16_t )irqMask & 0x00FF );
				WriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
			}

			void Calibrate( CalibrationParams calibParam ) {
				uint8_t cal = ( calibParam.ADCBulkPEnable << 5 ) |
							  ( calibParam.ADCBulkNEnable << 4 ) |
							  ( calibParam.ADCPulseEnable << 3 ) |
							  ( calibParam.PLLEnable << 2 ) |
							  ( calibParam.RC13MEnable << 1 ) |
							  ( calibParam.RC64KEnable );
				WriteCommand( RADIO_CALIBRATE, &cal, 1 );
			}

			void SetRegulatorMode( RadioRegulatorModes mode ) {
				WriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
			}

			void SetSaveContext( void ) {
				WriteCommand( RADIO_SET_SAVECONTEXT, 0, 0 );
			}

			void SetAutoTx( uint16_t time ) {
				uint16_t compensatedTime = time - ( uint16_t )AUTO_TX_OFFSET;
				uint8_t buf[2];

				buf[0] = ( uint8_t )( ( compensatedTime >> 8 ) & 0x00FF );
				buf[1] = ( uint8_t )( compensatedTime & 0x00FF );
				WriteCommand( RADIO_SET_AUTOTX, buf, 2 );
			}

			void SetAutoFs( bool enableAutoFs ) {
				WriteCommand( RADIO_SET_AUTOFS, ( uint8_t * )&enableAutoFs, 1 );
			}

			void SetLongPreamble( bool enable ) {
				WriteCommand( RADIO_SET_LONGPREAMBLE, ( uint8_t * )&enable, 1 );
			}

			void SetPayload( uint8_t *buffer, uint8_t size, uint8_t offset ) {
				WriteBuffer( offset, buffer, size );
			}

			uint8_t GetPayload( uint8_t *buffer, uint8_t *size , uint8_t maxSize ) {
				uint8_t offset;

				GetRxBufferStatus( size, &offset );
				if( *size > maxSize )
				{
					return 1;
				}
				ReadBuffer( offset, buffer, *size );
				return 0;
			}

			void SendPayload( uint8_t *payload, uint8_t size, TickTime timeout, uint8_t offset = 0 ) {
				SetPayload( payload, size, offset );
				SetTx( timeout );
			}

			uint8_t SetSyncWord( uint8_t syncWordIdx, uint8_t *syncWord ) {
				uint16_t addr;
				uint8_t syncwordSize = 0;

				switch( GetPacketType( true ) )
				{
				#ifdef GFSK_SUPPORT
					case PACKET_TYPE_GFSK:
						syncwordSize = 5;
						switch( syncWordIdx )
						{
							case 1:
								addr = REG_LR_SYNCWORDBASEADDRESS1;
								break;
							case 2:
								addr = REG_LR_SYNCWORDBASEADDRESS2;
								break;
							case 3:
								addr = REG_LR_SYNCWORDBASEADDRESS3;
								break;
							default:
								return 1;
						}
						break;
				#endif  // #ifdef GFSK_SUPPORT
				#ifdef FLRC_SUPPORT
					case PACKET_TYPE_FLRC:
						// For FLRC packet type, the SyncWord is one byte shorter and
						// the base address is shifted by one byte
						syncwordSize = 4;
						switch( syncWordIdx )
						{
							case 1:
								addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
								break;
							case 2:
								addr = REG_LR_SYNCWORDBASEADDRESS2 + 1;
								break;
							case 3:
								addr = REG_LR_SYNCWORDBASEADDRESS3 + 1;
								break;
							default:
								return 1;
						}
						break;
				#endif  // #ifdef FLRC_SUPPORT
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						// For Ble packet type, only the first SyncWord is used and its
						// address is shifted by one byte
						syncwordSize = 4;
						switch( syncWordIdx )
						{
							case 1:
								addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
								break;
							default:
								return 1;
						}
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
						return 1;
				}
				WriteRegister( addr, syncWord, syncwordSize );
				return 0;
			}

			void SetSyncWordErrorTolerance( uint8_t ErrorBits ) {
				ErrorBits = ( ReadRegister( REG_LR_SYNCWORDTOLERANCE ) & 0xF0 ) | ( ErrorBits & 0x0F );
				WriteRegister( REG_LR_SYNCWORDTOLERANCE, ErrorBits );
			}

			uint8_t SetCrcSeed( uint8_t *seed ) {
				uint8_t updated = 0;
				switch( GetPacketType( true ) )
				{
				#if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
						WriteRegister( REG_LR_CRCSEEDBASEADDR, seed, 2 );
						updated = 1;
						break;
				#endif  // #if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						WriteRegister(0x9c7, seed[2] );
						WriteRegister(0x9c8, seed[1] );
						WriteRegister(0x9c9, seed[0] );
						updated = 1;
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
						break;
				}
				return updated;
			}

			#ifdef BLE_SUPPORT
			void SetBleAccessAddress( uint32_t accessAddress ) {
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS, ( accessAddress >> 24 ) & 0x000000FF );
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 1, ( accessAddress >> 16 ) & 0x000000FF );
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 2, ( accessAddress >> 8 ) & 0x000000FF );
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 3, accessAddress & 0x000000FF );
			}

			void SetBleAdvertizerAccessAddress( void ) {
				SetBleAccessAddress( BLE_ADVERTIZER_ACCESS_ADDRESS );
			}
			#endif  // #ifdef BLE_SUPPORT

			void SetCrcPolynomial( uint16_t polynomial ) {
				switch( GetPacketType( true ) )
				{
				#if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
						uint8_t val[2];
						val[0] = ( uint8_t )( polynomial >> 8 ) & 0xFF;
						val[1] = ( uint8_t )( polynomial  & 0xFF );
						WriteRegister( REG_LR_CRCPOLYBASEADDR, val, 2 );
						break;
				#endif  // #if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
					default:
						break;
				}
			}

			void SetWhiteningSeed( uint8_t seed ) {
				switch( GetPacketType( true ) )
				{
				#if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT) || defined(BLE_SUPPORT)
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
					case PACKET_TYPE_BLE:
						WriteRegister( REG_LR_WHITSEEDBASEADDR, seed );
						break;
				#endif  // #if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT) || defined(BLE_SUPPORT)
					default:
						break;
				}
			}

			#ifdef RANGING_SUPPORT
			void SetRangingIdLength( RadioRangingIdCheckLengths length ) {
				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						WriteRegister( REG_LR_RANGINGIDCHECKLENGTH, ( ( ( ( uint8_t )length ) & 0x03 ) << 6 ) | ( ReadRegister( REG_LR_RANGINGIDCHECKLENGTH ) & 0x3F ) );
						break;
					default:
						break;
				}
			}

			void SetDeviceRangingAddress( uint32_t address ) {
				uint8_t addrArray[] = { uint8_t(address >> 24), uint8_t(address >> 16), uint8_t(address >> 8), uint8_t(address) };

				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						WriteRegister( REG_LR_DEVICERANGINGADDR, addrArray, 4 );
						break;
					default:
						break;
				}
			}

			void SetRangingRequestAddress( uint32_t address ) {
				uint8_t addrArray[] = { uint8_t(address >> 24), uint8_t(address >> 16), uint8_t(address >> 8), uint8_t(address) };

				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						WriteRegister( REG_LR_REQUESTRANGINGADDR, addrArray, 4 );
						break;
					default:
						break;
				}
			}

			double GetRangingResult( RadioRangingResultTypes resultType ) {
				uint32_t valLsb = 0;
				double val = 0.0;

				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						SetStandby( STDBY_XOSC );
						WriteRegister( 0x97F, ReadRegister( 0x97F ) | ( 1 << 1 ) ); // enable LORA modem clock
						WriteRegister( REG_LR_RANGINGRESULTCONFIG, ( ReadRegister( REG_LR_RANGINGRESULTCONFIG ) & MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )resultType ) & 0x03 ) << 4 ) );
						valLsb = ( ( ReadRegister( REG_LR_RANGINGRESULTBASEADDR ) << 16 ) | ( ReadRegister( REG_LR_RANGINGRESULTBASEADDR + 1 ) << 8 ) | ( ReadRegister( REG_LR_RANGINGRESULTBASEADDR + 2 ) ) );
						SetStandby( STDBY_RC );

						// Convertion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280
						switch( resultType )
						{
							case RANGING_RESULT_RAW:
								// Convert the ranging LSB to distance in meter
								// The theoretical conversion from register value to distance [m] is given by:
								// distance [m] = ( complement2( register ) * 150 ) / ( 2^12 * bandwidth[MHz] ) )
								// The API provide BW in [Hz] so the implemented formula is complement2( register ) / bandwidth[Hz] * A,
								// where A = 150 / (2^12 / 1e6) = 36621.09
								val = ( double )complement2( valLsb, 24 ) / ( double )GetLoRaBandwidth( ) * 36621.09375;
								break;

							case RANGING_RESULT_AVERAGED:
							case RANGING_RESULT_DEBIASED:
							case RANGING_RESULT_FILTERED:
								val = ( double )valLsb * 20.0 / 100.0;
								break;
							default:
								val = 0.0;
						}
						break;
					default:
						break;
				}
				return val;
			}

			void SetRangingCalibration( uint16_t cal ) {
				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						WriteRegister( REG_LR_RANGINGRERXTXDELAYCAL, ( uint8_t )( ( cal >> 8 ) & 0xFF ) );
						WriteRegister( REG_LR_RANGINGRERXTXDELAYCAL + 1, ( uint8_t )( ( cal ) & 0xFF ) );
						break;
					default:
						break;
				}
			}

			void RangingClearFilterResult( void ) {
				uint8_t regVal = ReadRegister( REG_LR_RANGINGRESULTCLEARREG );

				// To clear result, set bit 5 to 1 then to 0
				WriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal | ( 1 << 5 ) );
				WriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal & ( ~( 1 << 5 ) ) );
			}

			void RangingSetFilterNumSamples( uint8_t num ) {
				// Silently set 8 as minimum value
				WriteRegister( REG_LR_RANGINGFILTERWINDOWSIZE, ( num < DEFAULT_RANGING_FILTER_SIZE ) ? DEFAULT_RANGING_FILTER_SIZE : num );
			}

			void SetRangingRole( RadioRangingRoles role ) {
				uint8_t buf[1];

				buf[0] = role;
				WriteCommand( RADIO_SET_RANGING_ROLE, &buf[0], 1 );
			}
			#endif  // #ifdef RANGING_SUPPORT

			double GetFrequencyError( ) {
				uint8_t efeRaw[3] = {0};
				uint32_t efe = 0;
				double efeHz = 0.0;

				switch( GetPacketType( true ) )
				{
				#if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						efeRaw[0] = ReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
						efeRaw[1] = ReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
						efeRaw[2] = ReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
						efe = ( efeRaw[0]<<16 ) | ( efeRaw[1]<<8 ) | efeRaw[2];
						efe &= REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

						efeHz = 1.55 * ( double )complement2( efe, 20 ) / ( 1600.0 / ( double )GetLoRaBandwidth( ) * 1000.0 );
						break;
				#endif  // #if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)

					case PACKET_TYPE_NONE:
					case PACKET_TYPE_BLE:
					case PACKET_TYPE_FLRC:
					case PACKET_TYPE_GFSK:
						break;
				}

				return efeHz;
			}

			void SetPollingMode( void ) {
				PollingMode = true;
			}

			#ifdef LORA_SUPPORT
			int32_t GetLoRaBandwidth( ) {
				int32_t bwValue = 0;

				switch( LoRaBandwidth ) {
					case LORA_BW_0200:
						bwValue = 203125;
						break;
					case LORA_BW_0400:
						bwValue = 406250;
						break;
					case LORA_BW_0800:
						bwValue = 812500;
						break;
					case LORA_BW_1600:
						bwValue = 1625000;
						break;
					default:
						bwValue = 0;
				}
				return bwValue;
			}
			#endif  // #ifdef LORA_SUPPORT

			void SetInterruptMode( void ) {
				PollingMode = false;
			}

			void OnDioIrq( void ) {
				/*
				 * When polling mode is activated, it is up to the application to call
				 * ProcessIrqs( ). Otherwise, the driver automatically calls ProcessIrqs( )
				 * on radio interrupt.
				 */
				if( PollingMode == true ) {
					IrqState = true;
				} else {
					ProcessIrqs( );
				}
			}

			void ProcessIrqs( void ) {
				RadioPacketTypes packetType = PACKET_TYPE_NONE;

				if( PollingMode == true ) {
					if( IrqState == true ) {
			            disableIRQ( );
						IrqState = false;
			            enableIRQ( );
					} else {
						return;
					}
				}

				packetType = GetPacketType( true );
				uint16_t irqRegs = GetIrqStatus( );
				ClearIrqStatus( IRQ_RADIO_ALL );

				switch( packetType )
				{
				#if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT) || defined(BLE_SUPPORT)
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
					case PACKET_TYPE_BLE:
						switch( OperatingMode )
						{
							case MODE_RX:
								if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
								{
									if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
									{
										rxError( IRQ_CRC_ERROR_CODE );
									}
									else if( ( irqRegs & IRQ_SYNCWORD_ERROR ) == IRQ_SYNCWORD_ERROR )
									{
										rxError( IRQ_SYNCWORD_ERROR_CODE );
									}
									else
									{
										rxDone( );
									}
								}
								if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
								{
									rxSyncWordDone( );
								}
								if( ( irqRegs & IRQ_SYNCWORD_ERROR ) == IRQ_SYNCWORD_ERROR )
								{
									rxError( IRQ_SYNCWORD_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									rxTimeout( );
								}
								break;
							case MODE_TX:
								if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
								{
									txDone( );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									txTimeout( );
								}
								break;
							default:
								// Unexpected IRQ: silently returns
								break;
						}
						break;
				#endif  // #if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT) || defined(BLE_SUPPORT)
				#ifdef LORA_SUPPORT
					case PACKET_TYPE_LORA:
						switch( OperatingMode )
						{
							case MODE_RX:
								if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
								{
									if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
									{
										rxError( IRQ_CRC_ERROR_CODE );
									}
									else
									{
										rxDone( );
									}
								}
								if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
								{
									rxHeaderDone( );
								}
								if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
								{
									rxError( IRQ_HEADER_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									rxTimeout( );
								}
								if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_DISCARDED ) == IRQ_RANGING_SLAVE_REQUEST_DISCARDED )
								{
									rxError( IRQ_RANGING_ON_LORA_ERROR_CODE );
								}
								break;
							case MODE_TX:
								if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
								{
									txDone( );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									txTimeout( );
								}
								break;
							case MODE_CAD:
								if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
								{
									if( ( irqRegs & IRQ_CAD_DETECTED ) == IRQ_CAD_DETECTED )
									{
										cadDone( true );
									}
									else
									{
										cadDone( false );
									}
								}
								else if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									rxTimeout( );
								}
								break;
							default:
								// Unexpected IRQ: silently returns
								break;
						}
						break;
				#endif  // #ifdef LORA_SUPPORT
				#ifdef RANGING_SUPPORT
					case PACKET_TYPE_RANGING:
						switch( OperatingMode )
						{
							// MODE_RX indicates an IRQ on the Slave side
							case MODE_RX:
								if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_DISCARDED ) == IRQ_RANGING_SLAVE_REQUEST_DISCARDED )
								{
									rangingDone( IRQ_RANGING_SLAVE_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_VALID ) == IRQ_RANGING_SLAVE_REQUEST_VALID )
								{
									rangingDone( IRQ_RANGING_SLAVE_VALID_CODE );
								}
								if( ( irqRegs & IRQ_RANGING_SLAVE_RESPONSE_DONE ) == IRQ_RANGING_SLAVE_RESPONSE_DONE )
								{
									rangingDone( IRQ_RANGING_SLAVE_VALID_CODE );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									rangingDone( IRQ_RANGING_SLAVE_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
								{
									rxHeaderDone( );
								}
								if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
								{
									rxError( IRQ_HEADER_ERROR_CODE );
								}
								break;
							// MODE_TX indicates an IRQ on the Master side
							case MODE_TX:
								if( ( irqRegs & IRQ_RANGING_MASTER_TIMEOUT ) == IRQ_RANGING_MASTER_TIMEOUT )
								{
									rangingDone( IRQ_RANGING_MASTER_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_RANGING_MASTER_RESULT_VALID ) == IRQ_RANGING_MASTER_RESULT_VALID )
								{
									rangingDone( IRQ_RANGING_MASTER_VALID_CODE );
								}
								break;
							default:
								// Unexpected IRQ: silently returns
								break;
						}
						break;
				#endif  // #ifdef RANGING_SUPPORT
					default:
						// Unexpected IRQ: silently returns
						break;
				}
			}

			void txDone() {
				sdd1306->PlaceAsciiStr(0,0,"TXDONE!!");
				sdd1306->Display();
				delay(250);
			}

			void rxDone() {
			    PacketStatus packetStatus;
				GetPacketStatus(&packetStatus);
				uint8_t rxBufferSize = 0;
                GetPayload( rxBuffer, &rxBufferSize, LORA_BUFFER_SIZE );

				sdd1306->PlaceAsciiStr(0,0,"RXDONE!!");
				sdd1306->Display();
				delay(250);
			}

			void rxSyncWordDone() {
				sdd1306->PlaceAsciiStr(0,0,"RXSYNCDO");
				sdd1306->Display();
				delay(250);
			}

			void rxHeaderDone() {
				sdd1306->PlaceAsciiStr(0,0,"RXHEADER");
				sdd1306->Display();
				delay(250);
			}
			
			void txTimeout() {
				sdd1306->PlaceAsciiStr(0,0,"TXTIMOUT");
				sdd1306->Display();
				delay(250);
			}
			
			void rxTimeout() {
				sdd1306->PlaceAsciiStr(0,0,"RXTIMOUT");
				sdd1306->Display();
				delay(250);
			}
			
			void rxError(IrqErrorCode errCode) {
				sdd1306->PlaceAsciiStr(0,0,"RXERROR!");
				sdd1306->Display();
				delay(250);
			}

			void rangingDone(IrqRangingCode errCode) {
				// TODO
			}

			void cadDone(bool cadFlag) {
				// TODO
			}
			
			void SendBeacon() {
				const char *beacon = "DUCKPONDDUCKPOND";
				memcpy(txBuffer,beacon,16);
				SendBuffer();
			}

	private:
	
			void disableIRQ() {
				NVIC_DisableIRQ(PIN_INT0_IRQn);
			}

			void enableIRQ() {
				NVIC_EnableIRQ(PIN_INT0_IRQn);
			}

			uint8_t spiwrite(uint8_t val) {
				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));
				uint8_t read_value = 0;
				for (uint32_t c=0; c<8; c++) {
					read_value <<= 1;
					Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));
					if ((val&(1<<(7-c)))) {
						Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_MOSI>>8), (SPI_MOSI&0xFF));
					} else {
						Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_MOSI>>8), (SPI_MOSI&0xFF));
					}
					read_value |= Chip_GPIO_GetPinState(LPC_GPIO, (SPI_MISO>>8), (SPI_MISO&0xFF));
					Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));
				}
				return read_value;
			}
						
			int32_t complement2( const uint32_t num, const uint8_t bitCnt ) {
				int32_t retVal = int32_t(num);
				if( int32_t(num) >= 2<<( bitCnt - 2 ) ) {
					retVal -= 2<<( bitCnt - 1 );
				}
				return retVal;
			}

    RadioOperatingModes 	OperatingMode = MODE_SLEEP;
	RadioPacketTypes 		PacketType = PACKET_TYPE_NONE;
    RadioLoRaBandwidths 	LoRaBandwidth = LORA_BW_0200;
    bool 					IrqState = false;
    bool 					PollingMode = true;

};

class UI {
	
	EEPROM &settings;
	SDD1306 &sdd1306;
	
	uint32_t mode;
	uint32_t mode_start_time;
	
public:
	
	UI(EEPROM &_settings,
	   SDD1306 &_sdd1306):
		settings(_settings),
		sdd1306(_sdd1306) {
	}

	const uint32_t PRIMARY_BUTTON = 0x0119;
	const uint32_t SECONDARY_BUTTON = 0x0001;

	void Init() {
		
		mode = 0;
		
		Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PINT);

		Chip_IOCON_PinMuxSet(LPC_IOCON, uint8_t(PRIMARY_BUTTON>>8), uint8_t(PRIMARY_BUTTON&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, uint8_t(PRIMARY_BUTTON>>8), uint8_t(PRIMARY_BUTTON&0xFF));

		Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH1);
		Chip_PININT_EnableIntLow(LPC_PININT, Chip_PININT_GetHighEnabled(LPC_PININT) | PININTCH1);
		Chip_PININT_EnableIntHigh(LPC_PININT, Chip_PININT_GetLowEnabled(LPC_PININT) | PININTCH1);
		Chip_SYSCTL_SetPinInterrupt(1, uint8_t(PRIMARY_BUTTON>>8), uint8_t(PRIMARY_BUTTON&0xFF));  

		NVIC_ClearPendingIRQ(PIN_INT1_IRQn);  
		Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
		NVIC_EnableIRQ(PIN_INT1_IRQn);  
		
		Chip_IOCON_PinMuxSet(LPC_IOCON, uint8_t(SECONDARY_BUTTON>>8), uint8_t(SECONDARY_BUTTON&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, uint8_t(SECONDARY_BUTTON>>8), uint8_t(SECONDARY_BUTTON&0xFF));

		Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH2);
		Chip_PININT_EnableIntLow(LPC_PININT, Chip_PININT_GetHighEnabled(LPC_PININT) | PININTCH2);
		Chip_PININT_EnableIntHigh(LPC_PININT, Chip_PININT_GetLowEnabled(LPC_PININT) | PININTCH2);
		Chip_SYSCTL_SetPinInterrupt(2, uint8_t(SECONDARY_BUTTON>>8), uint8_t(SECONDARY_BUTTON&0xFF));  

		NVIC_ClearPendingIRQ(PIN_INT2_IRQn);  
		Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
		NVIC_EnableIRQ(PIN_INT2_IRQn);  
	}
	
	void HandleINT1IRQ() {
		static uint32_t fall_time = 0;
		if ( (Chip_PININT_GetFallStates(LPC_PININT) & PININTCH1) ) {
			Chip_PININT_ClearFallStates(LPC_PININT, PININTCH1);
			fall_time = system_clock_ms;
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
		}

		if ( (Chip_PININT_GetRiseStates(LPC_PININT) & PININTCH1) ) {
			Chip_PININT_ClearRiseStates(LPC_PININT, PININTCH1);
			if ( (system_clock_ms - fall_time) > 20) {
				settings.NextEffect();
			}
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
		}
	}
	
	void HandleINT2IRQ() {
		static uint32_t fall_time = 0;
		if ( (Chip_PININT_GetFallStates(LPC_PININT) & PININTCH2) ) {
			Chip_PININT_ClearFallStates(LPC_PININT, PININTCH2);
			fall_time = system_clock_ms;
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
		}

		if ( (Chip_PININT_GetRiseStates(LPC_PININT) & PININTCH2) ) {
			Chip_PININT_ClearRiseStates(LPC_PININT, PININTCH2);
			if ( (system_clock_ms - fall_time) > 20) {
				settings.NextBrightness();
			}
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
		}
	}
	
	void SetMode(uint32_t current_time, uint32_t _mode) {
		mode_start_time = current_time;
		mode = _mode;
	}

	void DisplayNow() {
		if ((system_clock_ms - mode_start_time) < 50) {
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,0,0xB8+c);
			}
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,1,0xC0+c);
			}
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,2,0xC8+c);
			}
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,3,0);
			}
			sdd1306.SetVerticalShift(0);
		}
		else if ((system_clock_ms - mode_start_time) < 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - 50;
			static uint8_t bounce[] = {
				0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1e, 0x1d, 0x1d, 
				0x1c, 0x1b, 0x1a, 0x18, 0x17, 0x16, 0x14, 0x12, 
				0x10, 0x0e, 0x0c, 0x0a, 0x08, 0x05, 0x03, 0x01, 
				0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x07, 0x08, 
				0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x07, 
				0x06, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 
				0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 
				0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 
			};
			uint32_t y = ltime/10;
			if (y >= 64) y = 63;
			sdd1306.Display();
			sdd1306.SetVerticalShift(bounce[y]);
		}
		else if ((system_clock_ms - mode_start_time) < 1000 + 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - (50 + 640 + 50);
			if (((ltime / 100)&1) == 0) {
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,0,0);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,1,0);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,2,0);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,3,0);
				}
				sdd1306.Display();
			} else if (((ltime / 100)&1) == 1) {
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,0,0xB8+c);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,1,0xC0+c);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,2,0xC8+c);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,3,0);
				}
				sdd1306.Display();
			}
		}
		else if ((system_clock_ms - mode_start_time) < 160 + 1000 + 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - (1000 + 50 + 640 + 50);
			static int8_t ease[] = {
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 
				0x02, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 
				0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0e, 0x0f, 0x11, 
				0x12, 0x14, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 
				0x1f,
			};
			sdd1306.SetVerticalShift(-ease[ltime/5]);
			sdd1306.SetCenterFlip(ltime/5);
			sdd1306.Display();
		} else {
			mode = 0;
			DisplayStatus();
			sdd1306.SetVerticalShift(0);
			sdd1306.SetCenterFlip(0);
			sdd1306.Display();
		}
	}
	
	void DisplayBar(uint8_t y, uint8_t val, uint8_t range) {
		if (y >= 4) {
			return;
		}
		if (range == 0) {
			const uint8_t val_to_chr[7*8] = {
				0x6B, 0x6E, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
				0x6D, 0x6E, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x6E, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x70, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x73,
			};
			for (uint32_t c=0; c<7; c++) {
				sdd1306.PlaceCustomChar(c+1,y,val_to_chr[val*7+c]);
			}
		} else {
			const uint8_t val_to_chr[7*14] = {
				0x6B, 0x6E, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
				0x6C, 0x6E, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
				0x6D, 0x6F, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x6E, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x6E, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x6F, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x6E, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x6F, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x70, 0x6E, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x70, 0x6F, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x71,
				0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x72,
				0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x73,
			};
			for (uint32_t c=0; c<7; c++) {
				sdd1306.PlaceCustomChar(c+1,y,val_to_chr[val*7+c]);
			}
		}
	}
	
	uint8_t NormBatteryChargeForBar() {
		static int32_t avg_buf[16];
		static int32_t avg_pos = 0;

		uint16_t adc5_value = 0;
		Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH5, ADC_DR_DONE_STAT) != SET) {}
		Chip_ADC_ReadValue(LPC_ADC, ADC_CH5, &adc5_value);
		
		avg_buf[(avg_pos++)&0xF] = (uint32_t(adc5_value) * 4800) / 1024;
		int32_t avg = 0;
		
		for (uint32_t c=0; c<16; c++) {
			avg += avg_buf[c];
		}
		
		int32_t charge = ((((avg / 16) - 2800) * 255 ) / (4200 - 2800));
		if (charge < 0) charge = 0;
		if (charge >= 256 ) charge = 255;
		return charge;
	}

	void DisplayStatus() {
		sdd1306.PlaceCustomChar(0,0,0xA9);
		sdd1306.PlaceCustomChar(1,0,0xAA);
		sdd1306.PlaceCustomChar(2,0,0xAB);
		sdd1306.PlaceCustomChar(3,0,0xAC);
		sdd1306.PlaceCustomChar(4,0,0xAD);
		sdd1306.PlaceCustomChar(5,0,0xAE);
		sdd1306.PlaceCustomChar(6,0,0xAF);
		sdd1306.PlaceCustomChar(7,0,0xA0+(system_clock_ms/0x400)%8);
		sdd1306.PlaceCustomChar(0,1,0x65);
		DisplayBar(1,uint8_t(settings.brightness), 0);
		sdd1306.PlaceCustomChar(0,2,0x66);
		DisplayBar(2,(NormBatteryChargeForBar() * 14 / 255), 1);
		sdd1306.PlaceCustomChar(0,3,0x67);
		char str[8];
		sprintf(str,"[%02d/%02d]",settings.program_curr,settings.program_count);
		sdd1306.PlaceAsciiStr(1,3,str);
		sdd1306.Display();
	}
	
	void Display() {
		switch (mode) {
			case	0:
					DisplayStatus();
					break;
			case	1:
					DisplayNow();
					break;
		}
	}
};

class Effects {

	EEPROM &settings;
	Random &random;
	LEDs &leds;
	SPI &spi;
	SDD1306 &sdd1306;
	UI &ui;

	uint32_t post_clock_ms;
	bool past_post_time;
	
public:
	
	Effects(EEPROM &_settings, 
			Random &_random, 
			LEDs &_leds, 
			SPI &_spi, 
			SDD1306 &_sdd1306,
			UI &_ui):

			settings(_settings),
			random(_random),
			leds(_leds),
			spi(_spi),
			sdd1306(_sdd1306),
			ui(_ui)	{
		post_clock_ms = system_clock_ms + 10;
		past_post_time = true;
	}	
	
	void RunForever() {
		while (1) {
			if (past_post_time) {
				past_post_time = false;
				switch(settings.program_curr) {
					case	0:
							color_ring();
							break;
					case	1:	
							fade_ring();
							break;
					case	2:
							rgb_walker();
							break;
					case	3:
							rgb_glow();
							break;
					case	4:
							rgb_tracer();
							break;
					case	5: 
							ring_tracer();
							break;
					case	6:
							light_tracer();
							break;
					case	7: 
							ring_bar_rotate();
							break;
					case	8: 
							ring_bar_move();
							break;
					case	9:
							sparkle();
							break;
					case	10:
							lightning();
							break;
					case	11:
							lightning_crazy();
							break;
					case 	12:
							rgb_vertical_wall();
							break;
					case 	13:
							rgb_horizontal_wall();
							break;
					case	14:
							shine_vertical();
							break;
					case	15:
							shine_horizontal();
							break;	
					case 	16:
							heartbeat();
							break;
					case 	17:
							brilliance();
							break;
					case    18:
							tingling();
							break;
					case    19:
							twinkle();
							break;
					case	20:
							simple_change_ring();
							break;
					case	21:
							simple_change_bird();
							break;
					case	22:
							simple_random();
							break;
					case	23:
							diagonal_wipe();
							break;
					case	24:
							shimmer_outside();
							break;
					case	25:
							shimmer_inside();
							break;
					case	26:
							red();
							break;
					default:
							color_ring();
							break;
				}
			}
            Chip_WWDT_Feed(LPC_WWDT);
			__WFI();
		}
	}

	void CheckPostTime() {
		if (system_clock_ms > post_clock_ms) {
			past_post_time = true;
			post_clock_ms = system_clock_ms + 250; // at minimum update every 0.25s
		}
	}
	
private:

	bool break_effect() {
		static uint32_t old_program_curr = 0;
		if (settings.program_curr != old_program_curr) {
			old_program_curr = settings.program_curr;
			return true;
		}
		return false;
	}
	
	bool post_frame(uint32_t ms) {
		post_clock_ms = system_clock_ms + ms;

		spi.push_frame(leds, settings.brightness);

		if (sdd1306.DevicePresent()) {
			ui.Display();
		}

		for ( ; ; ) {
			if (past_post_time) {
				past_post_time = false;
				return break_effect();
			}
			if (break_effect()) {
				return true;
			}
            Chip_WWDT_Feed(LPC_WWDT);
			__WFI();
		}
		return false;
	}
	
	void color_ring() {
		for (; ;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(5)) {
				return;
			}
		}
	}

	void fade_ring() {
		for (; ;) {
			rgba color;
			const rgba &rc = settings.ring_color;
			color = rgba(max(rc.r()-0x20UL,0UL), max(rc.g()-0x20UL,0UL), max(rc.b()-0x20UL,0UL));
			leds.set_ring(0, color);
			color = rgba(max(rc.r()-0x1AUL,0UL), max(rc.g()-0x1AUL,0UL), max(rc.b()-0x1AUL,0UL));
			leds.set_ring(1, color);
			leds.set_ring(7, color);
			color = rgba(max(rc.r()-0x18UL,0UL), max(rc.g()-0x18UL,0UL), max(rc.b()-0x18UL,0UL));
			leds.set_ring(2, color);
			leds.set_ring(6, color);
			color = rgba(max(rc.r()-0x10UL,0UL), max(rc.g()-0x10UL,0UL), max(rc.b()-0x10UL,0UL));
			leds.set_ring(3, color);
			leds.set_ring(5, color);
			color = rgba(max(rc.r()-0x00UL,0UL), max(rc.g()-0x00UL,0UL), max(rc.b()-0x00UL,0UL));
			leds.set_ring(4, color);

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(5)) {
				return;
			}
		}
	}

	void rgb_walker() {

		static uint8_t work_buffer[0x80] = { 0 };

		for (uint32_t c = 0; c < 0x80; c++) {
			work_buffer[c] = max(0UL,(sine_wave[c] - 0x80UL) - 0x20UL) ;
		}

		uint32_t walk = 0;
		uint32_t rgb_walk = 0;
		for (;;) {

			rgba color = rgba::hsvToRgb(rgb_walk/3, 255, 255);
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, (work_buffer[(((0x80/8)*d) + walk)&0x7F] * ((color.r())&0xFF)) >> 8,
						 	     (work_buffer[(((0x80/8)*d) + walk)&0x7F] * ((color.g())&0xFF)) >> 8,
						 		 (work_buffer[(((0x80/8)*d) + walk)&0x7F] * ((color.b())&0xFF)) >> 8);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			walk ++;
			walk &= 0x7F;

			rgb_walk ++;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			if (post_frame(5)) {
				return;
			}
		}
	}

	void rgb_glow() {
		uint32_t rgb_walk = 0;
		for (;;) {

			rgba color = rgba::hsvToRgb(rgb_walk, 255, 255);
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring_synced(d, (color / 4UL));
			}
			
			rgb_walk ++;
			if (rgb_walk >= 360) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void rgb_tracer() {
		uint32_t rgb_walk = 0;
		uint32_t walk = 0;
		int32_t switch_dir = 1;
		uint32_t switch_counter = 0;
		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d,0,0,0);
			}

			rgba color = rgba::hsvToRgb(rgb_walk/3, 255, 255);
			leds.set_ring_synced(walk&0x7, (color / 4UL));

			walk += switch_dir;

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			switch_counter ++;
			if (switch_counter > 64 && random.get(0,2)) {
				switch_dir *= -1;
				switch_counter = 0;
				walk += switch_dir;
				walk += switch_dir;
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void light_tracer() {
		uint32_t walk = 0;

		rgba gradient[8];
		const rgba &rc = settings.ring_color;
		gradient[7] = rgba(max(rc.ru()-0x40UL,0UL), max(rc.gu()-0x40UL,0UL), max(rc.bu()-0x40UL,0UL));
		gradient[6] = rgba(max(rc.ru()-0x40UL,0UL), max(rc.gu()-0x40UL,0UL), max(rc.bu()-0x40UL,0UL));
		gradient[5] = rgba(max(rc.ru()-0x30UL,0UL), max(rc.gu()-0x30UL,0UL), max(rc.bu()-0x30UL,0UL));
		gradient[4] = rgba(max(rc.ru()-0x18UL,0UL), max(rc.gu()-0x18UL,0UL), max(rc.bu()-0x18UL,0UL));
		gradient[3] = rgba(max(rc.ru()-0x00UL,0UL), max(rc.gu()-0x00UL,0UL), max(rc.bu()-0x00UL,0UL));
		gradient[2] = rgba(max(rc.ru(),0x10UL), max(rc.gu(),0x00UL), max(rc.bu(),0x20UL));
		gradient[1] = rgba(max(rc.ru(),0x30UL), max(rc.gu(),0x30UL), max(rc.bu(),0x30UL));
		gradient[0] = rgba(max(rc.ru(),0x40UL), max(rc.gu(),0x40UL), max(rc.bu(),0x40UL));

		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring((walk+d)&0x7, gradient[d]);
			}

			walk--;

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(100)) {
				return;
			}
		}
	}


	void ring_tracer() {
		uint32_t walk = 0;
		int32_t switch_dir = 1;
		uint32_t switch_counter = 0;
		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d,0,0,0);
			}

			leds.set_ring_synced((walk+0)&0x7, settings.ring_color);
			leds.set_ring_synced((walk+1)&0x7, settings.ring_color);
			leds.set_ring_synced((walk+2)&0x7, settings.ring_color);

			walk += switch_dir;

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			switch_counter ++;
			if (switch_counter > 64 && random.get(0,2)) {
				switch_dir *= -1;
				switch_counter = 0;
				walk += switch_dir;
				walk += switch_dir;
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void ring_bar_rotate() {
		uint32_t rgb_walk = 0;
		uint32_t walk = 0;
		int32_t switch_dir = 1;
		uint32_t switch_counter = 0;
		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d,0,0,0);
			}

			leds.set_ring_synced((walk+0)&0x7, settings.ring_color);
			leds.set_ring_synced((walk+4)&0x7, settings.ring_color);

			walk += switch_dir;

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			switch_counter ++;
			if (switch_counter > 64 && random.get(0,2)) {
				switch_dir *= -1;
				switch_counter = 0;
				walk += switch_dir;
				walk += switch_dir;
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void ring_bar_move() {
		uint32_t rgb_walk = 0;
		uint32_t walk = 0;
		int32_t switch_dir = 1;
		uint32_t switch_counter = 0;

		static const int8_t indecies0[] = {
			-1,
			-1,
			-1,
			-1,
			-1,
			-1,
			0,
			1,
			2,
			3,
			4
			-1,
			-1,
			-1,
			-1,
			-1,
		};

		static const int8_t indecies1[] = {
			-1,
			-1,
			-1,
			-1,
			-1,
			-1,
			0,
			7,
			6,
			5,
			4,
			-1,
			-1,
			-1,
			-1,
			-1,
		};

		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d,0,0,0);
			}

			if (indecies0[(walk)%15] >=0 ) {
				leds.set_ring_synced(indecies0[(walk)%15], settings.ring_color);	
			}
			if (indecies1[(walk)%15] >=0 ) {
				leds.set_ring_synced(indecies1[(walk)%15], settings.ring_color);
			}

			walk += switch_dir;

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			switch_counter ++;
			if (switch_counter > 64 && random.get(0,2)) {
				switch_dir *= -1;
				switch_counter = 0;
				walk += switch_dir;
				walk += switch_dir;
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void rgb_vertical_wall() {
		uint32_t rgb_walk = 0;
		for (;;) {

			rgba color;
			color = rgba::hsvToRgb(((rgb_walk+  0)/3)%360, 255, 255);
			leds.set_ring_synced(0, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+ 30)/3)%360, 255, 255);
			leds.set_ring_synced(1, (color / 4UL));
			leds.set_ring_synced(7, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+120)/3)%360, 255, 255);
			leds.set_ring_synced(2, (color / 4UL));
			leds.set_ring_synced(6, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+210)/3)%360, 255, 255);
			leds.set_ring_synced(3, (color / 4UL));
			leds.set_ring_synced(5, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+230)/3)%360, 255, 255);
			leds.set_ring_synced(4, (color / 4UL));

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(40)) {
				return;
			}
		}
	}

	void shine_vertical() {
		uint32_t rgb_walk = 0;
		rgba gradient[256];
		for (uint32_t c = 0; c < 128; c++) {
			uint32_t r = max(settings.ring_color.ru(),c/2);
			uint32_t g = max(settings.ring_color.gu(),c/2);
			uint32_t b = max(settings.ring_color.bu(),c/2);
			gradient[c] = rgba(r, g, b);
		}
		for (uint32_t c = 0; c < 128; c++) {
			uint32_t r = max(settings.ring_color.ru(),(128-c)/2);
			uint32_t g = max(settings.ring_color.gu(),(128-c)/2);
			uint32_t b = max(settings.ring_color.bu(),(128-c)/2);
			gradient[c+128] = rgba(r, g, b);
		}

		for (;;) {
			rgba color;
			color = gradient[((rgb_walk+ 0))%256];
			leds.set_ring_synced(0, color);
			color = gradient[((rgb_walk+10))%256];
			leds.set_ring_synced(1, color);
			leds.set_ring_synced(7, color);
			color = gradient[((rgb_walk+40))%256];
			leds.set_ring_synced(2, color);
			leds.set_ring_synced(6, color);
			color = gradient[((rgb_walk+70))%256];
			leds.set_ring_synced(3, color);
			leds.set_ring_synced(5, color);
			color = gradient[((rgb_walk+80))%256];
			leds.set_ring_synced(4, color);

			rgb_walk += 7;
			if (rgb_walk >= 256) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(80)) {
				return;
			}
		}
	}

	void shine_horizontal() {
		int32_t rgb_walk = 0;
		int32_t switch_dir = 1;

		rgba gradient[256];
		for (uint32_t c = 0; c < 128; c++) {
			uint32_t r = max(settings.ring_color.ru(),c/2);
			uint32_t g = max(settings.ring_color.gu(),c/2);
			uint32_t b = max(settings.ring_color.bu(),c/2);
			gradient[c] = rgba(r, g, b);
		}
		for (uint32_t c = 0; c < 128; c++) {
			uint32_t r = max(settings.ring_color.ru(),(128-c)/2);
			uint32_t g = max(settings.ring_color.gu(),(128-c)/2);
			uint32_t b = max(settings.ring_color.bu(),(128-c)/2);
			gradient[c+128] = rgba(r, g, b);
		}

		for (;;) {
			rgba color;
			color = gradient[((rgb_walk+ 0))%256];
			leds.set_ring_synced(6, color);
			color = gradient[((rgb_walk+10))%256];
			leds.set_ring_synced(7, color);
			leds.set_ring_synced(5, color);
			color = gradient[((rgb_walk+40))%256];
			leds.set_ring_synced(0, color);
			leds.set_ring_synced(4, color);
			color = gradient[((rgb_walk+70))%256];
			leds.set_ring_synced(1, color);
			leds.set_ring_synced(3, color);
			color = gradient[((rgb_walk+80))%256];
			leds.set_ring_synced(2, color);

			rgb_walk += 7*switch_dir;
			if (rgb_walk >= 256) {
				rgb_walk = 255;
				switch_dir *= -1;
			}
			if (rgb_walk < 0) {
				rgb_walk = 0;
				switch_dir *= -1;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(80)) {
				return;
			}
		}
	}

	void rgb_horizontal_wall() {
		uint32_t rgb_walk = 0;
		for (;;) {

			rgba color;
			color = rgba::hsvToRgb(((rgb_walk+  0)/3)%360, 255, 255);
			leds.set_ring_synced(6, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+ 30)/3)%360, 255, 255);
			leds.set_ring_synced(7, (color / 4UL));
			leds.set_ring_synced(5, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+120)/3)%360, 255, 255);
			leds.set_ring_synced(0, (color / 4UL));
			leds.set_ring_synced(4, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+210)/3)%360, 255, 255);
			leds.set_ring_synced(1, (color / 4UL));
			leds.set_ring_synced(3, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+230)/3)%360, 255, 255);
			leds.set_ring_synced(2, (color / 4UL));

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(40)) {
				return;
			}
		}
	}

	void lightning() {
		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, 0,0,0);
			}

			int index = random.get(0,128);
			leds.set_ring_all(index,0x40,0x40,0x40);

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(10)) {
				return;
			}
		}
	}

	void sparkle() {
		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, 0,0,0);
			}

			int index = random.get(0,16);
			leds.set_ring_all(index,random.get(0x00,0x10),random.get(0x00,0x10),random.get(0,0x10));

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void lightning_crazy() {
		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, 0,0,0);
			}

			int index = random.get(0,16);
			leds.set_ring_all(index,0x40,0x40,0x40);

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(10)) {
				return;
			}
		}
	}

	void heartbeat() {
		int32_t rgb_walk = 0;
		int32_t switch_dir = 1;
		for (; ;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, ((settings.bird_color.r())*rgb_walk)/256,
						 		 ((settings.bird_color.g())*rgb_walk)/256,
						 		 ((settings.bird_color.b())*rgb_walk)/256);
			}

			rgb_walk += switch_dir;
			if (rgb_walk >= 256) {
				rgb_walk = 255;
				switch_dir *= -1;
			}
			if (rgb_walk < 0) {
				rgb_walk = 0;
				switch_dir *= -1;
			}

			if (post_frame(8)) {
				return;
			}
		}
	}

	void brilliance() {
		int32_t current_wait = 0;
		int32_t wait_time = 0;
		int32_t rgb_walk = 0;
		int32_t switch_dir = 1;
		for (; ;) {
			rgba gradient[256];
			for (int32_t c = 0; c < 112; c++) {
				uint32_t r = settings.bird_color.r();
				uint32_t g = settings.bird_color.g();
				uint32_t b = settings.bird_color.b();
				gradient[c] = rgba(r, g, b);
			}
			for (uint32_t c = 0; c < 16; c++) {
				uint32_t r = max(settings.bird_color.ru(),c*8);
				uint32_t g = max(settings.bird_color.gu(),c*8);
				uint32_t b = max(settings.bird_color.bu(),c*8);
				gradient[c+112] = rgba(r, g, b);
			}
			for (uint32_t c = 0; c < 16; c++) {
				uint32_t r = max(settings.bird_color.ru(),(16-c)*8);
				uint32_t g = max(settings.bird_color.gu(),(16-c)*8);
				uint32_t b = max(settings.bird_color.bu(),(16-c)*8);
				gradient[c+128] = rgba(r, g, b);
			}
			for (int32_t c = 0; c < 112; c++) {
				uint32_t r = settings.bird_color.r();
				uint32_t g = settings.bird_color.g();
				uint32_t b = settings.bird_color.b();
				gradient[c+144] = rgba(r, g, b);
			}


			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				rgba color = gradient[((rgb_walk+ 0))%256];
				leds.set_bird(d, color);
			}

			rgb_walk += switch_dir;
			if (rgb_walk >= 256) {
				current_wait++;
				if (current_wait > wait_time) {
					wait_time = random.get(0,2000);
					rgb_walk = 0;
					current_wait = 0;
				} else {
					rgb_walk = 255;
				}
			}

			if (post_frame(10)) {
				return;
			}
		}
	}

	void tingling() {
		#define NUM_TINGLES 16
		struct tingle {
			bool active;
			int32_t wait;
			int32_t index;
			uint32_t progress;
			bool lightordark;
		} tingles[NUM_TINGLES];

		memset(tingles, 0, sizeof(tingles));

		for (; ;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			for (int32_t c = 0 ; c < NUM_TINGLES; c++) {
				if (tingles[c].active == 0) {
					tingles[c].wait = random.get(0,25);
					for (;;) {
						bool done = true;
						tingles[c].index = random.get(0,16);
						for (int32_t d = 0 ; d < NUM_TINGLES; d++) {
							if( d != c && 
								tingles[c].active && 
								tingles[c].index == tingles[d].index) {
								done = false;
								break;
							}
						}
						if (done) {
							break;
						}
					}
					tingles[c].index = random.get(0,16);
					tingles[c].progress = 0;
					tingles[c].lightordark = random.get(0,2);
					tingles[c].active = 1;
				} else if (tingles[c].progress >= 16) {
					tingles[c].active = 0;
				} else if (tingles[c].wait > 0) {
					tingles[c].wait --;
				} else {
					uint32_t r = 0,g = 0,b = 0;
					uint32_t progress = tingles[c].progress;
					if (progress > 8) {
						progress -= 8;
						progress = 8 - progress;
					}
					if (tingles[c].lightordark) {
						r = max(settings.ring_color.ru(),progress*8);
						g = max(settings.ring_color.gu(),progress*8);
						b = max(settings.ring_color.bu(),progress*8);					
					} else {
						r = (settings.ring_color.r())-progress*8;
						g = (settings.ring_color.g())-progress*8;
						b = (settings.ring_color.b())-progress*8;
						r = max(r,0UL);
						g = max(g,0UL);
						b = max(b,0UL);					
					}
					leds.set_ring_all(tingles[c].index, r, g, b);
					tingles[c].progress++;
				}
			}

			if (post_frame(20)) {
				return;
			}
		}
	}


	void twinkle() {
		#define NUM_TWINKLE 3

		struct tingle {
			bool active;
			int32_t wait;
			int32_t index;
			uint32_t progress;
		} tingles[NUM_TWINKLE];

		memset(tingles, 0, sizeof(tingles));

		for (; ;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			for (int32_t c = 0 ; c < NUM_TWINKLE; c++) {
				if (tingles[c].active == 0) {
					tingles[c].wait = random.get(0,50);
					for (;;) {
						bool done = true;
						tingles[c].index = random.get(0,16);
						for (int32_t d = 0 ; d < NUM_TWINKLE; d++) {
							if( d != c && 
								tingles[c].active && 
								tingles[c].index == tingles[d].index) {
								done = false;
								break;
							}
						}
						if (done) {
							break;
						}
					}
					tingles[c].index = random.get(0,16);
					tingles[c].progress = 0;
					tingles[c].active = 1;
				} else if (tingles[c].progress >= 16) {
					tingles[c].active = 0;
				} else if (tingles[c].wait > 0) {
					tingles[c].wait --;
				} else {
					uint32_t r = 0,g = 0,b = 0;
					uint32_t progress = tingles[c].progress;
					if (progress > 8) {
						progress -= 8;
						progress = 8 - progress;
					}

					r = max(settings.ring_color.ru(),progress*16);
					g = max(settings.ring_color.gu(),progress*16);
					b = max(settings.ring_color.bu(),progress*16);					
					leds.set_ring_all(tingles[c].index, r,  g, b);
					tingles[c].progress++;
				}
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void simple_change_ring() {
		int32_t index = 0;

		int32_t r = random.get(0x00,0x40);
		int32_t g = random.get(0x00,0x40);
		int32_t b = random.get(0x00,0x40);
		int32_t cr = 0;
		int32_t cg = 0;
		int32_t cb = 0;
		int32_t nr = 0;
		int32_t ng = 0;
		int32_t nb = 0;

		for (; ;) {

			if (index >= 600) {
				if (index == 600) {
					cr = r;
					cg = g;
					cb = b;
					nr = random.get(0x00,0x40);
					ng = random.get(0x00,0x40);
					nb = random.get(0x00,0x40);
				}
				if (index >= 664) {
					index = 0;
				} else {
					int32_t lft = index-600;
					int32_t rgt = 64-lft;
					r = (nr*lft + cr*rgt) / 64;
					g = (ng*lft + cg*rgt) / 64;
					b = (nb*lft + cb*rgt) / 64;
					index++;
				}
			} else {
				index++;
			}

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, r, g, b);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(15)) {
				return;
			}
		}
	}

	void simple_change_bird() {
		int32_t index = 0;

		int32_t r = random.get(0x00,0x40);
		int32_t g = random.get(0x00,0x40);
		int32_t b = random.get(0x00,0x40);
		int32_t cr = 0;
		int32_t cg = 0;
		int32_t cb = 0;
		int32_t nr = 0;
		int32_t ng = 0;
		int32_t nb = 0;

		for (; ;) {

			if (index >= 600) {
				if (index == 600) {
					cr = r;
					cg = g;
					cb = b;
					nr = random.get(0x00,0x40);
					ng = random.get(0x00,0x40);
					nb = random.get(0x00,0x40);
				}
				if (index >= 664) {
					index = 0;
				} else {
					int32_t lft = index-600;
					int32_t rgt = 64-lft;
					r = (nr*lft + cr*rgt) / 64;
					g = (ng*lft + cg*rgt) / 64;
					b = (nb*lft + cb*rgt) / 64;
					index++;
				}
			} else {
				index++;
			}

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, r,
						 		 g,
						 		 b);
			}

			if (post_frame(15)) {
				return;
			}
		}
	}

	void simple_random() {

		rgba colors[16];
		for (int32_t c = 0; c<16; c++) {
			colors[c] = rgba(random.get(0x00,0x40),random.get(0x00,0x40),random.get(0x00,0x40));
		}

		for (; ;) {

			uint32_t index = random.get(0x00,0x10);
			colors[index] = rgba(random.get(0x00,0x40),random.get(0x00,0x40),random.get(0x00,0x40));

			for (uint32_t d = 0; d < 16; d++) {
				leds.set_ring_all(d, colors[d]);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(20)) {
				return;
			}
		}
	}

	void diagonal_wipe() {

		int32_t walk = 0;
		int32_t wait = random.get(60,1500);
		int32_t dir = random.get(0,2);

		for (; ;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			int32_t i0 = -1;
			int32_t i1 = -1;

			if (dir) {
				if (walk < 10) {
					i0 = 7;
					i1 = 7;
				} else if (walk < 20) {
					i0 = 0;
					i1 = 6;
				} else if (walk < 30) {
					i0 = 1;
					i1 = 5;
				} else if (walk < 40) {
					i0 = 2;
					i1 = 4;
				} else if (walk < 50) {
					i0 = 3;
					i1 = 3;
				} else {
					i0 = -1;
					i1 = -1;
				}	
			} else {
				if (walk < 10) {
					i0 = 1;
					i1 = 1;
				} else if (walk < 20) {
					i0 = 0;
					i1 = 2;
				} else if (walk < 30) {
					i0 = 7;
					i1 = 3;
				} else if (walk < 40) {
					i0 = 6;
					i1 = 4;
				} else if (walk < 50) {
					i0 = 5;
					i1 = 5;
				} else {
					i0 = -1;
					i1 = -1;
				}	
			}
			
			walk ++;
			if (walk > wait) {
				walk = 0;
				wait = random.get(60,1024);
				dir = random.get(0,2);
			}

			if (i0 >= 0) leds.set_ring_synced(i0, 0x40,0x40,0x40);
			if (i1 >= 0) leds.set_ring_synced(i1, 0x40,0x40,0x40);

			if (post_frame(5)) {
				return;
			}
		}
	}

	void shimmer_outside() {
		int32_t walk = 0;
		int32_t wait = random.get(16,64);

		for (; ;) {

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			int32_t r = settings.ring_color.r();
			int32_t g = settings.ring_color.g();
			int32_t b = settings.ring_color.b();
			if (walk < 8) {
				r = max(int32_t(0),r - walk);
				g = max(int32_t(0),g - walk);
				b = max(int32_t(0),b - walk);
			} else if (walk < 16) {
				r = max(int32_t(0),r - (8-(walk-8)));
				g = max(int32_t(0),g - (8-(walk-8)));
				b = max(int32_t(0),b - (8-(walk-8)));
			}

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, r,
						 		 g,
						 		 b);
			}
			
			walk ++;
			if (walk > wait) {
				walk = 0;
				wait = random.get(16,64);

			}

			if (post_frame(2)) {
				return;
			}
		}
	}

	void shimmer_inside() {
		int32_t walk = 0;
		int32_t wait = random.get(16,64);

		for (; ;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			int32_t r = settings.ring_color.r();
			int32_t g = settings.ring_color.g();
			int32_t b = settings.ring_color.b();
			if (walk < 8) {
				r = max(int32_t(0),r - walk);
				g = max(int32_t(0),g - walk);
				b = max(int32_t(0),b - walk);
			} else if (walk < 16) {
				r = max(int32_t(0),r - (8-(walk-8)));
				g = max(int32_t(0),g - (8-(walk-8)));
				b = max(int32_t(0),b - (8-(walk-8)));
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, r,
								 g,
								 b);
			}
			
			walk ++;
			if (walk > wait) {
				walk = 0;
				wait = random.get(16,64);

			}

			if (post_frame(10)) {
				return;
			}
		}
	}

	void red() {

		int32_t wait = 1200;

		int32_t index = 0;

		int32_t br = settings.bird_color.r();
		int32_t bg = settings.bird_color.g();
		int32_t bb = settings.bird_color.b();

		int32_t rr = settings.ring_color.r();
		int32_t rg = settings.ring_color.g();
		int32_t rb = settings.ring_color.b();

		int32_t b1r = br;
		int32_t b1g = bg;
		int32_t b1b = bb;

		int32_t r1r = rr;
		int32_t r1g = rg;
		int32_t r1b = rb;

		for (; ;) {

			if (index >= 0) {
				if (index >= wait) {
					wait = random.get(1200,10000);
					index = 0;
				} else if (index >= 0 && index < 64) {
					int32_t rgt = index-0;
					int32_t lft = 64-rgt;
					b1r = (br*lft + 0x40*rgt) / 64;
					b1g = (bg*lft + 0x00*rgt) / 64;
					b1b = (bb*lft + 0x10*rgt) / 64;
					r1r = (rr*lft + 0x40*rgt) / 64;
					r1g = (rg*lft + 0x00*rgt) / 64;
					r1b = (rb*lft + 0x10*rgt) / 64;
					index++;
				} else if (index >= 600 && index < 664) {
					int32_t lft = index-600;
					int32_t rgt = 64-lft;
					b1r = (br*lft + 0x40*rgt) / 64;
					b1g = (bg*lft + 0x00*rgt) / 64;
					b1b = (bb*lft + 0x10*rgt) / 64;
					r1r = (rr*lft + 0x40*rgt) / 64;
					r1g = (rg*lft + 0x00*rgt) / 64;
					r1b = (rb*lft + 0x10*rgt) / 64;
					index++;
				} else {
					index++;
				}
			} else {
				index++;
			}

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, r1r, r1g, r1b);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, b1r, b1g, b1b);
			}

			if (post_frame(20)) {
				return;
			}
		}
	}
};  // class Effects

}  // namespace {

static LEDs *g_leds = 0;
static SPI *g_spi = 0;
static UI *g_ui = 0;
static Effects *g_effects = 0;
static SX1280 *g_sx1280 = 0;
static SDD1306 *g_sdd1306 = 0;

extern "C" {
	
	void SysTick_Handler(void)
	{	
		system_clock_ms++;
		
		if ( (system_clock_ms % (1024*32)) == 0) {
			g_ui->SetMode(system_clock_ms, 1);
		}
		
		if ( (system_clock_ms % (256)) == 0) {
			g_sx1280->ProcessIrqs();
		}
		
		if ( (system_clock_ms % (4096)) == 0) {
			g_sx1280->SendBeacon();
		}
		
		g_effects->CheckPostTime();
	}

	void TIMER32_0_IRQHandler(void)
	{
	}

	void UART_IRQHandler(void)
	{
	}
	
	void FLEX_INT0_IRQHandler(void)
	{
		g_sx1280->OnDioIrq();
		Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
	}

	void FLEX_INT1_IRQHandler(void)
	{	
		if (g_ui) {
			g_ui->HandleINT1IRQ();
		}
	}

	void FLEX_INT2_IRQHandler(void)
	{
		if (g_ui) {
			g_ui->HandleINT2IRQ();
		}
	}
	
	void USB_IRQHandler(void)
	{
		uint32_t *addr = (uint32_t *) LPC_USB->EPLISTSTART;

		/*	WORKAROUND for artf32289 ROM driver BUG:
			As part of USB specification the device should respond
			with STALL condition for any unsupported setup packet. The host will send
			new setup packet/request on seeing STALL condition for EP0 instead of sending
			a clear STALL request. Current driver in ROM doesn't clear the STALL
			condition on new setup packet which should be fixed.
		 */
		if ( LPC_USB->DEVCMDSTAT & _BIT(8) ) {	/* if setup packet is received */
			addr[0] &= ~(_BIT(29));	/* clear EP0_OUT stall */
			addr[2] &= ~(_BIT(29));	/* clear EP0_IN stall */
		}
		USBD_API->hw->ISR(g_hUsb);
	}
}

int main(void)
{
	SystemCoreClockUpdate();

	SDD1306 sdd1306;
	g_sdd1306 = &sdd1306;
	BQ24295 bq24295;

	Setup setup(sdd1306, bq24295);

	if (bq24295.DevicePresent()) {
		bq24295.SetBoostUpperTemperatureLimit(80);
		bq24295.SetInputCurrentLimit(900);
		bq24295.EnableInputLimits();
		bq24295.SetChipThermalRegulationThreshold(80);
		bq24295.SetBoostVoltage(5126);
	}
	
	delay(50);
	
	EEPROM settings;
	
	SPI spi;
	g_spi = &spi;

	LEDs leds;
	g_leds = &leds;
	
	spi.push_frame(leds, 0);

	if (sdd1306.DevicePresent()) {
		sdd1306.Init(); 
		sdd1306.Clear();
		sdd1306.DisplayBootScreen();
		static uint8_t bounce[] = {
			0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1e, 0x1d, 0x1d, 
			0x1c, 0x1b, 0x1a, 0x18, 0x17, 0x16, 0x14, 0x12, 
			0x10, 0x0e, 0x0c, 0x0a, 0x08, 0x05, 0x03, 0x01, 
			0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x07, 0x08, 
			0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x07, 
			0x06, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 
			0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 
		};
		sdd1306.SetVerticalShift(bounce[0]);
		sdd1306.Display();
		for (uint32_t y=0; y<64; y++) {
			sdd1306.SetVerticalShift(bounce[y]);
			delay(10);
		}
		delay(500);
		static int8_t ease[] = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 
			0x02, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 
			0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0e, 0x0f, 0x11, 
			0x12, 0x14, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 
			0x1f,
		};
		for (uint32_t x=0; x<=32; x++) {
			sdd1306.SetVerticalShift(-ease[x]);
			sdd1306.SetCenterFlip(x);
			sdd1306.Display();
			delay(1);
		}
		sdd1306.SetVerticalShift(0);
		sdd1306.SetCenterFlip(0);
	}

	settings.Load();
	
	UI ui(settings, sdd1306);
	// For IRQ handlers only
	g_ui = &ui;
	ui.Init();
	
	Random random(0xCAFFE);

#ifndef NO_FLASH
//	flash_storage.init();
#endif  // #ifndef NO_FLASH

	SX1280 sx1280;
	
	g_sx1280 = &sx1280;
	
	sx1280.Init(sdd1306, false);
	
	// start 1ms timer
	SysTick_Config(SystemCoreClock / 1000);

	Effects effects(settings, random, leds, spi, sdd1306, ui);
	
	g_effects = &effects;

	effects.RunForever();
	
	return 0;
}
