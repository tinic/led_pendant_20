#include <stdint.h>

#include "chip.h"
#include "gpio_11xx_1.h"
#include "gpiogroup_11xx.h"
#include "ssp_11xx.h"

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000
#define I2C_DEFAULT_SPEED    SPEED_400KHZ
#define I2C_FASTPLUS_BIT     0

#if (I2C_DEFAULT_SPEED > SPEED_400KHZ)
#undef  I2C_FASTPLUS_BIT
#define I2C_FASTPLUS_BIT IOCON_FASTI2C_EN
#endif

#ifdef NO_SX1280
#include "printf.h"
#endif  // #ifdef NO_SX1280


namespace {

template<class T> const T& max(const T &a, const T &b) { return (a>b) ? a : b; }
template<class T> const T& min(const T &a, const T &b) { return (a<b) ? a : b; }
template<class T> const T& abs(const T &a) { return (a<0) ? -a : a; }
template<class T> const T& constrain(const T& x, const T &a, const T &b) { return (x>b)?b:((x<a)?a:x); }

static volatile uint32_t system_clock_ms = 0;

typedef struct rgb_color {
	uint8_t red, green, blue;
	rgb_color() {};
	rgb_color(uint8_t r, uint8_t g, uint8_t b) : red(r), green(g), blue(b) {};
} rgb_color;

static rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v) {
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
	return rgb_color(r, g, b);
}

static const uint8_t gamma_curve[256] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 
	0x02, 0x02, 0x03, 0x03, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x05, 0x05, 0x06, 
	0x06, 0x06, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08, 0x09, 0x09, 0x09, 0x0a, 0x0a, 0x0a, 0x0b, 0x0b, 
	0x0c, 0x0c, 0x0d, 0x0d, 0x0d, 0x0e, 0x0e, 0x0f, 0x0f, 0x10, 0x10, 0x11, 0x11, 0x12, 0x12, 0x13, 
	0x13, 0x14, 0x15, 0x15, 0x16, 0x16, 0x17, 0x17, 0x18, 0x19, 0x19, 0x1a, 0x1b, 0x1b, 0x1c, 0x1d, 
	0x1d, 0x1e, 0x1f, 0x1f, 0x20, 0x21, 0x21, 0x22, 0x23, 0x24, 0x24, 0x25, 0x26, 0x27, 0x28, 0x28, 
	0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 
	0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 
	0x48, 0x49, 0x4a, 0x4b, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x54, 0x55, 0x56, 0x57, 0x58, 0x5a, 
	0x5b, 0x5c, 0x5d, 0x5f, 0x60, 0x61, 0x63, 0x64, 0x65, 0x67, 0x68, 0x69, 0x6b, 0x6c, 0x6d, 0x6f, 
	0x70, 0x72, 0x73, 0x75, 0x76, 0x77, 0x79, 0x7a, 0x7c, 0x7d, 0x7f, 0x80, 0x82, 0x83, 0x85, 0x87, 
	0x88, 0x8a, 0x8b, 0x8d, 0x8e, 0x90, 0x92, 0x93, 0x95, 0x97, 0x98, 0x9a, 0x9c, 0x9d, 0x9f, 0xa1, 
	0xa2, 0xa4, 0xa6, 0xa8, 0xa9, 0xab, 0xad, 0xaf, 0xb0, 0xb2, 0xb4, 0xb6, 0xb8, 0xba, 0xbb, 0xbd, 
	0xbf, 0xc1, 0xc3, 0xc5, 0xc7, 0xc9, 0xcb, 0xcd, 0xcf, 0xd1, 0xd3, 0xd5, 0xd7, 0xd9, 0xdb, 0xdd, 
	0xdf, 0xe1, 0xe3, 0xe5, 0xe7, 0xe9, 0xeb, 0xed, 0xef, 0xf1, 0xf4, 0xf6, 0xf8, 0xfa, 0xfc, 0xff
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
	0x404000,
	0x104020,
	0x005010,
	0x004040,
	0x083040,
	0x001050,
	0x300050,
	0x401040,
	0x501000,
	0x401818,
	0x400020,
	0x453000,
	0x452000,
	0x204000,
	0x201024,
	0x102014,
	0x201810,
	0x101820,
	0x303030,
	0x202020,
};

static const uint32_t ring_colors[] = {
	0x404000,
	0x104020,
	0x005010,
	0x004040,
	0x083040,
	0x001050,
	0x300050,
	0x401040,
	0x501000,
	0x401818,
	0x400020,
	0x453000,
	0x402000,
	0x204000,
	0x201024,
	0x102014,
	0x201810,
	0x101820,
	0x303030,
	0x202020,
};

static void delay(int32_t ms) {
	for (volatile uint32_t i = 0; i < ms*2400; i++) {}
}

class random {
public:

	random() {}

	#define rot(x,k) (((x)<<(k))|((x)>>(32-(k))))
	uint32_t get() {
		uint32_t e = a - rot(b, 27);
		a = b ^ rot(c, 17);
		b = c + d;
		c = d + e;
		d = e + a;
		return d;
	}

	void init(uint32_t seed) {
		uint32_t i;
		a = 0xf1ea5eed, b = c = d = seed;
		for (i=0; i<20; ++i) {
		    (void)get();
		}
	}

	uint32_t get(uint32_t lower, uint32_t upper) {
		return (get() % (upper-lower)) + lower;
	}

private:
	uint32_t a; 
	uint32_t b; 
	uint32_t c; 
	uint32_t d; 

} random;

static struct flash_access {

	static const uint32_t FLASH_MOSI0_PIN = 0x0006; // 0_6
	static const uint32_t FLASH_MISO0_PIN = 0x0009; // 0_9
	static const uint32_t FLASH_SCK0_PIN = 0x000A; // 0_10
	static const uint32_t FLASH_CSEL_PIN = 0x000A; // 0_10

	static const uint32_t ALT_SCK0_PIN = 0x0009; // 0_9

	flash_access() {
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
	static void push_byte(uint8_t byte) {
		while(!Chip_SSP_GetStatus(LPC_SSP0, SSP_STAT_TNF));
		Chip_SSP_SendFrame(LPC_SSP0, byte);
	}

	static uint8_t read_byte() {
		while(!Chip_SSP_GetStatus(LPC_SSP0, SSP_STAT_RNE));
		return Chip_SSP_ReceiveFrame(LPC_SSP0);
	}
} flash_access;

static struct eeprom_settings {

	eeprom_settings() {
		program_count = 0;
		program_curr = 0;
		program_change_count = 0;
	}

	void load() {
		unsigned int param[5] = { 0 };
		param[0] = 62; // Write EEPROM
		param[1] = 0;
		param[2] = (uintptr_t)this;
		param[3] = sizeof(eeprom_settings);
		param[4] = SystemCoreClock;
		unsigned int result[4] = { 0 };
		iap_entry(param, result);
	}

	void save() {
		unsigned int param[5] = { 0 };
		param[0] = 61; // Read EEPROM
		param[1] = 0;
		param[2] = (uintptr_t)this;
		param[3] = sizeof(eeprom_settings);
		param[4] = SystemCoreClock;
		unsigned int result[4] = { 0 };
		iap_entry(param, result);
	}

	uint32_t program_count;
	uint32_t program_curr;
	uint32_t program_change_count;
	uint32_t bird_color;
	uint32_t bird_color_index;
	uint32_t ring_color;
	uint32_t ring_color_index;
} eeprom_settings;

class spi;
static class leds {
public:

	static void set_ring(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		led_data[frnt_ring_indecies[index]*3+1] = r;
		led_data[frnt_ring_indecies[index]*3+0] = g;
		led_data[frnt_ring_indecies[index]*3+2] = b;
		led_data[back_ring_indecies[index]*3+1] = r;
		led_data[back_ring_indecies[index]*3+0] = g;
		led_data[back_ring_indecies[index]*3+2] = b;
	}

	static void set_ring_synced(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		led_data[frnt_ring_indecies[index]*3+1] = r;
		led_data[frnt_ring_indecies[index]*3+0] = g;
		led_data[frnt_ring_indecies[index]*3+2] = b;
		led_data[back_ring_indecies[(8-index)&7]*3+1] = r;
		led_data[back_ring_indecies[(8-index)&7]*3+0] = g;
		led_data[back_ring_indecies[(8-index)&7]*3+2] = b;
	}

	static void set_ring_all(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		if(index < 8) { 
			led_data[frnt_ring_indecies[index]*3+1] = r;
			led_data[frnt_ring_indecies[index]*3+0] = g;
			led_data[frnt_ring_indecies[index]*3+2] = b;
		} else if (index < 16) {
			led_data[back_ring_indecies[index-8]*3+1] = r;
			led_data[back_ring_indecies[index-8]*3+0] = g;
			led_data[back_ring_indecies[index-8]*3+2] = b;
		}
	}

	static void set_bird(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		led_data[frnt_bird_indecies[index]*3+1] = r;
		led_data[frnt_bird_indecies[index]*3+0] = g;
		led_data[frnt_bird_indecies[index]*3+2] = b;
		led_data[back_bird_indecies[index]*3+1] = r;
		led_data[back_bird_indecies[index]*3+0] = g;
		led_data[back_bird_indecies[index]*3+2] = b;
	}

private:
	friend class spi;

#define TOTAL_LEDS 			24
#define HALF_LEDS 			12

	static const uint8_t frnt_ring_indecies[];
	static const uint8_t back_ring_indecies[];
	static const uint8_t frnt_bird_indecies[];
	static const uint8_t back_bird_indecies[];

	static uint8_t led_data[TOTAL_LEDS*3];

} leds;

const uint8_t leds::frnt_ring_indecies[] = { 0x14, 0x15, 0x16, 0x17, 0x10, 0x11, 0x12, 0x13 };
const uint8_t leds::back_ring_indecies[] = { 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02 };
const uint8_t leds::frnt_bird_indecies[] = { 0x0D, 0x0C, 0x0E, 0x0F };
const uint8_t leds::back_bird_indecies[] = { 0x09, 0x0B, 0x0A, 0x08 };

uint8_t leds::led_data[TOTAL_LEDS*3] = { 0x00 } ;

static class spi {
public:
	static const uint32_t BOTTOM_LED_MOSI0_PIN = 0x0009; // 0_9
	static const uint32_t BOTTOM_LED_SCK0_PIN = 0x000A; // 0_10

	static const uint32_t ALT_SCK0_PIN = 0x0006; // 0_6

	static const uint32_t TOP_LED_MOSI1_PIN = 0x0116; // 1_22
	static const uint32_t TOP_LED_SCK1_PIN = 0x010F; // 1_15

	static void init() {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_MOSI0_PIN>>8), (BOTTOM_LED_MOSI0_PIN&0xFF), IOCON_FUNC1);
		// SCK0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_SCK0_PIN>>8), (BOTTOM_LED_SCK0_PIN&0xFF), IOCON_FUNC2);

		// Set to GPIO, shared with flash chip
		Chip_IOCON_PinMuxSet(LPC_IOCON, (ALT_SCK0_PIN>>8), (ALT_SCK0_PIN&0xFF), IOCON_FUNC0);
		
		// MOSI1
		Chip_IOCON_PinMuxSet(LPC_IOCON, (TOP_LED_MOSI1_PIN>>8), (TOP_LED_MOSI1_PIN&0xFF), IOCON_FUNC2);
		// SCK1
		Chip_IOCON_PinMuxSet(LPC_IOCON, (TOP_LED_SCK1_PIN>>8), (TOP_LED_SCK1_PIN&0xFF), IOCON_FUNC3);

		Chip_SSP_Init(LPC_SSP0);
	    Chip_SSP_SetMaster(LPC_SSP0, 1);
		Chip_SSP_SetClockRate(LPC_SSP0, 0, 8);
		Chip_SSP_SetFormat(LPC_SSP0, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
		Chip_SSP_Enable(LPC_SSP1);

	    Chip_SSP_SetMaster(LPC_SSP1, 1);
		Chip_SSP_Init(LPC_SSP1);
		Chip_SSP_SetClockRate(LPC_SSP1, 0, 8);
		Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
		Chip_SSP_Enable(LPC_SSP1);
	}

	static void push_frame(uint32_t brightness = 0x20)  {
	
		// SCK0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_SCK0_PIN>>8), (BOTTOM_LED_SCK0_PIN&0xFF), IOCON_FUNC2);
		// Set to GPIO, shared with flash chip
		Chip_IOCON_PinMuxSet(LPC_IOCON, (ALT_SCK0_PIN>>8), (ALT_SCK0_PIN&0xFF), IOCON_FUNC0);
		// Set format again
		Chip_SSP_SetClockRate(LPC_SSP0, 0, 8);
		Chip_SSP_SetFormat(LPC_SSP0, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);

		// Start frame
		push_byte0(0);
		push_byte0(0);
		push_byte0(0);
		push_byte0(0);

		push_byte1(0);
		push_byte1(0);
		push_byte1(0);
		push_byte1(0);

		// Frame data
		for (int32_t c=0; c<HALF_LEDS; c++) {
			push_byte0(0xE0 | brightness);
			push_byte0(leds::led_data[HALF_LEDS*0*3 + c*3+0]);
			push_byte0(leds::led_data[HALF_LEDS*0*3 + c*3+1]);
			push_byte0(leds::led_data[HALF_LEDS*0*3 + c*3+2]);

			push_byte1(0xE0 | brightness);
			push_byte1(leds::led_data[HALF_LEDS*1*3 + c*3+0]);
			push_byte1(leds::led_data[HALF_LEDS*1*3 + c*3+1]);
			push_byte1(leds::led_data[HALF_LEDS*1*3 + c*3+2]);
		}
		
		// End frame
		push_byte0(0xFF);
		push_byte0(0xFF);
		push_byte0(0xFF);
		push_byte0(0xFF);

		push_byte1(0xFF);
		push_byte1(0xFF);
		push_byte1(0xFF);
		push_byte1(0xFF);
	}

private:
	
	static void push_byte0(uint8_t byte) {
		while(!Chip_SSP_GetStatus(LPC_SSP0, SSP_STAT_TNF));
		Chip_SSP_SendFrame(LPC_SSP0, byte);
	}
	
	static void push_byte1(uint8_t byte) {
		while(!Chip_SSP_GetStatus(LPC_SSP1, SSP_STAT_TNF));
		Chip_SSP_SendFrame(LPC_SSP1, byte);
	}

} spi;

static void advance_mode(uint32_t mode) {
	switch(mode) {
		case	0:
				eeprom_settings.bird_color_index++; 
				eeprom_settings.bird_color_index %= 20; 
				eeprom_settings.bird_color = bird_colors[eeprom_settings.bird_color_index];
				break;
		case	1:
				eeprom_settings.ring_color_index++; 
				eeprom_settings.ring_color_index %= 20; 
				eeprom_settings.ring_color = ring_colors[eeprom_settings.ring_color_index];
				break;
	}
}

static void config_mode() {
}

static bool test_button() {
	static uint32_t last_config_time = 0;
	// Don't take into account this button press if we just
	// came out of configuration
	if ((system_clock_ms - last_config_time) < 1000) {
		return false;
	}
	if (Chip_GPIO_ReadPortBit(LPC_GPIO, 0, 2)) {
		uint32_t d_time = system_clock_ms;
		delay(100);
		for (;Chip_GPIO_ReadPortBit(LPC_GPIO, 0, 2);) {
			uint32_t u_time = system_clock_ms;
			// long press > 2 seconds gets us into config mode
			if ((u_time - d_time) > 2000) {
				config_mode();
				last_config_time = system_clock_ms;
				return false;
			}
		}
		// advance program if we did not end up in config mode
		eeprom_settings.program_curr++;
		eeprom_settings.program_change_count++;
		if (eeprom_settings.program_curr >= eeprom_settings.program_count) {
			eeprom_settings.program_curr = 0;
		}
#ifdef NO_SX1280
		printf("Setting program #%d\n", eeprom_settings.program_curr);
#endif  // #ifdef NO_SX1280
		eeprom_settings.save();
		return true;
	}
	return false;
}

static void color_ring() {
	for (; ;) {
		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, gamma_curve[((eeprom_settings.ring_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 0)&0xFF)]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[((eeprom_settings.bird_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 0)&0xFF)]);
		}

		delay(5);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void fade_ring() {
	for (; ;) {
		rgb_color color;
		int32_t col = eeprom_settings.ring_color;
		color = rgb_color(max(((col>>16)&0xFF)-0x20UL,0UL), max(((col>> 8)&0xFF)-0x20UL,0UL), max(((col>> 0)&0xFF)-0x20UL,0UL));
		leds::set_ring(0, gamma_curve[color.red],  gamma_curve[color.green], gamma_curve[color.blue]);
		color = rgb_color(max(((col>>16)&0xFF)-0x1AUL,0UL), max(((col>> 8)&0xFF)-0x1AUL,0UL), max(((col>> 0)&0xFF)-0x1AUL,0UL));
		leds::set_ring(1, gamma_curve[color.red],  gamma_curve[color.green], gamma_curve[color.blue]);
		leds::set_ring(7, gamma_curve[color.red],  gamma_curve[color.green], gamma_curve[color.blue]);
		color = rgb_color(max(((col>>16)&0xFF)-0x18UL,0UL), max(((col>> 8)&0xFF)-0x18UL,0UL), max(((col>> 0)&0xFF)-0x18UL,0UL));
		leds::set_ring(2, gamma_curve[color.red],  gamma_curve[color.green], gamma_curve[color.blue]);
		leds::set_ring(6, gamma_curve[color.red],  gamma_curve[color.green], gamma_curve[color.blue]);
		color = rgb_color(max(((col>>16)&0xFF)-0x10UL,0UL), max(((col>> 8)&0xFF)-0x10UL,0UL), max(((col>> 0)&0xFF)-0x10UL,0UL));
		leds::set_ring(3, gamma_curve[color.red],  gamma_curve[color.green], gamma_curve[color.blue]);
		leds::set_ring(5, gamma_curve[color.red],  gamma_curve[color.green], gamma_curve[color.blue]);
		color = rgb_color(max(((col>>16)&0xFF)-0x00UL,0UL), max(((col>> 8)&0xFF)-0x00UL,0UL), max(((col>> 0)&0xFF)-0x00UL,0UL));
		leds::set_ring(4, gamma_curve[color.red],  gamma_curve[color.green], gamma_curve[color.blue]);

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[((eeprom_settings.bird_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 0)&0xFF)]);
		}

		delay(5);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void rgb_walker() {

	static uint8_t work_buffer[0x80] = { 0 };

	for (uint32_t c = 0; c < 0x80; c++) {
		work_buffer[c] = max(0UL,(sine_wave[c] - 0x80UL) - 0x20UL) ;
	}

	uint32_t walk = 0;
	uint32_t rgb_walk = 0;
	uint32_t flash = 0;
	for (;;) {

		rgb_color color = hsvToRgb(rgb_walk/3, 255, 255);
		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, gamma_curve[(work_buffer[(((0x80/8)*d) + walk)&0x7F] * ((color.red)&0xFF)) >> 8],
					 	      gamma_curve[(work_buffer[(((0x80/8)*d) + walk)&0x7F] * ((color.green)&0xFF)) >> 8],
					 		  gamma_curve[(work_buffer[(((0x80/8)*d) + walk)&0x7F] * ((color.blue)&0xFF)) >> 8]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		walk ++;
		walk &= 0x7F;

		rgb_walk ++;
		if (rgb_walk >= 360*3) {
			rgb_walk = 0;
		}

		delay(5);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}
static void rgb_glow() {
	uint32_t rgb_walk = 0;
	uint32_t walk = 0;
	int32_t switch_dir = 1;
	uint32_t switch_counter = 0;
	for (;;) {

		rgb_color color = hsvToRgb(rgb_walk, 255, 255);
		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring_synced(d,
				gamma_curve[((color.red)&0xFF)/4],
				gamma_curve[((color.green)&0xFF)/4],
				gamma_curve[((color.blue)&0xFF)/4]
			);
		}
		
		rgb_walk ++;
		if (rgb_walk >= 360) {
			rgb_walk = 0;
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		delay(50);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void rgb_tracer() {
	uint32_t rgb_walk = 0;
	uint32_t walk = 0;
	int32_t switch_dir = 1;
	uint32_t switch_counter = 0;
	for (;;) {

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d,0,0,0);
		}

		rgb_color color = hsvToRgb(rgb_walk/3, 255, 255);
		leds::set_ring_synced(walk&0x7,
			gamma_curve[((color.red)&0xFF)/4],
			gamma_curve[((color.green)&0xFF)/4],
			gamma_curve[((color.blue)&0xFF)/4]
		);

		walk += switch_dir;

		rgb_walk += 7;
		if (rgb_walk >= 360*3) {
			rgb_walk = 0;
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		switch_counter ++;
		if (switch_counter > 64 && random.get(0,2)) {
			switch_dir *= -1;
			switch_counter = 0;
			walk += switch_dir;
			walk += switch_dir;
		}

		delay(50);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void light_tracer() {
	uint32_t walk = 0;

	rgb_color gradient[8];
	uint32_t col = eeprom_settings.ring_color;
	gradient[7] = rgb_color(max(((col>>16)&0xFF)-0x40UL,0UL), max(((col>> 8)&0xFF)-0x40UL,0UL), max(((col>> 0)&0xFF)-0x40UL,0UL));
	gradient[6] = rgb_color(max(((col>>16)&0xFF)-0x40UL,0UL), max(((col>> 8)&0xFF)-0x40UL,0UL), max(((col>> 0)&0xFF)-0x40UL,0UL));
	gradient[5] = rgb_color(max(((col>>16)&0xFF)-0x30UL,0UL), max(((col>> 8)&0xFF)-0x30UL,0UL), max(((col>> 0)&0xFF)-0x30UL,0UL));
	gradient[4] = rgb_color(max(((col>>16)&0xFF)-0x18UL,0UL), max(((col>> 8)&0xFF)-0x18UL,0UL), max(((col>> 0)&0xFF)-0x18UL,0UL));
	gradient[3] = rgb_color(max(((col>>16)&0xFF)-0x00UL,0UL), max(((col>> 8)&0xFF)-0x00UL,0UL), max(((col>> 0)&0xFF)-0x00UL,0UL));
	gradient[2] = rgb_color(max((col>>16)&0xFF,0x10UL), max((col>> 8)&0xFF,0x00UL), max((col>> 0)&0xFF,0x20UL));
	gradient[1] = rgb_color(max((col>>16)&0xFF,0x30UL), max((col>> 8)&0xFF,0x30UL), max((col>> 0)&0xFF,0x30UL));
	gradient[0] = rgb_color(max((col>>16)&0xFF,0x40UL), max((col>> 8)&0xFF,0x40UL), max((col>> 0)&0xFF,0x40UL));

	for (;;) {

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring((walk+d)&0x7,
				gamma_curve[((gradient[d].red)&0xFF)],
				gamma_curve[((gradient[d].green)&0xFF)],
				gamma_curve[((gradient[d].blue)&0xFF)]
			);
		}

		walk--;

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		delay(100);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}


static void ring_tracer() {
	uint32_t walk = 0;
	int32_t switch_dir = 1;
	uint32_t switch_counter = 0;
	for (;;) {

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d,0,0,0);
		}

		leds::set_ring_synced((walk+0)&0x7,
			gamma_curve[(eeprom_settings.ring_color>>16)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 8)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 0)&0xFF]
		);
		leds::set_ring_synced((walk+1)&0x7,
			gamma_curve[(eeprom_settings.ring_color>>16)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 8)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 0)&0xFF]
		);
		leds::set_ring_synced((walk+2)&0x7,
			gamma_curve[(eeprom_settings.ring_color>>16)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 8)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 0)&0xFF]
		);

		walk += switch_dir;

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		switch_counter ++;
		if (switch_counter > 64 && random.get(0,2)) {
			switch_dir *= -1;
			switch_counter = 0;
			walk += switch_dir;
			walk += switch_dir;
		}

		delay(50);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void ring_bar_rotate() {
	uint32_t rgb_walk = 0;
	uint32_t walk = 0;
	int32_t switch_dir = 1;
	uint32_t switch_counter = 0;
	for (;;) {

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d,0,0,0);
		}

		leds::set_ring_synced((walk+0)&0x7,
			gamma_curve[(eeprom_settings.ring_color>>16)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 8)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 0)&0xFF]
		);
		leds::set_ring_synced((walk+4)&0x7,
			gamma_curve[(eeprom_settings.ring_color>>16)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 8)&0xFF],
			gamma_curve[(eeprom_settings.ring_color>> 0)&0xFF]
		);

		walk += switch_dir;

		rgb_walk += 7;
		if (rgb_walk >= 360*3) {
			rgb_walk = 0;
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		switch_counter ++;
		if (switch_counter > 64 && random.get(0,2)) {
			switch_dir *= -1;
			switch_counter = 0;
			walk += switch_dir;
			walk += switch_dir;
		}

		delay(50);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void ring_bar_move() {
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
			leds::set_ring(d,0,0,0);
		}

		if (indecies0[(walk)%15] >=0 ) {
				leds::set_ring_synced(indecies0[(walk)%15],
				gamma_curve[(eeprom_settings.ring_color>>16)&0xFF],
				gamma_curve[(eeprom_settings.ring_color>> 8)&0xFF],
				gamma_curve[(eeprom_settings.ring_color>> 0)&0xFF]
			);	
		}
		if (indecies1[(walk)%15] >=0 ) {
			leds::set_ring_synced(indecies1[(walk)%15],
				gamma_curve[(eeprom_settings.ring_color>>16)&0xFF],
				gamma_curve[(eeprom_settings.ring_color>> 8)&0xFF],
				gamma_curve[(eeprom_settings.ring_color>> 0)&0xFF]
			);
		}

		walk += switch_dir;

		rgb_walk += 7;
		if (rgb_walk >= 360*3) {
			rgb_walk = 0;
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		switch_counter ++;
		if (switch_counter > 64 && random.get(0,2)) {
			switch_dir *= -1;
			switch_counter = 0;
			walk += switch_dir;
			walk += switch_dir;
		}

		delay(50);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void rgb_vertical_wall() {
	uint32_t rgb_walk = 0;
	for (;;) {

		rgb_color color;
		color = hsvToRgb(((rgb_walk+  0)/3)%360, 255, 255);
		leds::set_ring_synced(0, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		color = hsvToRgb(((rgb_walk+ 30)/3)%360, 255, 255);
		leds::set_ring_synced(1, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		leds::set_ring_synced(7, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		color = hsvToRgb(((rgb_walk+120)/3)%360, 255, 255);
		leds::set_ring_synced(2, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		leds::set_ring_synced(6, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		color = hsvToRgb(((rgb_walk+210)/3)%360, 255, 255);
		leds::set_ring_synced(3, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		leds::set_ring_synced(5, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		color = hsvToRgb(((rgb_walk+230)/3)%360, 255, 255);
		leds::set_ring_synced(4, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);

		rgb_walk += 7;
		if (rgb_walk >= 360*3) {
			rgb_walk = 0;
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		delay(40);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void shine_vertical() {
	uint32_t rgb_walk = 0;
	rgb_color gradient[256];
	for (uint32_t c = 0; c < 128; c++) {
		uint32_t r = max((eeprom_settings.ring_color>>16)&0xFF,c/2);
		uint32_t g = max((eeprom_settings.ring_color>> 8)&0xFF,c/2);
		uint32_t b = max((eeprom_settings.ring_color>> 0)&0xFF,c/2);
		gradient[c] = rgb_color(r, g, b);
	}
	for (uint32_t c = 0; c < 128; c++) {
		uint32_t r = max((eeprom_settings.ring_color>>16)&0xFF,(128-c)/2);
		uint32_t g = max((eeprom_settings.ring_color>> 8)&0xFF,(128-c)/2);
		uint32_t b = max((eeprom_settings.ring_color>> 0)&0xFF,(128-c)/2);
		gradient[c+128] = rgb_color(r, g, b);
	}

	for (;;) {
		rgb_color color;
		color = gradient[((rgb_walk+ 0))%256];
		leds::set_ring_synced(0, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		color = gradient[((rgb_walk+10))%256];
		leds::set_ring_synced(1, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		leds::set_ring_synced(7, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		color = gradient[((rgb_walk+40))%256];
		leds::set_ring_synced(2, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		leds::set_ring_synced(6, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		color = gradient[((rgb_walk+70))%256];
		leds::set_ring_synced(3, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		leds::set_ring_synced(5, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		color = gradient[((rgb_walk+80))%256];
		leds::set_ring_synced(4, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);

		rgb_walk += 7;
		if (rgb_walk >= 256) {
			rgb_walk = 0;
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		delay(80);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void shine_horizontal() {
	int32_t rgb_walk = 0;
	int32_t switch_dir = 1;

	rgb_color gradient[256];
	for (uint32_t c = 0; c < 128; c++) {
		uint32_t r = max((eeprom_settings.ring_color>>16)&0xFF,c/2);
		uint32_t g = max((eeprom_settings.ring_color>> 8)&0xFF,c/2);
		uint32_t b = max((eeprom_settings.ring_color>> 0)&0xFF,c/2);
		gradient[c] = rgb_color(r, g, b);
	}
	for (uint32_t c = 0; c < 128; c++) {
		uint32_t r = max((eeprom_settings.ring_color>>16)&0xFF,(128-c)/2);
		uint32_t g = max((eeprom_settings.ring_color>> 8)&0xFF,(128-c)/2);
		uint32_t b = max((eeprom_settings.ring_color>> 0)&0xFF,(128-c)/2);
		gradient[c+128] = rgb_color(r, g, b);
	}

	for (;;) {
		rgb_color color;
		color = gradient[((rgb_walk+ 0))%256];
		leds::set_ring_synced(6, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		color = gradient[((rgb_walk+10))%256];
		leds::set_ring_synced(7, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		leds::set_ring_synced(5, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		color = gradient[((rgb_walk+40))%256];
		leds::set_ring_synced(0, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		leds::set_ring_synced(4, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		color = gradient[((rgb_walk+70))%256];
		leds::set_ring_synced(1, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		leds::set_ring_synced(3, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);
		color = gradient[((rgb_walk+80))%256];
		leds::set_ring_synced(2, gamma_curve[((color.red)&0xFF)], gamma_curve[((color.green)&0xFF)], gamma_curve[((color.blue)&0xFF)]);

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
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		delay(80);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void rgb_horizontal_wall() {
	uint32_t rgb_walk = 0;
	for (;;) {

		rgb_color color;
		color = hsvToRgb(((rgb_walk+  0)/3)%360, 255, 255);
		leds::set_ring_synced(6, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		color = hsvToRgb(((rgb_walk+ 30)/3)%360, 255, 255);
		leds::set_ring_synced(7, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		leds::set_ring_synced(5, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		color = hsvToRgb(((rgb_walk+120)/3)%360, 255, 255);
		leds::set_ring_synced(0, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		leds::set_ring_synced(4, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		color = hsvToRgb(((rgb_walk+210)/3)%360, 255, 255);
		leds::set_ring_synced(1, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		leds::set_ring_synced(3, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);
		color = hsvToRgb(((rgb_walk+230)/3)%360, 255, 255);
		leds::set_ring_synced(2, gamma_curve[((color.red)&0xFF)/4], gamma_curve[((color.green)&0xFF)/4], gamma_curve[((color.blue)&0xFF)/4]);

		rgb_walk += 7;
		if (rgb_walk >= 360*3) {
			rgb_walk = 0;
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		delay(40);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void lightning() {
	for (;;) {

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, 0,0,0);
		}

		int index = random.get(0,128);
		leds::set_ring_all(index,0x40,0x40,0x40);

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		delay(10);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void sparkle() {
	for (;;) {

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, 0,0,0);
		}

		int index = random.get(0,16);
		leds::set_ring_all(index,random.get(0x00,0x10),random.get(0x00,0x10),random.get(0,0x10));

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		delay(50);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void lightning_crazy() {
	for (;;) {

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, 0,0,0);
		}

		int index = random.get(0,16);
		leds::set_ring_all(index,0x40,0x40,0x40);

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(eeprom_settings.bird_color>>16)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 8)&0xFF],
					 		  gamma_curve[(eeprom_settings.bird_color>> 0)&0xFF]);
		}

		delay(10);

		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void heartbeat() {
	int32_t rgb_walk = 0;
	int32_t switch_dir = 1;
	for (; ;) {
		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, gamma_curve[((eeprom_settings.ring_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 0)&0xFF)]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[(((eeprom_settings.bird_color>>16)&0xFF)*rgb_walk)/256],
					 		  gamma_curve[(((eeprom_settings.bird_color>> 8)&0xFF)*rgb_walk)/256],
					 		  gamma_curve[(((eeprom_settings.bird_color>> 0)&0xFF)*rgb_walk)/256]);
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

		delay(8);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void brilliance() {
	int32_t current_wait = 0;
	int32_t wait_time = 0;
	int32_t rgb_walk = 0;
	int32_t switch_dir = 1;
	for (; ;) {
		rgb_color gradient[256];
		for (int32_t c = 0; c < 112; c++) {
			uint32_t r = (eeprom_settings.bird_color>>16)&0xFF;
			uint32_t g = (eeprom_settings.bird_color>> 8)&0xFF;
			uint32_t b = (eeprom_settings.bird_color>> 0)&0xFF;
			gradient[c] = rgb_color(r, g, b);
		}
		for (uint32_t c = 0; c < 16; c++) {
			uint32_t r = max((eeprom_settings.bird_color>>16)&0xFF,c*8);
			uint32_t g = max((eeprom_settings.bird_color>> 8)&0xFF,c*8);
			uint32_t b = max((eeprom_settings.bird_color>> 0)&0xFF,c*8);
			gradient[c+112] = rgb_color(r, g, b);
		}
		for (uint32_t c = 0; c < 16; c++) {
			uint32_t r = max((eeprom_settings.bird_color>>16)&0xFF,(16-c)*8);
			uint32_t g = max((eeprom_settings.bird_color>> 8)&0xFF,(16-c)*8);
			uint32_t b = max((eeprom_settings.bird_color>> 0)&0xFF,(16-c)*8);
			gradient[c+128] = rgb_color(r, g, b);
		}
		for (int32_t c = 0; c < 112; c++) {
			uint32_t r = (eeprom_settings.bird_color>>16)&0xFF;
			uint32_t g = (eeprom_settings.bird_color>> 8)&0xFF;
			uint32_t b = (eeprom_settings.bird_color>> 0)&0xFF;
			gradient[c+144] = rgb_color(r, g, b);
		}


		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, gamma_curve[((eeprom_settings.ring_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 0)&0xFF)]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			rgb_color color = gradient[((rgb_walk+ 0))%256];
			leds::set_bird(d, gamma_curve[((color.red)&0xFF)], 
							  gamma_curve[((color.green)&0xFF)], 
							  gamma_curve[((color.blue)&0xFF)]);
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

		delay(10);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void tingling() {
	#define NUM_TINGLES 16
	struct tingle {
		bool active;
		int32_t wait;
		int32_t index;
		uint32_t progress;
		bool lightordark;
	} tingles[NUM_TINGLES] = {0};

	for (; ;) {

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, gamma_curve[((eeprom_settings.ring_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 0)&0xFF)]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[((eeprom_settings.bird_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 0)&0xFF)]);
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
					r = max((eeprom_settings.ring_color>>16)&0xFF,progress*8);
					g = max((eeprom_settings.ring_color>> 8)&0xFF,progress*8);
					b = max((eeprom_settings.ring_color>> 0)&0xFF,progress*8);					
				} else {
					r = ((eeprom_settings.ring_color>>16)&0xFF)-progress*8;
					g = ((eeprom_settings.ring_color>> 8)&0xFF)-progress*8;
					b = ((eeprom_settings.ring_color>> 0)&0xFF)-progress*8;
					r = max(r,0UL);
					g = max(g,0UL);
					b = max(b,0UL);					
				}
				leds::set_ring_all(tingles[c].index, gamma_curve[r], 
								  				 gamma_curve[g], 
								  				 gamma_curve[b]);
				tingles[c].progress++;
			}
		}

		delay(20);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}


static void twinkle() {
	#define NUM_TWINKLE 3
	struct tingle {
		bool active;
		int32_t wait;
		int32_t index;
		uint32_t progress;
	} tingles[NUM_TWINKLE] = {0};

	for (; ;) {

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, gamma_curve[((eeprom_settings.ring_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 0)&0xFF)]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[((eeprom_settings.bird_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 0)&0xFF)]);
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

				r = max((eeprom_settings.ring_color>>16)&0xFF,progress*16);
				g = max((eeprom_settings.ring_color>> 8)&0xFF,progress*16);
				b = max((eeprom_settings.ring_color>> 0)&0xFF,progress*16);					
				leds::set_ring_all(tingles[c].index, gamma_curve[r], 
								  				 gamma_curve[g], 
								  				 gamma_curve[b]);
				tingles[c].progress++;
			}
		}

		delay(50);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void simple_change_ring() {

	int32_t color_index = -1;

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
			leds::set_ring(d, gamma_curve[r],
					 		  gamma_curve[g],
					 		  gamma_curve[b]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[((eeprom_settings.bird_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 0)&0xFF)]);
		}

		delay(15);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void simple_change_bird() {

	int32_t color_index = -1;

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
			leds::set_ring(d, gamma_curve[((eeprom_settings.ring_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 0)&0xFF)]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[r],
					 		  gamma_curve[g],
					 		  gamma_curve[b]);
		}

		delay(15);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void simple_random() {

	rgb_color colors[16];
	for (int32_t c = 0; c<16; c++) {
		colors[c].red = random.get(0x00,0x40);
		colors[c].green = random.get(0x00,0x40);
		colors[c].blue = random.get(0x00,0x40);
	}

	for (; ;) {

		uint32_t index = random.get(0x00,0x10);
		colors[index].red = random.get(0x00,0x40);
		colors[index].green = random.get(0x00,0x40);
		colors[index].blue = random.get(0x00,0x40);

		for (uint32_t d = 0; d < 16; d++) {
			leds::set_ring_all(d, gamma_curve[colors[d].red],
					 		      gamma_curve[colors[d].green],
					 		      gamma_curve[colors[d].blue]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[((eeprom_settings.bird_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 0)&0xFF)]);
		}

		delay(20);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void diagonal_wipe() {

	int32_t walk = 0;
	int32_t wait = random.get(60,1500);
	int32_t dir = random.get(0,2);

	for (; ;) {
		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, gamma_curve[((eeprom_settings.ring_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 0)&0xFF)]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[((eeprom_settings.bird_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 0)&0xFF)]);
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

		if (i0 >= 0) leds::set_ring_synced(i0, 0x40,0x40,0x40);
		if (i1 >= 0) leds::set_ring_synced(i1, 0x40,0x40,0x40);

		delay(5);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void shimmer_outside() {
	int32_t walk = 0;
	int32_t wait = random.get(16,64);
	int32_t dir = random.get(0,2);

	for (; ;) {

		rgb_color color = rgb_color(((eeprom_settings.ring_color>>16)&0xFF),
								  ((eeprom_settings.ring_color>> 8)&0xFF),
								  ((eeprom_settings.ring_color>> 0)&0xFF));

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[((eeprom_settings.bird_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.bird_color>> 0)&0xFF)]);
		}

		int32_t r = color.red;
		int32_t g = color.green;
		int32_t b = color.blue;
		if (walk < 8) {
			r = max(int32_t(0),r - int32_t(walk));
			g = max(int32_t(0),g - int32_t(walk));
			b = max(int32_t(0),b - int32_t(walk));
		} else if (walk < 16) {
			r = max(int32_t(0),r - int32_t((8-(walk-8))));
			g = max(int32_t(0),g - int32_t((8-(walk-8))));
			b = max(int32_t(0),b - int32_t((8-(walk-8))));
		}

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, gamma_curve[r],
					 		  gamma_curve[g],
					 		  gamma_curve[b]);
		}
		
		walk ++;
		if (walk > wait) {
			walk = 0;
			wait = random.get(16,64);

		}

		delay(2);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void shimmer_inside() {
	int32_t walk = 0;
	int32_t wait = random.get(16,64);
	int32_t dir = random.get(0,2);

	for (; ;) {

		rgb_color color = rgb_color(((eeprom_settings.bird_color>>16)&0xFF),
								    ((eeprom_settings.bird_color>> 8)&0xFF),
								    ((eeprom_settings.bird_color>> 0)&0xFF));

		for (uint32_t d = 0; d < 8; d++) {
			leds::set_ring(d, gamma_curve[((eeprom_settings.ring_color>>16)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 8)&0xFF)],
					 		  gamma_curve[((eeprom_settings.ring_color>> 0)&0xFF)]);
		}

		int32_t r = color.red;
		int32_t g = color.green;
		int32_t b = color.blue;
		if (walk < 8) {
			r = max(int32_t(0),r - int32_t(walk));
			g = max(int32_t(0),g - int32_t(walk));
			b = max(int32_t(0),b - int32_t(walk));
		} else if (walk < 16) {
			r = max(int32_t(0),r - int32_t((8-(walk-8))));
			g = max(int32_t(0),g - int32_t((8-(walk-8))));
			b = max(int32_t(0),b - int32_t((8-(walk-8))));
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[r],
					 		  gamma_curve[g],
					 		  gamma_curve[b]);
		}
		
		walk ++;
		if (walk > wait) {
			walk = 0;
			wait = random.get(16,64);

		}

		delay(10);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

static void red() {

	int32_t wait = 1200;

	int32_t index = 0;

	int32_t br = ((eeprom_settings.bird_color>>16)&0xFF);
	int32_t bg = ((eeprom_settings.bird_color>> 8)&0xFF);
	int32_t bb = ((eeprom_settings.bird_color>> 0)&0xFF);

	int32_t rr = ((eeprom_settings.ring_color>>16)&0xFF);
	int32_t rg = ((eeprom_settings.ring_color>> 8)&0xFF);
	int32_t rb = ((eeprom_settings.ring_color>> 0)&0xFF);

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
			leds::set_ring(d, gamma_curve[r1r],
					 		  gamma_curve[r1g],
					 		  gamma_curve[r1b]);
		}

		for (uint32_t d = 0; d < 4; d++) {
			leds::set_bird(d, gamma_curve[b1r],
					 		  gamma_curve[b1g],
					 		  gamma_curve[b1b]);
		}

		delay(20);
		spi::push_frame();
		if (test_button()) {
			return;
		}
	}
}

#ifndef NO_SX1280
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
			
			const uint32_t BUSY_PIN = 0x0000;
			const uint32_t DIO1_PIN = 0x0008;
			const uint32_t DIO2_PIN = 0x0115;
			const uint32_t DIO3_PIN = 0x011F;
			const uint32_t RESET_PIN = 0x011F;
			
			const IRQn_Type INT_IRQn = PIN_INT0_IRQn;
			const uint32_t INT_NUM = 0;
			const uint32_t INT_CHN = PININTCH0;
			
			const uint32_t LORA_BUFFER_SIZE = 30;
			uint8_t txBuffer[30] = { 0 };
			uint8_t rxBuffer[30] = { 0 };
			
			const uint32_t RF_FREQUENCY = 2425000000UL;
			const uint32_t TX_OUTPUT_POWER = 13;

			const uint16_t TX_TIMEOUT_VALUE = 100; // ms
			const uint16_t RX_TIMEOUT_VALUE = 0xffff; // ms
			const RadioTickSizes RX_TIMEOUT_TICK_SIZE = RADIO_TICK_SIZE_1000_US;

			const uint16_t IrqMask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;

			// Standard values

			const uint32_t AUTO_TX_OFFSET = 0x21;
			const uint32_t MASK_RANGINGMUXSEL = 0xCF;
			const uint32_t DEFAULT_RANGING_FILTER_SIZE = 0x7F;
			const uint32_t MASK_FORCE_PREAMBLELENGTH = 0x8F;
			const uint32_t BLE_ADVERTIZER_ACCESS_ADDRESS = 0x8E89BED6;

			SX1280() { }
			
			void Init(bool pollMode) {

				Chip_IOCON_PinMuxSet(LPC_IOCON, (BUSY_PIN>>8), (BUSY_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (BUSY_PIN>>8), (BUSY_PIN&0xFF));
				
				Chip_IOCON_PinMuxSet(LPC_IOCON, (DIO1_PIN>>8), (DIO1_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (DIO1_PIN>>8), (DIO1_PIN&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (DIO2_PIN>>8), (DIO2_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (DIO2_PIN>>8), (DIO2_PIN&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (DIO3_PIN>>8), (DIO3_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (DIO3_PIN>>8), (DIO3_PIN&0xFF));
				
				Chip_IOCON_PinMuxSet(LPC_IOCON, (RESET_PIN>>8), (RESET_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
			
				if (!pollMode) {
					Chip_PININT_SetPinModeEdge(LPC_PININT, INT_CHN);
					Chip_PININT_EnableIntLow(LPC_PININT, INT_CHN);
					Chip_PININT_EnableIntHigh(LPC_PININT, INT_CHN);
					
					Chip_SYSCTL_SetPinInterrupt(INT_NUM, uint8_t(DIO1_PIN>>8), uint8_t(DIO1_PIN&0xFF));  
					
					NVIC_ClearPendingIRQ(INT_IRQn);  
					NVIC_EnableIRQ(INT_IRQn);  
				}

				putchar( 0x98 );                     // Reversed opcode for read register (0x19)
				putchar( 0x10 );                     // Reversed MSB register address (0x08)
				putchar( 0x18 );                     // Reversed LSB register address (0x18)
				putchar( 0x80 );            		 // Reversed value for reading only 1 byte (0x01)

				uint8_t regVal = getchar( ) & 0xF3;  // Read reversed value and mask it

				putchar( 0x18 );            		 // Reversed opcode for read register (0x18)
				putchar( 0x10 );            		 // Reversed MSB register address (0x08)
				putchar( 0x18 );            		 // Reversed LSB register address (0x18)
				putchar( 0x80 );            		 // Reversed value for writing only 1 byte (0x01)
				putchar( regVal );           	     // The new value of the register

				// After this point, the UART is running standard mode: 8 data bit, 1 even
				// parity bit, 1 stop bit, 115200 baud, LSB first
				delay( 10 );
				
				Wakeup();

				SetStandby( STDBY_RC );

				ModulationParams modulationParams;
				modulationParams.PacketType                  = PACKET_TYPE_LORA;
				modulationParams.Params.LoRa.SpreadingFactor = LORA_SF5;
				modulationParams.Params.LoRa.Bandwidth       = LORA_BW_0200;
				modulationParams.Params.LoRa.CodingRate      = LORA_CR_LI_4_5;
				SetPacketType( modulationParams.PacketType );
				SetModulationParams( &modulationParams );
	
				PacketParams PacketParams;
				PacketParams.PacketType                 	 = PACKET_TYPE_LORA;
				PacketParams.Params.LoRa.PreambleLength      = 0x0C;
				PacketParams.Params.LoRa.HeaderType          = LORA_PACKET_IMPLICIT;
				PacketParams.Params.LoRa.PayloadLength       = LORA_BUFFER_SIZE;
				PacketParams.Params.LoRa.Crc                 = LORA_CRC_ON;
				PacketParams.Params.LoRa.InvertIQ            = LORA_IQ_NORMAL;
				SetPacketParams( &PacketParams );

				SetRfFrequency( RF_FREQUENCY );
				SetBufferBaseAddresses( 0x00, 0x00 );
				SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_20_US );
				
				SetDioIrqParams( SX1280::IrqMask, SX1280::IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

		    	SetRx( ( TickTime ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
			}
			
			void SendBuffer() {
				SendPayload( txBuffer, LORA_BUFFER_SIZE, ( TickTime ) { RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } ); 
			}
			
			void Reset() {
				disableIRQ();
				delay( 20 );
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				Chip_GPIO_SetPinOutLow(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				delay( 50 );
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				delay( 20 );
				enableIRQ();
			}
			
			void Wakeup() {
				disableIRQ();

				putchar( RADIO_GET_STATUS );
				getchar();

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

				putchar( command );
				if(size > 0) {
					putchar( size );
					for( uint32_t i = 0; i < size; i++ ) {
						putchar( buffer[i] );
					}
				}
			
				if( command != RADIO_SET_SLEEP ) {
					WaitOnBusy( );
				}
			}

			void ReadCommand(RadioCommand command, uint8_t *buffer, uint32_t size ) {
				WaitOnBusy();

				putchar( command );

				// Behavior on the UART is different depending of the opcode command
				if( ( command == RADIO_GET_PACKETTYPE ) ||
					( command == RADIO_GET_RXBUFFERSTATUS ) ||
					( command == RADIO_GET_RSSIINST ) ||
					( command == RADIO_GET_PACKETSTATUS ) ||
					( command == RADIO_GET_IRQSTATUS ) ) {
					putchar( size );
				}

				for( uint32_t i = 0; i < size; i++ ) {
					 buffer[i] = getchar( );
				}

				WaitOnBusy( );
			}
			
			void WriteRegister( uint32_t address, uint8_t *buffer, uint32_t size ) {
				WaitOnBusy( );

				uint32_t addr = address;
				uint32_t i = 0;
				for( addr = address; ( addr + 255 ) < ( address + size ); ) {
					putchar( RADIO_WRITE_REGISTER );
					putchar( ( addr & 0xFF00 ) >> 8 );
					putchar( ( addr & 0x00FF )      );
					putchar( 255 );
					for( uint32_t lastAddr = addr + 255 ; addr < lastAddr; i++, addr++ ) {
						putchar( buffer[i] );
					}
				}

				putchar( RADIO_WRITE_REGISTER );
				putchar( ( addr & 0xFF00 ) >> 8 );
				putchar( ( addr & 0x00FF )      );
				putchar( address + size - addr );

				for( ; addr < ( address + size ); addr++, i++ ) {
					putchar( buffer[i] );
				}

				WaitOnBusy( );
			}

			void WriteRegister( uint32_t address, uint8_t value ) {
				WriteRegister( address, &value, 1 );
			}
			
			void ReadRegister( uint32_t address, uint8_t *buffer, uint32_t size ) {
				WaitOnBusy( );

				uint32_t addr = address;
				uint32_t i = 0;
				for( addr = address; ( addr + 255 ) < ( address + size ); ) {
					putchar( RADIO_READ_REGISTER );
					putchar( ( addr & 0xFF00 ) >> 8 );
					putchar( addr & 0x00FF );
					putchar( 255 );
					for( uint32_t lastAddr = addr + 255 ; addr < lastAddr; i++, addr++ ) {
						buffer[i] = getchar( );
					}
				}
				putchar( RADIO_READ_REGISTER );
				putchar( ( addr & 0xFF00 ) >> 8 );
				putchar( addr & 0x00FF );
				putchar( address + size - addr );
				for( ; addr < ( address + size ); addr++, i++ ) {
					buffer[i] = getchar( );
				}

				WaitOnBusy( );
			}

			uint8_t ReadRegister( uint32_t address )
			{
				uint8_t data;
				ReadRegister( address, &data, 1 );
				return data;
			}

			void WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size ) {
			    WaitOnBusy( );

				putchar( RADIO_WRITE_BUFFER );
				putchar( offset );
				putchar( size );
				for( uint32_t i = 0; i < size; i++ )
				{
					putchar( buffer[i] );
				}

				WaitOnBusy( );
			}

			void ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size ) {
			    WaitOnBusy( );

				putchar( RADIO_READ_BUFFER );
				putchar( offset );
				putchar( size );
				for( uint16_t i = 0; i < size; i++ )
				{
					buffer[i] = getchar( );
				}

				WaitOnBusy( );
			}


			void SetSleep( SleepParams sleepConfig )
			{
				uint8_t sleep = ( sleepConfig.WakeUpRTC << 3 ) |
								( sleepConfig.InstructionRamRetention << 2 ) |
								( sleepConfig.DataBufferRetention << 1 ) |
								( sleepConfig.DataRamRetention );

				OperatingMode = MODE_SLEEP;
				WriteCommand( RADIO_SET_SLEEP, &sleep, 1 );
			}

			void SetStandby( RadioStandbyModes standbyConfig )
			{
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

			void SetFs( void )
			{
				WriteCommand( RADIO_SET_FS, 0, 0 );
				OperatingMode = MODE_FS;
			}

			void SetTx( TickTime timeout )
			{
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

			void SetRx( TickTime timeout )
			{
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

			void SetRxDutyCycle( RadioTickSizes periodBase, uint16_t periodBaseCountRx, uint16_t periodBaseCountSleep )
			{
				uint8_t buf[5];

				buf[0] = periodBase;
				buf[1] = ( uint8_t )( ( periodBaseCountRx >> 8 ) & 0x00FF );
				buf[2] = ( uint8_t )( periodBaseCountRx & 0x00FF );
				buf[3] = ( uint8_t )( ( periodBaseCountSleep >> 8 ) & 0x00FF );
				buf[4] = ( uint8_t )( periodBaseCountSleep & 0x00FF );
				WriteCommand( RADIO_SET_RXDUTYCYCLE, buf, 5 );
				OperatingMode = MODE_RX;
			}

			void SetCad( void )
			{
				WriteCommand( RADIO_SET_CAD, 0, 0 );
				OperatingMode = MODE_CAD;
			}

			void SetTxContinuousWave( void )
			{
				WriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
			}

			void SetTxContinuousPreamble( void )
			{
				WriteCommand( RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
			}

			void SetPacketType( RadioPacketTypes packetType )
			{
				// Save packet type internally to avoid questioning the radio
				PacketType = packetType;

				WriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
			}

			RadioPacketTypes GetPacketType( bool returnLocalCopy )
			{
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
			
			void SetRfFrequency( uint32_t rfFrequency )
			{
				uint8_t buf[3];
				uint32_t freq = 0;

				const uint64_t XTAL_FREQ = 52000000;
				freq = uint32_t( (uint64_t(rfFrequency) * uint64_t(262144)) / uint64_t(XTAL_FREQ) );

				buf[0] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
				buf[1] = ( uint8_t )( ( freq >> 8  ) & 0xFF );
				buf[2] = ( uint8_t )( ( freq       ) & 0xFF );
				WriteCommand( RADIO_SET_RFFREQUENCY, buf, 3 );
			}

			void SetTxParams( int8_t power, RadioRampTimes rampTime )
			{
				uint8_t buf[2];

				// The power value to send on SPI/UART is in the range [0..31] and the
				// physical output power is in the range [-18..13]dBm
				buf[0] = power + 18;
				buf[1] = ( uint8_t )rampTime;
				WriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
			}

			void SetCadParams( RadioLoRaCadSymbols cadSymbolNum )
			{
				WriteCommand( RADIO_SET_CADPARAMS, ( uint8_t* )&cadSymbolNum, 1 );
				OperatingMode = MODE_CAD;
			}

			void SetBufferBaseAddresses( uint8_t txBaseAddress, uint8_t rxBaseAddress )
			{
				uint8_t buf[2];

				buf[0] = txBaseAddress;
				buf[1] = rxBaseAddress;
				WriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
			}

			void SetModulationParams( ModulationParams *modParams )
			{
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
					case PACKET_TYPE_NONE:
						buf[0] = 0;
						buf[1] = 0;
						buf[2] = 0;
						break;
				}
				WriteCommand( RADIO_SET_MODULATIONPARAMS, buf, 3 );
			}

			void SetPacketParams( PacketParams *packetParams )
			{
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

			void ForcePreambleLength( RadioPreambleLengths preambleLength )
			{
				WriteRegister( REG_LR_PREAMBLELENGTH, ( ReadRegister( REG_LR_PREAMBLELENGTH ) & MASK_FORCE_PREAMBLELENGTH ) | preambleLength );
			}

			void GetRxBufferStatus( uint8_t *rxPayloadLength, uint8_t *rxStartBufferPointer )
			{
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

			void GetPacketStatus( PacketStatus *packetStatus )
			{
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
					case PACKET_TYPE_NONE:
						// In that specific case, we set everything in the packetStatus to zeros
						// and reset the packet type accordingly
						for (uint32_t c=0; c<sizeof(PacketStatus); c++) { ((uint8_t*)packetStatus)[c] = 0; } 
						packetStatus->packetType = PACKET_TYPE_NONE;
						break;
				}
			}

			int8_t GetRssiInst( void )
			{
				uint8_t raw = 0;

				ReadCommand( RADIO_GET_RSSIINST, &raw, 1 );

				return ( int8_t ) ( -raw / 2 );
			}

			void SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
			{
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

			uint16_t GetIrqStatus( void )
			{
				uint8_t irqStatus[2];
				ReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
				return ( irqStatus[0] << 8 ) | irqStatus[1];
			}

			void ClearIrqStatus( uint16_t irqMask )
			{
				uint8_t buf[2];

				buf[0] = ( uint8_t )( ( ( uint16_t )irqMask >> 8 ) & 0x00FF );
				buf[1] = ( uint8_t )( ( uint16_t )irqMask & 0x00FF );
				WriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
			}

			void Calibrate( CalibrationParams calibParam )
			{
				uint8_t cal = ( calibParam.ADCBulkPEnable << 5 ) |
							  ( calibParam.ADCBulkNEnable << 4 ) |
							  ( calibParam.ADCPulseEnable << 3 ) |
							  ( calibParam.PLLEnable << 2 ) |
							  ( calibParam.RC13MEnable << 1 ) |
							  ( calibParam.RC64KEnable );
				WriteCommand( RADIO_CALIBRATE, &cal, 1 );
			}

			void SetRegulatorMode( RadioRegulatorModes mode )
			{
				WriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
			}

			void SetSaveContext( void )
			{
				WriteCommand( RADIO_SET_SAVECONTEXT, 0, 0 );
			}

			void SetAutoTx( uint16_t time )
			{
				uint16_t compensatedTime = time - ( uint16_t )AUTO_TX_OFFSET;
				uint8_t buf[2];

				buf[0] = ( uint8_t )( ( compensatedTime >> 8 ) & 0x00FF );
				buf[1] = ( uint8_t )( compensatedTime & 0x00FF );
				WriteCommand( RADIO_SET_AUTOTX, buf, 2 );
			}

			void SetAutoFs( bool enableAutoFs )
			{
				WriteCommand( RADIO_SET_AUTOFS, ( uint8_t * )&enableAutoFs, 1 );
			}

			void SetLongPreamble( bool enable )
			{
				WriteCommand( RADIO_SET_LONGPREAMBLE, ( uint8_t * )&enable, 1 );
			}

			void SetPayload( uint8_t *buffer, uint8_t size, uint8_t offset )
			{
				WriteBuffer( offset, buffer, size );
			}

			uint8_t GetPayload( uint8_t *buffer, uint8_t *size , uint8_t maxSize )
			{
				uint8_t offset;

				GetRxBufferStatus( size, &offset );
				if( *size > maxSize )
				{
					return 1;
				}
				ReadBuffer( offset, buffer, *size );
				return 0;
			}

			void SendPayload( uint8_t *payload, uint8_t size, TickTime timeout, uint8_t offset = 0 )
			{
				SetPayload( payload, size, offset );
				SetTx( timeout );
			}

			uint8_t SetSyncWord( uint8_t syncWordIdx, uint8_t *syncWord )
			{
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

			void SetSyncWordErrorTolerance( uint8_t ErrorBits )
			{
				ErrorBits = ( ReadRegister( REG_LR_SYNCWORDTOLERANCE ) & 0xF0 ) | ( ErrorBits & 0x0F );
				WriteRegister( REG_LR_SYNCWORDTOLERANCE, ErrorBits );
			}

			uint8_t SetCrcSeed( uint8_t *seed )
			{
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
			void SetBleAccessAddress( uint32_t accessAddress )
			{
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS, ( accessAddress >> 24 ) & 0x000000FF );
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 1, ( accessAddress >> 16 ) & 0x000000FF );
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 2, ( accessAddress >> 8 ) & 0x000000FF );
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 3, accessAddress & 0x000000FF );
			}

			void SetBleAdvertizerAccessAddress( void )
			{
				SetBleAccessAddress( BLE_ADVERTIZER_ACCESS_ADDRESS );
			}
			#endif  // #ifdef BLE_SUPPORT

			void SetCrcPolynomial( uint16_t polynomial )
			{
				uint8_t val[2];

				val[0] = ( uint8_t )( polynomial >> 8 ) & 0xFF;
				val[1] = ( uint8_t )( polynomial  & 0xFF );

				switch( GetPacketType( true ) )
				{
				#if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
						WriteRegister( REG_LR_CRCPOLYBASEADDR, val, 2 );
						break;
				#endif  // #if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
					default:
						break;
				}
			}

			void SetWhiteningSeed( uint8_t seed )
			{
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
			void SetRangingIdLength( RadioRangingIdCheckLengths length )
			{
				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						WriteRegister( REG_LR_RANGINGIDCHECKLENGTH, ( ( ( ( uint8_t )length ) & 0x03 ) << 6 ) | ( ReadRegister( REG_LR_RANGINGIDCHECKLENGTH ) & 0x3F ) );
						break;
					default:
						break;
				}
			}

			void SetDeviceRangingAddress( uint32_t address )
			{
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

			void SetRangingRequestAddress( uint32_t address )
			{
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

			double GetRangingResult( RadioRangingResultTypes resultType )
			{
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

			void SetRangingCalibration( uint16_t cal )
			{
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

			void RangingClearFilterResult( void )
			{
				uint8_t regVal = ReadRegister( REG_LR_RANGINGRESULTCLEARREG );

				// To clear result, set bit 5 to 1 then to 0
				WriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal | ( 1 << 5 ) );
				WriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal & ( ~( 1 << 5 ) ) );
			}

			void RangingSetFilterNumSamples( uint8_t num )
			{
				// Silently set 8 as minimum value
				WriteRegister( REG_LR_RANGINGFILTERWINDOWSIZE, ( num < DEFAULT_RANGING_FILTER_SIZE ) ? DEFAULT_RANGING_FILTER_SIZE : num );
			}

			void SetRangingRole( RadioRangingRoles role )
			{
				uint8_t buf[1];

				buf[0] = role;
				WriteCommand( RADIO_SET_RANGING_ROLE, &buf[0], 1 );
			}
			#endif  // #ifdef RANGING_SUPPORT

			double GetFrequencyError( )
			{
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

			void SetPollingMode( void )
			{
				PollingMode = true;
			}

			#ifdef LORA_SUPPORT
			int32_t GetLoRaBandwidth( )
			{
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

			void SetInterruptMode( void )
			{
				PollingMode = false;
			}

			void OnDioIrq( void )
			{
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

			void ProcessIrqs( void )
			{
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
				// TODO
			}

			void rxDone() {
			    PacketStatus packetStatus;
				GetPacketStatus(&packetStatus);
				uint8_t rxBufferSize = 0;
                GetPayload( rxBuffer, &rxBufferSize, LORA_BUFFER_SIZE );
			}

			void rxSyncWordDone() {
				// TODO
			
			}
			void rxHeaderDone() {
				// TODO
			}
			
			void txTimeout() {
				// TODO
			}
			
			void rxTimeout() {
				// TODO
			}
			
			void rxError(IrqErrorCode errCode) {
				// TODO
			}

			void rangingDone(IrqRangingCode errCode) {
				// TODO
			}

			void cadDone(bool cadFlag) {
				// TODO
			}

	private:
	
			void disableIRQ() {
				NVIC_DisableIRQ(INT_IRQn);
			}

			void enableIRQ() {
				NVIC_EnableIRQ(INT_IRQn);
			}


			void putchar(uint8_t c) {	
				while ((Chip_UART_ReadLineStatus(LPC_USART) & UART_LSR_THRE) == 0) { }
				Chip_UART_SendByte(LPC_USART, c);
			}

			uint8_t getchar() {
				while ((Chip_UART_ReadLineStatus(LPC_USART) & UART_LSR_RDR) == 0) { }
				return(Chip_UART_ReadByte(LPC_USART));
			}

			int32_t complement2( const uint32_t num, const uint8_t bitCnt )
			{
				int32_t retVal = ( int32_t )num;
				if( num >= 2<<( bitCnt - 2 ) ) {
					retVal -= 2<<( bitCnt - 1 );
				}
				return retVal;
			}

    RadioOperatingModes 	OperatingMode = MODE_SLEEP;
	RadioPacketTypes 		PacketType = PACKET_TYPE_NONE;
    RadioLoRaBandwidths 	LoRaBandwidth = LORA_BW_0200;
    bool 					IrqState = false;
    bool 					PollingMode = true;

} sx1280;

extern "C" {
	volatile void PIN_INT0_IRQHandler(void) {
		sx1280.OnDioIrq();
	}
}
#endif  // #ifndef NO_SX1280

class SDD1306 {

	public:
			static const uint32_t i2caddr = 0x3C;
			static const uint32_t width = 64;
			static const uint32_t height = 32;

			SDD1306() {
				present = false;
				Clear();
			}

			void Clear(uint32_t value = 0) {
				for (uint32_t c = 0; c < height; c++) {
					buffer[(c*(width+1))] = 0x40;
					for (uint32_t d = 0; d < width; d++) {
						buffer[(c*(width+1))+d+1] = value;
					}
				}
			}

			void delay(int32_t ms) const {
				for (volatile uint32_t i = 0; i < ms*2400; i++) {}
			}

			void WriteCommand(uint8_t v) const {
				uint8_t control[2] = { 0 };
				control[1] = v;
				Chip_I2C_MasterSend(I2C0, i2caddr, control, 2);
			}

			bool Present() const { return present; }

			void Display() {
				uint8_t *buffer_ptr = buffer;
				for (uint32_t y = 0; y < height/8; y ++) {
					WriteCommand(0xB0+y);
					WriteCommand(0x00); // 0x00 +
					WriteCommand(0x12); // 0x20 offset
					Chip_I2C_MasterSend(I2C0, i2caddr, buffer_ptr, width + 1);
					buffer_ptr += width + 1;
				}
			}

			void Init() const {
				if (!Present()) {	
					return;
				}

				Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 31);

				Chip_GPIO_SetPinState(LPC_GPIO, 1, 31, true);

				delay(1);

				Chip_GPIO_SetPinState(LPC_GPIO, 1, 31, false);

				delay(10);

				Chip_GPIO_SetPinState(LPC_GPIO, 1, 31, true);

				WriteCommand(0xAE);

				WriteCommand(0x00);
				WriteCommand(0x12);

				WriteCommand(0x00);

				WriteCommand(0xB0);

				WriteCommand(0x81);
				WriteCommand(0x4F);

				WriteCommand(0xA1);

				WriteCommand(0xA6);

				WriteCommand(0xA8);
				WriteCommand(0x1F);

				WriteCommand(0xC8);

				WriteCommand(0xD3);
				WriteCommand(0x00);

				WriteCommand(0xD5);
				WriteCommand(0x80);

				WriteCommand(0xD9);
				WriteCommand(0x1F);

				WriteCommand(0xDA);
				WriteCommand(0x12);

				WriteCommand(0xDB);
				WriteCommand(0x40);

				WriteCommand(0x8D);
				WriteCommand(0x14);

				WriteCommand(0xAF);

				WriteCommand(0x20);
				WriteCommand(0x02);

				WriteCommand(0xA6);
			}

	private:
			friend class Setup;

			bool present;

			uint8_t buffer[(width + 1) * height / 8];

} sdd1306;

class Setup {

	public:
			Setup() {
			}

			void InitGPIO() {
				Chip_GPIO_Init(LPC_GPIO);
			}
			
			void InitPININT() {
				Chip_PININT_Init(LPC_PININT);
			}

			void InitUART() {

				Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 13, IOCON_FUNC3 | IOCON_MODE_INACT);	/* PIO0_13 used for RXD */
				Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 14, IOCON_FUNC3 | IOCON_MODE_INACT);	/* PIO0_14 used for TXD */

				Chip_UART_Init(LPC_USART);
				Chip_UART_SetBaud(LPC_USART, 115200);
				Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS));
				Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));
				Chip_UART_TXEnable(LPC_USART);
#ifdef NO_SX1280
				printf("\r\n\r\n======= UART initializing =======\r\n");

				printf("\r\n\r\n======= UART intialized =======\r\n");
#endif  // #ifdef NO_SX1280
			}

			void InitI2C() {
#ifdef NO_SX1280
				printf("\r\n\r\n======= I2C initializing =======\r\n");
#endif  // #ifdef NO_SX1280
				Chip_SYSCTL_PeriphReset(RESET_I2C0);

				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 4, IOCON_FUNC1 | I2C_FASTPLUS_BIT);
				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5, IOCON_FUNC1 | I2C_FASTPLUS_BIT);

				Chip_I2C_Init(I2C0);
				Chip_I2C_SetClockRate(I2C0, I2C_DEFAULT_SPEED);

				NVIC_DisableIRQ(I2C0_IRQn);
				Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling);

#ifdef NO_SX1280
				ProbeI2CSlaves();

				printf("======== I2C intialized ========\r\n");
#endif  // #ifdef NO_SX1280
			}

#ifdef NO_SX1280
			void ProbeI2CSlaves() {
				int i;
				uint8_t ch[2];

				printf("Probing available I2C devices...\r\n");
				printf("\r\n     00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
				printf("\r\n====================================================");
				for (i = 0; i <= 0x7F; i++) {
					if (!(i & 0x0F)) {
						printf("\r\n%02X  ", i >> 4);
					}
					if ((i <= 7) || (i > 0x78)) {
						printf("   ");
						continue;
					}
					/* Address 0x48 points to LM75AIM device which needs 2 bytes be read */
					if (Chip_I2C_MasterRead(I2C0, i, ch, 1 + (i == 0x48)) > 0) {
						printf(" %02X", i);
						switch(i) {
							case	sdd1306.i2caddr:
									sdd1306.present = true;
									break;
						}
					}
					else {
						printf(" --");
					}
				}
				printf("\r\n");
			}
#endif  // #ifdef NO_SX1280

} setup;

static uint8_t screen_data[256] = { 0 };

};

extern "C" {
	volatile void SysTick_Handler(void)
	{	
		system_clock_ms++;
#ifndef NO_SX1280
		if ( (system_clock_ms % 1024) == 0) {
			sx1280.SendBuffer();
		}
#endif  // #ifndef NO_SX1280
	}

	volatile void TIMER32_0_IRQHandler(void)
	{
	}

	volatile void UART_IRQHandler(void)
	{
	}
}


int main(void)
{
	SystemCoreClockUpdate();

	// Init LPC hardware
	setup.InitPININT();
	setup.InitGPIO();
	setup.InitUART();
	setup.InitI2C();

	// OLED
	sdd1306.Init();

	// SX1280	
#ifndef NO_SX1280
	sx1280.Init(false);
#endif  // #ifndef NO_SX1280

	// 1s timer
	SysTick_Config(SystemCoreClock / 1000);

	eeprom_settings.load();

	eeprom_settings.program_count = 27;

	if (eeprom_settings.bird_color == 0 ||
		eeprom_settings.bird_color_index > 16 ||
		eeprom_settings.ring_color == 0 ||
		eeprom_settings.ring_color_index > 16 ) {
	 	eeprom_settings.bird_color = 0x404000;
		eeprom_settings.bird_color_index = 0;
	 	eeprom_settings.ring_color = 0x083040;
		eeprom_settings.ring_color_index = 0;
		eeprom_settings.save();
	}
	
	random.init(0xCAFFE);
	
	// Boot screen
	flash_access.read_data(0, &screen_data[0], sizeof(screen_data));

	while (1) {

		sdd1306.Display();
		sdd1306.Clear(0x00);
		delay(200);
		sdd1306.Display();
		sdd1306.Clear(0xFF);
		delay(200);

		switch(eeprom_settings.program_curr) {
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

	return 0;
}
