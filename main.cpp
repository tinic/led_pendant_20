#include <stdint.h>
#include "printf.h"
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

extern "C" {
	void SysTick_Handler(void)
	{
	}

	void TIMER32_0_IRQHandler(void)
	{
	}

	void UART_IRQHandler(void)
	{
	}
}

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

			void Delay(int32_t ms) const {
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

				Delay(1);

				Chip_GPIO_SetPinState(LPC_GPIO, 1, 31, false);

				Delay(10);

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

			void InitUART() {

				Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 13, IOCON_FUNC3 | IOCON_MODE_INACT);	/* PIO0_13 used for RXD */
				Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 14, IOCON_FUNC3 | IOCON_MODE_INACT);	/* PIO0_14 used for TXD */

				Chip_UART_Init(LPC_USART);
				Chip_UART_SetBaud(LPC_USART, 115200);
				Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS));
				Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));
				Chip_UART_TXEnable(LPC_USART);
				printf("\r\n\r\n======= UART initializing =======\r\n");

				printf("\r\n\r\n======= UART intialized =======\r\n");
			}

			void InitI2C() {
				printf("\r\n\r\n======= I2C initializing =======\r\n");
				Chip_SYSCTL_PeriphReset(RESET_I2C0);

				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 4, IOCON_FUNC1 | I2C_FASTPLUS_BIT);
				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5, IOCON_FUNC1 | I2C_FASTPLUS_BIT);

				Chip_I2C_Init(I2C0);
				Chip_I2C_SetClockRate(I2C0, I2C_DEFAULT_SPEED);

				NVIC_DisableIRQ(I2C0_IRQn);
				Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling);

				ProbeI2CSlaves();

				printf("======== I2C intialized ========\r\n");
			}

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

} setup;

int main(void)
{
	SystemCoreClockUpdate();

	// Init LPC hardware
	setup.InitGPIO();
	setup.InitUART();
	setup.InitI2C();

	// OLED
	sdd1306.Init();

	while (1) {
		sdd1306.Display();
		sdd1306.Clear(0x00);
		sdd1306.Delay(200);
		sdd1306.Display();
		sdd1306.Clear(0xFF);
		sdd1306.Delay(200);
		//__WFI();
	}

	return 0;
}
