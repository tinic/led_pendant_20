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

#define NO_SX1280
#ifndef NO_SX1280
class SX1280 {
	public:
			// Customization
			
			const uint32_t BUSY_PIN = 0x0000;
			const uint32_t DIO1_PIN = 0x0008;
			const uint32_t DIO2_PIN = 0x0115;
			const uint32_t DIO3_PIN = 0x011F;
			
			const IRQn_Type INT_IRQn = PIN_INT0_IRQn;
			const uint32_t INT_NUM = 0;
			const uint32_t INT_CHN = PININTCH0;

			// Standard values

			const uint32_t AUTO_TX_OFFSET = 0x21;
			const uint32_t MASK_RANGINGMUXSEL = 0xCF;
			const uint32_t DEFAULT_RANGING_FILTER_SIZE = 0x7F;
			const uint32_t MASK_FORCE_PREAMBLELENGTH = 0x8F;
			const uint32_t BLE_ADVERTIZER_ACCESS_ADDRESS = 0x8E89BED6;

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

			SX1280() { }

			void Delay(int32_t ms) const {
				for (volatile uint32_t i = 0; i < ms*2400; i++) {}
			}
			
			void Init(bool pollMode) {
			
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
				Delay( 10 );
				
				Wakeup();
			}
			
			void Reset() {
				disableIRQ();
#if 0 // TODO
				wait_ms( 20 );
				RadioReset.output( );
				RadioReset = 0;
				wait_ms( 50 );
				RadioReset = 1;
				RadioReset.input( ); // Using the internal pull-up
				wait_ms( 20 );
#endif  // #if 0 // TODO
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
				if( GetPacketType( true ) == PACKET_TYPE_RANGING )
				{
					SetRangingRole( RADIO_RANGING_ROLE_MASTER );
				}
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
				if( GetPacketType( true ) == PACKET_TYPE_RANGING )
				{
					SetRangingRole( RADIO_RANGING_ROLE_SLAVE );
				}
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
				this->PacketType = packetType;

				WriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
			}

			RadioPacketTypes GetPacketType( bool returnLocalCopy )
			{
				RadioPacketTypes packetType = PACKET_TYPE_NONE;
				if( returnLocalCopy == false )
				{
					ReadCommand( RADIO_GET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
					if( this->PacketType != packetType )
					{
						this->PacketType = packetType;
					}
				}
				else
				{
					packetType = this->PacketType;
				}
				return packetType;
			}

			void SetRfFrequency( uint32_t rfFrequency )
			{
				uint8_t buf[3];
				uint32_t freq = 0;

				const float XTAL_FREQ = 52000000.f;
				const float FREQ_STEP = ( ( float )( XTAL_FREQ / 262144.f ) );

				freq = ( uint32_t )( ( float ) rfFrequency / ( float ) FREQ_STEP );
				buf[0] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
				buf[1] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
				buf[2] = ( uint8_t )( freq & 0xFF );
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
				if( this->PacketType != modParams->PacketType )
				{
					this->SetPacketType( modParams->PacketType );
				}

				switch( modParams->PacketType )
				{
					case PACKET_TYPE_GFSK:
						buf[0] = modParams->Params.Gfsk.BitrateBandwidth;
						buf[1] = modParams->Params.Gfsk.ModulationIndex;
						buf[2] = modParams->Params.Gfsk.ModulationShaping;
						break;
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						buf[0] = modParams->Params.LoRa.SpreadingFactor;
						buf[1] = modParams->Params.LoRa.Bandwidth;
						buf[2] = modParams->Params.LoRa.CodingRate;
						this->LoRaBandwidth = modParams->Params.LoRa.Bandwidth;
						break;
					case PACKET_TYPE_FLRC:
						buf[0] = modParams->Params.Flrc.BitrateBandwidth;
						buf[1] = modParams->Params.Flrc.CodingRate;
						buf[2] = modParams->Params.Flrc.ModulationShaping;
						break;
					case PACKET_TYPE_BLE:
						buf[0] = modParams->Params.Ble.BitrateBandwidth;
						buf[1] = modParams->Params.Ble.ModulationIndex;
						buf[2] = modParams->Params.Ble.ModulationShaping;
						break;
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
				if( this->PacketType != packetParams->PacketType )
				{
					this->SetPacketType( packetParams->PacketType );
				}

				switch( packetParams->PacketType )
				{
					case PACKET_TYPE_GFSK:
						buf[0] = packetParams->Params.Gfsk.PreambleLength;
						buf[1] = packetParams->Params.Gfsk.SyncWordLength;
						buf[2] = packetParams->Params.Gfsk.SyncWordMatch;
						buf[3] = packetParams->Params.Gfsk.HeaderType;
						buf[4] = packetParams->Params.Gfsk.PayloadLength;
						buf[5] = packetParams->Params.Gfsk.CrcLength;
						buf[6] = packetParams->Params.Gfsk.Whitening;
						break;
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
					case PACKET_TYPE_FLRC:
						buf[0] = packetParams->Params.Flrc.PreambleLength;
						buf[1] = packetParams->Params.Flrc.SyncWordLength;
						buf[2] = packetParams->Params.Flrc.SyncWordMatch;
						buf[3] = packetParams->Params.Flrc.HeaderType;
						buf[4] = packetParams->Params.Flrc.PayloadLength;
						buf[5] = packetParams->Params.Flrc.CrcLength;
						buf[6] = packetParams->Params.Flrc.Whitening;
						break;
					case PACKET_TYPE_BLE:
						buf[0] = packetParams->Params.Ble.ConnectionState;
						buf[1] = packetParams->Params.Ble.CrcLength;
						buf[2] = packetParams->Params.Ble.BleTestPayload;
						buf[3] = packetParams->Params.Ble.Whitening;
						buf[4] = 0;
						buf[5] = 0;
						buf[6] = 0;
						break;
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
				this->WriteRegister( REG_LR_PREAMBLELENGTH, ( this->ReadRegister( REG_LR_PREAMBLELENGTH ) & MASK_FORCE_PREAMBLELENGTH ) | preambleLength );
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

					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						packetStatus->LoRa.RssiPkt = -( status[0] / 2 );
						( status[1] < 128 ) ? ( packetStatus->LoRa.SnrPkt = status[1] / 4 ) : ( packetStatus->LoRa.SnrPkt = ( ( status[1] - 256 ) /4 ) );
						break;

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

			void SendPayload( uint8_t *payload, uint8_t size, TickTime timeout, uint8_t offset )
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
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
						WriteRegister( REG_LR_CRCSEEDBASEADDR, seed, 2 );
						updated = 1;
						break;
					case PACKET_TYPE_BLE:
						this->WriteRegister(0x9c7, seed[2] );
						this->WriteRegister(0x9c8, seed[1] );
						this->WriteRegister(0x9c9, seed[0] );
						updated = 1;
						break;
					default:
						break;
				}
				return updated;
			}

			void SetBleAccessAddress( uint32_t accessAddress )
			{
				this->WriteRegister( REG_LR_BLE_ACCESS_ADDRESS, ( accessAddress >> 24 ) & 0x000000FF );
				this->WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 1, ( accessAddress >> 16 ) & 0x000000FF );
				this->WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 2, ( accessAddress >> 8 ) & 0x000000FF );
				this->WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 3, accessAddress & 0x000000FF );
			}

			void SetBleAdvertizerAccessAddress( void )
			{
				this->SetBleAccessAddress( BLE_ADVERTIZER_ACCESS_ADDRESS );
			}

			void SetCrcPolynomial( uint16_t polynomial )
			{
				uint8_t val[2];

				val[0] = ( uint8_t )( polynomial >> 8 ) & 0xFF;
				val[1] = ( uint8_t )( polynomial  & 0xFF );

				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
						WriteRegister( REG_LR_CRCPOLYBASEADDR, val, 2 );
						break;
					default:
						break;
				}
			}

			void SetWhiteningSeed( uint8_t seed )
			{
				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
					case PACKET_TYPE_BLE:
						WriteRegister( REG_LR_WHITSEEDBASEADDR, seed );
						break;
					default:
						break;
				}
			}

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
						this->SetStandby( STDBY_XOSC );
						this->WriteRegister( 0x97F, this->ReadRegister( 0x97F ) | ( 1 << 1 ) ); // enable LORA modem clock
						WriteRegister( REG_LR_RANGINGRESULTCONFIG, ( ReadRegister( REG_LR_RANGINGRESULTCONFIG ) & MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )resultType ) & 0x03 ) << 4 ) );
						valLsb = ( ( ReadRegister( REG_LR_RANGINGRESULTBASEADDR ) << 16 ) | ( ReadRegister( REG_LR_RANGINGRESULTBASEADDR + 1 ) << 8 ) | ( ReadRegister( REG_LR_RANGINGRESULTBASEADDR + 2 ) ) );
						this->SetStandby( STDBY_RC );

						// Convertion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280
						switch( resultType )
						{
							case RANGING_RESULT_RAW:
								// Convert the ranging LSB to distance in meter
								// The theoretical conversion from register value to distance [m] is given by:
								// distance [m] = ( complement2( register ) * 150 ) / ( 2^12 * bandwidth[MHz] ) )
								// The API provide BW in [Hz] so the implemented formula is complement2( register ) / bandwidth[Hz] * A,
								// where A = 150 / (2^12 / 1e6) = 36621.09
								val = ( double )complement2( valLsb, 24 ) / ( double )this->GetLoRaBandwidth( ) * 36621.09375;
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

			double GetFrequencyError( )
			{
				uint8_t efeRaw[3] = {0};
				uint32_t efe = 0;
				double efeHz = 0.0;

				switch( this->GetPacketType( true ) )
				{
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						efeRaw[0] = this->ReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
						efeRaw[1] = this->ReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
						efeRaw[2] = this->ReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
						efe = ( efeRaw[0]<<16 ) | ( efeRaw[1]<<8 ) | efeRaw[2];
						efe &= REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

						efeHz = 1.55 * ( double )complement2( efe, 20 ) / ( 1600.0 / ( double )this->GetLoRaBandwidth( ) * 1000.0 );
						break;

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
				this->PollingMode = true;
			}

			int32_t GetLoRaBandwidth( )
			{
				int32_t bwValue = 0;

				switch( this->LoRaBandwidth ) {
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

			void SetInterruptMode( void )
			{
				this->PollingMode = false;
			}

			void OnDioIrq( void )
			{
				/*
				 * When polling mode is activated, it is up to the application to call
				 * ProcessIrqs( ). Otherwise, the driver automatically calls ProcessIrqs( )
				 * on radio interrupt.
				 */
				if( this->PollingMode == true ) {
					this->IrqState = true;
				} else {
					this->ProcessIrqs( );
				}
			}

			void ProcessIrqs( void )
			{
				RadioPacketTypes packetType = PACKET_TYPE_NONE;

				if( this->PollingMode == true ) {
					if( this->IrqState == true ) {
			            disableIRQ( );
						this->IrqState = false;
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
					default:
						// Unexpected IRQ: silently returns
						break;
				}
			}

			void txDone() {
				// TODO
			}

			void rxDone() {
				// TODO
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

    RadioOperatingModes OperatingMode = MODE_SLEEP;
	RadioPacketTypes PacketType = PACKET_TYPE_NONE;
    RadioLoRaBandwidths LoRaBandwidth = LORA_BW_0200;
    bool IrqState = false;
    bool PollingMode = true;

} sx1280;

extern "C" {
	void PIN_INT0_IRQHandler(void) {
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
