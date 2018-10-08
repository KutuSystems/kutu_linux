/*
 *  IND.h -- Register definitions for IND implementation
 *
 *  Greg Smart
 *
 *  Version 0.1 10/03/14
 *
 */

#ifndef _IND_H
#define _IND_H


/*
** configuration constants
*/
#define GENERATE_PPS			(0x0001)
#define DEBUG_START_DMA			(0x0002)
#define DMA_HALT			(0x0004)
#define DMA_RESET			(0x0008)
#define FPGA_RESET			(0x0010)
#define ADC_TEST_DATA_POST_FIFO		(0x0020)
#define PPS_DEBUG_MODE			(0x0040)
#define DMA_DEBUG_MODE			(0x0080)
#define DEBUG_SELECT_CH0		(0x0000)
#define DEBUG_SELECT_CH1		(0x0100)
#define DEBUG_SELECT_CH2		(0x0200)
#define DEBUG_SELECT_CH_OFF		(0x0300)
#define DEBUG_SELECT_ACTIVE		(0x0800)
#define SIGNED_DATA			(0x1000)
#define ADC_TEST_DATA_PRE_FIFO		(0x2000)

#define CONFIG_MODE_MASK		( ~( GENERATE_PPS	\
					   | DEBUG_START_DMA	\
					   | DMA_HALT		\
					   | DMA_RESET		\
					   | FPGA_RESET		\
					) )

#define MODE_NORMAL			(0x00)
#define MODE_DMA_DEBUG			(DMA_DEBUG_MODE)
#define MODE_TRIGGER_DMA		(DMA_DEBUG_MODE | DEBUG_START_DMA)
#define MODE_PPS_DEBUG			(PPS_DEBUG_MODE)
#define MODE_TRIGGER_PPS		(PPS_DEBUG_MODE | GENERATE_PPS)
#define MODE_SYSTEM_HALT		(PPS_DEBUG_MODE)

#define MODE_CH_AUTO			(0x00)
#define MODE_CH_AUTO_INV		(~(DEBUG_SELECT_ACTIVE | DEBUG_SELECT_CH_OFF))
#define MODE_CH_0			(DEBUG_SELECT_ACTIVE | DEBUG_SELECT_CH0)
#define MODE_CH_1			(DEBUG_SELECT_ACTIVE | DEBUG_SELECT_CH1)
#define MODE_CH_2			(DEBUG_SELECT_ACTIVE | DEBUG_SELECT_CH2)
#define MODE_CH_OFF			(DEBUG_SELECT_ACTIVE | DEBUG_SELECT_CH_OFF)

#define MODE_UNSIGNED			(0)
#define MODE_SIGNED			(SIGNED_DATA)

#define PEAK_START_DISABLE		(0x00ffffff)
#define PEAK_STOP_DISABLE		(0x00ffffff)


#define DISABLE_INTERRUPT		(0)
#define ENABLE_INTERRUPT		(1)

#define DMA_LENGTH			(128*1024*1024)

#define IND_DEBUG_READ        	(1)
#define IND_DEBUG_WRITE       	(2)
#define IND_DEBUG_DMA_READ    	(3)
#define IND_DEBUG_DMA_WRITE   	(4)

/*
** Status constants
*/
#define BIT_SPI_BUSY            (0x00000001)
#define BIT_S2MM_ERR            (0x00000002)
#define BIT_MM2S_RD_CMPLT       (0x00000004)
#define BIT_MM2S_ERR            (0x00000008)
#define BIT_SPI_ERR             (0x00000010)
#define BIT_INTERRUPT_ACTIVE    (0x00000020)
#define BIT_FPGA_RESET_STATUS   (0x00000040)
#define BIT_ADC_TEST_STATUS     (0x00000080)
#define BIT_PPS_DEBUG_STATUS    (0x00000100)
#define BIT_DMA_RESET_STATUS    (0x00000200)
#define BIT_DMA_DEBUG_STATUS    (0x00000400)
#define BIT_INTERRUPT_EN_STATUS (0x00000800)

#define STAT_BAT_LOW_STATUS     (0x00001000)
#define STAT_AC_OK_STATUS       (0x00002000)
#define STAT_NOT_RESTART_REQ    (0x00004000)        // PM MCU has requested a restart
#define STAT_NOT_SHUTDOWN_REQ   (0x00008000)        // PM MCU has requested a shutdown

/*
 * CTRL bit definitions
 */
#define CTRL_RESET_3G         	(0x00000001)
#define CTRL_POWER_3G         	(0x00000002)
#define CTRL_EN_SELECT        	(0x00000004)

#define CTRL_NOT_OS_RUNNING     (0x00000008)        // output low to indicate to PM MCU that we are up and running ok
#define CTRL_NOT_SPARE_MCU      (0x00000010)        // a spare signal to PM MCU (could be input or output)?


/*
 * LED definitions
 */
#define LED_RUNNING           	(0x00000001)
#define LED_ALERT             	(0x00000002)
#define LED_SPARE_IND1        	(0x00000004)    // Spare LED on IND1 3G board.
#define LED_PPS_OK            	(0x00000008)
#define LED_3G_OK             	(0x00000010)
#define LED_WS_OK             	(0x00000020)
#define LED_POWER_OK            (0x00000040)
#define LED_BATTERY_OK          (0x00000080)

#define LED_SPARE1_3G_IND2      (0x00000100)
#define LED_SPARE2_3G_IND2      (0x00000200)    // Spare LED on IND2 3G board.
#define LED_SPARE3_3G_IND2      (0x00000400)
#define LED_SPARE4_3G_IND2      (0x00000800)
#define LED_SPARE1_RF_IND2      (0x00001000)
#define LED_SPARE2_RF_IND2      (0x00002000)
#define LED_SPARE3_RF_IND2      (0x00004000)
#define LED_SPARE4_RF_IND2      (0x00008000)

#define LED_DEBUG0_IND2         (0x10000000)
#define LED_DEBUG1_IND2         (0x20000000)
#define LED_DEBUG2_IND2         (0x40000000)
#define LED_DEBUG3_IND2         (0x80000000)

#define LED_SPARE             	((LED_SPARE_IND1) | (LED_SPARE2_3G_IND2))

/*
 * SPI definitions
 */
#define SPI_CTRL_WRITE        	(0x00000000)
#define SPI_CTRL_READ         	(0x00020000)
#define SPI_DEVICE_AD9467     	(0x00000000)
#define SPI_DEVICE_AD9517     	(0x00010000)



enum IND_user_cmds
{
   IND_USER_RESET,
   IND_USER_DMA_RESET,
   IND_USER_SET_MODE,
   IND_USER_SET_ADDRESS,
   IND_USER_DMA_TEST,
   IND_USER_TRIG_PPS,
   IND_USER_SPI_ACCESS,
   IND_USER_STATUS,
   IND_USER_SET_LEDS,			/* use IND_USER_MODIFY_LEDS to set and clear */
   IND_USER_CLEAR_LEDS,			/* use IND_USER_MODIFY_LEDS to set and clear */
   IND_USER_SET_CTRL,			/* use IND_USER_MODIFY_CTRL to set and clear */
   IND_USER_CLEAR_CTRL,			/* use IND_USER_MODIFY_CTRL to set and clear */
   IND_USER_SET_INTERRUPT,
   IND_USER_GET_SEM,
   IND_USER_SET_SEM,
   IND_USER_REG_DEBUG,
   IND_USER_MODIFY_LEDS,
   IND_USER_MODIFY_CTRL,
   IND_USER_READ_MAXMIN,		/* IND_USER_READ_MAXMIN_NORMAL */
   IND_USER_READ_MAXMIN_NORMAL,		/* IND_USER_READ_MAXMIN_NORMAL */
   IND_USER_FPGA_VERSION,
   IND_USER_ADC_CLOCK_COUNT_PER_PPS,
   IND_USER_ADC_OFFSET_SET,
   IND_USER_ADC_OFFSET_GET,
   IND_USER_READ_MAXMIN_SQUARED,
   IND_USER_CAPTURE_INFO_0_GET,
   IND_USER_CAPTURE_INFO_1_GET,
};

/*
 *  Structure to set and clear bits for the following ioctl commands.
 *      IND_USER_MODIFY_LEDS
 *      IND_USER_MODIFY_CTRL
 */
typedef struct IND_bit_flag_struct {
   __u32                            set;
   __u32                            clear;
   __u32                            toggle;
} IND_bit_flag_t;


struct IND_spi_cmd_struct {
   __u32                           port_device[16];
   __u32                           port_addr[16];
   __u32                           port_data[16];
   __u32                           num_spi_writes;
   __u32                           num_spi_reads;
} ;

struct IND_debug_struct {
   __u32                           cmd;
   __u32                           reg;
   __u32                           data;
} ;

struct IND_maxmin_struct {
   // version 1 : peak values and indices.
   __s32                           max_ch0_data;
   __u32                           max_ch0_addr;
   __s32                           min_ch0_data;
   __u32                           min_ch0_addr;
   __s32                           max_ch1_data;
   __u32                           max_ch1_addr;
   __s32                           min_ch1_data;
   __u32                           min_ch1_addr;
   __s32                           max_ch2_data;
   __u32                           max_ch2_addr;
   __s32                           min_ch2_data;
   __u32                           min_ch2_addr;
   // version 2 : add peak counts.
   __u32                           max_ch0_count;
   __u32                           min_ch0_count;
   __u32                           max_ch1_count;
   __u32                           min_ch1_count;
   __u32                           max_ch2_count;
   __u32                           min_ch2_count;
} ;

/*
 *  Structure to get the FPGA version.
 *      reserved -- upper 16 bits.
 *          set to zeros.
 *      version  -- lower 16 bits.
 *          version_major -- bits [15:8]
 *          version_minor -- bits [7:0]
 */
struct IND_fpga_version_struct {
   __u16                           _reserved_0;
   __u8                            major;
   __u8                            minor;
} __packed;

/*
 *  struct IND_cmd_struct.
 *  This structure points to the first block where the registers are located
 */

struct IND_cmd_struct {
   __u32                            config;
   __u32                            interrupt;
   __u32                            address;
   __u32                            capture_count;
   __u32                            delay_count;
   __u32                            peak_detect_start;
   __u32                            peak_detect_end;
   __s32                            adc_offset;
} ;

/*
 *  struct IND_capture_info.
 *  This structure contains info regarding a capture bank (address).
 */

struct IND_capture_info {
	struct timespec			irq_time;
	__u32				int_status;
	__u32				irq_count;
	__u32				semaphore;
	__u32				adc_clock_count_per_pps;
	__u32				bank;
	struct IND_maxmin_struct	maxmin_normal;
	struct IND_maxmin_struct	maxmin_squared;
} ;

/*
 *  IOCTL definitions.
 */

#define IND_IOCTL_BASE	't'

#define IND_USER_RESET                      _IOWR(IND_IOCTL_BASE, 0x80, struct IND_cmd_struct)
#define IND_USER_DMA_RESET                  _IOWR(IND_IOCTL_BASE, 0x81, struct IND_cmd_struct)
#define IND_USER_SET_MODE                   _IOWR(IND_IOCTL_BASE, 0x82, struct IND_cmd_struct)
#define IND_USER_SET_ADDRESS                _IOWR(IND_IOCTL_BASE, 0x83, struct IND_cmd_struct)
#define IND_USER_DMA_TEST                   _IOWR(IND_IOCTL_BASE, 0x84, struct IND_cmd_struct)
#define IND_USER_TRIG_PPS                   _IOWR(IND_IOCTL_BASE, 0x85, struct IND_cmd_struct)
#define IND_USER_SPI_ACCESS                 _IOWR(IND_IOCTL_BASE, 0x86, struct IND_cmd_struct)
#define IND_USER_STATUS                     _IOWR(IND_IOCTL_BASE, 0x87, struct IND_cmd_struct)
#define IND_USER_SET_LEDS                   _IOWR(IND_IOCTL_BASE, 0x88, struct IND_cmd_struct)
#define IND_USER_CLEAR_LEDS                 _IOWR(IND_IOCTL_BASE, 0x89, struct IND_cmd_struct)
#define IND_USER_SET_CTRL                   _IOWR(IND_IOCTL_BASE, 0x8a, struct IND_cmd_struct)
#define IND_USER_CLEAR_CTRL                 _IOWR(IND_IOCTL_BASE, 0x8b, struct IND_cmd_struct)
#define IND_USER_SET_INTERRUPT              _IOWR(IND_IOCTL_BASE, 0x8c, struct IND_cmd_struct)
#define IND_USER_GET_SEM                    _IOWR(IND_IOCTL_BASE, 0x8d, struct IND_cmd_struct)
#define IND_USER_SET_SEM                    _IOWR(IND_IOCTL_BASE, 0x8e, struct IND_cmd_struct)
#define IND_USER_REG_DEBUG                  _IOWR(IND_IOCTL_BASE, 0x8f, struct IND_cmd_struct)
#define IND_USER_MODIFY_LEDS                _IOWR(IND_IOCTL_BASE, 0x90, struct IND_bit_flag_struct)
#define IND_USER_MODIFY_CTRL                _IOWR(IND_IOCTL_BASE, 0x91, struct IND_bit_flag_struct)
#define IND_USER_READ_MAXMIN                _IOR( IND_IOCTL_BASE, 0x92, struct IND_maxmin_struct)
#define IND_USER_READ_MAXMIN_NORMAL         _IOR( IND_IOCTL_BASE, 0x92, struct IND_maxmin_struct)
#define IND_USER_FPGA_VERSION               _IOWR(IND_IOCTL_BASE, 0x93, struct IND_fpga_version_struct)
#define IND_USER_ADC_CLOCK_COUNT_PER_PPS    _IOWR(IND_IOCTL_BASE, 0x94, __u32)
#define IND_USER_ADC_OFFSET_SET             _IOW( IND_IOCTL_BASE, 0x95, __s32)
#define IND_USER_ADC_OFFSET_GET             _IOR( IND_IOCTL_BASE, 0x96, __s32)
#define IND_USER_READ_MAXMIN_SQUARED        _IOR( IND_IOCTL_BASE, 0x97, struct IND_maxmin_struct)
#define IND_USER_CAPTURE_INFO_0_GET         _IOR( IND_IOCTL_BASE, 0x98, struct IND_capture_info)
#define IND_USER_CAPTURE_INFO_1_GET         _IOR( IND_IOCTL_BASE, 0x99, struct IND_capture_info)

#endif /* _IND_H */
