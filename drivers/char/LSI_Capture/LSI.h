/*
 *  LSI_Capture.h -- Register definitions for LSI_Capture implementation
 *
 *  Greg Smart
 *
 *  Version 0.2 29/04/16
 *
 */

#ifndef _LSI_Capture_H
#define _LSI_Capture_H

#define LSI_VERSION_MAJOR 0
#define LSI_VERSION_MINOR 22

/*
** configuration constants
*/
#define GENERATE_PPS             0x01
#define START_DMA                0x02
#define DMA_HALT                 0x04
#define DMA_RESET                0x08
#define FPGA_RESET               0x10
#define ADC_TEST_DATA            0x20
#define PPS_DEBUG_MODE           0x40
#define SINGLE_MODE              0x80
#define USE_HISTOGRAM            0x100
#define USE_CAPTURE              0x200
#define USE_TEST_DATA            0x400
#define USE_PEAK_DETECT          0x800
#define CHANNEL_SELECT           0x3f0000
#define ARM_SYNC                 0x1000000
#define USE_GATE                 0x2000000

#define DEBUG_LED                (1<<12)
#define FPGA_ID_SHIFT            2
#define FPGA_ID_MASK             0x03
#define SLOT_ID_SHIFT            21
#define SLOT_ID_MASK             0x0f
#define SUBRACK_ID_SHIFT         25
#define SUBRACK_ID_MASK          0x0f

#define CONFIG_MODE_MASK         (START_DMA | ADC_TEST_DATA | PPS_DEBUG_MODE | CHANNEL_SELECT)


#define MODE_NORMAL              0x00
#define MODE_DMA_DEBUG           (DMA_DEBUG_MODE)
#define MODE_TRIGGER_DMA         (DMA_DEBUG_MODE|DEBUG_START_DMA)
#define MODE_PPS_DEBUG           (PPS_DEBUG_MODE)
#define MODE_TRIGGER_PPS         (PPS_DEBUG_MODE|GENERATE_PPS)
#define MODE_SYSTEM_HALT         (PPS_DEBUG_MODE)

#define DISABLE_INTERRUPT        0
#define ENABLE_INTERRUPT         1

/*
** Status constants
*/
#define BIT_GEN_BUSY             0x01
#define BIT_S2MM_ERR             0x02
#define BIT_MM2S_RD_CMPLT        0x04

#define BIT_ADC_ACTIVE           0x10
#define BIT_INTERRUPT_ACTIVE     0x20
#define BIT_FPGA_RESET_STATUS    0x40
#define BIT_ADC_TEST_STATUS      0x80
#define BIT_DMA_RESET_STATUS     0x200
#define BIT_INTERRUPT_EN_STATUS  0x800

#define DMA_LENGTH	(256*1024*1024)

#define LSI_DEBUG_READ        1
#define LSI_DEBUG_WRITE       2
#define LSI_DEBUG_DMA_READ    3
#define LSI_DEBUG_DMA_WRITE   4


/* SPI definitions
*/
#define SPI_CTRL_WRITE        0x0
#define SPI_CTRL_READ         0x00800000
#define SPI_DEVICE_0          0x0
#define SPI_DEVICE_1          0x01000000




enum LSI_Capture_user_cmds
{
   LSI_USER_RESET,
   LSI_USER_DMA_RESET,
   LSI_USER_SET_MODE,
   LSI_USER_SET_ADDRESS,
   LSI_USER_TRIG_PPS,
   LSI_USER_SPI_ACCESS,
   LSI_USER_STATUS,
   LSI_USER_SET_INTERRUPT,
   LSI_USER_GET_SEM,
   LSI_USER_SET_SEM,
   LSI_USER_REG_DEBUG,
   LSI_USER_INIT_LMK03000,
   LSI_USER_WRITE_TAPS,
   LSI_USER_READ_TAPS,
   LSI_USER_VERSION,
   LSI_USER_FPGA_VERSION,
   LSI_USER_SET_PN9_TEST,
   LSI_USER_READ_PN9_TEST,
   LSI_USER_SET_INPUT_SCALE,
   LSI_USER_READ_GPIO
};

/*
 *  struct LSI_registers.
 *  This structure points to the first block where the registers are located
 */

struct LSI_cmd_struct {
   __u32                            config;
   __u32                            interrupt;
   __u32                            capture_address;
   __u32                            histogram_address;
   __u32                            test_data_address;
   __u32                            histogram_count;
   __u32                            histogram_rate;
   __u32                            alpha;
   __u32                            sync_armed;
   __u32                            ddt;
   __u32                            capture_count;
   __u32                            capture_channel;
   __u32                            single_channel;
} ;

struct LSI_pn9_struct {
   __u32                            command;
   __u32                            status[5];
} ;

struct LSI_scale_struct {
   __u32                            channel;
   __s32                            offset;
   __u32                            gain;
} ;


/*
** Structure to set and clear bits for the following ioctl commands.
**   LSI_USER_MODIFY_LEDS
**   LSI_USER_MODIFY_CTRL
*/
typedef struct LSI_bit_flag_struct {
   __u32                            set;
   __u32                            clear;
} LSI_bit_flag_t;


struct LSI_spi_cmd_struct {
   __u32                           port_device[16];
   __u32                           port_addr[16];
   __u32                           port_data[16];
   __u32                           num_spi_writes;
   __u32                           num_spi_reads;
} ;

struct LSI_adc_tap_struct {
   __u32                           clk_taps;
   __u32                           frame_taps;
   __u32                           adc0_0_taps;
   __u32                           adc0_1_taps;
   __u32                           adc1_0_taps;
   __u32                           adc1_1_taps;
   __u32                           adc2_0_taps;
   __u32                           adc2_1_taps;
   __u32                           adc3_0_taps;
   __u32                           adc3_1_taps;
   __u32                           device;
   __u32                           frame_status;
} ;

struct LSI_debug_struct {
   __u32                           cmd;
   __u32                           reg;
   __u32                           data;
} ;

struct LSI_gpio_struct {
   __u32                           data0;
   __u32                           data1;
} ;



#define LSI_IOCTL_BASE	't'

#define LSI_USER_RESET              _IOWR(LSI_IOCTL_BASE, 0x80, struct LSI_cmd_struct)
#define LSI_USER_DMA_RESET          _IOWR(LSI_IOCTL_BASE, 0x81, struct LSI_cmd_struct)
#define LSI_USER_SET_MODE           _IOWR(LSI_IOCTL_BASE, 0x82, struct LSI_cmd_struct)
#define LSI_USER_SET_ADDRESS        _IOWR(LSI_IOCTL_BASE, 0x83, struct LSI_cmd_struct)
#define LSI_USER_TRIG_PPS           _IOWR(LSI_IOCTL_BASE, 0x84, struct LSI_cmd_struct)
#define LSI_USER_SPI_ACCESS         _IOWR(LSI_IOCTL_BASE, 0x85, struct LSI_cmd_struct)
#define LSI_USER_STATUS             _IOWR(LSI_IOCTL_BASE, 0x86, struct LSI_cmd_struct)
#define LSI_USER_SET_INTERRUPT      _IOWR(LSI_IOCTL_BASE, 0x87, struct LSI_cmd_struct)
#define LSI_USER_GET_SEM            _IOWR(LSI_IOCTL_BASE, 0x88, struct LSI_cmd_struct)
#define LSI_USER_SET_SEM            _IOWR(LSI_IOCTL_BASE, 0x89, struct LSI_cmd_struct)
#define LSI_USER_REG_DEBUG          _IOWR(LSI_IOCTL_BASE, 0x8a, struct LSI_cmd_struct)
#define LSI_USER_INIT_LMK03000      _IOWR(LSI_IOCTL_BASE, 0x8b, struct LSI_cmd_struct)
#define LSI_USER_WRITE_TAPS         _IOWR(LSI_IOCTL_BASE, 0x8c, struct LSI_adc_tap_struct)
#define LSI_USER_READ_TAPS          _IOWR(LSI_IOCTL_BASE, 0x8d, struct LSI_adc_tap_struct)
#define LSI_USER_VERSION            _IOWR(LSI_IOCTL_BASE, 0x8e, struct LSI_cmd_struct)
#define LSI_USER_FPGA_VERSION       _IOWR(LSI_IOCTL_BASE, 0x8f, struct LSI_cmd_struct)
#define LSI_USER_SET_PN9_TEST       _IOWR(LSI_IOCTL_BASE, 0x90, struct LSI_pn9_struct)
#define LSI_USER_READ_PN9_TEST      _IOWR(LSI_IOCTL_BASE, 0x91, struct LSI_pn9_struct)
#define LSI_USER_SET_INPUT_SCALE    _IOWR(LSI_IOCTL_BASE, 0x92, struct LSI_pn9_struct)
#define LSI_USER_READ_GPIO          _IOWR(LSI_IOCTL_BASE, 0x93, struct LSI_gpio_struct)

#endif /* _LSI_H */
