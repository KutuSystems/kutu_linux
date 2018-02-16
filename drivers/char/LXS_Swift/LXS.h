/*
 *  LXS_Capture.h -- Register definitions for LXS_Capture implementation
 *
 *  Greg Smart
 *
 *  Version 0.2 29/04/16
 *
 */

#ifndef _LXS_Capture_H
#define _LXS_Capture_H

#define LXS_VERSION_MAJOR 1
#define LXS_VERSION_MINOR 1

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
#define USE_BRAM                 0x1000
#define CHANNEL_SELECT           0x3f0000
#define ARM_SYNC                 0x1000000
#define USE_GATE                 0x2000000
#define LOOP_ACTIVE              0x4000000

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

#define LXS_DEBUG_READ        1
#define LXS_DEBUG_WRITE       2
#define LXS_DEBUG_DMA_READ    3
#define LXS_DEBUG_DMA_WRITE   4

/* SPI definitions
*/
#define SPI_DEVICE_ADC0       0
#define SPI_DEVICE_ADC1       1
#define SPI_DEVICE_LMK        2

#define SPI_DEVICE_WRITE      0
#define SPI_DEVICE_READ       1


enum LXS_Capture_user_cmds
{
   LXS_USER_RESET,
   LXS_USER_DMA_RESET,
   LXS_USER_SET_MODE,
   LXS_USER_SET_ADDRESS,
   LXS_USER_TRIG_PPS,
   LXS_USER_SPI_ACCESS,
   LXS_USER_STATUS,
   LXS_USER_SET_INTERRUPT,
   LXS_USER_GET_SEM,
   LXS_USER_SET_SEM,
   LXS_USER_REG_DEBUG,
   LXS_USER_INIT_LMK0482X,
   LXS_USER_WRITE_BRAM,
   LXS_USER_READ_BRAM,
   LXS_USER_VERSION,
   LXS_USER_FPGA_VERSION,
   LXS_USER_SET_INPUT_SCALE,
   LXS_USER_READ_GPIO
};

/*
 *  struct LXS_registers.
 *  This structure points to the first block where the registers are located
 */

struct LXS_cmd_struct {
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

struct LXS_pn9_struct {
   __u32                            command;
   __u32                            status[5];
} ;

struct LXS_scale_struct {
   __u32                            channel;
   __s32                            offset;
   __u32                            gain;
} ;

struct LXS_bram_struct {
   __u32                            address;
   __u32                            value;
} ;


/*
** Structure to set and clear bits for the following ioctl commands.
**   LXS_USER_MODIFY_LEDS
**   LXS_USER_MODIFY_CTRL
*/
typedef struct LXS_bit_flag_struct {
   __u32                            set;
   __u32                            clear;
} LXS_bit_flag_t;


struct LXS_spi_cmd_struct {
   __u32                           device;
   __u32                           addr;
   __u32                           data;
   __u32                           rd_nwr;
} ;

struct LXS_debug_struct {
   __u32                           cmd;
   __u32                           reg;
   __u32                           data;
} ;

struct LXS_gpio_struct {
   __u32                           data0;
   __u32                           data1;
} ;



#define LXS_IOCTL_BASE	't'

#define LXS_USER_RESET              _IOWR(LXS_IOCTL_BASE, 0x80, struct LXS_cmd_struct)
#define LXS_USER_DMA_RESET          _IOWR(LXS_IOCTL_BASE, 0x81, struct LXS_cmd_struct)
#define LXS_USER_SET_MODE           _IOWR(LXS_IOCTL_BASE, 0x82, struct LXS_cmd_struct)
#define LXS_USER_SET_ADDRESS        _IOWR(LXS_IOCTL_BASE, 0x83, struct LXS_cmd_struct)
#define LXS_USER_TRIG_PPS           _IOWR(LXS_IOCTL_BASE, 0x84, struct LXS_cmd_struct)
#define LXS_USER_SPI_ACCESS         _IOWR(LXS_IOCTL_BASE, 0x85, struct LXS_cmd_struct)
#define LXS_USER_STATUS             _IOWR(LXS_IOCTL_BASE, 0x86, struct LXS_cmd_struct)
#define LXS_USER_SET_INTERRUPT      _IOWR(LXS_IOCTL_BASE, 0x87, struct LXS_cmd_struct)
#define LXS_USER_GET_SEM            _IOWR(LXS_IOCTL_BASE, 0x88, struct LXS_cmd_struct)
#define LXS_USER_SET_SEM            _IOWR(LXS_IOCTL_BASE, 0x89, struct LXS_cmd_struct)
#define LXS_USER_REG_DEBUG          _IOWR(LXS_IOCTL_BASE, 0x8a, struct LXS_cmd_struct)
#define LXS_USER_INIT_LMK0482X      _IOWR(LXS_IOCTL_BASE, 0x8b, struct LXS_cmd_struct)
#define LXS_USER_WRITE_BRAM         _IOWR(LXS_IOCTL_BASE, 0x8c, struct LXS_cmd_struct)
#define LXS_USER_READ_BRAM          _IOWR(LXS_IOCTL_BASE, 0x8d, struct LXS_cmd_struct)
#define LXS_USER_VERSION            _IOWR(LXS_IOCTL_BASE, 0x8e, struct LXS_cmd_struct)
#define LXS_USER_FPGA_VERSION       _IOWR(LXS_IOCTL_BASE, 0x8f, struct LXS_cmd_struct)
#define LXS_USER_SET_INPUT_SCALE    _IOWR(LXS_IOCTL_BASE, 0x90, struct LXS_pn9_struct)
#define LXS_USER_READ_GPIO          _IOWR(LXS_IOCTL_BASE, 0x91, struct LXS_gpio_struct)

#endif /* _LXS_H */
