/*
 *  IND_system.h -- Register definitions for IND implementation
 *
 *  Greg Smart
 *
 *  Version 0.1 11/11/15
 *
 */

/*--------------------------------------------------------------------
 *
 *  IND register include file
 *  This file contains all the macros and defines needed by the driver
 *  and user programs using it.
 *
 *--------------------------------------------------------------------*/

#ifndef _IND_SYSTEM_H
#define _IND_SYSTEM_H

/* general register memory location */

#define IND_BASE			0x43C00000

#define R_IND_REG_BASE			0x0000
#define R_IND_SPI_BASE			0x0800
#define R_IND_MAXMIN_BASE		0x1000
#define R_IND_MAXMIN_NORMAL_BASE	(0x1000)	// base address for min/max normal (voltage) signals.
#define R_IND_MAXMIN_SQUARED_BASE	(0x1080)	// base address for min/max squared ("energy") signals.

#define R_IND_STATUS			0x0000
#define R_SPI_READ_ADDR			0x0800

// FPGA user read/write registers.
// address decoder uses bits 5..2 => address offsets of 0x00..0x3C
#define R_DMA_WRITE_ADDR		0x0000
#define R_DMA_READ_ADDR			0x0004
#define R_DMA_SIZE_ADDR			0x0008
#define R_MODE_CONFIG_ADDR		0x000C
#define R_INTERRUPT_ADDR		0x0010

#define R_SPI_DATA_ADDR			0x0014   	// read address on 64 byte boundaries
#define R_SPI_DEVICE_ADDR		0x0018
#define R_CAPTURE_COUNT_ADDR		0x001C
#define R_DELAY_COUNT_ADDR		0x0020
#define R_GPIO_CTRL_ADDR		0x0024
#define R_GPIO_LED_ADDR			0x0028
#define R_PEAK_START_ADDR		0x002C
#define R_PEAK_END_ADDR			0x0030
#define R_ADC_OFFSET			0x0034
#define R_RESERVED_0038			0x0038		// reserved/available for future use
#define R_RESERVED_003C			0x003C		// reserved/available for future use

//----------------------------------------------------------------------------
// field offsets for maxmin structure.
//----------------------------------------------------------------------------

#define R_MAX_CH0_VAL_OFFSET		(0x0000)
#define R_MAX_CH0_LOC_OFFSET		(0x0004)
#define R_MIN_CH0_VAL_OFFSET		(0x0008)
#define R_MIN_CH0_LOC_OFFSET		(0x000C)

#define R_MAX_CH1_VAL_OFFSET		(0x0010)
#define R_MAX_CH1_LOC_OFFSET		(0x0014)
#define R_MIN_CH1_VAL_OFFSET		(0x0018)
#define R_MIN_CH1_LOC_OFFSET		(0x001C)

#define R_MAX_CH2_VAL_OFFSET		(0x0020)
#define R_MAX_CH2_LOC_OFFSET		(0x0024)
#define R_MIN_CH2_VAL_OFFSET		(0x0028)
#define R_MIN_CH2_LOC_OFFSET		(0x002C)

#define R_MIN_CH0_COUNT_OFFSET		(0x0040)
#define R_MAX_CH0_COUNT_OFFSET		(0x0044)

#define R_MIN_CH1_COUNT_OFFSET		(0x0050)
#define R_MAX_CH1_COUNT_OFFSET		(0x0054)

#define R_MIN_CH2_COUNT_OFFSET		(0x0060)
#define R_MAX_CH2_COUNT_OFFSET		(0x0064)

//----------------------------------------------------------------------------

#define R_FPGA_VERSION_ADDR		(0x1800)
#define R_CLOCK_COUNT_PER_PPS_ADDR	(0x2000)

//----------------------------------------------------------------------------

#define IND_REG_BASE             (IND_BASE + R_IND_REG_BASE)
#define IND_SPI_BASE             (IND_BASE + R_IND_SPI_BASE)

#define IND_STATUS               (IND_BASE + R_IND_STATUS)

#define DMA_WRITE_ADDR           (IND_BASE + R_DMA_WRITE_ADDR)
#define DMA_READ_ADDR            (IND_BASE + R_DMA_READ_ADDR)
#define MODE_CONFIG_ADDR         (IND_BASE + R_MODE_CONFIG_ADDR)
#define INTERRUPT_ADDR           (IND_BASE + R_INTERRUPT_ADDR)
#define SPI_DATA_ADDR            (IND_BASE + R_SPI_DATA_ADDR)
#define SPI_DEVICE_ADDR          (IND_BASE + R_SPI_DEVICE_ADDR)
#define SPI_CAPTURE_COUNT_ADDR   (IND_BASE + R_CAPTURE_COUNT_ADDR)

/*
 * interrupt constants
 */
#define K_ENABLE_INTERRUPT       0x01
#define K_DISABLE_INTERRUPT      0x02
#define K_CLEAR_INTERRUPT        0x03

/*
** SPI constants

#define SPI_PORT_LOW             0x01
#define SPI_PORT_HIGH            0x02
#define SPI_PORT_BOTH            0x03
*/

#define SPI_MAX_WAIT_COUNT 1000000
#define MAX_WAIT_COUNT     10000


/*
 *  struct IND_axi_lite_base.
 *  This structure provides general access for the 3 blocks within the system
 */

struct IND_axi_lite_base {
   u32                           registers[512];
   u32                           fifo[512];
   u32                           bram[1024];
} ;

/*
 *  struct IND_registers.
 *  This structure points to the first block where the registers are located
 */

struct IND_registers {
   u32                           pulse_rate_gen;      // 0x0
   u32                           pulse_width_gen;     // 0x4
   u32                           sweep_test;          // 0x8
   u32                           run_test;            // 0xC
   u32                           data_read_addr;      // 0x10
   u32                           data_read_stride;    // 0x14
} ;

#define MAX_DEVICES     4

/**
 * struct IND_drvdata - Device Configuration driver structure
 *
 * @dev: Pointer to the device structure
 * @cdev: Instance of the cdev structure
 * @devt: Pointer to the dev_t structure
 * @class: Pointer to device class
 * @fclk_class: Pointer to fclk device class
 * @dma_done: The dma_done status bit for the DMA command completion
 * @error_status: The error status captured during the DMA transfer
 * @irq: Interrupt number
 * @clk: Peripheral clock for devcfg
 * @fclk: Array holding references to the FPGA clocks
 * @fclk_exported: Flag inidcating whether an FPGA clock is exported
 * @is_open: The status bit to indicate whether the device is opened
 * @sem: Instance for the mutex
 * @lock: Instance of spinlock
 * @base_address: The virtual device base address of the device registers
 * @is_partial_bitstream: Status bit to indicate partial/full bitstream
 */
struct IND_drvdata {
   struct device *dev;
   struct cdev cdev;
   dev_t devt;
   struct class *class;
   int irq;
   atomic_t irq_count;
   int dma_block_count;
   struct clk *clk;
   volatile bool dma_done;
   volatile int error_status;
   bool is_open;
   struct mutex mutex;
   spinlock_t lock;
   void __iomem *base;
   uint32_t config_state;
   uint32_t led_status;
   uint32_t ctrl_status;
   uint32_t int_status;
   atomic_t semaphore;
   char *dma_addr;
   dma_addr_t dma_handle;
   struct list_head dev_list;
   wait_queue_head_t irq_wait_queue;
   struct IND_cmd_struct command;
   uint32_t bank;
   struct IND_capture_info capture_info[2];	/* bank 0, 1 */
};

static inline void IND_write_reg(struct IND_drvdata *IND, size_t reg, uint32_t val)
{
    writel(val, IND->base + reg);
}

static inline uint32_t IND_read_reg(struct IND_drvdata *IND, size_t reg)
{
    return readl(IND->base + reg);
}

//
// IND_Status()
//
// read the FOS status registers
//
// status if system is running test, sweep or is idle
//
static inline u32 IND_Status(struct IND_drvdata *IND)
{
   u32 status;

   status = IND_read_reg(IND, R_IND_STATUS) | IND->int_status;
//   status &= 0x000001ff;

   return status;
}

//
//
//
//
//  Prototypes
//
//
//
//
//

//
// IND_Open()
//
// Open IND system and mmap registers for user access.
//
// Returns pointer to virtual address.
//
u32 IND_Open(u32 init_fpga);

//
// IND_Close()
//
// Close IND system and unmap memory.  Function is called with
// virtual address that was returned when opened.
//
u32 IND_Close(int fd);

//
// IND_Set_User_Mode()
//
// Setup the system configuration
//
int IND_Set_User_Mode(struct IND_drvdata *IND, void *user_ptr);

//
// IND_Run_Scan()
//
// Initiate a scan
//
// Function will return immediately after initiating test
// before test completion
//
int IND_Run_Scan(struct IND_drvdata *IND, void *user_ptr);

//
// IND_SPI_Write()
//
// Write a command to SPI port
//
int IND_SPI_Access(struct IND_drvdata *IND, void *user_ptr);

/*
 * IOCTL to read maxmin registers and copy to user memory.
 */
int IND_Maxmin_Read( struct IND_drvdata *IND, void *user_ptr, size_t base );

/*
 * Read maxmin registers and populate a IND_maxmin structure.
 *
 */
void _ind_maxmin_read( struct IND_maxmin_struct *maxmin, struct IND_drvdata *IND, size_t base );

/*
 * IOCTL to read capture information and copy to user memory.
 */
int IND_capture_info_get( struct IND_drvdata *IND, void *user_ptr, uint32_t bank );

#endif /* _IND_SYSTEM_H */
