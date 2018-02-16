/*
 *  LXS_system.h -- Register definitions for LXS implementation
 *
 *  Greg Smart
 *
 *  Version 0.1 11/11/15
 *
 */

/*--------------------------------------------------------------------
 *
 *  LXS register include file
 *  This file contains all the macros and defines needed by the driver
 *  and user programs using it.
 *
 *--------------------------------------------------------------------*/

#ifndef _LXS_SYSTEM_H
#define _LXS_SYSTEM_H

/* general register memory location */

#define LXS_BASE                 0x43C00000
#define LXS_LMK_BASE             0x43C10000
#define LXS_ADC_BASE             0x43C20000

#define R_LXS_REG_BASE           0x0000
#define R_LXS_SPI_BASE           0x0800

#define R_LXS_STATUS             0x0000
#define R_SPI_READ_ADDR          0x0800

#define R_BRAM_ADDR              0x2000

#define SPI_ADC0_ADDR            0x0000
#define SPI_ADC1_ADDR            0x0004
#define SPI_LMK_ADDR             0x0008
#define SPI_STATUS_ADDR          0x000C


#define R_MODE_CONFIG_ADDR       0x0000
#define R_INTERRUPT_ADDR         0x0004
#define R_HISTOGRAM_RATE_ADDR    0x0008
#define R_HISTOGRAM_NUM_ADDR     0x000C

#define R_CAPTURE_COUNT_ADDR     0x0010

#define R_CAPT_DMA_ADDR          0x0018
#define R_CAPT_SIZE_ADDR         0x001C

#define R_HIST_DMA_ADDR          0x0020
#define R_HIST_SIZE_ADDR         0x0024

#define R_TEST_DMA_ADDR          0x0028
#define R_TEST_SIZE_ADDR         0x002C

#define R_ALPHA_ADDR             0x0030
#define R_ALPHA_SQ_ADDR          0x0034
#define R_DDT_ADDR               0x0038

#define R_OFFSET_BASE_ADDR       0x0100   // base address for 40 32-bit words
#define R_SCALE_BASE_ADDR        0x0200   // base address for 40 32-bit words

#define R_FPGA_VERSION_ADDR      0x0800

// LMK registers
#define R_SPI_ADC0_ADDR          (0x10000 + SPI_ADC0_ADDR)
#define R_SPI_ADC1_ADDR          (0x10000 + SPI_ADC1_ADDR)
#define R_SPI_LMK_ADDR           (0x10000 + SPI_LMK_ADDR)
#define R_SPI_STATUS_ADDR        (0x10000 + SPI_STATUS_ADDR)
#define R_SPI_CONTROL_ADDR       (0x10000 + SPI_STATUS_ADDR)

#define SPI_CTRL_WRITE        0x0
#define SPI_CTRL_READ         0x40000000

/*
** configuration constants
*/
#define GENERATE_PPS             0x01
#define DEBUG_START_DMA          0x02
#define PPS_DEBUG_MODE           0x40
#define DMA_DEBUG_MODE           0x80

#define MODE_NORMAL              0x00
#define MODE_DMA_DEBUG           (DMA_DEBUG_MODE)
#define MODE_TRIGGER_DMA         (DMA_DEBUG_MODE|DEBUG_START_DMA)
#define MODE_PPS_DEBUG           (PPS_DEBUG_MODE)
#define MODE_TRIGGER_PPS         (PPS_DEBUG_MODE|GENERATE_PPS)

/*
 * interrupt constants
 */
#define K_ENABLE_INTERRUPT       0x01
#define K_DISABLE_INTERRUPT      0x02
#define K_CLEAR_INTERRUPT        0x03

/*
** SPI constants
*/

#define SPI_MAX_WAIT_COUNT 1000000
#define MAX_WAIT_COUNT     10000


/*
 *  struct LXS_axi_lite_base.
 *  This structure provides general access for the 3 blocks within the system
 */

struct LXS_axi_lite_base {
   u32                           registers[512];
   u32                           fifo[512];
   u32                           bram[1024];
} ;

/*
 *  struct LXS_registers.
 *  This structure points to the first block where the registers are located
 */

struct LXS_registers {
   u32                           pulse_rate_gen;      // 0x0
   u32                           pulse_width_gen;     // 0x4
   u32                           sweep_test;          // 0x8
   u32                           run_test;            // 0xC
   u32                           data_read_addr;      // 0x10
   u32                           data_read_stride;    // 0x14
} ;

#define MAX_DEVICES     4

/**
 * struct LXS_drvdata - Device Configuration driver structure
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
 * @is_open: The status bit to LXSicate whether the device is opened
 * @sem: Instance for the mutex
 * @lock: Instance of spinlock
 * @base_address: The virtual device base address of the device registers
 * @is_partial_bitstream: Status bit to indicate partial/full bitstream
 */
struct LXS_drvdata {
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
   void __iomem *gpio_base;
   uint32_t current_control;
   uint32_t config_state;
   uint32_t int_status;
   atomic_t semaphore;
   char *dma_addr;
   dma_addr_t dma_handle;
   struct list_head dev_list;
   wait_queue_head_t irq_wait_queue;
};

static inline void LXS_write_reg(struct LXS_drvdata *LXS, unsigned int reg, uint32_t val)
{
	writel(val, LXS->base + reg);
}

static inline uint32_t LXS_read_reg(struct LXS_drvdata *LXS, unsigned int reg)
{
	return(readl(LXS->base + reg));
}

static inline void LXS_write_gpio(struct LXS_drvdata *LXS, unsigned int reg, uint32_t val)
{
	writel(val, LXS->gpio_base + reg);
}

static inline uint32_t LXS_read_gpio(struct LXS_drvdata *LXS, unsigned int reg)
{
	return(readl(LXS->gpio_base + reg));
}


//
// LXS_Status()
//
// read the FOS status registers
//
// status if system is running test, sweep or is idle
//
static inline u32 LXS_Status(struct LXS_drvdata *LXS)
{
   u32 status;

   status = LXS_read_reg(LXS, R_LXS_STATUS)|LXS->int_status;
//   status &= 0x000001ff;

   return status;
}

//
//  Prototypes
//

//
// LXS_Open()
//
// Open LXS system and mmap registers for user access.
//
// Returns pointer to virtual address.
//
u32 LXS_Open(u32 init_fpga);

//
// LXS_Close()
//
// Close LXS system and unmap memory.  Function is called with
// virtual address that was returned when opened.
//
u32 LXS_Close(int fd);

//
// LXS_Set_User_Mode()
//
// Setup the system configuration
//
int LXS_Set_User_Mode(struct LXS_drvdata *LXS, struct LXS_cmd_struct *cmd);

//
// LXS_Run_Scan()
//
// Initiate a scan
//
// Function will return immediately after initiating test
// before test completion
//
int LXS_Run_Scan(struct LXS_drvdata *LXS, void *user_ptr);

//
// LXS_SPI_Write()
//
// Write a command to SPI port
//
int LXS_SPI_Access(struct LXS_drvdata *LXS, void *user_ptr);

//
// LXS_Write_bram()
//
// Set the user operation mode
//
int LXS_Write_bram(struct LXS_drvdata *LXS, void *user_ptr);

//
// LXS_Read_bram()
//
// Set the user operation mode
//
int LXS_Read_bram(struct LXS_drvdata *LXS, void *user_ptr);


#endif /* _LXS_SYSTEM_H */
