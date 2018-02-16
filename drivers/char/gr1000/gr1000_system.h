/*
 *  gr1000_system.h -- Register definitions for GR1000 implementation
 *
 *  Greg Smart
 *
 *  Version 0.1 10/03/14
 *
 */

/*--------------------------------------------------------------------
 *
 *  GR1000 register include file
 *  This file contains all the macros and defines needed by the driver
 *  and user programs using it.
 *
 *--------------------------------------------------------------------*/

#ifndef _GR1000_SYSTEM_H
#define _GR1000_SYSTEM_H

/* general register memory location */

#define GR1000_BASE              0x43C00000

#define R_GR1000_REG_BASE        0x0000
#define R_GR1000_FIFO_BASE       0x0800
#define R_GR1000_BRAM_BASE       0x1000

#define R_GR1000_STATUS          0x0000

#define R_DMA_WRITE_ADDR         0x0000
#define R_DMA_READ_ADDR          0x0004
#define R_DMA_SIZE_ADDR          0x0008
#define R_MODE_CONFIG_ADDR       0x000C
#define R_INTERRUPT_ADDR         0x0010

#define R_SPI_DATA_ADDR          0x0014   // read address on 64 byte boundaries
#define R_SPI_DEVICE_ADDR        0x0018
#define R_CAPTURE_COUNT_ADDR     0x001C

#define GR1000_REG_BASE          (GR1000_BASE + R_GR1000_REG_BASE)
#define GR1000_FIFO_BASE         (GR1000_BASE + R_GR1000_FIFO_BASE)
#define GR1000_BRAM_BASE         (GR1000_BASE + R_GR1000_BRAM_BASE)

#define GR1000_STATUS            (GR1000_BASE + R_GR1000_STATUS)

#define DMA_WRITE_ADDR           (GR1000_BASE + R_DMA_WRITE_ADDR)
#define DMA_READ_ADDR            (GR1000_BASE + R_DMA_READ_ADDR)
#define MODE_CONFIG_ADDR         (GR1000_BASE + R_MODE_CONFIG_ADDR)
#define INTERRUPT_ADDR           (GR1000_BASE + R_INTERRUPT_ADDR)
#define SPI_DATA_ADDR            (GR1000_BASE + R_SPI_DATA_ADDR)
#define SPI_DEVICE_ADDR          (GR1000_BASE + R_SPI_DEVICE_ADDR)
#define SPI_CAPTURE_COUNT_ADDR   (GR1000_BASE + R_SPI_DEVICE_ADDR)

/*
** configuration constants
*/
#define GENERATE_PPS             0x01
#define DEBUG_START_DMA          0x02
#define DMA_HALT                 0x04
#define DMA_RESET                0x08
#define FPGA_RESET               0x10
#define ADC_TEST_DATA            0x20
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
#define ENABLE_INTERRUPT         0x01
#define DISABLE_INTERRUPT        0x02
#define CLEAR_INTERRUPT          0x03

/*
** SPI constants
*/
#define SPI_PORT_LOW             0x01
#define SPI_PORT_HIGH            0x02
#define SPI_PORT_BOTH            0x03

/*
** Status constants
*/
#define STAT_SPI_BUSY            0x01
#define STAT_S2MM_ERR            0x02
#define STAT_MM2S_RD_CMPLT       0x04
#define STAT_MM2S_ERR            0x08
#define STAT_SPI_ERR             0x10
#define STAT_INTERRUPT_ACTIVE    0x20

#define SPI_MAX_WAIT_COUNT 1000000
#define MAX_WAIT_COUNT     10000


/*
 *  struct GR1000_axi_lite_base.
 *  This structure provides general access for the 3 blocks within the system
 */

struct GR1000_axi_lite_base {
   u32                           registers[512];
   u32                           fifo[512];
   u32                           bram[1024];
} ;

/*
 *  struct GR1000_registers.
 *  This structure points to the first block where the registers are located
 */

struct GR1000_registers {
   u32                           pulse_rate_gen;      // 0x0
   u32                           pulse_width_gen;     // 0x4
   u32                           sweep_test;          // 0x8
   u32                           run_test;            // 0xC
   u32                           data_read_addr;      // 0x10
   u32                           data_read_stride;    // 0x14
} ;

#define MAX_DEVICES     4

/**
 * struct gr1000_drvdata - Device Configuration driver structure
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
struct gr1000_drvdata {
   struct device *dev;
   struct cdev cdev;
   dev_t devt;
   struct class *class;
   int irq;
   uint32_t irq_count;
   int dma_block_count;
   struct clk *clk;
   volatile bool dma_done;
   volatile int error_status;
   bool is_open;
   struct mutex mutex;
   spinlock_t lock;
   void __iomem *base;
   uint32_t config_state;
   char *dma_addr;
   dma_addr_t dma_handle;
	struct list_head dev_list;
};

static inline void gr1000_write_reg(struct gr1000_drvdata *gr1000, unsigned int reg, uint32_t val)
{
	writel(val, gr1000->base + reg);
}

static inline uint32_t gr1000_read_reg(struct gr1000_drvdata *gr1000, unsigned int reg)
{
	return(readl(gr1000->base + reg));
}

//
// GR1000_Status()
//
// read the FOS status registers
//
// status if system is running test, sweep or is idle
//
static inline u32 GR1000_Status(struct gr1000_drvdata *gr1000)
{
   u32 status;

   status = gr1000_read_reg(gr1000, R_GR1000_STATUS);
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
// GR1000_Open()
//
// Open GR1000 system and mmap registers for user access.
//
// Returns pointer to virtual address.
//
u32 GR1000_Open(u32 init_fpga);

//
// GR1000_Close()
//
// Close GR1000 system and unmap memory.  Function is called with
// virtual address that was returned when opened.
//
u32 GR1000_Close(int fd);

//
// GR1000_Set_User_Mode()
//
// Setup the system configuration
//
int GR1000_Set_User_Mode(struct gr1000_drvdata *gr1000, u32 arg);


//
// GR1000_Run Test()
//
// Initiate a COTDR, BOTBA or ROTDR test
//
// Function will return immediately after initiating test
// before test completion
//
int GR1000_Run_Scan(struct gr1000_drvdata *gr1000, void *user_ptr);

//
// GR1000_SPI_Write()
//
// Write a command to SPI port
//
int GR1000_SPI_Write(struct gr1000_drvdata *gr1000, void *user_ptr);


#endif /* _GR1000_SYSTEM_H */
