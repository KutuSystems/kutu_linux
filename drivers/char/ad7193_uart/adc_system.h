/*
 *  ADC_system.h -- Register definitions for ADC implementation
 *
 *  Haig Parseghian
 *
 *  Version 1.0 9/04/15
 *
 */

/*--------------------------------------------------------------------
 *
 *  ADC register include file
 *  This file contains all the macros and defines needed by the driver
 *  and user programs using it.
 *
 *--------------------------------------------------------------------*/

#ifndef _ADC_SYSTEM_H
#define _ADC_SYSTEM_H

struct adc_drvdata {
   struct device *dev;
   struct cdev cdev;
   dev_t devt;
   struct class *class;
   int irq;
   volatile int error_status;
   bool is_open;
   struct mutex mutex;
   spinlock_t lock;
   void __iomem *base;
   uint32_t run_state;
};

void update_struct(struct adc_drvdata *adc, struct uart_data_struct *U1, int channel);

//bool rx_empty(int addr int channel)

static inline void adc_write_reg(struct adc_drvdata *adc, unsigned int reg, uint32_t val)
{
	writel(val, adc->base + reg);
}

static inline uint32_t adc_read_reg(struct adc_drvdata *adc, unsigned int reg)
{
	return(readl(adc->base + reg));
}

/*bool rx_empty(int addr int channel){
	int status; 
	int i;
	
	i = channel * 16;
	status = adc_read_reg(adc, UART_STATUS0_ADDR(i))
}*/

void update_struct(struct adc_drvdata *adc, struct uart_data_struct *U1, int channel){
	__u32 uart_word = 0;
	__u32 adc_val = 0;
	int i = 0;
	int empty;
	int status;

 	memset(U1->uart_rx, 0, 16);

	U1->channel = channel;
	adc_val  = adc_read_reg(adc, ADC_CH0_ADDR + (channel * 4));
	U1->adc_raw	= adc_val;
	status = adc_read_reg(adc, UART_STATUS0_ADDR + (channel * 16));
	empty = (status>>2)&0x1;

	while(!empty){
		uart_word = adc_read_reg(adc, UART_RX0_ADDR + (channel * 16));
		U1->uart_rx[i]	= uart_word;
		i++;
		status = adc_read_reg(adc, UART_STATUS0_ADDR + (channel * 16));
		empty = (status>>2)&0x1;
	}

return;

}

#endif /* _ADC_SYSTEM_H */
