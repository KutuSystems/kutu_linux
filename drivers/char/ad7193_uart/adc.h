/*
 *  ADC.h -- Register definitions for ADC implementation
 *
 *  Haig Parseghian
 *
 *  Version 1.0 9/04/15
 *
 */

#ifndef _ADC_H
#define _ADC_H


/*
** System constants
*/
//uart rx channel address
#define UART_RX0_ADDR            0x00
#define UART_RX1_ADDR            0x10
#define UART_RX2_ADDR            0x20
#define UART_RX3_ADDR            0x30
#define UART_RX4_ADDR            0x40
#define UART_RX5_ADDR            0x50
#define UART_RX6_ADDR            0x60
#define UART_RX7_ADDR            0x70
//uart tx channel address
#define UART_TX0_ADDR            0x04
#define UART_TX1_ADDR            0x14
#define UART_TX2_ADDR            0x24
#define UART_TX3_ADDR            0x34
#define UART_TX4_ADDR            0x44
#define UART_TX5_ADDR            0x54
#define UART_TX6_ADDR            0x64
#define UART_TX7_ADDR            0x74
//uart baud rate address
#define UART_BAUD0_ADDR          0x08
#define UART_BAUD1_ADDR          0x18
#define UART_BAUD2_ADDR          0x28
#define UART_BAUD3_ADDR          0x38
#define UART_BAUD4_ADDR          0x48
#define UART_BAUD5_ADDR          0x58
#define UART_BAUD6_ADDR          0x68
#define UART_BAUD7_ADDR          0x78
//uart status address
#define UART_STATUS0_ADDR        0x0C
#define UART_STATUS1_ADDR        0x1C
#define UART_STATUS2_ADDR        0x2C
#define UART_STATUS3_ADDR        0x3C
#define UART_STATUS4_ADDR        0x4C
#define UART_STATUS5_ADDR        0x5C
#define UART_STATUS6_ADDR        0x6C
#define UART_STATUS7_ADDR        0x7C
//adc channel address
#define ADC_CH0_ADDR             0x80
#define ADC_CH1_ADDR             0x84
#define ADC_CH2_ADDR             0x88
#define ADC_CH3_ADDR             0x8C
#define ADC_CH4_ADDR             0x90
#define ADC_CH5_ADDR             0x94
#define ADC_CH6_ADDR             0x98
#define ADC_CH7_ADDR             0x9C
//adc config address
#define ADC_CONFIG_ADDR          0xC0


struct uart_data_struct{
   __u32	channel;	
   char	uart_rx[16];
   __u32        adc_raw;
};


#define ADC_IOCTL_BASE	'W'

#define UART_RX0         _IOWR(ADC_IOCTL_BASE, 0x40, int)
#define UART_RX1         _IOWR(ADC_IOCTL_BASE, 0x41, int)
#define UART_RX2         _IOWR(ADC_IOCTL_BASE, 0x42, int)
#define UART_RX3         _IOWR(ADC_IOCTL_BASE, 0x43, int)
#define UART_RX4         _IOWR(ADC_IOCTL_BASE, 0x44, int)
#define UART_RX5         _IOWR(ADC_IOCTL_BASE, 0x45, int)
#define UART_RX6         _IOWR(ADC_IOCTL_BASE, 0x46, int)
#define UART_RX7         _IOWR(ADC_IOCTL_BASE, 0x47, int)

#define UART_ADC0         _IOWR(ADC_IOCTL_BASE, 0x50,struct uart_data_struct)
#define UART_ADC1         _IOWR(ADC_IOCTL_BASE, 0x51,struct uart_data_struct)
#define UART_ADC2         _IOWR(ADC_IOCTL_BASE, 0x52,struct uart_data_struct)
#define UART_ADC3         _IOWR(ADC_IOCTL_BASE, 0x53,struct uart_data_struct)
#define UART_ADC4         _IOWR(ADC_IOCTL_BASE, 0x54,struct uart_data_struct)
#define UART_ADC5         _IOWR(ADC_IOCTL_BASE, 0x55,struct uart_data_struct)
#define UART_ADC6         _IOWR(ADC_IOCTL_BASE, 0x56,struct uart_data_struct)
#define UART_ADC7         _IOWR(ADC_IOCTL_BASE, 0x57,struct uart_data_struct)

#define UART_STATUS0     _IOWR(ADC_IOCTL_BASE, 0x60, int)
#define UART_STATUS1     _IOWR(ADC_IOCTL_BASE, 0x61, int)
#define UART_STATUS2     _IOWR(ADC_IOCTL_BASE, 0x62, int)
#define UART_STATUS3     _IOWR(ADC_IOCTL_BASE, 0x63, int)
#define UART_STATUS4     _IOWR(ADC_IOCTL_BASE, 0x64, int)
#define UART_STATUS5     _IOWR(ADC_IOCTL_BASE, 0x65, int)
#define UART_STATUS6     _IOWR(ADC_IOCTL_BASE, 0x66, int)
#define UART_STATUS7     _IOWR(ADC_IOCTL_BASE, 0x67, int)

#define UART_BAUD0       _IOWR(ADC_IOCTL_BASE, 0x70, int)
#define UART_BAUD1       _IOWR(ADC_IOCTL_BASE, 0x71, int)
#define UART_BAUD2       _IOWR(ADC_IOCTL_BASE, 0x72, int)
#define UART_BAUD3       _IOWR(ADC_IOCTL_BASE, 0x73, int)
#define UART_BAUD4       _IOWR(ADC_IOCTL_BASE, 0x74, int)
#define UART_BAUD5       _IOWR(ADC_IOCTL_BASE, 0x75, int)
#define UART_BAUD6       _IOWR(ADC_IOCTL_BASE, 0x76, int)
#define UART_BAUD7       _IOWR(ADC_IOCTL_BASE, 0x77, int)

#define UART_TX0         _IOWR(ADC_IOCTL_BASE, 0x80, int)
#define UART_TX1         _IOWR(ADC_IOCTL_BASE, 0x81, int)
#define UART_TX2         _IOWR(ADC_IOCTL_BASE, 0x82, int)
#define UART_TX3         _IOWR(ADC_IOCTL_BASE, 0x83, int)
#define UART_TX4         _IOWR(ADC_IOCTL_BASE, 0x84, int)
#define UART_TX5         _IOWR(ADC_IOCTL_BASE, 0x85, int)
#define UART_TX6         _IOWR(ADC_IOCTL_BASE, 0x86, int)
#define UART_TX7         _IOWR(ADC_IOCTL_BASE, 0x87, int)

#define ADC_RAW0         _IOWR(ADC_IOCTL_BASE, 0x88, int)
#define ADC_RAW1         _IOWR(ADC_IOCTL_BASE, 0x89, int)
#define ADC_RAW2         _IOWR(ADC_IOCTL_BASE, 0x90, int)
#define ADC_RAW3         _IOWR(ADC_IOCTL_BASE, 0x91, int)
#define ADC_RAW4         _IOWR(ADC_IOCTL_BASE, 0x92, int)
#define ADC_RAW5         _IOWR(ADC_IOCTL_BASE, 0x93, int)
#define ADC_RAW6         _IOWR(ADC_IOCTL_BASE, 0x94, int)
#define ADC_RAW7         _IOWR(ADC_IOCTL_BASE, 0x95, int)

#define ADC_CONFIG       _IOWR(ADC_IOCTL_BASE, 0x96, int)


#endif /* _ADC_H */
