/*Sensor: GY-NEO6MV2-GPS*/
/*Author: Nguyen Khang Huy*/
/*Date: 29/8/2022*/
/*Status: In Progress*/
/*Note: Bug ...*/
/*Pin IO*/
/*VCC: 3.3V*/
/*GND: GND*/
/*TX: PB0*/
/*RX: PB1*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/pin_map.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/i2c.h"

//#include "utils/uartstdio.h"
#include "uartstdio.h"
#include <string.h>
#include "BH1750.h"


unsigned short data;
//char Data;
//uint8_t data_high, data_low;

int main()
     	{
		 /* Initial and configure Uart 0 */
		 SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
		 //SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		 GPIOPinConfigure(GPIO_PA0_U0RX);
		 GPIOPinConfigure(GPIO_PA1_U0TX);
		 GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		 UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
		 //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

		 UARTStdioConfig(0, 9600, SysCtlClockGet());

		 /* Configure Uart 1 */
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);			 //Configure Port B
		 GPIOPinConfigure(GPIO_PB0_U1RX);
		 GPIOPinConfigure(GPIO_PB1_U1TX);
		 GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		 UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
		 initI2C();
		 bh1750_init();

   while(1)
      {
	   	   i2cReadData(BH1750_ADDR,&data);
		   UARTprintf("Light : %d Lux\n", data);
		   //HWREG(I2C1_BASE + I2C_O_MDR) = 10;
		   //Data = I2CMasterDataGet(I2C1_BASE);
		   //Data = HWREG(I2C1_BASE + I2C_O_MDR);
		   //Data = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7);
		   //UARTprintf("Data: %d\n", Data);
      }
}
