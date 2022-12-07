/*Sensor: Grove Sound Sensor*/
/*Author: Nguyen Khang Huy*/
/*Date: 20/11/2022*/
/*Status: In Progress*/
/*Note: Bug ...*/
/*Pin IO*/
/*VCC: 3.3V*/
/*GND: GND*/
/*DATA: PE3*/
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
#include "driverlib/pin_map.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"

#include "driverlib/ssi.h"
#include "driverlib/systick.h"

//#include "utils/uartstdio.h"
#include "uartstdio.h"
#include <string.h>

char a[100];
int i,j;
uint32_t ui32ADC0Value;
int main()
{
		 /* Initial and configure Uart 0 */
		 SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		 GPIOPinConfigure(GPIO_PA0_U0RX);
		 GPIOPinConfigure(GPIO_PA1_U0TX);
		 GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		 UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
		 //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

		 UARTStdioConfig(0, 9600, SysCtlClockGet());

		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

		 /* Setup ADC */
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
		 ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
		 ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
		 ADCSequenceEnable(ADC0_BASE, 1);

   while(1)
      {
			ADCIntClear(ADC0_BASE, 1);
			ADCProcessorTrigger(ADC0_BASE, 1);
			while(!ADCIntStatus(ADC0_BASE, 1, false))
			{
			}
		   ADCSequenceDataGet(ADC0_BASE, 1, &ui32ADC0Value);
		   ui32ADC0Value = ui32ADC0Value >> 2;
		   UARTprintf("Do am dat: %d\n",ui32ADC0Value);
      }
}
