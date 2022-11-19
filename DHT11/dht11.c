/*Sensor: DHT11*/
/*Author: Nguyen Khang Huy*/
/*Date: 17/9/2022*/
/*Status: Done*/
/*Note: Bug - Cannot run loop while(1) (cause: communicate to DHT11 failed in loop second ): RESOLVED*/
/*Pin IO*/
/*Vcc: 3.3V*/
/*GND*/
/*Data: PF1*/
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

#include "driverlib/ssi.h"
#include "driverlib/systick.h"

//#include "utils/uartstdio.h"
#include "uartstdio.h"
#include <string.h>

char a[100];
int a_int[100];
int i, j;
int do_am;
int nhietdo;
/*Conver string to int*/
void stringtoint(char *kitu, int *number){
	*number = (*kitu - '0');
}
int main()
{
		 /* Initial and configure Uart*/
		 SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		 GPIOPinConfigure(GPIO_PA0_U0RX);
		 GPIOPinConfigure(GPIO_PA1_U0TX);
		 GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		 UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
		 //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
		 UARTStdioConfig(0, 115200, SysCtlClockGet());
		 //Configure Port F
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		 //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1); 	// Chân Data
		 //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

		  /* Cau hinh timer 2	*/
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
		 TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
		 TimerEnable(TIMER2_BASE,TIMER_A);
   while(1)
      {
	   	   	  do_am=0;
	   	   	  nhietdo=0;
	   	   	  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
	   	   	  GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);   //Keo chan Data xuong 0
	          SysCtlDelay(25*(SysCtlClockGet()/3/1000));
	          GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);   //Dua chan Data len 1
	          GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
	          SysCtlDelay(40*(SysCtlClockGet()/3/1000000));
	          //UARTprintf("Start");
	          if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1){
	        	  //UARTCharPut(UART0_BASE, 'F');
	        	  continue;
	          }
	          /*else {
				  UARTCharPut(UART0_BASE, 'P');
			  }*/
	          //while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1);	//Cho den khi DHT11 keo chan Data xuong muc thap
	          //GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);

	          while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == 0);	//Cho den khi DHT11 keo chan Data len muc cao
	          /*if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1){
				  UARTCharPut(UART0_BASE, 'H');
			  }
			  else {
				  //UARTCharPut(UART0_BASE, 'L');
				  continue;
			  }*/
	          if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == 0){
				  continue;
			  }

	          //GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
	          //while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1);	//Cho den khi DHT11 keo chan Data xuong muc thap trong vong 80us
	          SysCtlDelay(80*(SysCtlClockGet()/3/1000000));	//Delay 80us
	          //while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1);	//Cho den khi DHT11 keo chan Data xuong muc thap
	          for(i=0;i<40;i++){
			   //SysCtlDelay(5*(SysCtlClockGet()/3/1000000)); //Delay 5us
	           //UARTCharPut(UART0_BASE, '4');
	   		   while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) != GPIO_PIN_1);	//Cho den khi DHT11 keo chan Data len muc cao
	   		   SysCtlDelay(50*(SysCtlClockGet()/3/1000000)); //Delay 50us
	        	 //UARTCharPut(UART0_BASE, j);
	   		   if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == 0){
	   			   a[i] = '0';
	   			   //UARTCharPut(UART0_BASE,a[i]);
	   			   SysCtlDelay(10*(SysCtlClockGet()/3/1000000));
	   		   }
	   		   else{
	   			   a[i] = '1';

	   			   //UARTCharPut(UART0_BASE,a[i]);
	   			   //while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1);	//Cho den khi DHT11 keo chan Data xuong muc thap --------- bug tại đây
	   			   SysCtlDelay(20*(SysCtlClockGet()/3/1000000));
	   		   }
	   		   stringtoint(&a[i],&a_int[i]);
	   		   //UARTCharPut(UART0_BASE,a[i]); 		//Print chuoi doc duoc tu xung dht11
	          }

	          //UARTCharPut(UART0_BASE,'E');
	          int tmp=7;
	          for(j=0;j<8;j++){
	        	  do_am += (pow(2,tmp))*(a_int[j]);
	        	  nhietdo += (pow(2,tmp))*(a_int[j+16]);
	        	  tmp--;
	          }
	          UARTprintf(" DHT11: ");
	          UARTprintf(" Do am: %d ",do_am);
	          UARTprintf(" Nhiet do: %d \n",nhietdo);

		 }

}

