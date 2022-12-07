#include <stdbool.h>
#include <stdint.h>
#include "stdlib.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/adc.h"

#include "inc/tm4c123gh6pm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "priorities.h"
#include "BH1750.h"

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

/* Create Queue */
xQueueHandle xQueue;
xQueueHandle xQueue1;
xQueueHandle xQueueTemp;
xQueueHandle xQueueHum;

/* Create Semaphore */
xSemaphoreHandle xLightSemaphore;
xSemaphoreHandle xFailLightSemaphore;
xSemaphoreHandle xOffLightSemaphore;
xSemaphoreHandle xOnLightSemaphore;

static void vDHT11Task(void *pvParameters);
static void vUARTTask(void *pvParameters);
static void vBH1750Task(void *pvParameters);
static void vSoildTask(void *pvParameters);
static void vMotorTask(void *pvParameters);
static void vLightTask(void *pvParameters);
static void vLoraTask(void *pvParameters);
static void vPressButtonTask(void *pvParameters);
static void vManualTask(void *pvParameters);
void stringtoint(char *kitu, int *number);
static int lt(int a, int n);

/* Configure UART 0 */
void ConfigureUART0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 9600, SysCtlClockGet());

}

/* Configure UART 1 */
void ConfigureUART1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);            //Configure Port B
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

/* Setup ADC */
void SetupADC(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);
}
int main(){
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    ConfigureUART0();
    ConfigureUART1();
    SetupADC();
    //Configure Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

     /* Configure timer 2   */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
    TimerEnable(TIMER2_BASE,TIMER_A);

    /*Create Queue*/
    xQueue = xQueueCreate( 10, sizeof(long));
    xQueue1 = xQueueCreate( 10, sizeof(long));
    xQueueTemp = xQueueCreate( 10, sizeof(long));
    xQueueHum = xQueueCreate( 10, sizeof(long));

    /*Create binary semaphore*/
    xLightSemaphore = xSemaphoreCreateBinary();
    xFailLightSemaphore = xSemaphoreCreateBinary();
    xOffLightSemaphore = xSemaphoreCreateBinary();
    xOnLightSemaphore = xSemaphoreCreateBinary();

    xTaskCreate( vBH1750Task, "BH1750", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate( vSoildTask, "Soild", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate( vUARTTask, "UART", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
    xTaskCreate( vLightTask, "Light", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate( vPressButtonTask, "Button", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate( vManualTask, "Manual", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate( vDHT11Task, "DHT11", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

    vTaskStartScheduler();
    for(;;)
    {

    }
}

static void vDHT11Task(void *pvParameters){
        const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
        char a[100];
        int a_int[100];
        int i, j;
        int do_am;
        int nhietdo;
        for (;;){
            do_am=0;
            nhietdo=0;
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);   //Keo chan Data xuong 0
            //SysCtlDelay(25*(SysCtlClockGet()/3/1000));
            vTaskDelay(pdMS_TO_TICKS(25));
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);   //Dua chan Data len 1
            GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
            SysCtlDelay(40*(SysCtlClockGet()/3/1000000));
            //vTaskDelay(pdMS_TO_TICKS(40)/1000);
            //UARTprintf("Start");
            if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1){
                //UARTCharPut(UART0_BASE, 'F');
                continue;
            }
            /*else {
                UARTCharPut(UART0_BASE, 'P');
            }*/
            //while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1);  //Cho den khi DHT11 keo chan Data xuong muc thap
            //GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);

            while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == 0); //Cho den khi DHT11 keo chan Data len muc cao
            if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1){
                UARTCharPut(UART0_BASE, 'H');
            }
            else {
                UARTCharPut(UART0_BASE, 'L');
                continue;
            }
            if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == 0){
                continue;
            }

            //GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
            //while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1);  //Cho den khi DHT11 keo chan Data xuong muc thap trong vong 80us
            SysCtlDelay(80*(SysCtlClockGet()/3/1000000)); //Delay 80us
            //vTaskDelay(pdMS_TO_TICKS(80)/1000);
            //while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1);  //Cho den khi DHT11 keo chan Data xuong muc thap
            for(i=0;i<40;i++){
             //UARTCharPut(UART0_BASE, '4');
            while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) != GPIO_PIN_1);   //Cho den khi DHT11 keo chan Data len muc cao
            SysCtlDelay(50*(SysCtlClockGet()/3/1000000)); //Delay 50us
            //vTaskDelay(pdMS_TO_TICKS(50)/1000);
               //UARTCharPut(UART0_BASE, j);
            if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == 0){
                 a[i] = '0';
                 UARTCharPut(UART0_BASE,a[i]);
                 SysCtlDelay(10*(SysCtlClockGet()/3/1000000));
                 //vTaskDelay(pdMS_TO_TICKS(5)/1000);
            }
            else{
                 a[i] = '1';

                 UARTCharPut(UART0_BASE,a[i]);
                 //while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == GPIO_PIN_1); //Cho den khi DHT11 keo chan Data xuong muc thap --------- bug tại đây
                 SysCtlDelay(20*(SysCtlClockGet()/3/1000000));
                 //vTaskDelay(pdMS_TO_TICKS(10)/1000);
            }
            stringtoint(&a[i],&a_int[i]);
             //UARTCharPut(UART0_BASE,a[i]);      //Print chuoi doc duoc tu xung dht11
            }

            //UARTCharPut(UART0_BASE,'E');
            int tmp=7;
            for(j=0;j<8;j++){
                do_am += (lt(2,tmp))*(a_int[j]);
                nhietdo += (lt(2,tmp))*(a_int[j+16]);
                tmp--;
            }
            //UARTprintf(" DHT11: ");
            //UARTprintf("Do am : %d \n", do_am);
            //xQueueSendToBack( xQueueTemp, &do_am, xTicksToWait );
            //xQueueSendToBack( xQueueHum, &nhietdo, xTicksToWait );
       }
}

void stringtoint(char *kitu, int *number)
{
    *number = (*kitu - '0');
}

static void vBH1750Task(void *pvParameters)
{
    const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
    unsigned short data;
    initI2C();
    bh1750_init();
    for(;;){
        i2cReadData(BH1750_ADDR,&data);
        //UARTprintf("Light : %d Lux\n", data);
        xQueueSendToBack( xQueue, &data, xTicksToWait );
        //vTaskDelay(( rand() & 0x1FF ));
        //vTaskDelay(pdMS_TO_TICKS(250));
        if (data <= 100){
            xSemaphoreGive(xLightSemaphore);
        }
        else {
            xSemaphoreGive(xFailLightSemaphore);
        }
    }
}

static void vSoildTask(void *pvParameters)
{
    uint32_t ui32ADC0Value;
    const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
    for(;;)
    {
        ADCIntClear(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC0_BASE, 1);
        while(!ADCIntStatus(ADC0_BASE, 1, false))
        {
        }
       ADCSequenceDataGet(ADC0_BASE, 1, &ui32ADC0Value);
       ui32ADC0Value = ui32ADC0Value >> 2;
       xQueueSendToBack( xQueue1, &ui32ADC0Value, xTicksToWait );
       //UARTprintf("Do am dat: %d\n",ui32ADC0Value);
       //vTaskDelay(( rand() & 0x1FF ));
       //vTaskDelay(pdMS_TO_TICKS(250));
    }

}

static void vUARTTask(void *pvParameters){
    const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
    for(;;){
        unsigned short light, soild, temp, humid;
         vTaskDelay(pdMS_TO_TICKS(250));
         xQueueReceive( xQueue, &light, xTicksToWait );
         xQueueReceive( xQueue1, &soild, xTicksToWait );
         //xQueueReceive( xQueueTemp, &temp, xTicksToWait );
         //xQueueReceive( xQueueHum, &humid, xTicksToWait );

         //   Data was successfully received from the queue, print out the received     value.
         UARTprintf("Light : %d Lux\n", light);
         UARTprintf("Do am dat : %d \n", soild);
         //UARTprintf("Nhiet do : %d oC\n", temp);
         //UARTprintf("Do am : %d \n", humid);

    }
}

static void vLightTask(void *pvParameters){
    const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
    for(;;){
        //xSemaphoreTake( xLightSemaphore, portMAX_DELAY );
        if(xSemaphoreTake( xLightSemaphore, portMAX_DELAY ) == pdTRUE){
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);
        }
        if(xSemaphoreTake( xFailLightSemaphore, portMAX_DELAY ) == pdTRUE){
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0);
        }
    }
}

static void vPressButtonTask(void *pvParameters){
    const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 |GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    /* Configure button*/
    GPIOPadConfigSet( GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 |GPIO_PIN_2 | GPIO_PIN_3, 0);
    for(;;)
    {
        //Check status button
        while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4));
        //SysCtlDelay(100*(SysCtlClockGet()/3/1000));
        vTaskDelay(150/portTICK_PERIOD_MS);         // debounced
        if (!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4))
        {
            vTaskPrioritySet(NULL,3);
            vTaskPrioritySet(vManualTask,4);
            //reverse led state
            if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
            {
                xSemaphoreGive(xOffLightSemaphore);
            }
            else
            {
                xSemaphoreGive(xOnLightSemaphore);
            }
            vTaskPrioritySet(vLightTask,1);
            vTaskPrioritySet(NULL,1);
        }
    }
}

static void vManualTask(void *pvParameters)
{
    const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
    for(;;){
        //xSemaphoreTake( xLightSemaphore, portMAX_DELAY );
        if(xSemaphoreTake( xOffLightSemaphore, portMAX_DELAY ) == pdTRUE)
        {
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0);
        }
        if(xSemaphoreTake( xOnLightSemaphore, portMAX_DELAY ) == pdTRUE)
        {
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);
        }
    }
}

static void vMotorTask(void *pvParameters)
{
    const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
    for(;;){
        if(xSemaphoreTake( xLightSemaphore, portMAX_DELAY ) == pdTRUE)
        {
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,GPIO_PIN_3);
        }
        if(xSemaphoreTake( xFailLightSemaphore, portMAX_DELAY ) == pdTRUE)
        {
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0);
        }
    }
}

static int lt(int a, int n)
{
    if(n == 1) {
        return a;
    } else {
        int temp = pow(a, n/2);
        if(n % 2 == 0)
            return temp * temp;
        else
            return temp * temp * a;
    }
}
