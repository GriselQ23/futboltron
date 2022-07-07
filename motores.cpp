/*********************************************************************
 *
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *  rosserial_tivac chatter tutorial
 *
 *  On this demo your TivaC Launchpad Connected will publish a string
 *  over the topic "/chatter".
 *
 * Full guide: http://wiki.ros.org/rosserial_tivac/Tutorials
 *
 *********************************************************************/

#include <stdbool.h>
#include <stdint.h>
// TivaC specific includes
extern "C"
{
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include "inc/hw_ints.h"
  #include "inc/hw_memmap.h"
  #include "driverlib/interrupt.h"
  #include "driverlib/pin_map.h"
  #include "driverlib/timer.h"
}
// ROS includes
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h> 
// ROS nodehandle
ros::NodeHandle nh;
int counter=0;
uint32_t current_time=0;
float recibe=0;
float direction=0;
//ros::Publisher chatter("number", &float_msg);
//char hello[13] = "Hello world!";

extern void GPIOJ_Interrupt_Handler(void);

uint32_t g_ui32SysClock;
uint8_t pulses=0;
uint8_t carry=0;
float fdata=0;

//void TwistMessage(const geometry_msgs::Twist& msg){
	//float_msg.data = msg.angular.x;
	//fdata= msg.angular.x;
  //if(fdata>0) 
    //{
     //carry=1; 
    //} 
    //else
    //{
      //carry=0;
    //} 
//}
// L0 step 
// L1 dir
void Callback(const geometry_msgs::Point& abc)
{
  recibe=abc.x;
  direction=abc.y;
  if(recibe>0) 
  {
    carry=1;
  } 
  else
  {
    carry=0;
    //SysCtlDelay(200000);
  }

  if(direction>0) 
  {
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,2);
  } 
  else
  {
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,0);
  }
}

void Callback(const geometry_msgs::Point& abc3)
{
  recibe=abc3.x;
  direction=abc3.y;
  if(recibe>0) 
  {
    carry=1;
  } 
  else
  {
    carry=0;
    //SysCtlDelay(200000);
  }

  if(direction>0) 
  {
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,2);
  } 
  else
  {
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,0);
  }
}

void GPIOJ_Interrupt_Handler(void)
{
	int status =0;
	status = GPIOIntStatus(GPIO_PORTJ_BASE,true);
	GPIOIntClear(GPIO_PORTJ_BASE,status);
    counter++;
    //SysCtlDelay(40000000);
}
void
Timer0IntHandler(void)
{
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_0,pulses);
    
    pulses=pulses+carry;
    ROM_IntMasterDisable();
    //chatter.publish(&float_msg);
    ROM_IntMasterEnable();
}
ros::Subscriber<geometry_msgs::Point> subscriptor("Tiva2", &Callback);

//ros::Subscriber<geometry_msgs::Twist> subs_pose("poses",TwistMessage);
int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();

  // Run from the PLL at 120 MHz.
  MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), TM4C129FREQ);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);  

  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0);
  GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_0|GPIO_PIN_1);
  //GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_0);
  GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  TimerLoadSet(TIMER0_BASE, TIMER_A, 100000);
  TimerEnable(TIMER0_BASE, TIMER_A);

  GPIOIntTypeSet(GPIO_PORTJ_BASE,GPIO_INT_PIN_0,GPIO_FALLING_EDGE);
	GPIOIntRegister(GPIO_PORTJ_BASE,GPIOJ_Interrupt_Handler);
	GPIOIntEnable(GPIO_PORTJ_BASE,GPIO_INT_PIN_0);

	ROM_IntEnable(INT_TIMER0A);
	ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntRegister(TIMER0_BASE, TIMER_A,Timer0IntHandler);
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);

  // ROS nodehandle initialization and topic registration
  nh.initNode();
  //nh.advertise(chatter);
  nh.subscribe(subscriptor);
  while (1)
  {
    current_time=TimerValueGet(TIMER0_BASE, TIMER_A);

    // Publish message to be transmitted.
    //int_msg.data = current_time;
    //chatter.publish(&int_msg);

    // Handle all communications and callbacks.
    nh.spinOnce();
    // Delay for a bit.
    nh.getHardware()->delay(100);
  }
}
