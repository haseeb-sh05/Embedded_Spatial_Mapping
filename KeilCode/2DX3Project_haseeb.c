/*  Modified code from Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 28, 2025 
						by Haseeb Shaikh
							- major modifications made to meet final project requirmenets 

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include "math.h"

#include <stdio.h>



#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

// Global variable for LED toggle state
volatile uint8_t LED_State = 0;
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

// Initialize Motor Pins (H0-H3)
void PortH_Init(void){
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;		// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        			// configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;     				// disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;        				// enable digital I/O on Port H pins (PH0-PH3)
																									// configure Port H as GPIO
  GPIO_PORTH_AMSEL_R &= ~0x0F;     				// disable analog functionality on Port H	pins (PH0-PH3)	
	return;
}

// Initialize onboard LEDs
void PortN_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	// allow time for clock to stabilize
		GPIO_PORTN_DIR_R |= 0x03;        								// make PN0:1 out (PN0 on-board LED D2; PN1 on-board LED D1)
		GPIO_PORTN_AFSEL_R &= ~0x03;     								// disable alt funct on PN0:1
		GPIO_PORTN_DEN_R |= 0x03;        								// enable digital I/O on PN0:1
    //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
		GPIO_PORTN_AMSEL_R &= ~0x03;     								// disable analog functionality on PN0:1	
	return;
}

void PortF_Init(void){
	//Use PortF pins (PF0 and PF4) for ouput
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;		// activate clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	// allow time for clock to stabilize
	GPIO_PORTF_DIR_R |= 0x11;        			// configure Port F pins (PF0 and PF4) as output
  GPIO_PORTF_AFSEL_R &= ~0x11;     				// disable alt funct on Port F pins (PF0 and PF4)
  GPIO_PORTF_DEN_R |= 0x11;        				// enable digital I/O on Port F pins (PF0 and PF4)
																									// configure Port F as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;     				// disable analog functionality on Port F	pins (PF0 and PF4)	
	return;
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// Global variable visible in Watch window of debugger
// Increments at least once per button press
volatile unsigned long FallingEdges = 0;

// Give clock to Port J and initalize PJ1 as Digital Input GPIO
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};			// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    									// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     									// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 						// Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;								// Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;									// Enable weak pull up resistor on PJ1
}


// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
	
		FallingEdges = 0;             										// Initialize counter

		GPIO_PORTJ_IS_R &= ~0x03;   										// (Step 1) PJ1 is Edge-sensitive 
		GPIO_PORTJ_IBE_R &= ~0x03;  									// PJ1 is not triggered by both edges 
		GPIO_PORTJ_IEV_R &= ~0x03;  									// PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x03; 									// Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x03;  									// Arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000; 									// (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000;								// (Step 4) Set interrupt priority to 5

		EnableInt();												// (Step 3) Enable Global Interrupt. lets go!
}


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
uint16_t wordData;
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum; 
uint8_t RangeStatus;
uint8_t dataReady;

//Define measurement variables 
uint16_t x = 0;
int indexNum = 0;
uint16_t measurements[32] = {0};
uint16_t xValues[32] = {0};

void spinCCW(){																			
	uint32_t delay = 1;			
	
	for(int i=0; i< 512; i++){	//rotating motor CCW to return to home position				
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(delay);			
		GPIO_PORTH_DATA_R = 0b00001100;			
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00000110;			
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00000011;			
		SysTick_Wait1ms(delay);
	}
}

//Spin function to control the motor rotation and measurments 
void spinTOF(){																			// Complete function spin to implement the Full-step Stepping Method
	uint32_t delay = 11;			
	for(int i=0; i< 512; i++)
	{
		if ((i%16) == 0) //take measurement every 11.25 degreess
		{
			while (dataReady == 0)
			{
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
        VL53L1_WaitMs(dev, 5);
			}
		dataReady = 0;
			
	  //do{
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance); 
			FlashLED4(1); //Measurement status LED
		//}while(RangeStatus != 0);
	
		measurements[indexNum] = Distance;
		xValues[indexNum] = x;
	
	  status = VL53L1X_ClearInterrupt(dev); 
		indexNum++;
		}
		
		//rotating motor CW
		GPIO_PORTH_DATA_R = 0b00000011; 
		SysTick_Wait1ms(delay);			
		GPIO_PORTH_DATA_R = 0b00000110;			
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00001100;			
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00001001;			
		SysTick_Wait1ms(delay);
	}
	
	GPIO_PORTN_DATA_R ^= 0b00000010; //Additional LED
	spinCCW();
	GPIO_PORTN_DATA_R ^= 0b00000010; //Additonal LED
	x+= 100; //deafault displacement set to 100 mm
	VL53L1X_StopRanging(dev);
}


// Interrupt Service Routine (ISR) for Port J
void GPIOJ_IRQHandler(void) {
		FallingEdges = FallingEdges + 1;	// Increase the global counter variable ;Observe in Debug Watch Window

		indexNum = 0;
	// 1 Wait for device ToF booted
	while(sensorState==0){
	status = VL53L1X_BootState(dev, &sensorState);
	SysTick_Wait10ms(10);
	}
 
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	/* 2 Initialize the sensor with the default setting  */
	status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);


	/* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
	//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
	//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
	//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

	status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging
	
	spinTOF(); //invoke spin function to start rotating motor and collecting measurements through ToF sensor
	
	//Transmit data
	int input = 0;
	while(1) //only exits after uses presses enter key
		{
			input = UART_InChar(); 
			if (input == 's')
			{
				break;
			}
		}
	for(int i = 0; i < 32; i++) //trasnmit data (32 measurments) to PC via UART
	{		
		FlashLED2(1); //UART transmission Status LED
		sprintf(printf_buffer,"%u,%u\r\n",xValues[i],measurements[i]); 		// print readings to UART
		UART_printf(printf_buffer);
	  SysTick_Wait10ms(50);
  }
	GPIO_PORTJ_ICR_R = 0x02;     					// Acknowledge flag by setting proper bit in ICR register

}



//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

int main(void) {

	//initialize
	PLL_Init();	//bus speed set to 12 MHz
	SysTick_Init();
	PortH_Init();
	PortN_Init();
	PortF_Init();
	PortJ_Init();
	I2C_Init();
	UART_Init();
	onboardLEDs_Init();
	
	// Enable interrupts for PJ1 
	PortJ_Interrupt_Init();
	
	while (1) // Wait for an interrput to be triggered
	{
		WaitForInt();
	}
	return 0;
/*	//uncomment this code and comment everything from PortJ_Init(); to return 0; in main and demonstrate bus speed on AD3
	while(1)
	{
		SysTick_Wait10ms(1);
		GPIO_PORTN_DATA_R ^= 0b100;
	}
	*/
		
}