//###########################################################################
//
// FILE:    GpioToggle.c
//
// TITLE:   GPIO toggle test program.
//
//! \addtogroup cpu01_example_list
//! <h1>GPIO toggle test program (GpioToggle)</h1>
//!
//! Three different examples are included. Select the example
//! (data, set/clear or toggle) to execute before compiling using
//! the #define statements found at the top of the code.
//!
//! Toggle all of the GPIO PORT pins
//!
//! The pins can be observed using Oscilloscope.
//!
//
//###########################################################################
// $TI Release: F2837xD Support Library v200 $
// $Release Date: Tue Jun 21 13:00:02 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include "F28x_Project.h"

//
// Select the example to compile in.  Only one example should be set as 1
// the rest should be set as 0.
//
#define EXAMPLE1 0  // Use DATA registers to toggle I/O's
#define EXAMPLE2 1  // Use SET/CLEAR registers to toggle I/O's
#define EXAMPLE3 0  // Use TOGGLE registers to toggle I/O's

//
// Function Prototypes
//
void delay_loop(void);
void Gpio_select(void);
void Gpio_example1(void);
void Gpio_example2(void);
void Gpio_example3(void);


//
// Main
//
void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
   InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
   Gpio_select();

//
// Step 3. Clear all __interrupts and initialize PIE vector table:
// Disable CPU __interrupts
//
   DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
   InitPieCtrl();

//
// Disable CPU __interrupts and clear all CPU __interrupt flags:
//
   IER = 0x0000;
   IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
   InitPieVectTable();

//
// Step 4. User specific code:
//
#if EXAMPLE1

    //
    // This example uses DATA registers to toggle I/O's
    //
    Gpio_example1();

#endif  // - EXAMPLE1

#if EXAMPLE2

    //
    // This example uses SET/CLEAR registers to toggle I/O's
    //
    Gpio_example2();

#endif

#if EXAMPLE3

    //
    // This example uses TOGGLE registers to toggle I/O's
    //
    Gpio_example3();

#endif
}

//
// delay_loop - Delay function
//
void delay_loop()
{
    short i;
    for (i = 0; i < 1000; i++) {}
}

//
// Gpio_example1 - Example 1: Toggle I/Os using DATA registers
//
void Gpio_example1(void)
{
   for(;;)
   {

       GPIO_WritePin(0, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(0, 0);
       DELAY_US(100*500);

       GPIO_WritePin(1, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(1, 0);
       DELAY_US(100*500);

       GPIO_WritePin(2, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(2, 0);
       DELAY_US(100*1000);

       GPIO_WritePin(3, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(3, 0);
       DELAY_US(100*1000);

       GPIO_WritePin(4, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(4, 0);
       DELAY_US(100*1000);

       GPIO_WritePin(5, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(5, 0);
       DELAY_US(100*1000);

       GPIO_WritePin(4, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(4, 0);
       DELAY_US(100*1000);

       GPIO_WritePin(3, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(3, 0);
       DELAY_US(100*1000);

       GPIO_WritePin(2, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(2, 0);
       DELAY_US(100*1000);

       GPIO_WritePin(1, 1);
       DELAY_US(100*1000);
       GPIO_WritePin(1, 0);
       DELAY_US(100*500);

    }
}

//
// Gpio_example2 - Example 2: Toggle I/Os using SET/CLEAR registers
//
void Gpio_example2(void)
{



   for(;;)
   {
       /*
              GpioDataRegs.GPASET.bit.GPIO0 = 1;
              DELAY_US(55000);
              GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
              GpioDataRegs.GPASET.bit.GPIO1 = 1;
              DELAY_US(55000);
              GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
              GpioDataRegs.GPASET.bit.GPIO2 = 1;
              DELAY_US(55000);
              GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
              GpioDataRegs.GPASET.bit.GPIO3 = 1;
              DELAY_US(55000);
              GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
              GpioDataRegs.GPASET.bit.GPIO4 = 1;
              DELAY_US(55000);
              GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
              GpioDataRegs.GPASET.bit.GPIO5 = 1;
              DELAY_US(55000);
              GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
              GpioDataRegs.GPASET.bit.GPIO4 = 1;
              DELAY_US(55000);
              GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
              GpioDataRegs.GPASET.bit.GPIO3 = 1;
              DELAY_US(55000);
              GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
              GpioDataRegs.GPASET.bit.GPIO2 = 1;
              DELAY_US(55000);
              GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
*/

//       GpioDataRegs.GPASET.bit.GPIO0 = 1;
//       GpioDataRegs.GPASET.bit.GPIO2 = 1;
//       GpioDataRegs.GPASET.bit.GPIO4 = 1;
//       GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
//       GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
//       GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
//       DELAY_US(1250);
//       GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
//       GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
//       GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
//       GpioDataRegs.GPASET.bit.GPIO1 = 1;
//       GpioDataRegs.GPASET.bit.GPIO3 = 1;
//       GpioDataRegs.GPASET.bit.GPIO5 = 1;
//       DELAY_US(1250);
       GpioDataRegs.GPATOGGLE.bit.GPIO14 = 1;
//       DELAY_US(250);   //2kHz
       DELAY_US(63);   //8kHz


    }
}
//
// Gpio_example3 - Example 3: Toggle I/Os using TOGGLE registers
//
void Gpio_example3(void)
{
   //
   // Set pins to a known state
   //
   //GpioDataRegs.GPASET.all = 0xFFFFFFFF;
   //GpioDataRegs.GPACLEAR.all = 0xFFFFFFFF;

   //GpioDataRegs.GPBSET.all = 0x00000AAA;
   //GpioDataRegs.GPBCLEAR.all = 0x00001555;

   //
   // Use TOGGLE registers to flip the state of
   // the pins.
   // Any bit set to a 1 will flip state (toggle)
   // Any bit set to a 0 will not toggle.
   //

   for(;;)
   {
      GpioDataRegs.GPATOGGLE.all =0x000000AA;
      DELAY_US(100000);
      GpioDataRegs.GPATOGGLE.all =0x000000FF;
      DELAY_US(100000);
      GpioDataRegs.GPATOGGLE.all =0x000000FF;
      DELAY_US(100000);
      GpioDataRegs.GPATOGGLE.all =0x000000FF;
      DELAY_US(100000);
      GpioDataRegs.GPATOGGLE.all =0x00000055;
      //GpioDataRegs.GPBTOGGLE.all =0x00001FFF;

      delay_loop();
   }
}

//
// Gpio_select - Configure GPIO muxing and pin outputs
//
void Gpio_select(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all = 0x00000000;  // All GPIO
    GpioCtrlRegs.GPAMUX2.all = 0x00000000;  // All GPIO
    GpioCtrlRegs.GPBMUX1.all = 0x00000000;  // All GPIO
    GpioCtrlRegs.GPADIR.all = 0xFFFFFFFF;   // All outputs
    GpioCtrlRegs.GPBDIR.all = 0x00001FFF;   // All outputs
    EDIS;
}

//
// End of file
//
