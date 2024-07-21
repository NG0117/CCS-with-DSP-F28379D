#include "F28x_Project.h"

#define DELAY (CPU_RATE/1000*6*510)

volatile Uint32 Xint1Count;
Uint32 LoopCount;
int i =0;

interrupt void xint1_isr(void);

void main(void)
{
   Uint32 TempX1Count;

   InitSysCtrl();
   InitGpio();
   DINT;
   InitPieCtrl();
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();


   EALLOW;
   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;  // GPIO模式
   GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;   // 設置為輸出
   GpioDataRegs.GPACLEAR.bit.GPIO7 = 1; // 初始化輸出狀態為低電平
   EDIS;

   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.XINT1_INT = &xint1_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers


   Xint1Count = 0; // Count XINT1 interrupts
   LoopCount = 0;  // Count times through idle loop

   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
   IER |= M_INT1;                              // Enable CPU INT1
   EINT;                                       // Enable Global Interrupts

//
// GPIO0 is XINT1, GPIO1 is XINT2
//
   GPIO_SetupXINT1Gpio(6);


//
// Configure XINT1
//
   XintRegs.XINT1CR.bit.POLARITY = 0;          // Falling edge interrupt


//
// Enable XINT1 and XINT2
//
   XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1


//
// Step 5. IDLE loop:
//
   for(;;)
   {
      TempX1Count = Xint1Count;


      //
      // Trigger both XINT1
      //
      //GpioDataRegs.GPBSET.bit.GPIO34 = 1;      // GPIO34 is high
      //GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;    // Lower GPIO30, trigger XINT1
      while(Xint1Count == TempX1Count) {}

      //
      // Check that the counts were incremented properly and get ready
      // to start over.
      //
      if(Xint1Count == TempX1Count + 1 )
      {
          LoopCount++;
          //GpioDataRegs.GPASET.bit.GPIO30 = 1;   // raise GPIO30
          //GpioDataRegs.GPACLEAR.bit.GPIO31 = 1; // lower GPIO31
      }
      else
      {
        // asm("      ESTOP0"); // stop here
      }
   }
}

//
// xint1_isr - External Interrupt 1 ISR
//
interrupt void xint1_isr(void)
{
    //GpioDataRegs.GPBCLEAR.all = 0x4;   // GPIO34 is low
    Xint1Count++;
    GpioDataRegs.GPATOGGLE.bit.GPIO7 = 1;
    i;
    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


//
// End of file
//
