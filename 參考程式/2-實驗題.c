(a)
#include "F28x_Project.h"

__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
int count = 0;

void main(void)
{

    InitSysCtrl();

    InitGpio();
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(3, GPIO_OUTPUT, GPIO_PUSHPULL);

    DINT;

    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    EALLOW;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    EDIS;

    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer1, 200, 50);
    //ConfigCpuTimer(&CpuTimer2, 200, 50);

    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    IER |= M_INT13;
    IER |= M_INT14;

    EINT;  // Enable Global interrupt INTM
    ERTM;

    while(1)
        {

        }


}

__interrupt void cpu_timer1_isr(void)
{
   //CpuTimer1.InterruptCount++;
    count++;
   GpioDataRegs.GPATOGGLE.bit.GPIO2 = 1;

   if (count >= 4) {

       PieCtrlRegs.PIEIER1.bit.INTx14 = 1;
       GpioDataRegs.GPATOGGLE.bit.GPIO3 = 1;

       count = 0;

   }



}

//
// cpu_timer2_isr CPU Timer2 ISR
//
__interrupt void cpu_timer2_isr(void)
{
    CpuTimer2.InterruptCount++;



}





(b)

#include "F28x_Project.h"

__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
int count = 0;

void main(void)
{

    InitSysCtrl();

    InitGpio();
    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(3, GPIO_OUTPUT, GPIO_PUSHPULL);

    DINT;

    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    EALLOW;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    EDIS;

    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    //ConfigCpuTimer(&CpuTimer2, 200, 50);

    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    IER |= M_INT13;
    IER |= M_INT14;

    EINT;  // Enable Global interrupt INTM
    ERTM;


    while(1)
        {

        }


}

__interrupt void cpu_timer1_isr(void)
{
   //CpuTimer1.InterruptCount++;
   // count++;
    GpioDataRegs.GPASET.bit.GPIO10 = 1;
    DELAY_US(30000);
    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
    DELAY_US(10000);


}

//
// cpu_timer2_isr CPU Timer2 ISR
//
__interrupt void cpu_timer2_isr(void)
{
    CpuTimer2.InterruptCount++;



}

