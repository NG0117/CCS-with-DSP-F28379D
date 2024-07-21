#include "F28x_Project.h"

void ConfigureADC(void);
void ConfigureEPWM(void);
void SetupADCEpwm(Uint16 channel);
interrupt void adca1_isr(void);

#define RESULTS_BUFFER_SIZE 256

Uint16 AdcaResults[RESULTS_BUFFER_SIZE];
Uint16 resultsIndex;
volatile Uint16 bufferFull;
volatile uint64_t conversionTime;
uint64_t timeStart = 0;

void main(void)
{

    InitSysCtrl();
    InitGpio(); // Skipped for this example
    DINT;

    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    EDIS;

    ConfigureADC();

    ConfigureEPWM();

    SetupADCEpwm(0);

    IER |= M_INT1; //Enable group 1 interrupts
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);

    for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
    {
        AdcaResults[resultsIndex] = 0;
    }
    resultsIndex = 0;
    bufferFull = 0;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    do
    {

        EPwm4Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
        EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

        while(!bufferFull);
        bufferFull = 0; //clear the buffer full flag

        EPwm4Regs.ETSEL.bit.SOCAEN = 0;  //disable SOCA
        EPwm4Regs.TBCTL.bit.CTRMODE = 3; //freeze counter

        //asm("   ESTOP0");
    }while(1);
}

void ConfigureADC(void)
{
    EALLOW;

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    DELAY_US(1000);

    EDIS;
}

void ConfigureEPWM(void)
{
    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm4Regs.ETSEL.bit.SOCAEN    = 0;    // Disable SOC on A group
    EPwm4Regs.ETSEL.bit.SOCASEL    = 4;   // Select SOC on up-count
    EPwm4Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
    EPwm4Regs.CMPA.bit.CMPA = 0x0800;     // Set compare A value to 2048 counts
    EPwm4Regs.TBPRD = 0x1000;             // Set period to 4096 counts
    EPwm4Regs.TBCTL.bit.CTRMODE = 3;      // freeze counter
    EDIS;
}

void SetupADCEpwm(Uint16 channel)
{
    Uint16 acqps;

    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 140; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 140; //320ns
    }

    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11; //trigger on ePWM1 SOCA/C
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

interrupt void adca1_isr(void)
{
    AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0;
    if(RESULTS_BUFFER_SIZE <= resultsIndex)
    {
        resultsIndex = 0;
        bufferFull = 1;
    }

    //uint64_t timeStart = CpuTimer1Regs.TIM.all;
    uint64_t timeEnd = CpuTimer1Regs.TIM.all;
//    conversionTime = timeEnd - timeStart;
GpioDataRegs.GPATOGGLE.bit.GPIO7 = 1;


    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
