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

#define EPWM2_MAX_DB   0x03FF
#define EPWM2_MIN_DB   0
#define DB_UP          1
#define DB_DOWN        0

Uint32 EPwm2TimerIntCount;
Uint16 EPwm2_DB_Direction;

void InitEPwm2Example(void);
__interrupt void epwm2_isr(void);

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
    PieVectTable.EPWM2_INT = &epwm2_isr;
    EDIS;

    ConfigureADC();

    ConfigureEPWM();

    SetupADCEpwm(0);

    IER |= M_INT1; //Enable group 1 interrupts
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(14, GPIO_OUTPUT, GPIO_PUSHPULL);

    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    InitEPwm2Gpio();

    for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
    {
        AdcaResults[resultsIndex] = 0;
    }
    resultsIndex = 0;
    bufferFull = 0;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    InitEPwm2Example();
    IER |= M_INT3;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

    do
    {

        EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
        EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

        while(!bufferFull);
        bufferFull = 0; //clear the buffer full flag

        EPwm1Regs.ETSEL.bit.SOCAEN = 0;  //disable SOCA
        EPwm1Regs.TBCTL.bit.CTRMODE = 3; //freeze counter

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
    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL    = 4;   // Select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
    EPwm1Regs.CMPA.bit.CMPA = 0x0800;     // Set compare A value to 2048 counts
    EPwm1Regs.TBPRD = 0x1000;             // Set period to 4096 counts
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;      // freeze counter
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
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
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

    if (AdcaResultRegs.ADCRESULT0 <= 1500) {
        EPwm2Regs.CMPA.bit.CMPA = 75;
    } else if (AdcaResultRegs.ADCRESULT0 > 1500 && AdcaResultRegs.ADCRESULT0 <=3000) {
        EPwm2Regs.CMPA.bit.CMPA = 125;
    } else if (AdcaResultRegs.ADCRESULT0 >3000) {
        EPwm2Regs.CMPA.bit.CMPA = 175;
    }




    //uint64_t timeStart = CpuTimer1Regs.TIM.all;
    uint64_t timeEnd = CpuTimer1Regs.TIM.all;
    conversionTime = timeEnd - timeStart;

    GpioDataRegs.GPASET.bit.GPIO14 = 1;
    DELAY_US(conversionTime);
    GpioDataRegs.GPACLEAR.bit.GPIO14 = 1;
    DELAY_US(conversionTime);
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void epwm2_isr(void)
{

    EPwm2TimerIntCount++;

    //
    // Clear INT flag for this timer
    //
    EPwm2Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


void InitEPwm2Example()
{
    EPwm2Regs.TBPRD = 250;                       // Set timer period
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV4;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Setup compare
    //



    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on Zero
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;

    //EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
    //EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    //EPwm1Regs.DBRED.bit.DBRED = EPWM1_MIN_DB;
    //EPwm1Regs.DBFED.bit.DBFED = EPWM1_MIN_DB;
    EPwm2_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;          // Generate INT on 3rd event
}
