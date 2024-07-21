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


#define EPWM1_MAX_DB   0x03FF
#define EPWM1_MIN_DB   0x0064
#define DB_UP          1
#define DB_DOWN        0
Uint32 EPwm1TimerIntCount;
Uint16 EPwm1_DB_Direction;
void InitEPwm1Example(void);
__interrupt void epwm1_isr(void);


#define DELAY (CPU_RATE/1000*6*510)
volatile Uint32 Xint1Count;
Uint32 LoopCount;
interrupt void xint1_isr(void);

bool outflag = 0;
int count = 0;
float val = 0;

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
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    PieVectTable.EPWM1_INT = &epwm1_isr;
    PieVectTable.XINT1_INT = &xint1_isr;
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
    InitEPwm1Gpio();

    for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
    {
        AdcaResults[resultsIndex] = 0;
    }
    resultsIndex = 0;
    bufferFull = 0;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    InitEPwm1Example();
    IER |= M_INT3;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;



    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;  // GPIO模式
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;   // 設置為輸出
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1; // 初始化輸出狀態為低電平
    EDIS;

    Xint1Count = 0; // Count XINT1 interrupts
    LoopCount = 0;  // Count times through idle loop

    IER |= M_INT1;                              // Enable CPU INT1
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4

    EINT;                                       // Enable Global Interrupts

    GPIO_SetupXINT1Gpio(6);
    XintRegs.XINT1CR.bit.POLARITY = 0;          // Falling edge interrupt
    XintRegs.XINT1CR.bit.ENABLE = 1;


    for(;;)
       {
          TempX1Count = Xint1Count;



                  EPwm2Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
                  EPwm2Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

                  while(!bufferFull);
                  bufferFull = 0; //clear the buffer full flag

                  EPwm2Regs.ETSEL.bit.SOCAEN = 0;  //disable SOCA
                  EPwm2Regs.TBCTL.bit.CTRMODE = 3; //freeze counter

                  //asm("   ESTOP0");



          while(Xint1Count == TempX1Count) {}


          if(Xint1Count == TempX1Count + 1 )
          {
              LoopCount++;

          }
          else
          {

          }


       }

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

interrupt void xint1_isr(void)
{
    //GpioDataRegs.GPBCLEAR.all = 0x4;   // GPIO34 is low
    Xint1Count++;
    GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1;

    //
    // Acknowledge this interrupt to get more from group 1
    //
    outflag = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//    PieCtrlRegs.PIEIFR1.bit.INTx4 = 1;

}

void ConfigureEPWM(void)
{
    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm2Regs.ETSEL.bit.SOCAEN    = 0;    // Disable SOC on A group
    EPwm2Regs.ETSEL.bit.SOCASEL    = 4;   // Select SOC on up-count
    EPwm2Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
    EPwm2Regs.CMPA.bit.CMPA = 1250;     // Set compare A value to 2048 counts
    EPwm2Regs.TBPRD = 2500;             // Set period to 4096 counts
    EPwm2Regs.TBCTL.bit.CTRMODE = 3;      // freeze counter
    EDIS;
}

void SetupADCEpwm(Uint16 channel)
{
    Uint16 acqps;

    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7; //trigger on ePWM1 SOCA/C
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

interrupt void adca1_isr(void)
{
    GpioDataRegs.GPATOGGLE.bit.GPIO14 = 1;
    AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0;


    //uint64_t timeStart = CpuTimer1Regs.TIM.all;
    uint64_t timeEnd = CpuTimer1Regs.TIM.all;
//    conversionTime = timeEnd - timeStart;

//
//
    if (outflag == 1) {
        count ++ ;
        val += 7;

//        if (count >= 9 ) {
//            count = 0;
//            outflag = 0;
//            GpioDataRegs.GPATOGGLE.bit.GPIO14 = 1;
//            EPwm1Regs.CMPA.bit.CMPA = 1157;
//
//        }
        if(count > 332) {
            count = 0;
            val = 7;
            outflag = 0;
        }
        EPwm1Regs.CMPA.bit.CMPA = val;

    }


    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

__interrupt void epwm1_isr(void)
{

    EPwm1TimerIntCount++;

    //
    // Clear INT flag for this timer
    //


//    if (outflag == 1) {
//            count ++ ;
//            val += 1157;
//            EPwm1Regs.CMPA.bit.CMPA = val;
//            if (count >= 9 ) {
//                count = 0;
//                outflag = 0;
//                GpioDataRegs.GPATOGGLE.bit.GPIO14 = 1;
//                EPwm1Regs.CMPA.bit.CMPA = 1157;
//
//            }
//
//        }


    EPwm1Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //






    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


void InitEPwm1Example()
{
    EPwm1Regs.TBPRD = 2500;                       // Set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Setup compare
    //



    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
    EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
//    EPwm1Regs.CMPA.bit.CMPA = 75;
    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED.bit.DBRED = 50;
    EPwm1Regs.DBFED.bit.DBFED = 50;
    EPwm1_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;          // Generate INT on 3rd event
}
