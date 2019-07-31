/* DESCRIPTION
 * Sample code structure when with ADC analog input and driving an LED with PWM
 * This example: Turns a LED on at two different brightness level depending on ADC value
*/

#include <stdio.h>
#include <msp430.h>
#include "driverlib.h"
#include "Board.h"
#include "hal_LCD.h"

void displayTemp();
void displayMois();


volatile unsigned short *Result = (volatile unsigned short *) &BAKMEM5;
volatile unsigned int holdCount = 0;
volatile unsigned char * S1buttonDebounce = &BAKMEM2_L;       // S1 button debounce flag
volatile unsigned char * S2buttonDebounce = &BAKMEM2_H;       // S2 button debounce flag
volatile unsigned char * count = &BAKMEM7_H;            // Count variable

#define TIMER_PERIOD 511
int DUTY_CYCLE = 0;

char hexaKeys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

void Key();
volatile int ZoneNUM = 0;
volatile int KeyPressed = 0;
volatile int Stop1 = 0;
volatile int Stop2 = 0;

// TimerA0 UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 2MHz
        30000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};


void Init_RTC()
{
    // Set RTC modulo to 327-1 to trigger interrupt every ~10 ms
    RTC_setModulo(RTC_BASE, 326);
    RTC_enableInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT);
}


void main (void)
{

     WDT_A_hold(WDT_A_BASE);
     PMM_unlockLPM5();           // Need this for LED to turn on- in case of "abnormal off state"

         P1DIR |= 0x01;              // Set P1.0 to output direction  LED1
         P4DIR |= 0x01;              // Set P4.0 to output direction  LED2

         Timer_A_initUpModeParam param = {0};
         param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
         param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
         param.timerPeriod = TIMER_PERIOD;
         param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
         param.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
         param.timerClear = TIMER_A_DO_CLEAR;
         param.startTimer = true;
         Timer_A_initUpMode(TIMER_A0_BASE, &param);



         /*GPIOS initialization KeyPad*/


         GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);                  // Column 1: Output direction
         GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);

         GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN1);                  // Column 2: Output direction
         GPIO_setOutputLowOnPin (GPIO_PORT_P8, GPIO_PIN1);

         GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);                  // Column 3: Output direction
         GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);


         // Rows ARE ISR TRIGGERS

         GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);     // Row 1: Input direction
         GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
         GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN6);
         GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);
         GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);

         GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);     // Row 2: Input direction
         GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN3, GPIO_HIGH_TO_LOW_TRANSITION);
         GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN3);
         GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
         GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN3);


         /*GPIOS initialization Mux Inputs*/
         GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);  //Enable (Active Low)
         GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);

         GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);  //S2
         GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

         GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);  //S1
         GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);

         GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);  //S0
         GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);

         //Output A: ADC



         /*GPIOS initialization Buzzer      PWM See Below */
         GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);  //Buzzer
         GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);


         /*GPIOS initialization LED Outputs*/
         GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);  //Red Led
         GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);

         GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);  //Blue LED
         GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);

     // PWM set-up

     //GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);     // Connect P1.6 to LED

     Timer_A_initCompareModeParam initComp2Param = {0};
     initComp2Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
     initComp2Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
     initComp2Param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;



     // ADC set-up

     GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_ADC8, GPIO_PIN_ADC8, GPIO_FUNCTION_ADC8); // ADC Anolog Input Mux OUTPUT



     PMM_unlockLPM5();

     ADC_init(0x0700, ADC_SAMPLEHOLDSOURCE_SC, ADC_CLOCKSOURCE_ADCOSC, ADC_CLOCKDIVIDER_1);
     ADC_enable(0x0700);
     ADC_setupSamplingTimer(0x0700, ADC_CYCLEHOLD_16_CYCLES, ADC_MULTIPLESAMPLESDISABLE);        // timer trigger needed to start every ADC conversion
     ADC_configureMemory(0x0700, ADC_INPUT_A8, ADC_VREFPOS_INT, ADC_VREFNEG_AVSS);
     ADC_clearInterrupt(0x0700, ADC_COMPLETED_INTERRUPT);       //  bit mask of the interrupt flags to be cleared- for new conversion data in the memory buffer
     ADC_enableInterrupt(0x0700, ADC_COMPLETED_INTERRUPT);       //  enable source to reflected to the processor interrupt

     while (PMM_REFGEN_NOTREADY == PMM_getVariableReferenceVoltageStatus()) ;

     PMM_enableInternalReference();      // disabled by default

     __bis_SR_register(GIE);

     // Configure button S1 (P1.2) interrupt
        GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
        GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
        GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

        // Configure button S2 (P2.6) interrupt
        GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
        GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);


        // Set RTC modulo to 8192-1 to trigger interrupt every ~250 ms
                       RTC_setModulo(RTC_BASE, 8191);
                       RTC_enableInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT);
                       RTC_start(RTC_BASE, RTC_CLOCKSOURCE_XT1CLK);


     Init_LCD();

     *S1buttonDebounce = *S2buttonDebounce = 0;

     for (;;)
     {
         __delay_cycles(300000);
         GPIO_setOutputLowOnPin (GPIO_PORT_P2, GPIO_PIN7);//Enable
         GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
         GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);

         _EINT();        // Start interrupt

         PMM_unlockLPM5();           // Need this for LED to turn on- in case of "abnormal off state"

         //displayScrollText("WELCOME TO THE FR4133 LAUNCHPAD");

         switch (ZoneNUM){     // interrupt vector register never has a value that is odd or larger than 12 (stated)

         case  2:
             GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);//S2
             GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);//S1
             GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);//S0
             break; //MOIS Sensor 2

         case  1:
             GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);//S2
             GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);//S1
             GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);//S0
         break;

         case  3:
             GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);//S2
             GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);//S1
             GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);//S0
             break;

         case  4:
             GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);//S2
             GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);//S1
             GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);//S0
                 break;

         case  6:
             GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);//S2
             GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);//S1
             GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);//S0
             break;

         case  5:
             GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);//S2
             GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);//S1
             GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);//S0
             break;

         default: break;
         }

         if( 0<ZoneNUM<7  ){
             KeyPressed = 0;
             ADC_startConversion(0x0700, ADC_SINGLECHANNEL);

             initComp2Param.compareValue = DUTY_CYCLE;
             Timer_A_initCompareMode(TIMER_A0_BASE, &initComp2Param);
             if(ZoneNUM == 4 || ZoneNUM ==5 || ZoneNUM ==6){
                 if(Stop1 == 0){
                     displayTemp();
                 }
             }
             else if(ZoneNUM == 1 || ZoneNUM == 2 || ZoneNUM == 3){
                 if(Stop2 == 0){
                     displayMois();
                 }
             }
         }  else{
     // __bis_SR_register(LPM4_bits + GIE);     // Need this for interrupts or else "abnormal termination"
      //__no_operation();           //For debugger

         }
     }
}


void Key()
{
    /*if (KeyPressed == 0){
        //all column high*/
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);                               // Column 1- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN1);                               // Column 2- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);                               // Column 3- LOW


        //Column 1111111111
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);                                // Column 1- LOW
        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)  {    // Row 1 to GND
            ZoneNUM = 1;                                                                // MOIS Sensor 1
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW) {// Row 2
            ZoneNUM = 4;
        }
                                                                                        // TEMP Sensor 1
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);                               // Column 1- HIGH


        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN1);                                // Column 1- LOW
                if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)  {    // Row 1 to GND
                    ZoneNUM = 2;                                                                // MOIS Sensor 1
                    KeyPressed = 1;
                }
                if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW) {// Row 2
                    ZoneNUM = 5;
                    KeyPressed = 1;
                }
                                                                                                // TEMP Sensor 1
                GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN1);                               // Column 1- HIGH


        //Column 3333333333
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);                                // Column 1- LOW
                      if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)  {    // Row 1 to GND
                          ZoneNUM = 3;                                                                // MOIS Sensor 1
                          KeyPressed = 1;
                      }
                      if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW) {// Row 2
                          ZoneNUM = 6;
                          KeyPressed = 1;
                      }
                                                                                                      // TEMP Sensor 1
                      GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);                               // Column 1- HIGH

        //all column low
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);                                // Column 1- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN1);                               // Column 2- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);                               // Column 3- LOW


}

/*
#pragma vector = PORT1_VECTOR       // Using PORT1_VECTOR interrupt because P1.4 and P1.5 are in port 1
__interrupt void PORT1_ISR_KeyPad(void)
{
    Key();

    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);                  // Row 1

    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);                  // Row 2

}*/






void displayTemp()
{
    clearLCD();

    // Pick C or F depending on tempUnit state
    int deg;
    //if (*tempUnit == 0)
    //{
        showChar('C',pos6);
        deg = (*Result / 2);
   /* }
    else
    {
        showChar('F',pos6);
        deg = *degF;
    }*/

    // Handle negative values
    if (deg < 0)
    {
        deg *= -1;
        // Negative sign
        LCDMEM[pos1+1] |= 0x04;
    }

    // Handles displaying up to 999.9 degrees
    if (deg>=1000)
        showChar((deg/1000)%10 + '0',pos2);
    if (deg>=100)
        showChar((deg/100)%10 + '0',pos3);
    if (deg>=10)
        showChar((deg/10)%10 + '0',pos4);
    if (deg>=1)
        showChar((deg/1)%10 + '0',pos5);

    // Decimal point
    LCDMEM[pos4+1] |= 0x01;

    // Degree symbol
    LCDMEM[pos5+1] |= 0x04;
}


void displayMois()
{
    clearLCD();

    // Pick C or F depending on tempUnit state
    int deg;
    //if (*tempUnit == 0)
    //{
        //showChar('C',pos6);
    deg = (*Result / 2);
   /* }
    else
    {
        showChar('F',pos6);
        deg = *degF;
    }*/

    // Handle negative values
    if (deg < 0)
    {
        deg *= -1;
        // Negative sign
        LCDMEM[pos1+1] |= 0x04;
    }

    // Handles displaying up to 999.9 degrees
    /*if (deg>=1000)
        showChar((deg/1000)%10 + '0',pos1);*/
    if (deg>=100)
        showChar((deg/100)%10 + '0',pos1);
    if (deg>=10)
        showChar((deg/10)%10 + '0',pos2);
    if (deg>=1)
        showChar((deg/1)%10 + '0',pos3);
    showChar('G' ,pos4);
    showChar('M' ,pos5);

    showChar('3' ,pos6);


}



/*
 * PORT1 Interrupt Service Routine
 * Handles S1 button press interrupt
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR_PB(void)
{


    P1OUT |= BIT0;    // Turn LED1 On
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case P1IV_NONE : break;
        case P1IV_P1IFG0 : break;
        case P1IV_P1IFG1 : break;
        case P1IV_P1IFG2 :
            if ((*S1buttonDebounce) == 0)
            {
                *S1buttonDebounce = 1;                        // First high to low transition
                holdCount = 0;



                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }
            break;
        case P1IV_P1IFG3 :
            if(Stop1 == 0){
                Key();
            }else if(Stop1 == 1){
                //Stop = 0;

            }

            GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);                  // Row 2
            break;
        case P1IV_P1IFG4 : break;
        case P1IV_P1IFG5 : break;
        case P1IV_P1IFG6 :
            if(Stop2 == 0){
                Key();

            }else if(Stop2 == 1){
                //Stop = 0;

            }
            GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);                  // Row 1
            break;
        case P1IV_P1IFG7 : break;
    }



           //GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);                  // Row 1

//           GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);                  // Row 2
}

/*
 * PORT2 Interrupt Service Routine
 * Handles S2 button press interrupt
 */
#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    P4OUT |= BIT0;    // Turn LED2 On
    switch(__even_in_range(P2IV, P2IV_P2IFG7))
    {
        case P2IV_NONE : break;
        case P2IV_P2IFG0 : break;
        case P2IV_P2IFG1 : break;
        case P2IV_P2IFG2 : break;
        case P2IV_P2IFG3 : break;
        case P2IV_P2IFG4 : break;
        case P2IV_P2IFG5 : break;
        case P2IV_P2IFG6 :
            if ((*S2buttonDebounce) == 0)
            {
                *S2buttonDebounce = 1;                        // First high to low transition
                holdCount = 0;



                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }

        case P2IV_P2IFG7 : break;
    }
}


/*
 * Timer A0 Interrupt Service Routine
 * Used as button debounce timer
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
    // Both button S1 & S2 held down
    if (!(P1IN & BIT2)  )
    {
        holdCount++;
        if (holdCount == 40)
        {
            Timer_A_stop(TIMER_A0_BASE);
            Stop1 ^= 1;
            //GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
            //GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);
            GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN4);       // turn off the LED
            GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN5);       // turn off the LED
            GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN7);
            *S1buttonDebounce = 0;
            __bic_SR_register_on_exit(LPM3_bits);                // exit LPM3
        }
    }
    else if (!(P2IN & BIT6)){
        holdCount++;
        if (holdCount == 40)
        {
            Timer_A_stop(TIMER_A0_BASE);
            Stop2 ^= 1;
            //GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
            //GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);
            GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN4);       // turn off the LED
            GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN5);       // turn off the LED
            GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN7);
            *S2buttonDebounce = 0;
            __bic_SR_register_on_exit(LPM3_bits);                // exit LPM3
        }
    }

    // Button S1 released
    if (P1IN & BIT2)
    {
        *S1buttonDebounce = 0;                                   // Clear button debounce
        P1OUT &= ~BIT0;
    }

    // Button S2 released
    if (P2IN & BIT6)
    {
        *S2buttonDebounce = 0;                                   // Clear button debounce
        P4OUT &= ~BIT0;
    }


}



/*
 * RTC Interrupt Service Routine
 * Wakes up every ~10 milliseconds to update stowatch
 */
#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
{
    switch(__even_in_range(RTCIV, RTCIV_RTCIF))
    {
        case RTCIV_NONE : break;
        case RTCIV_RTCIF:

                // Since RTC runs at 32768 Hz and isn't fast enough to count 10 ms exactly
                // offset RTC counter every 100 10ms intervals to add up to 1s
                // (327 * 32) + (328 * 68) = 32768
                if((*count)==31)
                {
                    // Set RTC to interrupt after 328 XT1 cycles
                    RTC_setModulo(RTC_BASE, 327);
                }
                else if((*count)==99)
                {
                    // Set RTC to interrupt after 327 XT1 cycles
                    RTC_setModulo(RTC_BASE, 326);
                    (*count)=0;
                }
                (*count)++;
                __bic_SR_register_on_exit(LPM3_bits);            // exit LPM3

            break;
    }
}

//ADC ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC_VECTOR)))
#endif
void ADC_ISR (void)
{
    switch (__even_in_range(ADCIV,12)){     // interrupt vector register never has a value that is odd or larger than 12 (stated)
        case  0: break; //No interrupt
        case  2: break; //conversion result overflow
        case  4: break; //conversion time overflow
        case  6: break; //ADCHI
        case  8: break; //ADCLO
        case 10: break; //ADCIN
        case 12:        //ADCIFG0 is ADC interrupt flag

            *Result = ADC_getResults(0x0700);
           // GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
            if(Stop2 == 0){
                if( ZoneNUM == 1){
                    GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN4);
                    GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    if ((ADC_getResults(0x0700) >= 0x298))  {    //100 333G/M3
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);       // turn on the LED
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                    else{
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN5);       // turn off the LED
                        GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                }

                else if ( ZoneNUM == 2){
                    GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN4);
                    GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    if ((ADC_getResults(0x0700) <= 0x0D0))  {    //100 333G/M3
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);       // turn on the LED
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                    else{
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN5);       // turn off the LED
                        GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                }

                else if(ZoneNUM == 3){
                    GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN4);
                    GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    if ((ADC_getResults(0x0700) >= 0x298 || ADC_getResults(0x0700) <= 0x0D0))  {    //100 333G/M3
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);       // turn on the LED
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                    else{
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN5);       // turn off the LED
                        GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                }
            }
            if(Stop1 == 0){
                if(ZoneNUM == 4){
                    GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN5);
                    GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    if ((ADC_getResults(0x0700) >= 0x213 ))  {  //26.5 &&    26              // 20.8C
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);        // turn on the LED
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                    else{
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN4);       // turn off the LED
                        GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                }
                else if( ZoneNUM ==5){
                    GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN5);
                    GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                        if ((ADC_getResults(0x0700) <= 0x207))  {  //26.5 &&    26              // 20.8C
                            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);        // turn on the LED
                            GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN7);
                        }
                        else{
                            GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN4);       // turn off the LED
                            GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                        }
                }
                else if( ZoneNUM ==6){
                    GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN5);
                    GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    if ((ADC_getResults(0x0700) >= 0x213 || ADC_getResults(0x0700) <= 0x207))  {  //26.5 &&    26              // 20.8C
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);        // turn on the LED
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                    else{
                        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN4);       // turn off the LED
                        GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN7);
                    }
                }
            }

            break;
        default: break;
    }
}
