/****************************************************************************
 Module
   ADC_HAL.c

 Revision
   1.0.1

 Description
   This implements a hardware abstraction layer for the PIC32 ADC

 Notes

****************************************************************************/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

/**************************************************************************
  Function
     InitADC

 Parameters
     None

 Returns
     None

 Description
     Initializes the ADC
 Notes
 
 *************************************************************************/
void InitADC(void) {
  /* 
    Step 1: Initialize the ADC calibration values by
    copying them from the factory programmed
    DEVADCx Flash locations starting at 0xBFC45000
    into the ADCxCFG registers starting at 0xBF887D00,
    respectively. 
   */
   ADC0CFG = DEVADC0;        //Load ADC0 Calibration values    
   ADC1CFG = DEVADC1;        //Load ADC1 Calibration values
   ADC2CFG = DEVADC2;        //Load ADC2 Calibration values    
   ADC3CFG = DEVADC3;        //Load ADC3 Calibration values
   ADC4CFG = DEVADC4;        //Load ADC4 Calibration values
   ADC7CFG = DEVADC7;        //Load ADC7 Calibration values
   
  /*
   Step 2: The user writes all the essential ADC
    configuration SFRs including the ADC control clock
    and all ADC core clocks setup:
    -ADCCON1, keeping the ON bit = 0
    -ADCCON2, especially paying attention to ADCDIV<6:0> and SAMC<9:0>
    -ADCANCON, keeping all analog enables ANENx bit = 0, WKUPCLKCNT bit = 0xA
    -ADCCON3, keeping all DIGEN5x = 0, especially paying attention to ADCSEL<1:0>, CONCLKDIV <5:0>, and VREFSEL<2:0>
    -ADCxTIME, ADCDIVx<6:0>, and SAMCx<9:0>
    -ADCTRGMODE, ADCIMCONx, ADCTRGSNS, ADCCSSx, ADCGIRQENx, ADCTRGx, ADCBASE
    -Comparators, Filters, etc.
   */
   
   /* Configure ADCCON1 */ 
   ADCCON1 = 0; // No ADCCON1 features are enabled including: Stop-in-Idle, turbo, 
                // CVD mode, Fractional mode and scan trigger source. 
   ADCCON1bits.SELRES = 0b11; // ADC7 resolution is 12 bits 
   ADCCON1bits.STRGSRC = 0b00001; // Select scan trigger.
   
   // Set these to 0 since Vdd > 2.5 V
   ADCCON1bits.AICPMPEN = 0;
   CFGCONbits.IOANCPEN = 0; 
   
   /* Configure ADCCON2 */ 
   ADCCON2bits.SAMC = 5; // ADC7 sampling time = 7 * TAD7 
   ADCCON2bits.ADCDIV = 1; // ADC7 clock freq is half of control clock = TAD7
   ADCCON2bits.EOSIEN = 1; // End of scan interrupt enabled
   
   /* Initialize warm up time register */ 
   ADCANCON = 0; 
   ADCANCONbits.WKUPCLKCNT = 0xA; // Wakeup exponent
   
   /* Clock setting */ 
//   ADCCON3 = 0;
   ADCCON3bits.ADCSEL = 0; // Select Tclk = PBCLK3 (50 MHz)
   ADCCON3bits.CONCLKDIV = 1; // Control clock frequency is half of input clock (TQ =25 MHz)
   ADCCON3bits.VREFSEL = 0; // Select AVDD and AVSS as reference source
   
   // ADCxTIME registers
   ADC4TIMEbits.ADCEIS = 0b000; // Data ready interrupt 1 ADC clock early
   ADC4TIMEbits.SELRES = 0b11; // 12 bit resolution
   ADC4TIMEbits.ADCDIV = 1; // ADC4 clock freq is half of control clock
   ADC4TIMEbits.SAMC = 5; // ADC4 sampling time = 7*TAD4
   
   // ADCTRGMODE
   ADCTRGMODE = 0; // Use AN4 for ADC4, don't use presynchronized triggers, don't use synchronous sampling
   
   /* Select ADC input mode */ 
   ADCIMCON1bits.SIGN4 = 0; // unsigned data format 
   ADCIMCON1bits.DIFF4 = 0; // Single ended mode 
   ADCIMCON1bits.SIGN6 = 0; // unsigned data format 
   ADCIMCON1bits.DIFF6 = 0; // Single ended mode 
   ADCIMCON3bits.SIGN37 = 0; // unsigned data format 
   ADCIMCON3bits.DIFF37 = 0; // Single ended mode 
   
   /* Configure ADCGIRQENx */ 
   ADCGIRQEN1 = 0; // No interrupts
   ADCGIRQEN2 = 0; // No interrupts
  
   /* Configure ADCCSSx */ 
   ADCCSS1 = 0; // Clear all bits 
   ADCCSS2 = 0; 
   ADCCSS1bits.CSS4 = 1; // AN4 set for scan 
   ADCCSS1bits.CSS6 = 1; // AN6 set for scan 
   ADCCSS2bits.CSS37 = 1; // AN37 set for scan 
   
   // Also need to set trigger source for AN4/AN6 since class1/class2
   ADCTRG2bits.TRGSRC4 = 0b00011; // STRIG
   ADCTRG2bits.TRGSRC6 = 0b00011; // STRIG
   
   // Not using digital comparator, don't worry about ADCCMPENx
   ADCCMPEN1 = 0;
   ADCCMPEN2 = 0;
   ADCCMPEN3 = 0;
   ADCCMPEN4 = 0;
   ADCCMPEN5 = 0;
   ADCCMPEN6 = 0;
   
   /* Configure ADCCMPCONx */ 
   ADCCMPCON1 = 0; // No digital comparators are used. Setting the ADCCMPCONx 
   ADCCMPCON2 = 0; // register to '0' ensures that the comparator is disabled.
   ADCCMPCON3 = 0;
   ADCCMPCON4 = 0;
   ADCCMPCON5 = 0;
   ADCCMPCON6 = 0;
   
   /* Configure ADCFLTRx */ 
   ADCFLTR1 = 0; // No oversampling filters are used. 
   ADCFLTR2 = 0; 
   ADCFLTR3 = 0; 
   ADCFLTR4 = 0; 
   ADCFLTR5 = 0; 
   ADCFLTR6 = 0; 
   
   // ADCFSTAT: not using FIFO so dont worry about it
   
   // ADCBASE: Not using interrupts so don't worry about it
   
   // ADCTRGSNS: leave at default-use poitive edge of trigger
   ADCTRGSNS = 0;
   
   /* Early interrupt */ 
   ADCEIEN1 = 0; // No early interrupt 
   ADCEIEN2 = 0;
   
   /* 
     Enable local/global interrupts
    */
   INTCONbits.MVEC = 1; // Use multivector mode
   PRISSbits.PRI4SS = 0b0100; // Priority 4 interrupt use shadow set 4
   IPC11bits.ADCIP = 4; // ADC global interrupt priority
   
   // Clear interrupt flags
  IFS1CLR = _IFS1_ADCIF_MASK;
  
  // Local disable interrupts
  IEC1CLR = _IEC1_ADCIE_MASK;
  
  __builtin_enable_interrupts(); // Global enable interrupts
  
   /*
    Step 4: The user sets the ON bit to 1, which enables 
    * the ADC control clock.
   */
   ADCCON1bits.ON = 1;
   
   /*
    Step 5: The user waits for the interrupt/polls the
    BGVRRDY bit (ADCCON2<31>) and the WKRDYx
    bit (ADCANCON<15,13:8>) = 1, which signals that
    the device analog environment (band gap and VREF)
    is ready.
   */
   while(!ADCCON2bits.BGVRRDY); // Wait until the reference voltage is ready 
   while(ADCCON2bits.REFFLT); // Wait if there is a fault with the reference voltage
   
    /*
    Step 3: The user sets the ANENx bit to 1 for the ADC 
    SAR Cores needed (which internally in the ADC module 
    enables the control clock to generate by division the 
    core clocks for the desired ADC SAR Cores, which in turn 
    enables the bias circuitry for these ADC SAR Cores).
   */ 
   ADCANCONbits.ANEN7 = 1; // Enable, ADC 7
   while(!ADCANCONbits.WKRDY7); // Wait until ADC7 is ready
   
   ADCANCONbits.ANEN4 = 1; // Enable ADC 4
   while (!ADCANCONbits.WKRDY4); // Wait until ADC4 is ready
   DB_printf("ADC4 Ready! \r\n");
   
   /*
    Step 6: Set the DIGENx bit (ADCCON3<15,13:8>) to
    1, which enables the digital circuitry to immediately
    begin processing incoming triggers to perform data
    conversions. 
   */
   ADCCON3bits.DIGEN4 = 1 ; // Enable ADC4
   ADCCON3bits.DIGEN7 = 1; // Enable ADC7
}

/**************************************************************************
  Function
     ReadADCValues

 Parameters
     uint32_t* Results: a pointer to a vector which the ADC results will be
                        placed

 Returns
     None

 Description
     Places the readings from the ADC into Results
 Notes
 
 *************************************************************************/
void ReadADC(uint16_t *Results){
    
    IEC1SET = _IEC1_ADCIE_MASK; // Enable local interrupt
    ADCCON3bits.GSWTRG = 1; // Trigger a conversion    
}