/****************************************************************************
 Module
   ADC_HAL.c

 Description
   Shared PIC32 ADC HAL for cliff sensors and motor current (DRV8842 ISEN).
   Scan: AN4, AN6, AN29 (right ISEN/RA1), AN36 (left ISEN/RJ9), AN37.
   InitADC is idempotent so MotorSM and ReflectService may both call it.
****************************************************************************/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ADC_HAL.h"
#include "dbprintf.h"
#include <sys/attribs.h>
#include <xc.h>

/* I_mA = counts * Vref / 4095 / Rsense * 1000; Vref=3.3, Rsense=0.25 */
#define ADC_TO_MA_NUM 13200u  /* 3.3 * 1000 / 0.25 */
#define ADC_TO_MA_DEN 4095u

static bool adc_initialized = false;
static volatile bool conversion_done = false;
static volatile uint16_t cliff_results[3];   /* AN6, AN37, AN4 */
static volatile uint16_t motor_right_an29; /* RA1 / right motor ISEN */
static volatile uint16_t motor_left_an36;  /* RJ9 / left motor ISEN */

/* Peak capture: ISEN is only valid during PWM on-time, so track max over a window */
static volatile bool capture_active = false;
static volatile uint16_t max_left_counts = 0;
static volatile uint16_t max_right_counts = 0;

void InitADC(void)
{
  if (adc_initialized) {
    return;
  }

  /* Factory calibration */
  ADC0CFG = DEVADC0;
  ADC1CFG = DEVADC1;
  ADC2CFG = DEVADC2;
  ADC3CFG = DEVADC3;
  ADC4CFG = DEVADC4;
  ADC7CFG = DEVADC7;

  ADCCON1 = 0;
  ADCCON1bits.SELRES = 0b11;     /* ADC7: 12-bit */
  ADCCON1bits.STRGSRC = 0b00001; /* scan trigger */
  ADCCON1bits.AICPMPEN = 0;
  CFGCONbits.IOANCPEN = 0;

  ADCCON2bits.SAMC = 5;
  ADCCON2bits.ADCDIV = 1;
  ADCCON2bits.EOSIEN = 1;        /* end-of-scan IRQ */

  ADCANCON = 0;
  ADCANCONbits.WKUPCLKCNT = 0xA;

  ADCCON3bits.ADCSEL = 0;        /* PBCLK3 */
  ADCCON3bits.CONCLKDIV = 1;
  ADCCON3bits.VREFSEL = 0;       /* AVDD/AVSS */

  ADC4TIMEbits.ADCEIS = 0b000;
  ADC4TIMEbits.SELRES = 0b11;
  ADC4TIMEbits.ADCDIV = 1;
  ADC4TIMEbits.SAMC = 5;

  ADCTRGMODE = 0;

  /* Unsigned, single-ended for all scanned channels */
  ADCIMCON1bits.SIGN4 = 0;
  ADCIMCON1bits.DIFF4 = 0;
  ADCIMCON1bits.SIGN6 = 0;
  ADCIMCON1bits.DIFF6 = 0;
  ADCIMCON2bits.SIGN29 = 0;
  ADCIMCON2bits.DIFF29 = 0;
  ADCIMCON3bits.SIGN36 = 0;
  ADCIMCON3bits.DIFF36 = 0;
  ADCIMCON3bits.SIGN37 = 0;
  ADCIMCON3bits.DIFF37 = 0;

  ADCGIRQEN1 = 0;
  ADCGIRQEN2 = 0;

  /* Scan: cliffs AN4/AN6/AN37 + motor ISEN AN29 (right) / AN36 (left) */
  ADCCSS1 = 0;
  ADCCSS2 = 0;
  ADCCSS1bits.CSS4 = 1;
  ADCCSS1bits.CSS6 = 1;
  ADCCSS1bits.CSS29 = 1;
  ADCCSS2bits.CSS36 = 1;
  ADCCSS2bits.CSS37 = 1;

  /* Class 1/2 channels need explicit scan trigger source */
  ADCTRG2bits.TRGSRC4 = 0b00011; /* STRIG */
  ADCTRG2bits.TRGSRC6 = 0b00011;

  ADCCMPEN1 = 0;
  ADCCMPEN2 = 0;
  ADCCMPEN3 = 0;
  ADCCMPEN4 = 0;
  ADCCMPEN5 = 0;
  ADCCMPEN6 = 0;
  ADCCMPCON1 = 0;
  ADCCMPCON2 = 0;
  ADCCMPCON3 = 0;
  ADCCMPCON4 = 0;
  ADCCMPCON5 = 0;
  ADCCMPCON6 = 0;

  ADCFLTR1 = 0;
  ADCFLTR2 = 0;
  ADCFLTR3 = 0;
  ADCFLTR4 = 0;
  ADCFLTR5 = 0;
  ADCFLTR6 = 0;

  ADCTRGSNS = 0;
  ADCEIEN1 = 0;
  ADCEIEN2 = 0;

  INTCONbits.MVEC = 1;
  PRISSbits.PRI4SS = 0b0100;
  IPC11bits.ADCIP = 4;
  IFS1CLR = _IFS1_ADCIF_MASK;
  IEC1CLR = _IEC1_ADCIE_MASK;
  __builtin_enable_interrupts();

  ADCCON1bits.ON = 1;
  while (!ADCCON2bits.BGVRRDY) {
    ;
  }
  while (ADCCON2bits.REFFLT) {
    ;
  }

  ADCANCONbits.ANEN7 = 1;
  while (!ADCANCONbits.WKRDY7) {
    ;
  }
  ADCANCONbits.ANEN4 = 1;
  while (!ADCANCONbits.WKRDY4) {
    ;
  }

  ADCCON3bits.DIGEN4 = 1;
  ADCCON3bits.DIGEN7 = 1;

  adc_initialized = true;
}

void ReadADC(void)
{
  conversion_done = false;
  IEC1SET = _IEC1_ADCIE_MASK;  // Enable local interrupt
  ADCCON3bits.GSWTRG = 1;  // Trigger a conversion    
}

bool ADC_ConversionReady(void)
{
  return conversion_done;
}

void GetCliffADC(uint16_t out[3])
{
  out[0] = cliff_results[0];
  out[1] = cliff_results[1];
  out[2] = cliff_results[2];
}

void GetMotorCurrentADC(uint16_t *left_counts, uint16_t *right_counts)
{
  if (left_counts) {
    *left_counts = motor_left_an36;
  }
  if (right_counts) {
    *right_counts = motor_right_an29;
  }
}

/**************************************************************************
  Function
     MotorCurrentCountsTomA

 Parameters
     counts: 12-bit ADC reading from a DRV8842 ISEN pin

 Returns
     Estimated motor current in milliamps

 Description
     Converts ISEN ADC counts to mA assuming AVDD=3.3 V reference and a
     0.25 ohm sense resistor (V_ISEN = I * R_sense).
 *************************************************************************/
uint32_t MotorCurrentCountsTomA(uint16_t counts)
{
  return ((uint32_t)counts * ADC_TO_MA_NUM) / ADC_TO_MA_DEN;
}

void StartMotorCurrentMaxCapture(void)
{
  max_left_counts = 0;
  max_right_counts = 0;
  capture_active = true;
}

void StopMotorCurrentMaxCapture(void)
{
  capture_active = false;
}

void GetMotorCurrentMaxADC(uint16_t *left_counts, uint16_t *right_counts)
{
  if (left_counts) {
    *left_counts = max_left_counts;
  }
  if (right_counts) {
    *right_counts = max_right_counts;
  }
}

void __ISR(_ADC_VECTOR, IPL4SRS) ADCHandler(void)
{
  uint32_t status = ADCCON2; /* reading ADCCON2 also clears EOSRDY */
  if ((status >> 29) & 1) {
    IFS1CLR = _IFS1_ADCIF_MASK;  // Clear the ADC interrupt flag
    IEC1CLR = _IEC1_ADCIE_MASK;  // Disable local interrupt

    cliff_results[0] = (uint16_t)ADCDATA6;
    cliff_results[1] = (uint16_t)ADCDATA37;
    cliff_results[2] = (uint16_t)ADCDATA4;
    motor_right_an29 = (uint16_t)ADCDATA29;
    motor_left_an36 = (uint16_t)ADCDATA36;

    if (capture_active) {
      if (motor_left_an36 > max_left_counts) {
        max_left_counts = motor_left_an36;
      }
      if (motor_right_an29 > max_right_counts) {
        max_right_counts = motor_right_an29;
      }
    }

    conversion_done = true;
  } else {
    DB_printf("Unexpected ADC interrupt\r\n");
  }
}
