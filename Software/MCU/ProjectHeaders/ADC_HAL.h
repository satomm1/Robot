/* 
 * File:   ADC_HAL.h
 * Author: satom
 *
 * PIC32 ADC HAL: cliff sensors + DRV8842 motor current (ISEN).
 */

#ifndef ADC_HAL_H
#define ADC_HAL_H

#include <stdint.h>
#include <stdbool.h>

void InitADC(void);
void ReadADC(void);
bool ADC_ConversionReady(void);

/* Cliff: [0]=AN6, [1]=AN37, [2]=AN4 */
void GetCliffADC(uint16_t out[3]);

/* Motor ISEN (0.25 ohm): left = RJ9/AN36, right = RA1/AN29 */
void GetMotorCurrentADC(uint16_t *left_counts, uint16_t *right_counts);

/* Convert 12-bit ADC counts on ISEN to milliamps.
 * V_ISEN = counts * 3.3 / 4095; I = V_ISEN / 0.25 ohm. */
uint32_t MotorCurrentCountsTomA(uint16_t counts);

/* Always-on peaks for control (PWM on-time); decay each control tick */
void GetMotorCurrentPeakADC(uint16_t *left_counts, uint16_t *right_counts);
void DecayMotorCurrentPeaks(void);

/* Peak capture over a PWM-aware window (updated in EOS ISR while active) */
void StartMotorCurrentMaxCapture(void);
void StopMotorCurrentMaxCapture(void);
void GetMotorCurrentMaxADC(uint16_t *left_counts, uint16_t *right_counts);
/* Samples in the 'i' window with I > 2.0 A, plus total completed scans */
void GetMotorCurrentCaptureOverCounts(uint16_t *left_over, uint16_t *right_over,
                                      uint16_t *samples);

#endif /* ADC_HAL_H */
