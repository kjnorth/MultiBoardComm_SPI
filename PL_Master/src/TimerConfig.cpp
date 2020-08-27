/** 
 * @Author: Kodiak North 
 * @Date: 2020-08-26 14:01:36 
 * @Desc: implements the TimerConfig class
 */

#include "TimerConfig.h"

// **** FUNCTION NOTES ****
/** @InitTimernISRs
 * @Desc: initializes Timernx compare match interrupts on all x channels
 * @Param - freqHzx: interrupt frequency in Hz for the x channel
 * @Note: pass 0 for freqHzx if don't want that channel to interrupt,
 * or do not start that channel with the StartTimernISRs function
 * @Note: Timernx ISRs do not start firing until StartTimernISRs is called
 */

/** @StartTimernISRs
 * @Desc: Starts a timer's output compare interrupts on channels
 * passed in startMask param
 * @Param - startMask: a mask such as (A | C) 
 * @Note: does not start channel if it's already started */

/** @StopTimernISRs
 * @Desc: Stops a timer's output compare interrupts on channels
 * passed in stopMask param
 * @Param - stopMask: a mask such as (A | C) 
 * @Note: does not stop channel if it's already stopped */
// **** END FUNCTION NOTES ****

// **** PRIVATE DEFINES ****
#define CLK_SPEED           16000000
/** NOTE: if changing PRESCALER_Tx, must change TCCRxB register appropriately to match 
 * NOTE: min and max frequencies are extremely unreliable. pick frequency well in between range */
#define PRESCALER_T0        8 // min freq is 31 Hz, max freq is 2 MHz
#define PRESCALER_T1        8 // min freq is 31 Hz, max freq is 2 MHz
#define PRESCALER_T2        1024 // min freq is 62 Hz, max freq is 15.5 kHz
#define PRESCALER_T3        8 // min freq is 31 Hz, max freq is 2 MHz
#define PRESCALER_T4        8 // min freq is 31 Hz, max freq is 2 MHz
#define PRESCALER_T5        8 // min freq is 31 Hz, max freq is 2 MHz
#define TIMER_8BIT_MAX      255 // timer 0 and 2 are 8 bit timers
#define TIMER_16BIT_MAX     65535 // timer 1, 3, 4, and 5 are 16 bit timers
#define OCRnx_CONFIG_PERIOD(freqHz,prescaler) ((CLK_SPEED / (freqHz * prescaler)) - 1) // n: 0-5; x: A,B,C
// **** END PRIVATE DEFINES ****

// **** DEFINITION OF GLOBAL VARIABLES ****
timer_ocr_period_t timer1;
timer_ocr_period_t timer2;
timer_ocr_period_t timer3;
timer_ocr_period_t timer4;
timer_ocr_period_t timer5;
// **** END DEFINITION OF GLOBAL VARIABLES ****

// PWM unusable on Mega pins 11, 12; on Uno/Nano pins 9, 10
void TimerConfig::InitTimer1ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC) {
  TCCR1A = 0; TCCR1B = 0; TCCR1C = 0; // set TCCRnx registers to 0
  TCNT1 = 0; // set initial counter value to 0
  /** NOTE: if changing TCCRxB prescaler bits, must make PRESCALER_Tx match new value in #define above */
  TCCR1B |= (1 << CS11); // set CS11 bit for 8 prescaler
  timer1.aPeriod = OCRnx_CONFIG_PERIOD(freqHzA, PRESCALER_T1);
  timer1.bPeriod = OCRnx_CONFIG_PERIOD(freqHzB, PRESCALER_T1);
  timer1.cPeriod = OCRnx_CONFIG_PERIOD(freqHzC, PRESCALER_T1);

  if (timer1.aPeriod <= TIMER_16BIT_MAX) {
    OCR1A = timer1.aPeriod;
  }
  else {
    OCR1A = 0;
    LogInfo("TimerConfig::InitTimer1ISRs - ERROR timer1 A frequency is too low, increase frequency or alter prescaler\n");
  }

  if (timer1.bPeriod <= TIMER_16BIT_MAX) {
    OCR1B = timer1.bPeriod;
  }
  else {
    OCR1B = 0;
    LogInfo("TimerConfig::InitTimer1ISRs - ERROR timer1 B frequency is too low, increase frequency or alter prescaler\n");
  }

  if (timer1.cPeriod <= TIMER_16BIT_MAX) {
    OCR1C = timer1.cPeriod;
  }
  else {
    OCR1C = 0;
    LogInfo("TimerConfig::InitTimer1ISRs - ERROR timer1 C frequency is too low, increase frequency or alter prescaler\n");
  }
}

void TimerConfig::StartTimer1ISRs(uint8_t startMask) {
  uint8_t aStatus = TIMSK1 & (1 << OCIE1A); // nonzero if A channel is already started
  uint8_t bStatus = TIMSK1 & (1 << OCIE1B); // nonzero if B channel is already started
  uint8_t cStatus = TIMSK1 & (1 << OCIE1C); // nonzero if C channel is already started

  if ((startMask & A) && !aStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 |= (1 << OCIE1A);
      OCR1A = TCNT1 + timer1.aPeriod;
    }
  }

  if ((startMask & B) && !bStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 |= (1 << OCIE1B);
      OCR1B = TCNT1 + timer1.bPeriod;
    }
  }

  if ((startMask & C) && !cStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 |= (1 << OCIE1C);
      OCR1C = TCNT1 + timer1.cPeriod;
    }
  }
}

void TimerConfig::StopTimer1ISRs(uint8_t stopMask) {
  uint8_t aStatus = TIMSK1 & (1 << OCIE1A); // nonzero if A channel is started
  uint8_t bStatus = TIMSK1 & (1 << OCIE1B); // nonzero if B channel is started
  uint8_t cStatus = TIMSK1 & (1 << OCIE1C); // nonzero if C channel is started

  if ((stopMask & A) && aStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 &= ~(1 << OCIE1A);
      OCR1A = TCNT1 + timer1.aPeriod;
    }
  }

  if ((stopMask & B) && bStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 &= ~(1 << OCIE1B);
      OCR1B = TCNT1 + timer1.bPeriod;
    }
  }

  if ((stopMask & C) && cStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 &= ~(1 << OCIE1C);
      OCR1C = TCNT1 + timer1.cPeriod;
    }
  }
}

// PWM unusable on Mega pins 9, 10; on Uno/Nano pins 3, 11
void TimerConfig::InitTimer2ISRs(uint16_t freqHzA, uint16_t freqHzB) {

}

void TimerConfig::StartTimer2ISRs(uint8_t startMask) {

}

void TimerConfig::StopTimer2ISRs(uint8_t stopMask) {

}

#if defined(__AVR_ATmega2560__) // only prototype/implement these functions if using Atmega2560 chip
// PWM unusable on Mega pins 2, 3, 5
void TimerConfig::InitTimer3ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC) {

}

void TimerConfig::StartTimer3ISRs(uint8_t startMask) {

}

void TimerConfig::StopTimer3ISRs(uint8_t stopMask) {

}

// PWM unusable on Mega pins 6, 7, 8
void TimerConfig::InitTimer4ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC) {
  return;
}

void TimerConfig::StartTimer4ISRs(uint8_t startMask) {
  return;
}

void TimerConfig::StopTimer4ISRs(uint8_t stopMask) {
  return;
}

// PWM unusable on Mega pins 44, 45, 46
void TimerConfig::InitTimer5ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC) {
  return;
}

void TimerConfig::StartTimer5ISRs(uint8_t startMask) {
  return;
}

void TimerConfig::StopTimer5ISRs(uint8_t stopMask) {
  return;
}
#endif