#pragma once

#ifdef STM32F4xx
#define MOT1_A  PB4 // 5 or 3 - PB4 or PB3
#define MOT1_B  PC7 // 9 or 10 - PC7 or PB6
#define MOT1_C  PB10 // 6, 11, or 13 - PB10, PA7, or PA5
#define MOT1_EN PA9 // 8
// #define MOT2_A  PB3 // 3
// #define MOT2_B  PC7 // 9
// #define MOT2_C  PA7 // 11
// #define MOT2_EN PB10 // 7
// encoder pinouts
#define ENC1_A PA2 // 2
#define ENC1_B PB3 // 4
// #define ENC2_A A4
// #define ENC2_B 12

#define CS_A A0
#define CS_B A2
#endif

#ifdef STM32F7xx
#define MOT1_A  D5 // PE11 // D5    TIM1 
#define MOT1_B  D9 // PD15 // D9    TIM4
#define MOT1_C  D6 // PE9 // D6     TIM1
#define MOT1_EN D8 // PF12 // D8

#define MOT2_A  D3 // PB3 // D3
#define MOT2_B  D10 // PD14 // D10
#define MOT2_C  D11 // PA7 // D11
#define MOT2_EN D7 // PF13 // D7

// encoder pinouts
// #define ENC1_A PA2 // 2
// #define ENC1_B PB3 // 4
// #define ENC2_A A4
// #define ENC2_B 12

#define CS_A A0
#define CS_B A2

#define CS2_A PC_0_ALT2 // A1 
#define CS2_B PF_3      // A3 // PA_1_ALT1 
#endif

#ifdef ARDUINO_B_G431B_ESC1

#endif

#ifdef ESP32
#define MOT1_A  26 // 5 or 3 - PB4 or PB3
#define MOT1_B  27 // 9 or 10 - PC7 or PB6
#define MOT1_C  33 // 6, 11, or 13 - PB10, PA7, or PA5
#define MOT1_EN 12 // 8
// #define MOT2_A  PB3 // 3
// #define MOT2_B  PC7 // 9
// #define MOT2_C  PA7 // 11
// #define MOT2_EN PB10 // 7
// encoder pinouts
// #define ENC1_A PA2 // 2
// #define ENC1_B PB3 // 4
// #define ENC2_A A4
// #define ENC2_B 12

#define CS_A 35
#define CS_B 34
#endif