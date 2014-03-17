/*
 * vars.h
 *
 *  Created on: Mar 15, 2014
 *      Author: harrison
 */

#ifndef VARS_H_
#define VARS_H_

//speed pid vars//
#define kp 2
#define kd 5
#define ki 0

//Gyro Ports//
#define GYROADC ADC0_SE13
#define ANGLEADC ADC0_SE12
#define RZADC ADC1_SE10
#define RXADC ADC1_SE11



//Encoders Ports Definition//
#ifdef FTM1_QDPHA
#undef FTM1_QDPHA
#define FTM1_QDPHA PTB0
#endif

#ifdef FTM1_QDPHB
#undef FTM1_QDPHB
#define FTM1_QDPHB PTB1
#endif

#ifdef FTM2_QDPHA
#undef FTM2_QDPHA
#define FTM2_QDPHA PTB18
#endif

#ifdef FTM2_QDPHB
#undef FTM2_QDPHB
#define FTM2_QDPHB PTB19
#endif

#ifdef FTM2_QDPHB
#undef FTM2_QDPHB
#define FTM2_QDPHB PTB19
#endif

//Motors Ports Definition//
#ifdef FTM0_CH0
#undef FTM0_CH0
#define FTM0_CH0 PTC1
#endif

#ifdef FTM0_CH1
#undef FTM0_CH1
#define FTM0_CH1 PTC2
#endif

#endif /* VARS_H_ */
