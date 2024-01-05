/*
 * PIDFilter.h
 *
 *  Created on: Dec 18, 2023
 *      Author: dkdll
 */

#ifndef INC_PIDFILTER_H_
#define INC_PIDFILTER_H_

#include <stdlib.h>
#include "main.h"

#define CW 0
#define CCW 1
#define RESET 0
#define SET 1

#define CPR 8192 //Encoder PPR * 4
#define Tc 0.01 //TMR IT

#define sumELimit 1000 //Don't touch
#define division 1000 //Don't touch

typedef struct _DIRECTION
{
	int FrontMotorDirection;
	int FrontEncoderDirection;
}DIRECTION;

typedef struct _ENCODER
{
	int now;
	int past;
	int m1;

	float RPM;
	long long DEGREE;
}ENCODER;

typedef struct _DUTY
{
	int duty;
	int dutylimit;
}DUTY;

typedef struct _PID
{
    float input;
	float target;

	float E;
	float E_old;

	float sumE;
	float diffE;

	float kP;
	float kI;
	float kD;

	float output;
	float outputlimit;
}PID;

typedef struct _FIR
{
	int n_fir;

	double *b_fir;

	double *buffer;

	double output;
}FIR;

typedef struct _IIR
{
	int n_iir;

	double *a_iir;
	double *b_iir;

	double *buffer;

	double *outputBuffer;
	double output;
}IIR;

void Get_Motor_Status(ENCODER* dst, TIM_TypeDef* TIMx);
void Duty_Control_Velocity(DUTY* dst, DIRECTION* DIRx, GPIO_TypeDef* GPIOx, uint16_t PINx, TIM_TypeDef* TIMx, uint8_t CHx, int target);
void PID_Control(PID* dst, float target, float input);
void PID_Control_Velocity(PID* dst, DIRECTION* DIRx, ENCODER* ENx, GPIO_TypeDef* GPIOx, uint16_t PINx, TIM_TypeDef* TIMx, uint8_t CHx, int target, FIR* fir);

void FIR_Init(FIR *fir, int n_fir, double* b_fir);
void FIR_Filter(FIR *fir, double rawdata);
void IIR_Init(IIR* iir, int n_iir, double* a_iir, double* b_iir);
void IIR_Filter(IIR* iir, double rawdata);
void tx_data(unsigned int rawdata, unsigned int filtereddata);

#endif /* INC_PIDFILTER_H_ */
