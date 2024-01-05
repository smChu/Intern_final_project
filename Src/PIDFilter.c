/*
 * PIDFilter.c
 *
 *  Created on: Dec 18, 2023
 *      Author: dkdll
 */

#include "PIDFilter.h"

//filter
void FIR_Init(FIR *fir, int n_fir, double* b_fir)
{
	fir->n_fir = n_fir;

	fir->b_fir = (double*)malloc(sizeof(double)*(n_fir+1));
	fir->buffer = (double*)malloc(sizeof(double)*(n_fir+1));

	fir->b_fir = b_fir;
}

void FIR_Filter(FIR *fir, double rawdata)
{
	fir->output = 0;

	for(int i = 0; i < fir->n_fir; i++)
	{
		fir->buffer[i] = fir->buffer[i+1];
	}
	fir->buffer[fir->n_fir] = rawdata;

	for(int k = 0; k < fir->n_fir+1; k++)
	{
		fir->output += fir->b_fir[k] * fir->buffer[fir->n_fir-k];
	}
}

void IIR_Init(IIR* iir, int n_iir, double* a_iir, double* b_iir)
{
	iir->n_iir = n_iir;

	iir->a_iir = (double*)malloc(sizeof(double)*(n_iir+1));
	iir->b_iir = (double*)malloc(sizeof(double)*(n_iir+1));
	iir->buffer = (double*)malloc(sizeof(double)*(n_iir+1));
	iir->outputBuffer = (double*)malloc(sizeof(double)*(n_iir+1));

	iir->a_iir = a_iir;
	iir->b_iir = b_iir;
}

void IIR_Filter(IIR* iir, double rawdata)
{
	iir->output = 0;

	for(int i = 0; i < iir->n_iir; i++)
	{
		iir->buffer[i] = iir->buffer[i+1];
		iir->outputBuffer[i] = iir->outputBuffer[i+1];
	}
	iir->buffer[iir->n_iir] = rawdata;

	for(int k = 0; k < iir->n_iir+1; k++)
	{
		iir->outputBuffer[iir->n_iir] += iir->b_iir[k] * iir->buffer[iir->n_iir-k] - iir->a_iir[k] * iir->outputBuffer[iir->n_iir-k];
	}

	iir->output = iir->outputBuffer[iir->n_iir];
}

void tx_data(unsigned int rawdata, unsigned int filtereddata)
{
	unsigned char data[10] = {0, };

	data[0] = (rawdata % 10000)/1000 + 48;
	data[1] = (rawdata % 1000)/100 + 48;
	data[2] = (rawdata % 100)/10 + 48;
	data[3] = (rawdata % 10) + 48;

	data[4] = 32;

	data[5] = (filtereddata % 10000)/1000 + 48;
	data[6] = (filtereddata % 1000)/100 + 48;
	data[7] = (filtereddata % 100)/10 + 48;
	data[8] = (filtereddata % 10) + 48;

	data[9] = 13;

	for(int i = 0; i < 10; i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART3));
		LL_USART_TransmitData8(USART3, data[i]);
	}
}

//motor control
void Get_Motor_Status(ENCODER* dst, TIM_TypeDef* TIMx)
{
	dst->past = dst->now;
	dst->now = TIMx->CNT;
	dst->m1 = dst->now - dst->past;

	if(dst->m1 < -60000)
	{
		dst->m1 += 65535;
	}
	else if(dst->m1 > 60000)
	{
		dst->m1 -= 65535;
	}

	dst->RPM = (60.0 * dst->m1) / (Tc * CPR);
	dst->DEGREE += (dst->m1 / (CPR / 360.0));
}

void Duty_Control_Velocity(DUTY* dst, DIRECTION* DIRx, GPIO_TypeDef* GPIOx, uint16_t PINx, TIM_TypeDef* TIMx, uint8_t CHx, int target)
{
	dst->duty = target;

	if(dst->duty > dst->dutylimit)
	{
		dst->duty = dst->dutylimit;
	}
	else if(dst->duty < -dst->dutylimit)
	{
		dst->duty = -dst->dutylimit;
	}

	if(dst->duty < 0)
	{
		if(DIRx->FrontMotorDirection == RESET)
		{
			LL_GPIO_SetOutputPin(GPIOx, PINx);
			//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_SET);

			switch(CHx)
			{
			case 1:
			{
				LL_TIM_OC_SetCompareCH1(TIMx, -dst->duty);
				//TIMx->CCR1 = -dst->duty;
				break;
			}
			case 2:
			{
				LL_TIM_OC_SetCompareCH2(TIMx, -dst->duty);
				//TIMx->CCR2 = -dst->duty;
				break;
			}
			case 3:
			{
				LL_TIM_OC_SetCompareCH3(TIMx, -dst->duty);
				//TIMx->CCR3 = -dst->duty;
				break;
			}
			case 4:
			{
				LL_TIM_OC_SetCompareCH4(TIMx, -dst->duty);
				//TIMx->CCR4 = -dst->duty;
				break;
			}
			}
		}
		else
		{
			LL_GPIO_ResetOutputPin(GPIOx, PINx);
			//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_RESET);

			switch(CHx)
			{
			case 1:
			{
				LL_TIM_OC_SetCompareCH1(TIMx, -dst->duty);
				//TIMx->CCR1 = -dst->duty;
				break;
			}
			case 2:
			{
				LL_TIM_OC_SetCompareCH2(TIMx, -dst->duty);
				//TIMx->CCR2 = -dst->duty;
				break;
			}
			case 3:
			{
				LL_TIM_OC_SetCompareCH3(TIMx, -dst->duty);
				//TIMx->CCR3 = -dst->duty;
				break;
			}
			case 4:
			{
				LL_TIM_OC_SetCompareCH4(TIMx, -dst->duty);
				//TIMx->CCR4 = -dst->duty;
				break;
			}
			}
		}
	}
	else
	{
		if(DIRx->FrontMotorDirection == RESET)
		{
			LL_GPIO_ResetOutputPin(GPIOx, PINx);
			//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_RESET);

			switch(CHx)
			{
			case 1:
			{
				LL_TIM_OC_SetCompareCH1(TIMx, dst->duty);
				//TIMx->CCR1 = dst->duty;
				break;
			}
			case 2:
			{
				LL_TIM_OC_SetCompareCH2(TIMx, dst->duty);
				//TIMx->CCR2 = dst->duty;
				break;
			}
			case 3:
			{
				LL_TIM_OC_SetCompareCH3(TIMx, dst->duty);
				//TIMx->CCR3 = dst->duty;
				break;
			}
			case 4:
			{
				LL_TIM_OC_SetCompareCH4(TIMx, dst->duty);
				//TIMx->CCR4 = dst->duty;
				break;
			}
			}
		}
		else
		{
			LL_GPIO_SetOutputPin(GPIOx, PINx);
			//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_SET);

			switch(CHx)
			{
			case 1:
			{
				LL_TIM_OC_SetCompareCH1(TIMx, dst->duty);
				//TIMx->CCR1 = dst->duty;
				break;
			}
			case 2:
			{
				LL_TIM_OC_SetCompareCH2(TIMx, dst->duty);
				//TIMx->CCR2 = dst->duty;
				break;
			}
			case 3:
			{
				LL_TIM_OC_SetCompareCH3(TIMx, dst->duty);
				//TIMx->CCR3 = dst->duty;
				break;
			}
			case 4:
			{
				LL_TIM_OC_SetCompareCH4(TIMx, dst->duty);
				//TIMx->CCR4 = dst->duty;
				break;
			}
			}
		}
	}
}

void PID_Control(PID* dst, float target, float input)
{
	dst->input = input;
	dst->target = target;

	dst->E = dst->target - dst->input;
	dst->sumE += dst->E;
	dst->diffE = dst->E - dst->E_old;

	if(dst->sumE > sumELimit)
	{
		dst->sumE = sumELimit;
	}
	else if(dst->sumE < -sumELimit)
	{
		dst->sumE = -sumELimit;
	}

	dst->output = (dst->kP * dst->E + dst->kI * dst->sumE + dst->kD * dst->diffE) / division;

	dst->E_old = dst->E;

	if(dst->output > dst->outputlimit)
	{
		dst->output = dst->outputlimit;
	}
	else if(dst->output < -dst->outputlimit)
	{
		dst->output = -dst->outputlimit;
	}
}

void PID_Control_Velocity(PID* dst, DIRECTION* DIRx, ENCODER* ENx, GPIO_TypeDef* GPIOx, uint16_t PINx, TIM_TypeDef* TIMx, uint8_t CHx, int target, FIR* fir)
{
	if(DIRx->FrontEncoderDirection == CCW)
	{
		PID_Control(dst, -target, ENx->RPM);
	}
	else
	{
		PID_Control(dst, target, ENx->RPM);
	}

//	FIR_Filter(fir, dst->output);
//	dst->output = fir->output;

	//motor
	if(dst->output < 0)
	{
		if(DIRx->FrontEncoderDirection == CCW)
		{
			if(DIRx->FrontMotorDirection == RESET)
			{
				LL_GPIO_ResetOutputPin(GPIOx, PINx);
				//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_RESET);
			}
			else
			{
				LL_GPIO_SetOutputPin(GPIOx, PINx);
				//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_SET);
			}
		}
		else
		{
			if(DIRx->FrontMotorDirection == RESET)
			{
				LL_GPIO_SetOutputPin(GPIOx, PINx);
				//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_SET);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOx, PINx);
				//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_RESET);
			}
		}

		switch(CHx)
		{
		case 1:
		{
			LL_TIM_OC_SetCompareCH1(TIMx, -dst->output);
			//TIMx->CCR1 = -dst->output;
			break;
		}
		case 2:
		{
			LL_TIM_OC_SetCompareCH2(TIMx, -dst->output);
			//TIMx->CCR2 = -dst->output;
			break;
		}
		case 3:
		{
			LL_TIM_OC_SetCompareCH3(TIMx, -dst->output);
			//TIMx->CCR3 = -dst->output;
			break;
		}
		case 4:
		{
			LL_TIM_OC_SetCompareCH4(TIMx, -dst->output);
			//TIMx->CCR4 = -dst->output;
			break;
		}
		}
	}
	else
	{
		if(DIRx->FrontEncoderDirection == CCW)
		{
			if(DIRx->FrontMotorDirection == RESET)
			{
				LL_GPIO_SetOutputPin(GPIOx, PINx);
				//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_SET);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOx, PINx);
				//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_RESET);
			}
		}
		else
		{
			if(DIRx->FrontMotorDirection == RESET)
			{
				LL_GPIO_ResetOutputPin(GPIOx, PINx);
				//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_RESET);
			}
			else
			{
				LL_GPIO_SetOutputPin(GPIOx, PINx);
				//HAL_GPIO_WritePin(GPIOx, PINx, GPIO_PIN_SET);
			}
		}

		switch(CHx)
		{
		case 1:
		{
			LL_TIM_OC_SetCompareCH1(TIMx, dst->output);
			//TIMx->CCR1 = dst->output;
			break;
		}
		case 2:
		{
			LL_TIM_OC_SetCompareCH2(TIMx, dst->output);
			//TIMx->CCR2 = dst->output;
			break;
		}
		case 3:
		{
			LL_TIM_OC_SetCompareCH3(TIMx, dst->output);
			//TIMx->CCR3 = dst->output;
			break;
		}
		case 4:
		{
			LL_TIM_OC_SetCompareCH4(TIMx, dst->output);
			//TIMx->CCR4 = dst->output;
			break;
		}
		}
	}
}

