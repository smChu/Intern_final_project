/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PIDFilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//direction
extern DIRECTION Direction1;
extern DIRECTION Direction2;

extern DUTY MotorDutyVelocity1;
extern DUTY MotorDutyVelocity2;

//encoder
extern ENCODER Encoder1;
extern ENCODER Encoder2;

//pid
extern PID RFPID;
extern PID RBPID;

//filter
extern FIR RBfir;
extern FIR RFfir;
extern double f[];

int target1 = 0;
int target2 = 0;

int nowrpm1;
int nowrpm2;
int nowduty1;
int nowduty2;
double FPID1;
double FPID2;

//uart
extern uint8_t rxBuffer[256];
int data[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 10};
int restoredDXL[5] = {2300,2048,2048,2048,230};
unsigned int DXL1, DXL2, DXL3, DXL4,DXL5;
int torque_flag = 0;

int a = 2350;
int b = 2480;
int c = 1026;
int d = 1618;

int errorCheck;
int allcnt;
int errorcnt;

int cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void PID_MOTORS()
{
	Get_Motor_Status(&Encoder1, TIM3); //NP
	Get_Motor_Status(&Encoder2, TIM4); //NP

	//RB
	PID_Control_Velocity(&RBPID, &Direction1, &Encoder1, GPIOC, LL_GPIO_PIN_8, TIM1, 1, target1, &RBfir);
	//RF
	PID_Control_Velocity(&RFPID, &Direction2, &Encoder2, GPIOC, LL_GPIO_PIN_9, TIM1, 2, target2, &RFfir);
}

void Duty_Velocity()
{
	Get_Motor_Status(&Encoder1, TIM3); //NP
	Get_Motor_Status(&Encoder2, TIM4); //NP

	Duty_Control_Velocity(&MotorDutyVelocity1, &Direction1, GPIOC, LL_GPIO_PIN_8, TIM1, 1, target1);
	Duty_Control_Velocity(&MotorDutyVelocity2, &Direction2, GPIOC, LL_GPIO_PIN_9, TIM1, 2, target2);
}


unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void Torque_Enable_MX_Sync_Write()//index
{
   unsigned char txBuf[26];
   unsigned short crc, CRC_L, CRC_H;
   // header 1~4
   txBuf[0] = 0xFF;
   txBuf[1] = 0xFF;
   txBuf[2] = 0xFD;
   txBuf[3] = 0x00;
   // broadcast id
   txBuf[4] = 0xFE;
   // length
   txBuf[5] = 0x13;
   txBuf[6] = 0x00;
   // inst: sync write instruction packet
   txBuf[7] = 0x83;
   // address
   txBuf[8] = 0x40;
   txBuf[9] = 0x00;
   // length per id
   txBuf[10] = 0x01;
   txBuf[11] = 0x00;
   // id 1 torque enable
   txBuf[12] = 0x01;
   txBuf[13] = 0x01;
   // id 2 torque enable
   txBuf[14] = 0x02;
   txBuf[15] = 0x01;
   // id 3 torque enable
   txBuf[16] = 0x03;
   txBuf[17] = 0x01;
   // id 4 torque enable
   txBuf[18] = 0x04;
   txBuf[19] = 0x01;

   // id 5 torque enable
   txBuf[20] = 0x05;
   txBuf[21] = 0x01;
   // id 6 torque enable
   txBuf[22] = 0x06;
   txBuf[23] = 0x01;

   //crc
   crc = update_crc(0, txBuf, 24);
   CRC_L = (crc & 0xFF);
   CRC_H = (crc >> 8) & 0xFF;
   txBuf[24] = CRC_L;
   txBuf[25] = CRC_H;

   for (int i = 0; i < 26; i++) {
      while (!LL_USART_IsActiveFlag_TXE(USART3));
      LL_USART_TransmitData8(USART3, txBuf[i]);
   }
}

void Driving_MX_Write(unsigned int id, unsigned int velocity, unsigned int position)
{
   unsigned char txBuf[20];
   unsigned char ID = (unsigned char) id;
   unsigned char Pos_L, Pos_H;
   Pos_L = (unsigned char) (position & 0xFF);
   Pos_H = (unsigned char) ((position >> 8) & 0xFF);
   unsigned char Vel_L, Vel_H;
   Vel_L = (unsigned char) (velocity & 0xFF);
   Vel_H = (unsigned char) ((velocity >> 8) & 0xFF);
   unsigned short crc, CRC_L, CRC_H;
   // header 1~4
   txBuf[0] = 0xFF;
   txBuf[1] = 0xFF;
   txBuf[2] = 0xFD;
   txBuf[3] = 0x00;
   txBuf[4] = ID;
   // length
   txBuf[5] = 0x0D;
   txBuf[6] = 0x00;
   // inst: write
   txBuf[7] = 0x03;
   // address
   txBuf[8] = 0x70; //profile velocity Address: 112 = 0x70
   txBuf[9] = 0x00;
   // profile velocity
   txBuf[10] = Vel_L; //112: profile velocity 0x2D = 10rpm
   txBuf[11] = Vel_H;
   txBuf[12] = 0x00;
   txBuf[13] = 0x00; //4byte
   // Goal Position
   txBuf[14] = Pos_L;   //116: Goal Position 0~4095
   txBuf[15] = Pos_H;
   txBuf[16] = 0x00;
   txBuf[17] = 0x00;   //4byte
   // crc
   crc = update_crc(0, txBuf, 18);
   CRC_L = (crc & 0xFF);
   CRC_H = (crc >> 8) & 0xFF;
   txBuf[18] = CRC_L;
   txBuf[19] = CRC_H;
   //transmit to DXL
   for (int i = 0; i < 20; i++) {
      while (!LL_USART_IsActiveFlag_TXE(USART3));
      LL_USART_TransmitData8(USART3, txBuf[i]);
   }
}

void Driving_MX_WheelMode_Write(int velocity) {
   unsigned char txBuf[16];
   unsigned char Vel_1, Vel_2, Vel_3, Vel_4;
   Vel_1 = (unsigned char) (velocity & 0xFF);
   Vel_2 = (unsigned char) ((velocity >> 8) & 0xFF);
   Vel_3 = (unsigned char) ((velocity >> 16) & 0xFF);
   Vel_4 = (unsigned char) ((velocity >> 24) & 0xFF);
   unsigned short crc, CRC_L, CRC_H;
   // header 1~4
   txBuf[0] = 0xFF;
   txBuf[1] = 0xFF;
   txBuf[2] = 0xFD;
   txBuf[3] = 0x00;
   // ID
   txBuf[4] = 0x06;
   // length
   txBuf[5] = 0x09;
   txBuf[6] = 0x00;
   // inst: write
   txBuf[7] = 0x03;
   // address
   txBuf[8] = 0x68; //goal velocity Address: 104 = 0x68
   txBuf[9] = 0x00;
   // goal velocity
   txBuf[10] = Vel_1; //104: goal velocity
   txBuf[11] = Vel_2;
   txBuf[12] = Vel_3;
   txBuf[13] = Vel_4; //4byte
   // crc
   crc = update_crc(0, txBuf, 14);
   CRC_L = (crc & 0xFF);
   CRC_H = (crc >> 8) & 0xFF;
   txBuf[14] = CRC_L;
   txBuf[15] = CRC_H;
   //transmit to DXL
   for (int i = 0; i < 16; i++) {
      while (!LL_USART_IsActiveFlag_TXE(USART3));
      LL_USART_TransmitData8(USART3, txBuf[i]);
   }
}

void SET_DualMode_MX_Write()
{
   unsigned char txBuf[13];
   unsigned short crc, CRC_L, CRC_H;
   // header 1~4
   txBuf[0] = 0xFF;
   txBuf[1] = 0xFF;
   txBuf[2] = 0xFD;
   txBuf[3] = 0x00;
   // ID
   txBuf[4] = 0x03;
   // length
   txBuf[5] = 0x06;
   txBuf[6] = 0x00;
   // inst: write
   txBuf[7] = 0x03;
   // address
   txBuf[8] = 0x0A;//driving mode: 0x0A
   txBuf[9] = 0x00;
   // set dual mode
   txBuf[10] = 0x02;

   // crc
   crc = update_crc(0, txBuf, 11);
   CRC_L = (crc & 0xFF);
   CRC_H = (crc >> 8) & 0xFF;
   txBuf[11] = CRC_L;
   txBuf[12] = CRC_H;
   //transmit to DXL
   for (int i = 0; i < 13; i++) {
      while (!LL_USART_IsActiveFlag_TXE(USART3));
      LL_USART_TransmitData8(USART3, txBuf[i]);
   }
}

void SET_WheelMode_MX_Write()
{
   unsigned char txBuf[14];
   unsigned short crc, CRC_L, CRC_H;
   // header 1~4
   txBuf[0] = 0xFF;
   txBuf[1] = 0xFF;
   txBuf[2] = 0xFD;
   txBuf[3] = 0x00;
   // ID
   txBuf[4] = 0x06;
   // length
   txBuf[5] = 0x07;
   txBuf[6] = 0x00;
   // inst: write
   txBuf[7] = 0x03;
   // address
   txBuf[8] = 0x0A;//driving mode: 0x0A
   txBuf[9] = 0x00;
   // set normal mode
   txBuf[10] = 0x00;
   // operate with goal velocity
   txBuf[11] = 0x01;

   // crc
   crc = update_crc(0, txBuf, 12);
   CRC_L = (crc & 0xFF);
   CRC_H = (crc >> 8) & 0xFF;
   txBuf[12] = CRC_L;
   txBuf[13] = CRC_H;
   //transmit to DXL
   for (int i = 0; i < 14; i++) {
      while (!LL_USART_IsActiveFlag_TXE(USART3));
      LL_USART_TransmitData8(USART3, txBuf[i]);
   }
}

void Driving_MX_Sync_Write(unsigned int position1, unsigned int position2, unsigned int position3, unsigned int position4)
{
   unsigned char txBuf[50];
   unsigned char Pos_L1, Pos_H1, Pos_L2, Pos_H2, Pos_L3, Pos_H3, Pos_L4, Pos_H4;
   Pos_L1 = (unsigned char) (position1 & 0xFF);
   Pos_H1 = (unsigned char) ((position1 >> 8) & 0xFF);
   Pos_L2 = (unsigned char) (position2 & 0xFF);
   Pos_H2 = (unsigned char) ((position2 >> 8) & 0xFF);
   Pos_L3 = (unsigned char) (position3 & 0xFF);
   Pos_H3 = (unsigned char) ((position3 >> 8) & 0xFF);
   Pos_L4 = (unsigned char) (position4 & 0xFF);
   Pos_H4 = (unsigned char) ((position4 >> 8) & 0xFF);
   unsigned short crc, CRC_L, CRC_H;
   //header 1~4
   txBuf[0] = 0xFF;
   txBuf[1] = 0xFF;
   txBuf[2] = 0xFD;
   txBuf[3] = 0x00;
   //id
   txBuf[4] = 0xFE;
   //length
   txBuf[5] = 0x2B;
   txBuf[6] = 0x00;
   //inst: sync write instruction packet
   txBuf[7] = 0x83;
   //address
   txBuf[8] = 0x70;
   txBuf[9] = 0x00;
   txBuf[10] = 0x08;
   txBuf[11] = 0x00;

   txBuf[12] = 0x01; //id 1
   txBuf[13] = 0x2D; //velocity
   txBuf[14] = 0x00;
   txBuf[15] = 0x00;
   txBuf[16] = 0x00;
   txBuf[17] = Pos_L1; //position
   txBuf[18] = Pos_H1;
   txBuf[19] = 0x00;
   txBuf[20] = 0x00;

   txBuf[21] = 0x02; //id 2
   txBuf[22] = 0x2D; //velocity
   txBuf[23] = 0x00;
   txBuf[24] = 0x00;
   txBuf[25] = 0x00;
   txBuf[26] = Pos_L2; //position
   txBuf[27] = Pos_H2;
   txBuf[28] = 0x00;
   txBuf[29] = 0x00;

   txBuf[30] = 0x04; //id 4
   txBuf[31] = 0x2D; //velocity
   txBuf[32] = 0x00;
   txBuf[33] = 0x00;
   txBuf[34] = 0x00;
   txBuf[35] = Pos_L3; //position
   txBuf[36] = Pos_H3;
   txBuf[37] = 0x00;
   txBuf[38] = 0x00;

   txBuf[39] = 0x05; //id 5
   txBuf[40] = 0x2D; //velocity
   txBuf[41] = 0x00;
   txBuf[42] = 0x00;
   txBuf[43] = 0x00;
   txBuf[44] = Pos_L4; //position
   txBuf[45] = Pos_H4;
   txBuf[46] = 0x00;
   txBuf[47] = 0x00;

   //crc
   crc = update_crc(0, txBuf, 48);
   CRC_L = (crc & 0xFF);
   CRC_H = (crc >> 8) & 0xFF;
   txBuf[48] = CRC_L;
   txBuf[49] = CRC_H;

   for (int i = 0; i < 50; i++) {
      while (!LL_USART_IsActiveFlag_TXE(USART3));
      LL_USART_TransmitData8(USART3, txBuf[i]);
   }
}

void readPacket(uint8_t *buffer) //serial to here
{
   errorCheck = 0;
   unsigned short received_crc;
   unsigned short packet_crc;

   if (buffer[0] == 0xFF);
   else
      errorCheck = 1;
   if (buffer[1] == 0xFF);
   else
      errorCheck = 1;
   if (buffer[2] == 0xFD);
   else
      errorCheck = 1;
   if (buffer[3] == 0x00);
   else
      errorCheck = 1;

   allcnt++;

   if(errorCheck == 1)
   {
      errorcnt++;
   }

   received_crc = (buffer[17] << 8) | buffer[16]; //crc index
   packet_crc = update_crc(0, buffer, 16);

   if ((received_crc == packet_crc) && (errorCheck == 0)) {
      for (int i = 0; i < 12; i++) {
         data[i] = buffer[4 + i];
      }
   }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if (LL_USART_IsActiveFlag_IDLE(USART3)) {

		readPacket(rxBuffer);


		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

		LL_DMA_ClearFlag_DME1(DMA1);
		LL_DMA_ClearFlag_FE1(DMA1);
		LL_DMA_ClearFlag_HT1(DMA1);
		LL_DMA_ClearFlag_TC1(DMA1);
		LL_DMA_ClearFlag_TE1(DMA1);

		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);

		LL_USART_ClearFlag_IDLE(USART3);
	}
  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM6))
	{
		PID_MOTORS();

		//Duty_Velocity();
		//DXL
				for (int i = 0; i < 5; i++) {
					restoredDXL[i] = (data[1 + (2 * i)] << 8) | data[2 * i];
				}

				if (restoredDXL[4] >= 230 && restoredDXL[4] <= 460) {
					restoredDXL[4] -= 230;
				} else if (restoredDXL[4] >= 0 && restoredDXL[4] <= 229) {
					restoredDXL[4] = restoredDXL[4] - 230;
				}

				DXL1 = restoredDXL[0];
				DXL2 = restoredDXL[1];
				DXL3 = restoredDXL[2];
				DXL4 = restoredDXL[3];
				DXL5 = restoredDXL[4];

				//right
				if (data[11] >= 10 && data[11] <= 20) {
					target1 = (2) * (data[11] - 10); //b
					target2 = (2.8) * (data[11] - 10); //f
				} else if (data[11] >= 0 && data[11] <= 10) {
					target1 = (-1) * (2) * (10 - data[11]);
					target2 = (-1) * (2.8) * (10 - data[11]);
				}

				//torque enable, set wheel mode, set dual mode
				if (torque_flag == 0) {
					Torque_Enable_MX_Sync_Write();
					LL_mDelay(5);
					SET_WheelMode_MX_Write();
					LL_mDelay(5);
					SET_DualMode_MX_Write();
					LL_mDelay(5);
					torque_flag = 1;
				}

				//      Driving_MX_Write(1, 45, restoredDXL[0]);
				//      LL_mDelay(5);
				//      Driving_MX_Write(2, 45, restoredDXL[1]);
				//      LL_mDelay(5);
				//      Driving_MX_Write(4, 45, restoredDXL[2]);
				//      LL_mDelay(5);

				//      Driving_MX_Write(1, 45, a);
				//      LL_mDelay(5);
				//      Driving_MX_Write(2, 45, b);
				//      LL_mDelay(5);
				//      Driving_MX_Write(4, 45, c);
				//      LL_mDelay(5);
				//      Driving_MX_Write(5, 45, d);
				//      LL_mDelay(5);

				if (DXL1 == 0 && DXL2 == 0 && DXL3 == 0 && DXL4 == 0) {
					Driving_MX_Sync_Write(a, b, c, d);
				} else {
					Driving_MX_Sync_Write(DXL1, DXL2, DXL3, DXL4);
					LL_mDelay(5);
					Driving_MX_WheelMode_Write(DXL5);
				}
				LL_mDelay(5);


		nowrpm1 = Encoder1.RPM;
		nowrpm2 = Encoder2.RPM;
		nowduty1 = RBPID.output; //MAX 900
		nowduty2 = RFPID.output;
		FPID1 = RBfir.output;
		FPID2 = RFfir.output;


		cnt++;

		LL_TIM_ClearFlag_UPDATE(TIM6);
	}
  /* USER CODE END TIM6_DAC_IRQn 0 */

  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
