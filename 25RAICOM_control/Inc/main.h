/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define ADC_BAT_Pin GPIO_PIN_10
#define ADC_BAT_GPIO_Port GPIOF
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define HW0_Pin GPIO_PIN_0
#define HW0_GPIO_Port GPIOC
#define HW1_Pin GPIO_PIN_1
#define HW1_GPIO_Port GPIOC
#define HW2_Pin GPIO_PIN_2
#define HW2_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOD
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2??��??��
	short Y_data; //2 bytes //2??��??��
	short Z_data; //2 bytes //2??��??��
}Mpu6050_Data;


// ��Ƭ�����͸�minipc���ݵĽṹ�嶨��
typedef struct {
    float       curr_yaw;
    float       curr_pitch;
    float       curr_omega;
    uint8_t     state;
    uint8_t     autoaim;
    uint8_t     enemy_color;
} __attribute__((packed)) OutputData;

// minipc���͸���Ƭ�����ݵĽṹ�嶨��
typedef struct {
    float       shoot_yaw;
    float       shoot_pitch;
    uint8_t     fire;
    uint8_t     target_id;
} __attribute__((packed)) InputData;

typedef struct  {
    uint8_t     sof;
    uint8_t     crc8;
} __attribute__((packed))FrameHeader;

typedef struct {
    uint16_t    crc16;
} __attribute__((packed))FrameTailer ;

typedef struct  {            // ���鴫����ص�����?
    FrameHeader frame_header;
    InputData   input_data;
    FrameTailer frame_tailer;   
} __attribute__((packed))RECEIVE_DATA;

typedef struct  {          // ��ش������������
    FrameHeader frame_header;
    OutputData  output_data;
    FrameTailer frame_tailer;
	
} __attribute__((packed))SEND_DATA;

#define RECEIVE_DATA_SIZE 12
#define SEND_DATA_SIZE 19
#define FRAME_HEADER      0X7B //Frame_header 
#define FRAME_TAIL        0X7D //Frame_tail   
#define FRAME_HEADER_SOF  0xA5  // ֡ͷ��ʼ��־
#define FRAME_HEADER_CRC8 0xFF  // ֡ͷ�̶�У��ֵ
#define FRAME_TAILER_CRC16_init 0xFFFF  // ֡β�̶�У��ֵ


#define GYROSCOPE_RATIO 0.00026644f
#define ACCEL_RATIO 1671.84f

void data_transition(void);
void USART1_SEND(void);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
