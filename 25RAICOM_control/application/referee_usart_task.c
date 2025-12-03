/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM²ÃÅÐÏµÍ³Êý¾Ý´¦Àí
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "RM_Cilent_UI.h"
#include "string.h"

Graph_Data G1,G2,G3,G4,G5;

/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          µ¥×Ö½Ú½â°ü
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

int Char_sizeof=0;
char Dr_Char_1[]={" \nyaw\npit\n"};
char Dr_Char_2[30]={0};
static uint8_t Clien_character[60];
/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ²ÃÅÐÏµÍ³ÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
//void referee_usart_task(void const * argument)
//{
//    init_referee_struct_data();
//    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
//    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);

//    while(1)
//    {

//        referee_unpack_fifo_data();
//        osDelay(10);
//    }
//}
uint32_t uiTick;
//extern uint32_t uwTick;
#define LASER_HEADER_SOF      0x80u
#define LASER_FRAME_LEN       10u
#define LASER_PAYLOAD_LEN     7u     /* ?????? */

/* ??????? unpack_data_t,?????? */
typedef unpack_data_t laser_unpack_t;
laser_unpack_t laser_unpack_obj;
fifo_s_t laser_fifo;            /* ????? FIFO,?? = 32 ?? */
extern float laser_y;
void laser_unpack_fifo_data(void);
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
    
    while(1)
    {
        laser_unpack_fifo_data();
        osDelay(10);
    }
}

void laser_data_solve(uint8_t *packet, float *distance)
{
    char tmp[8] = {0};
    memcpy(tmp, &packet[3], 7);      /* 3~9 ? ASCII */
    *distance = strtof(tmp, NULL);
}

void laser_unpack_fifo_data(void)
{
    uint8_t byte = 0;
    laser_unpack_t *p_obj = &laser_unpack_obj;

    while (fifo_s_used(&laser_fifo))
    {
        byte = fifo_s_get(&laser_fifo);

        switch (p_obj->unpack_step)
        {
        /* ---------- STEP 1 : ??? ---------- */
        case STEP_HEADER_SOF:
            if (byte == LASER_HEADER_SOF)
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_LENGTH_LOW;
            }
            else
            {
                p_obj->index = 0;
            }
            break;

        /* ---------- STEP 2 : ??? ---------- */
        case STEP_LENGTH_LOW:
            p_obj->data_len = byte;                         /* ???? 0x06 */
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
            break;

        /* ---------- STEP 3 : ??? ---------- */
        case STEP_LENGTH_HIGH:
            p_obj->data_len |= (byte << 8);                 /* ????? 0x00 */
            p_obj->protocol_packet[p_obj->index++] = byte;

            /* ????? 10 ??,???????? */
            if (p_obj->data_len == LASER_PAYLOAD_LEN)
            {
                p_obj->unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
            break;

        /* ---------- STEP 4 : ??? ---------- */
        case STEP_FRAME_SEQ:
            /* ??????? seq,??????????? */
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
            break;

        /* ---------- STEP 5 : ???(?????? CRC,????) ---------- */
        case STEP_HEADER_CRC8:
            p_obj->protocol_packet[p_obj->index++] = byte;
            /* ?????? 8 ????,??????“?????” */
            p_obj->unpack_step = STEP_DATA_CRC16;
            break;

        /* ---------- STEP 6 : ??? + ???? ---------- */
        case STEP_DATA_CRC16:
            if (p_obj->index < LASER_FRAME_LEN)
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }

            /* ?? 10 ????? */
            if (p_obj->index >= LASER_FRAME_LEN)
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;

                /* ????:??? = ~(sum[0..8]) + 1 */
                uint8_t sum = 0;
                for (uint8_t i = 0; i < LASER_FRAME_LEN - 1; i++)
                    sum += p_obj->protocol_packet[i];

                uint8_t expect_cs = (uint8_t)(~sum + 1);
                if (expect_cs == p_obj->protocol_packet[LASER_FRAME_LEN - 1])
                {
                    /* ????,????? */
                    laser_data_solve(p_obj->protocol_packet, &laser_y);
                }
            }
            break;

        default:
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
            break;
        }
    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          µ¥×Ö½Ú½â°ü
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
    }
}


