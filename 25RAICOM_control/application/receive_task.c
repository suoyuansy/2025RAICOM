#include "receive_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "remote_control.h"

#include "calibrate_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "oled_task.h"
#include "referee_usart_task.h"
#include "usb_task.h"
#include "voltage_task.h"

//#ifndef RX_BUFFER_SIZE
//#define RX_BUFFER_SIZE 140
//#endif
//extern uint8_t* rx_buffer; // DMA 缓冲区
//extern InputData inputdata;
//uint16_t rx_write_pos = 0;         // 写指针
//uint16_t rx_read_pos = 0;          // 读指针
//uint8_t buffer1[sizeof(RECEIVE_DATA)];//用于存储完整的数据包
//uint8_t Usart_Receive[1];//用于接收单个字节的数据
//uint16_t received = 0;//当前接收到的数据长度
//uint8_t Count=0;//接收状态标签


//void receive_task(void)
//{
//	while(1){
//    uint16_t dma_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx); // 获取 DMA 当前接收位置
//    while (rx_write_pos != dma_pos)
//    {
//        uint8_t received_byte = rx_buffer[rx_write_pos];
//        rx_write_pos = (rx_write_pos + 1) % RX_BUFFER_SIZE;

//        // 处理接收到的字节
//        if (Count == 0)
//        {
//            if (received_byte == FRAME_HEADER_SOF)  
//            {
//                buffer1[received++] = received_byte;  // 存储帧头起始标志
//                Count = 1;  // 进入帧头接收阶段
//            }
//            else
//            {
//                received = 0;  // 不是帧头标志则丢弃数据
//            }
//        }
//			else if (Count == 1)  // 接收帧头的crc8字段
//        {
//            buffer1[received++] = received_byte;

//            // 检查帧头是否接收完成
//            if (received == sizeof(FrameHeader))
//            {
//                FrameHeader *header = (FrameHeader *)buffer1;

//                // 检查sof和crc8是否符合固定值
//                if (header->sof == FRAME_HEADER_SOF && header->crc8 == 0x00)
//                {
//                    Count = 2;  // 帧头校验通过，准备接收数据部分
//                }
//                else
//                {
//                    received = 0;  // 帧头不匹配，重置接收
//                    Count = 0;
//                }
//            }
//        }
//				 else if (Count == 2)  // 接收数据部分以及帧尾
//        {
//            buffer1[received++] = received_byte;

//            // 检查数据包是否接收完整
//            if (received == sizeof(FrameHeader) + sizeof(InputData) + sizeof(FrameTailer))
//            {
//                //FrameTailer *tail = (FrameTailer *)(buffer + sizeof(FrameHeader) + sizeof(InputData));

//                // 检查帧尾
//                //if (verify_CRC16_check_sum((uint8_t*)buffer, sizeof(RECEIVE_DATA)))
//                //{
//									
//                    //数据包检验成功，解析数据部分
//                    memcpy(&inputdata, buffer1 + sizeof(FrameHeader), sizeof(InputData));
//							      //memset(buffer, 0, sizeof(RECEIVE_DATA));
//                //}

//                //无论是否校验成功，都重置接收状态
//                received = 0;
//                Count = 0;
//            }
//        }
//				vTaskDelay(2); // Avoid busy-waiting
//    }
//		
//	}
//}











