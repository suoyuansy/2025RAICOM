/**
 * @file usb_task.h
 * @brief usb输出错误信息
 * ----------------------------------------------------------------------------
 * @version 1.0.0.0
 * @author RM
 * @date 2018-12-26
 * @remark 官步初始代码
 * ----------------------------------------------------------------------------
 * @version 1.0.0.1
 * @author 周明杨
 * @date 2024-12-30
 * @remark 优化整体架构
 */

/*-----------------------------------预处理-----------------------------------*/

#ifndef USB_TASK_H
#define USB_TASK_H

#include "struct_typedef.h"

#define USB_FRAME_DATA_MAX_SIZE 128

/*-----------------------------------外部函数声明-----------------------------------*/

extern void usb_task(void const *argument);

typedef enum
{
  USB_STEP_HEADER_SOF = 0,
  USB_STEP_LENGTH_ID_CRC8,
  USB_STEP_DATA,
  USB_STEP_CHECKSUM,
} usb_unpack_step_e;

// 工具结构体
typedef __packed struct
{
  float x;
  float y;
  float z;
} my_point3_t;

typedef __packed struct
{
  float x;
  float y;
} my_point_t;

// 视觉数据结构体
typedef __packed struct
{
  uint8_t vision_flag;
  uint8_t class_id;
  my_point3_t dis3;
} vision_data_t;

// 激光测距数据结构体
typedef __packed struct
{
  my_point_t dis2;
} laser_data_t;

// usb数据结构
typedef __packed struct
{
  // 位姿数据
  my_point_t pos;
  float orientation;
  // 视觉数据
  vision_data_t vision_data;
  // 激光测距数据
  laser_data_t laser_data;
  // 时间戳数据
  uint32_t game_tick;
} my_usb_data_t;

// 数据接收与解包
typedef __packed struct
{
  uint8_t SOF;  // 起始标志 (0x5A)
  uint8_t LEN;  // 数据段长度
  uint8_t ID;   // 数据段ID
  uint8_t CRC8; // CRC8 校验码
} usb_frame_header_t;

typedef __packed struct
{
  usb_frame_header_t *header;
  my_usb_data_t *data;
} usb_frame_t;

typedef __packed struct
{
  // 全部帧数据缓冲区
  uint8_t buffer[128]; // 总长度不超过 4(SOF+LEN+ID+CRC8) + 20(data) + 2(checksum) = 26
  // 解析指针
  uint16_t index;
  // 解析后帧数据
  usb_frame_t frame;
  // 解析步骤
  usb_unpack_step_e unpack_step;
} usb_unpack_data_t;

typedef __packed struct
{
    uint8_t game_progress;
    uint16_t stage_remain_time;
} usb_game_status_t;

typedef __packed struct
{
    usb_frame_header_t header; // SOF, LEN, ID, CRC8
    usb_game_status_t data;    // 新增的游戏状态数据
    uint16_t checksum;         // CRC16 校验值
} usb_game_status_frame_t;

#endif
