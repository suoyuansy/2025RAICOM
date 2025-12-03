/**
 * @file usb_task.c
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

/*-----------------------------------头文件引用-----------------------------------*/

#include "usb_task.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "detect_task.h"
#include "voltage_task.h"
#include "CRC8_CRC16.h"

/*-----------------------------------内部函数声明-----------------------------------*/

void usb_data_unpacket(uint8_t byte);

void usb_receive_init();

void send_usb_game_status(uint8_t progress, uint16_t remain_time);

/*-----------------------------------变量声明-----------------------------------*/

// 用于解包USB数据的中间结构体
usb_unpack_data_t usb_unpack_obj;

// 数据帧头
usb_frame_header_t usb_frame_header;

// 解包完得到的USB接收数据
my_usb_data_t usb_data;

const error_t *error_list_usb_local;

/*-----------------------------------函数实现-----------------------------------*/

// USB任务
void usb_task(void const *argument)
{
    // USB设备初始化
    MX_USB_DEVICE_Init();
    // USB接收初始化
    usb_receive_init();
    // 错误捕获
    error_list_usb_local = get_error_list_point();
    // 初始化数据接收步骤
    usb_unpack_obj.unpack_step = USB_STEP_HEADER_SOF;
    // 初始化字节接收索引
    usb_unpack_obj.index = 0;

    while (1)
    {
    }
}

// USB接收初始化
void usb_receive_init()
{
    // 初始化用于解包的工具人结构体
    memset(&usb_unpack_obj, 0, sizeof(usb_unpack_data_t));
    // 将结构体的帧头指针绑定到帧头变量
    usb_unpack_obj.frame.header = &usb_frame_header;
    // 将结构体的数据指针绑定到数据变量
    usb_unpack_obj.frame.data = &usb_data;
}

// USB接收中断
extern void CDC_Receive_FS_Callback(uint8_t *Buf, uint32_t Len)
{
    for (uint32_t i = 0; i < Len; i++)
    {
        usb_data_unpacket(Buf[i]);
    }
}

// USB解包数据
void usb_data_unpacket(uint8_t byte)
{
    // 接收数据缓冲区
    usb_unpack_data_t *p_obj = &usb_unpack_obj;

    // 判断接受阶段
    switch (p_obj->unpack_step)
    {
    // 帧头SOF阶段
    case USB_STEP_HEADER_SOF:
        if (byte == 0x5A)	// 如果是0x5A就继续，进入下一阶段：帧头数据接收与校验阶段
        {
            p_obj->buffer[p_obj->index++] = byte;
            p_obj->unpack_step = USB_STEP_LENGTH_ID_CRC8;
        }
        else				// 如果不是0x5A就重新接收，直到接收到0x5A
        {
            p_obj->index = 0;
        }
        break;

    // 帧头数据接收与校验阶段
    case USB_STEP_LENGTH_ID_CRC8:
        // 逐字节接收帧头数据
        p_obj->buffer[p_obj->index++] = byte;
        // 如果接收索引增加到4，说明帧头接收完毕开始校验
        if (p_obj->index == sizeof(usb_frame_header_t))
        {
            // 将帧头数据从缓冲区拷贝到帧结构体中
            memcpy(p_obj->frame.header, p_obj->buffer, sizeof(usb_frame_header_t));

            // CRC8 校验：传入起始地址为buffer，校验长度为帧头长度4字节
            if (verify_CRC8_check_sum(p_obj->buffer, sizeof(usb_frame_header_t)))
            {
                // 校验通过，进入数据段接收阶段
                p_obj->unpack_step = USB_STEP_DATA;
            }
            else
            {
                // 校验失败，重新等待接收SOF
                p_obj->index = 0;
                p_obj->unpack_step = USB_STEP_HEADER_SOF;
            }
        }
        break;

    // 数据段阶段
    case USB_STEP_DATA:
        // 逐字节接收数据段
        p_obj->buffer[p_obj->index++] = byte;
        // 如果接收索引增加到（帧头长度 + 数据段长度），说明数据段接收完毕，将数据段内容拷贝备份
        if (p_obj->index == sizeof(usb_frame_header_t) + p_obj->frame.header->LEN)
        {
            // 进入数据段整体校验阶段
            p_obj->unpack_step = USB_STEP_CHECKSUM;
        }
        break;

    // 整体校验阶段
    case USB_STEP_CHECKSUM:
        // 逐字节接收CRC16（一共要接收2字节）
        p_obj->buffer[p_obj->index++] = byte;
        // 如果接收索引增加到（帧头长度 + 数据长度 + 2字节CRC16），说明CRC16接收完毕，开始校验
        if (p_obj->index == sizeof(usb_frame_header_t) + p_obj->frame.header->LEN + 2)
        {
            // CRC16 校验：传入起始地址为buffer，校验长度为（帧头长度4字节 + 数据段长度LEN + CRC16长度2字节）
            if (verify_CRC16_check_sum(p_obj->buffer, sizeof(usb_frame_header_t) + p_obj->frame.header->LEN + 2))
            {
                // 校验完成，将解析得到的数据段拷贝到usb_data
                // 拷贝目的地：data    拷贝起始点：buffer位移4字节    拷贝长度：数据段长度
                memcpy(p_obj->frame.data, p_obj->buffer + sizeof(usb_frame_header_t), p_obj->frame.header->LEN);
            }
            // 不管是否解析完成，都重置数据
            p_obj->index = 0;
            p_obj->unpack_step = USB_STEP_HEADER_SOF;
        }
        break;

    default:
        p_obj->unpack_step = USB_STEP_HEADER_SOF;
        p_obj->index = 0;
        break;
    }
}

