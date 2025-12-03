/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_can1[6];
static motor_measure_t motor_can2[6];		

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  motor3508_tx_message;
static uint8_t              motor3508_can_send_data[8];
static CAN_TxHeaderTypeDef  motor2006_tx_message;
static uint8_t              motor2006_can_send_data[8];
//static CAN_TxHeaderTypeDef  motor_damiao_tx_message;
//static uint8_t              motor_damiao_can_send_data[8];
#define P_MIN -12.5		//位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -18			//转矩最大值
#define T_MAX 18			//转矩最小值

CANx_t CAN_2;

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
    CAN_RxHeaderTypeDef rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  if (hcan == &hcan1) // 接收CAN1数据
    {
    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
//        case CAN_2006_M1_ID:
//        case CAN_2006_M2_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_can1[i], rx_data);
            detect_hook(CHASSIS_MOTOR1_TOE + i);
            break;
        }

        default:
        {
            break;
        }
    }
	}
 if (hcan == &hcan2) // CAN2
    {
        switch (rx_header.StdId)
        {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            {
                static uint8_t i = 0;
                // Get motor id for CAN2 (4 x 3508 motors)
                i = rx_header.StdId - CAN_3508_M1_ID; // Motor index for 3508 motors
                get_motor_measure(&motor_can2[i], rx_data);
                detect_hook(CHASSIS_MOTOR1_TOE + i);
                break;
            }
				case CAN_DAMIAO_M1_ID:
				{
				 CAN_2.id[0]= rx_data[0] & 0xFF;
				 CAN_2.err[0]=(rx_data[0] >> 4) & 0x0F;
				 CAN_2.p_int[0]=(rx_data[1]<<8)|rx_data[2];
	       CAN_2.v_int[0]=(rx_data[3]<<4)|(rx_data[4]>>4);
	       CAN_2.t_int[0]=((rx_data[4]&0xF)<<8)|rx_data[5];
	       CAN_2.position[0] = uint_to_float(CAN_2.p_int[0], P_MIN, P_MAX, 16); // (-12.5,12.5)
	       CAN_2.velocity[0] = uint_to_float(CAN_2.v_int[0], V_MIN, V_MAX, 12); // (-45.0,45.0)
	       CAN_2.torque[0] = uint_to_float(CAN_2.t_int[0], T_MIN, T_MAX, 12); // (-18.0,18.0)
					  break;
				}
				case CAN_DAMIAO_M2_ID:
				{
				 CAN_2.id[1]= rx_data[0] & 0xFF;
				 CAN_2.err[1]=(rx_data[0] >> 4) & 0x0F;
				 CAN_2.p_int[1]=(rx_data[1]<<8)|rx_data[2];
	       CAN_2.v_int[1]=(rx_data[3]<<4)|(rx_data[4]>>4);
	       CAN_2.t_int[1]=((rx_data[4]&0xF)<<8)|rx_data[5];
	       CAN_2.position[1] = uint_to_float(CAN_2.p_int[1], P_MIN, P_MAX, 16); // (-12.5,12.5)
	       CAN_2.velocity[1] = uint_to_float(CAN_2.v_int[1], V_MIN, V_MAX, 12); // (-45.0,45.0)
	       CAN_2.torque[1] = uint_to_float(CAN_2.t_int[1], T_MIN, T_MAX, 12); // (-18.0,18.0)
					  break;
				}
				case CAN_DAMIAO_M3_ID:
				{
				 CAN_2.id[2]= rx_data[0] & 0xFF;
				 CAN_2.err[2]=(rx_data[0] >> 4) & 0x0F;			
				 CAN_2.p_int[2]=(rx_data[1]<<8)|rx_data[2];
	       CAN_2.v_int[2]=(rx_data[3]<<4)|(rx_data[4]>>4);
	       CAN_2.t_int[2]=((rx_data[4]&0xF)<<8)|rx_data[5];
	       CAN_2.position[2] = uint_to_float(CAN_2.p_int[2], P_MIN, P_MAX, 16); // (-12.5,12.5)
	       CAN_2.velocity[2] = uint_to_float(CAN_2.v_int[2], V_MIN, V_MAX, 12); // (-45.0,45.0)
	       CAN_2.torque[2] = uint_to_float(CAN_2.t_int[2], T_MIN, T_MAX, 12); // (-18.0,18.0)
					  break;
				}
            default:
                break;
        }
    }
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
//void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
//{
//    uint32_t send_mail_box;
//    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
//    gimbal_tx_message.IDE = CAN_ID_STD;
//    gimbal_tx_message.RTR = CAN_RTR_DATA;
//    gimbal_tx_message.DLC = 0x08;
//    gimbal_can_send_data[0] = (yaw >> 8);
//    gimbal_can_send_data[1] = yaw;
//    gimbal_can_send_data[2] = (pitch >> 8);
//    gimbal_can_send_data[3] = pitch;
//    gimbal_can_send_data[4] = (shoot >> 8);
//    gimbal_can_send_data[5] = shoot;
//    gimbal_can_send_data[6] = (rev >> 8);
//    gimbal_can_send_data[7] = rev;
//    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
//}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_motor_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor3508_tx_message.StdId = CAN_MOTOR_3508_ALL_ID;
    motor3508_tx_message.IDE = CAN_ID_STD;
    motor3508_tx_message.RTR = CAN_RTR_DATA;
    motor3508_tx_message.DLC = 0x08;
    motor3508_can_send_data[0] = motor1 >> 8;
    motor3508_can_send_data[1] = motor1;
    motor3508_can_send_data[2] = motor2 >> 8;
    motor3508_can_send_data[3] = motor2;
    motor3508_can_send_data[4] = motor3 >> 8;
    motor3508_can_send_data[5] = motor3;
    motor3508_can_send_data[6] = motor4 >> 8;
    motor3508_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&motor3508_CAN, &motor3508_tx_message, motor3508_can_send_data, &send_mail_box);
}

void CAN_cmd_motor_2006(int16_t motor1, int16_t motor2,int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor2006_tx_message.StdId = CAN_MOTOR_2006_ALL_ID; 
    motor2006_tx_message.IDE = CAN_ID_STD; 
    motor2006_tx_message.RTR = CAN_RTR_DATA; 
    motor2006_tx_message.DLC = 0x08; 

    motor2006_can_send_data[0] = motor1 >> 8;
    motor2006_can_send_data[1] = motor1;
    motor2006_can_send_data[2] = motor2 >> 8;
    motor2006_can_send_data[3] = motor2;
	  motor2006_can_send_data[4] = motor3 >> 8;
    motor2006_can_send_data[5] = motor3;
    motor2006_can_send_data[6] = motor4 >> 8;
    motor2006_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&motor2006_CAN, &motor2006_tx_message, motor2006_can_send_data, &send_mail_box);
}

/**
 * @brief  达妙电机位置速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel)
{

	 static CAN_TxHeaderTypeDef Tx_Header;
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

		Tx_Header.StdId=id;
		Tx_Header.IDE=CAN_ID_STD;
		Tx_Header.RTR=CAN_RTR_DATA;
		Tx_Header.DLC=0x08;

    CAN_2.Tx_Data[0] = *pbuf;;
    CAN_2.Tx_Data[1] = *(pbuf+1);
    CAN_2.Tx_Data[2] = *(pbuf+2);
    CAN_2.Tx_Data[3] = *(pbuf+3);
    CAN_2.Tx_Data[4] = *vbuf;
    CAN_2.Tx_Data[5] = *(vbuf+1);
    CAN_2.Tx_Data[6] = *(vbuf+2);
    CAN_2.Tx_Data[7] = *(vbuf+3);

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_2.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_2.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_2.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
			
    }
    }

}
 


const motor_measure_t *get_motor_2006_1_measure_point(void)
{
    return &motor_can1[4];
}
const motor_measure_t *get_motor_2006_2_measure_point(void)
{
    return &motor_can1[5];
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_can1[(i & 0x03)];
}

const motor_measure_t *get_3508_motor_measure_point(uint8_t i)
{
    return &motor_can2[(i & 0x03)];
}


//板1->板2 发送数据
void CAN_cmd_contact(int16_t Current_1, int16_t Current_2, int16_t Current_3, int16_t Current_4)
{

    uint32_t send_mail_box;

    chassis_tx_message.StdId = CAN_CONTACT_ID;  // 消息的标准ID
    chassis_tx_message.IDE = CAN_ID_STD;        // 标准格式
    chassis_tx_message.RTR = CAN_RTR_DATA;      // 数据帧
    chassis_tx_message.DLC = 0x08;              // 数据长度为8字节
    
    chassis_can_send_data[0] = (Current_1 >> 8) & 0xFF;  
    chassis_can_send_data[1] = Current_1 & 0xFF;         
    chassis_can_send_data[2] = (Current_2 >> 8) & 0xFF;  
    chassis_can_send_data[3] = Current_2 & 0xFF;         
    chassis_can_send_data[4] = (Current_3 >> 8) & 0xFF;  
    chassis_can_send_data[5] = Current_3 & 0xFF;         
    chassis_can_send_data[6] = (Current_4 >> 8) & 0xFF;  
    chassis_can_send_data[7] = Current_4 & 0xFF;         

    // 发送CAN消息
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

