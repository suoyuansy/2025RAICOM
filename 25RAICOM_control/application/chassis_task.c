#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "tim.h"
#include "usb_task.h"
#include "stdlib.h"

#define rc_deadband_limit(input, output, dealine)        \
	{                                                    \
		if ((input) > (dealine) || (input) < -(dealine)) \
		{                                                \
			(output) = (input);                          \
		}                                                \
		else                                             \
		{                                                \
			(output) = 0;                                \
		}                                                \
	}

void laser_init(laser_decoder_t *d);
bool_t unpack_usart_laser_data(laser_decoder_t *d, uint8_t b, float *distance);

void check_stage_ok();
void check_stage_ok_catching_debug();
void check_stage_ok_moving_debug();
bool_t ymove_till_dis(chassis_move_t *chassis_move_control, float target_dis);
bool_t xmove_till_dis(chassis_move_t *chassis_move_control, float target_dis);
bool_t adjust_orientation(chassis_move_t *chassis_move_control, float target_angle);
void adjust_pose(chassis_move_t *chassis_move_control);

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

// 底盘运动数据
chassis_move_t chassis_move;

uint8_t rx1_buf;
uint8_t rx6_buf;

extern TIM_HandleTypeDef htim1;
extern CAN_HandleTypeDef hcan2;

fp32 damiao_setpos[3] = {0, 0, 0};
uint8_t Data_Enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};	  // 电机使能命令
uint8_t Data_Failure[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};	  // 电机失能命令
uint8_t Data_Save_zero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 电机保存零点命令
uint8_t CANx_SendStdData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *pData, uint16_t Len)
{
	static CAN_TxHeaderTypeDef Tx_Header;

	Tx_Header.StdId = ID;
	Tx_Header.ExtId = 0;
	Tx_Header.IDE = 0;
	Tx_Header.RTR = 0;
	Tx_Header.DLC = Len;
	/*找到空的发送邮箱，把数据发送出去*/
	if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
	{
		if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX2);
		}
	}
	return 0;
}

void chassis_task(void const *pvParameters)
{

	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);
	while (1)
	{
		// check_stage_ok();
		check_stage_ok_catching_debug();
		// check_stage_ok_moving_debug();
		chassis_set_mode(&chassis_move);
		chassis_feedback_update(&chassis_move);
		chassis_set_contorl(&chassis_move);
		chassis_control_loop(&chassis_move);
		// 当遥控器掉线的时候，发送给底盘电机零电流.
		if (toe_is_error(DBUS_TOE))
		{
			CAN_cmd_chassis(0, 0, 0, 0);
		}
		else
		{
			// 发送控制电流
			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current, chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		}

		// os delay
		// 系统延时
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

/**
 * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
 *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
 * @param[out]     chassis_move_init: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
	{
		return;
	}

	// chassis motor speed PID
	// 底盘速度环pid值
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

	// chassis angle PID
	// 底盘角度pid值
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
	const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	uint8_t i;

	// in beginning， chassis mode is raw
	// 底盘开机状态为原始
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
	// get remote control point
	// 获取遥控器指针
	chassis_move_init->chassis_RC = get_remote_control_point();
	// get gyro sensor euler angle point
	// 获取陀螺仪姿态角指针
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();

	// get chassis motor data point,  initialize motor speed PID
	// 获取底盘电机数据指针，初始化PID
	for (i = 0; i < 4; i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	// initialize angle PID
	// 初始化角度PID
	PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

	// first order low-pass filter  replace ramp function
	// 用一阶滤波代替斜波函数生成
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

	// max and min speed
	// 最大 最小速度
	chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X * 2.0f / 3.0f;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X * 2.0f / 3.0f;

	chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y * 2.0f / 3.0f;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y * 2.0f / 3.0f;

	// update data
	// 更新一下数据
	chassis_feedback_update(chassis_move_init);
}

/**
 * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
 * @param[out]     chassis_move_mode: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}
	// in file "chassis_behaviour.c"
	chassis_behaviour_mode_set(chassis_move_mode);
}

/**
 * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
 * @param[out]     chassis_move_transit: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
 * @param[out]     chassis_move_transit:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	if (chassis_move_transit == NULL)
	{
		return;
	}

	if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
	{
		return;
	}
}

/**
 * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL)
	{
		return;
	}

	uint8_t i = 0;
	for (i = 0; i < 4; i++)
	{
		// update motor speed, accel is differential of speed PID
		// 更新电机速度，加速度是速度的PID微分
		chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
		chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}

	// calculate vertical speed, horizontal speed ,rotation speed, left hand rule
	// 更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
	chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

	// 麦轮改全向轮
	//		chassis_move_update->vx = (chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	//    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	//    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

	// calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
	// 计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
	chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
	chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
	chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}
/**
 * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
 *
 * @param[out]     vx_set: vertical speed set-point
 * @param[out]     vy_set: horizontal speed set-point
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
	{
		return;
	}

	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	// deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
	// 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE); // input output deadline,输出的值为遥控器原始数值
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE); // input output deadline,输出的值为遥控器原始数值

	vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;  // 将遥控器原始数值转化为速度米每秒
	vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN; // 将遥控器原始数值转化为速度米每秒

	// keyboard set speed set-point
	// 键盘控制
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	{
		vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	{
		vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
	}

	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
	{
		vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	{
		vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
	}

	// first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
	// 一阶低通滤波代替斜波作为底盘速度输入
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel); // 结构体  input（m/s），滤波输出的结果保存在chassis_cmd_slow_set_vx.out
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel); // 结构体  input（m/s），滤波输出的结果保存在chassis_cmd_slow_set_vx.out
	// stop command, need not slow change, set zero derectly
	// 停止信号，不需要缓慢加速，直接减速到零
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}

	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}

	*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out; // 赋值单位为电流
	*vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out; // 赋值单位为电流
}

uint32_t slowTick;

process_mode_e process_mode;
extern my_usb_data_t usb_data;
bool_t is_stage_ok = 0;

uint32_t init_motor1 = 1000;  // 上下电机的初始位置
uint32_t init_motor2 = 200;	  // 前后电机的初始位置
uint32_t max_motor1 = 3800;	  // 上下电机的最高位置，最低位置默认为下侧的机械限位
uint32_t max_motor2 = 1000;	  // 前后电机的最前位置，最后位置默认为前侧的机械限位
int32_t real_motor1 = 0;	  // 上下电机的实时位置
int32_t real_motor2 = 0;	  // 前后电机的实时位置
uint16_t single_run_time = 1; // 步进电机单次运动时间

uint32_t duoji_angle_open1 = 500;   // 夹爪舵机张开角度
uint32_t duoji_angle_open2 = 1000;   // 夹爪舵机张开角度
uint32_t duoji_angle_close = 1975; // 夹爪舵机闭合角度
uint32_t duoji_angle_up = 2500;	   // 倒斗舵机存储果实角度
uint32_t duoji_angle_down = 1500;  // 倒斗舵机放置果实角度

float damiao_init = 2.0f; // 达秒电机初始位置
float damiao_lay = 0.75f; // 达秒电机放置位置
// damiao_lay=damiao_init+3.1415926535/2.0;

uint32_t dis_ignore = 650;		// 视觉忽视距离
uint32_t dis_catch_min = 125;	// 能够抓取的最近距离
uint32_t dis_catch_max = 185;	// 能够抓取的最远距离
uint8_t window_th_x_catch = 10;	// opencv坐标系下x轴方向的控制阈值，此处使用底盘的前后移动来实现
uint8_t window_th_y_catch_max = 2; // opencv坐标系下y轴方向的控制阈值，此处使用上下机械臂的移动来实现
uint8_t window_th_y_catch_min = 22;

uint16_t window_aim_tick = 0; // 处在窗口中心或者抓取阈值范围内的有效时间

uint8_t version_flag = 0; // 视觉检测标志
uint8_t step_flag = 0;
uint8_t overstep_tick = 0; // 机械臂超出机械限位后用作底盘运动赋值标志
uint32_t overstep_uwtick=0;

// 手动模式调试用
int8_t last_s_left_daodou_duoji_up = RC_SW_UP;
int8_t last_s_left_jiazhua_duoji_down = RC_SW_DOWN;

uint8_t left_daodou_duoji_up_times = 0;
uint8_t left_jiazhua_duoji_down_times = 0;

chassis_mode_e last_chassis_mode;
double v3_channel, v0_channel, v2_channel, v1_channel = 0;

// 手动模式下测试步进电机计时器
uint32_t testTick;
uint32_t deltaTick;
int16_t last_rcch4 = 0;
int16_t last_rcch2 = 0;

// 运动过程变量
uint8_t move_to_g2_step;
uint8_t move_to_f_step;
uint16_t xmove_check_tick;
uint16_t ymove_check_tick;

// 旋转变量
float angle_thre_vel_change = PI / 9;
float angle_thre_ok = PI / 60;
float init_orientation;
float oppo_orientation;

float my_abs(float data)
{
	if (data < 0)
		return -data;
	else
		return data;
}

laser_decoder_t laser1_decoder;
laser_decoder_t laser2_decoder;
float laser_x;
float laser_y;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		unpack_usart_laser_data(&laser1_decoder, rx1_buf, &laser_x);
		
		HAL_UART_Receive_IT(&huart1,&rx1_buf,1);
	}
	
//	if (huart->Instance == USART6)
//	{
//		unpack_usart_laser_data(&laser2_decoder, rx6_buf, &laser_y);
//		
//		HAL_UART_Receive_IT(&huart6,&rx6_buf,1);
//	}
}

void laser_init(laser_decoder_t *d)
{
    memset(d, 0, sizeof(*d));
    d->state = LASER_WAIT_ADDR;
}

bool_t unpack_usart_laser_data(laser_decoder_t *d, uint8_t b, float *distance)
{
    bool_t frame_ok = 0;

    switch (d->state)
    {
    case LASER_WAIT_ADDR:
        if (b == LASER_ADDR)
        {
            d->buf[0] = b;
            d->sum    = b;
            d->idx    = 1;
            d->state  = LASER_WAIT_LEN;
        }
        break;

    case LASER_WAIT_LEN:
        if (b == 0x06u)            /* ???? 0x06 */
        {
            d->buf[d->idx++] = b;
            d->sum += b;
            d->state = LASER_WAIT_CMD;
        }
        else
            d->state = LASER_WAIT_ADDR;   /* ??,???? */
        break;

    case LASER_WAIT_CMD:
        if (b == 0x83u)            /* ???? 0x83 */
        {
            d->buf[d->idx++] = b;
            d->sum += b;
            d->state = LASER_PAYLOAD;
        }
        else
            d->state = LASER_WAIT_ADDR;
        break;

    case LASER_PAYLOAD:            /* ? 7 ??:3X 3X 3X 2E 3X 3X 3X */
        d->buf[d->idx++] = b;
        d->sum += b;

        if (d->idx == 10)          /* ??? 10 Byte(?? CS)*/
            d->state = LASER_WAIT_CS;
        break;

    case LASER_WAIT_CS:
    {
        uint8_t cs = (uint8_t)(~(d->sum) + 1);   /* ??? */
        if (b == cs)
        {
            /* ?????:xxx.xxx */
            char tmp[8] = {0};
            for (int i = 0; i < 7; ++i)
                tmp[i] = (char)d->buf[3 + i];     /* 3X..3X 2E 3X..3X */
            *distance = strtof(tmp, NULL);        /* ?? float */
            frame_ok = 1;
        }
        /* ??????,?????? */
        d->state = LASER_WAIT_ADDR;
        break;
    }

    default:
        d->state = LASER_WAIT_ADDR;
        break;
    }

    return frame_ok;
}

bool_t ymove_till_dis(chassis_move_t *chassis_move_control, float target_dis)
{
	if (laser_y - target_dis > DIS_OFFSET)
	{
		// 左侧距离大于目标右侧距离，向左移动
		chassis_move_control->vy_set = 0.02f;
	}
	else if (laser_y - target_dis < -DIS_OFFSET)
	{
		// 左侧距离小于目标右侧距离，向右移动
		chassis_move_control->vy_set = -0.02f;
	}
	else
	{
		// 左侧距离在可接受范围内，认为已经到达目标dis
		chassis_move_control->vy_set = 0.0f;
		return 1;
	}
	return 0;
}

bool_t xmove_till_dis(chassis_move_t *chassis_move_control, float target_dis)
{
	if (laser_x - target_dis > DIS_OFFSET)
	{
		// 后侧距离大于目标后侧距离，向后移动
		chassis_move_control->vx_set = -0.02f;
	}
	else if (laser_x - target_dis < -DIS_OFFSET)
	{
		// 后侧距离小于目标后侧距离，向前移动
		chassis_move_control->vx_set = 0.02f;
	}
	else
	{
		// 后侧距离在可接受范围内，认为已经到达目标dis
		chassis_move_control->vx_set = 0.0f;
		return 1;
	}
	return 0;
}

bool_t adjust_orientation(chassis_move_t *chassis_move_control, float target_angle)
{
	float angle_offset = target_angle - usb_data.orientation;
	if (angle_offset - PI > 0)
		angle_offset -= 2 * PI;
	else if (angle_offset + PI < 0)
		angle_offset += 2 * PI;
	// 根据旋转角度给定旋转速度，远时快，近时慢
	if (angle_offset - angle_thre_vel_change > 0 || angle_offset + angle_thre_vel_change < 0)
	{
		if (angle_offset > 0)
		{
			chassis_move_control->wz_set = 2.0f;
		}
		else
		{
			chassis_move_control->wz_set = -2.0f;
		}
	}
	else
	{
		if (angle_offset > 0)
		{
			chassis_move_control->wz_set = 0.5f;
		}
		else
		{
			chassis_move_control->wz_set = -0.5f;
		}
	}
	// 判断是否完成
	if (angle_offset - angle_thre_ok < 0 && angle_offset + angle_thre_ok > 0)
	{
		chassis_move_control->wz_set = 0.0f;
		return 1;
	}
	return 0;
}

void adjust_pose(chassis_move_t *chassis_move_control)
{
	if(process_mode == GAME_CATCHING1){
		adjust_orientation(chassis_move_control, init_orientation);
	}else if(process_mode == GAME_CATCHING2){
		adjust_orientation(chassis_move_control, oppo_orientation);
	}
	ymove_till_dis(chassis_move_control, CATCH_ADJUST_DIS);
}

void check_stage_ok()
{
	if (is_stage_ok)
	{
		is_stage_ok = 0;
		switch (process_mode)
		{
		case GAME_UNSTART:
			break;
		case GAME_SETUP:
			process_mode = GAME_MOVING_TO_G1;
			break;
		case GAME_MOVING_TO_G1:
			process_mode = GAME_CATCHING1;
			break;
		case GAME_CATCHING1:
			process_mode = GAME_MOVING_TO_G2;
			break;
		case GAME_MOVING_TO_G2:
			process_mode = GAME_CATCHING2;
			break;
		case GAME_CATCHING2:
			process_mode = GAME_MOVING_TO_F;
			break;
		case GAME_MOVING_TO_F:
			process_mode = GAME_LAYING;
			break;
		case GAME_LAYING:
			process_mode = GAME_FINISHED;
			break;
		case GAME_FINISHED:
			break;
		default:
			break;
		}
	}
}

void check_stage_ok_catching_debug()
{
	if (is_stage_ok)
	{
		is_stage_ok = 0;
		switch (process_mode)
		{
		case GAME_UNSTART:
			break;
		case GAME_SETUP:
			process_mode = GAME_CATCHING1;
			break;
		case GAME_CATCHING1:
			process_mode = GAME_LAYING;
			break;
		case GAME_LAYING:
			process_mode = GAME_FINISHED;
			break;
		case GAME_FINISHED:
			break;
		default:
			break;
		}
	}
}

void check_stage_ok_moving_debug()
{
	if (is_stage_ok)
	{
		is_stage_ok = 0;
		switch (process_mode)
		{
		case GAME_UNSTART:
			break;
		case GAME_SETUP:
			process_mode = GAME_MOVING_TO_G1;
			break;
		case GAME_MOVING_TO_G1:
			process_mode = GAME_TEST_CATCHING1;
			break;
		case GAME_TEST_CATCHING1:
			process_mode = GAME_MOVING_TO_G2;
			break;
		case GAME_MOVING_TO_G2:
			process_mode = GAME_TEST_CATCHING2;
			break;
		case GAME_TEST_CATCHING2:
			process_mode = GAME_MOVING_TO_F;
			break;
		case GAME_MOVING_TO_F:
			process_mode = GAME_LAYING;
			break;
		case GAME_LAYING:
			process_mode = GAME_FINISHED;
			break;
		case GAME_FINISHED:
			break;
		default:
			break;
		}
	}
}

static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

	if (chassis_move_control == NULL)
	{
		return;
	}

	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f; // 单位m/s

	// 开环静止
	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
	{
		chassis_move_control->vx_set = 0.0f;
		chassis_move_control->vy_set = 0.0f;
		chassis_move_control->wz_set = 0.0f;
		chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
		chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
	}
	// 闭环静止
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		if (last_chassis_mode != CHASSIS_VECTOR_NO_MOVE)
		{
			while (CAN_2.err[1] == 1)
			{
				CANx_SendStdData(&hcan2, 0x102, Data_Failure, 8);
			}
		}
		chassis_move_control->vx_set = 0.0f;
		chassis_move_control->vy_set = 0.0f;
		chassis_move_control->wz_set = 0.0f;
	}
	// 初始化阶段
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_AUTO_SETTING)
	{
		// 流程变量初始化
		move_to_g2_step = 0;
		move_to_f_step = 0;
		xmove_check_tick = 0;
		ymove_check_tick = 0;
		laser_x = 0.0f;
		laser_y = 0.0f;
		// 位姿角度初始化
		init_orientation = usb_data.orientation;
		oppo_orientation = init_orientation + PI;
		if (oppo_orientation - PI > 0)
			oppo_orientation -= 2 * PI;
		else if (oppo_orientation + PI < 0)
			oppo_orientation += 2 * PI;
		// 机械臂初始化
		// ------------------------------------------------------------------------------------------------
		// 以下内容应该放在车刚启动时，防止因为移动导致电机位置变化，这里仅做该阶段的调试用；
		// 使达秒电机手动调整好位置后固定
		// 使步进电机比赛开始即达到初始位置，防止后续因为初始化位置导致浪费时间
		// 确保夹爪张开，倒斗在可存储位置

		// 使能2号达秒电机
		while (CAN_2.err[1] == 0)
		{
			CANx_SendStdData(&hcan2, 0x102, Data_Enable, 8);
		}
		// 1号电机达到指定高度
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET); // 上升
		TIM12->ARR = 124;
		TIM12->CCR1 = (uint32_t)(((float)TIM12->ARR + 1.0f) / 2.0f + 0.5f);
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
		HAL_Delay(init_motor1);
		HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
		real_motor1 = init_motor1; // 记录电机初始位置
		HAL_Delay(500);
		// 3号电机伸长到指定位置
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET); // 前进
		TIM8->ARR = 124;
		TIM8->CCR3 = (uint32_t)(((float)TIM8->ARR + 1.0f) / 2.0f + 0.5f);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		HAL_Delay(init_motor2);
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
		real_motor2 = init_motor2; // 记录电机初始位置
		// 夹爪舵机到达张开位置
		TIM1->ARR = 19999;
		TIM1->CCR1 = duoji_angle_open2;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_Delay(500);
		// 倒斗舵机到达存储位置
		TIM1->ARR = 19999;
		TIM1->CCR4 = duoji_angle_up;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
		HAL_Delay(500);
		// 记录达秒电机初始位置
		damiao_init = CAN_2.position[1];
		damiao_lay = damiao_init - 1.25f;
		// ------------------------------------------------------------------------------------------------
		is_stage_ok = 1;
	}
	// 运动阶段
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_AUTO_MOVING)
	{
		if (process_mode == GAME_MOVING_TO_G1)
		{
			if (ymove_till_dis(chassis_move_control, CATCH_ADJUST_DIS))
				is_stage_ok = 1;
		}
		else if (process_mode == GAME_MOVING_TO_G2)
		{
			switch (move_to_g2_step)
			{
			case 0: // 初始化（暂空）
				move_to_g2_step++;
				break;
			case 1: // 往前一步
				if (xmove_till_dis(chassis_move_control, MOVE_TEMP1_BACK_DIS))
					move_to_g2_step++;
				break;
			case 2: // 往左过去
				if (ymove_till_dis(chassis_move_control, MOVE_TEMP2_LEFT_DIS))
					move_to_g2_step++;
				break;
			case 3: // 往后
				if (xmove_till_dis(chassis_move_control, CATCH1_END_CHECK_DIS))
					move_to_g2_step++;
				break;
			case 4: // 旋转
				if (adjust_orientation(chassis_move_control, oppo_orientation))
					move_to_g2_step++;
				break;
			case 5: // 往左
				if (ymove_till_dis(chassis_move_control, CATCH_ADJUST_DIS))
					is_stage_ok = 1;
				break;
			default:
				break;
			}
		}
		else if (process_mode == GAME_MOVING_TO_F)
		{
			switch (move_to_f_step)
			{
			case 0:
				move_to_f_step++;
				break;
			case 1: // 向右
				if (ymove_till_dis(chassis_move_control, MOVE_TEMP3_LEFT_DIS))
					move_to_f_step++;
				break;
			case 2: // 向后
				if (xmove_till_dis(chassis_move_control, MOVE_TEMP4_BACK_DIS))
					move_to_f_step++;
				break;
			case 3: // 旋转
				if (adjust_orientation(chassis_move_control, init_orientation))
					move_to_f_step++;
				break;
			case 4: // 向左
				if (ymove_till_dis(chassis_move_control, LAY_LEFT_DIS))
					move_to_f_step++;
				break;
			case 5: // 向后
				if (xmove_till_dis(chassis_move_control, LAY_BACK_DIS))
					is_stage_ok = 1;
				break;
			default:
				break;
			}
		}
		else if (process_mode == GAME_TEST_CATCHING1)
		{
			if (xmove_till_dis(chassis_move_control, CATCH1_END_CHECK_DIS))
				is_stage_ok = 1;
			else
				adjust_pose(chassis_move_control);
		}
		else if (process_mode == GAME_TEST_CATCHING2)
		{
			if (xmove_till_dis(chassis_move_control, CATCH2_END_CHECK_DIS))
				is_stage_ok = 1;
			else
				adjust_pose(chassis_move_control);
		}
	}

	// 放置阶段
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_AUTO_LAY)
	{
	}
	// 抓取阶段
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_AUTO_CATCH)
	{
		if (last_chassis_mode != CHASSIS_VECTOR_AUTO_CATCH)
		{
			// 以下内容放在每次进入这个模式时
			step_flag = 0;	  // 重置放置阶段
			version_flag = 0; // 重置视觉检测标志
			overstep_tick=0;
			// 将车速初始化为0
			chassis_move_control->vy_set = 0;
			chassis_move_control->vx_set = 0;
			chassis_move_control->wz_set = 0;
		}
		
		//adjust_pose(chassis_move_control);

		// 持续前进
		if (step_flag == 0)
		{
			chassis_move_control->vx_set = 0.15f;
			if (usb_data.vision_data.vision_flag == 1 && usb_data.vision_data.dis3.z <= dis_ignore) // my_dis.distance<=dis_ignore&&
			{
				version_flag++;
				if (version_flag >= 3)
				{
					step_flag = 1;
					version_flag = 0;
				}
			}

		} // 0.1f
		// 检测到目标，记录时钟
		if (step_flag == 1)
		{
			slowTick = uwTick;
			step_flag = 2;
		}
		// 过运动1s
		if (step_flag == 2)
		{
			if (uwTick - slowTick < 1000)
			{
				chassis_move_control->vx_set = 0.04f;
			}
			else
			{
				chassis_move_control->vx_set = 0.0f;
				step_flag = 3;
			}
		}
		// 窗口检测与调整
		if (step_flag == 3) // 先调整底盘x
		{

			if (usb_data.vision_data.dis3.x <= -window_th_x_catch) // 目标在小车前方，需要向前移动
			{
				chassis_move_control->vx_set = 0.04f;
			}
			if (usb_data.vision_data.dis3.x >= window_th_x_catch)
			{
				chassis_move_control->vx_set = -0.04f;
			}

			if (my_abs(usb_data.vision_data.dis3.x) < window_th_x_catch)
			{
				chassis_move_control->vx_set = 0;
			}

			if (chassis_move_control->vx_set == 0)
			{
				window_aim_tick++;
			}
			else
			{
				window_aim_tick = 0;
			}

			if (window_aim_tick >= 500)
			{
				window_aim_tick = 0;
				step_flag = 4; // 底盘前后调整完毕，接下来调整上下机械臂
			}
		}

		if (step_flag == 4) // 再调整上下机械臂
		{
			if(real_motor1<=max_motor1&&real_motor1>=0)
			{
			if (usb_data.vision_data.dis3.y <= -window_th_y_catch_min) // 目标在夹爪下方，需要机械臂向下移动
			{
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET); // 下降
				TIM12->ARR = 124;
				TIM12->CCR1 = (uint32_t)(((float)TIM12->ARR + 1.0f) / 2.0f + 0.5f);
				HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
				HAL_Delay(single_run_time);
				real_motor1 -= single_run_time;
				HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
			}

			if (usb_data.vision_data.dis3.y >= window_th_y_catch_max)
			{
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET); // 上升
				TIM12->ARR = 124;
				TIM12->CCR1 = (uint32_t)(((float)TIM12->ARR + 1.0f) / 2.0f + 0.5f);
				HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
				HAL_Delay(single_run_time);
				real_motor1 += single_run_time;
				HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
			}
			if (usb_data.vision_data.dis3.y < window_th_y_catch_max&&usb_data.vision_data.dis3.y>-window_th_y_catch_min)
			{
				window_aim_tick++;
			}
			else
			{
				window_aim_tick = 0;
			}

			if (window_aim_tick >= 500)
			{
				
				window_aim_tick = 0;
				step_flag = 5; // 上下机械臂调整完毕，接下来调整前后机械臂
				
			}

			// 需要对机械臂做出保护，若超过机械限位，则认为该果实无法摘取，放弃并将机械臂归位，这部分是必须要完成的，防止小车卡死在原地
			// 为防止该果实被重新检测而陷入死循环，向前走一段距离，使得该果实被略过
			// 或者基于初始位置，对能够抓取的果实进行阈值判断，超过上下机械臂运动范围之外的果实不予理睬
		}
			else if (real_motor1 >= max_motor1) // 上下机械臂超过最高位置
			{
				if(overstep_tick==0)
				{
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET); // 下降
				TIM12->ARR = 124;
				TIM12->CCR1 = (uint32_t)(((float)TIM12->ARR + 1.0f) / 2.0f + 0.5f);
				HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
				HAL_Delay(max_motor1 - init_motor1);

				HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
				overstep_tick=1;	
				overstep_uwtick=uwTick;
				}
				chassis_move_control->vx_set = 0.2f; // 向前快速运动，这个行为要在step_flag被赋值为0之前完成
				if (overstep_tick== 1)			 // 速度被传递出去之后再延时
				{
					if(uwTick-overstep_uwtick>1500)
					{
					real_motor1 = init_motor1;
					overstep_tick = 0;
					step_flag = 0;
					}
				}
				
			}
			else if (real_motor1 <= 0) // 上下机械臂超过最低位置
			{
				if(overstep_tick==0)
				{
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET); // 上升
				TIM12->ARR = 124;
				TIM12->CCR1 = (uint32_t)(((float)TIM12->ARR + 1.0f) / 2.0f + 0.5f);
				HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
				HAL_Delay(init_motor1);

				HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
				overstep_tick=1;	
				overstep_uwtick=uwTick;
				}
				chassis_move_control->vx_set = 0.2f;
				if (overstep_tick == 1) // 速度被传递出去之后再延时
				{
					if(uwTick-overstep_uwtick>1500)
					{
					real_motor1 = init_motor1;
					overstep_tick = 0;
					step_flag = 0;
					}
				}
			}
		}
		if (step_flag == 5) // 最后调整前后机械臂
		{
			if(real_motor2<=max_motor2&&real_motor2>=0)
		{
			if (usb_data.vision_data.dis3.z >= dis_catch_max) // 目标在爪子前方，机械臂需要向前运动
			{
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET); // 前进
				TIM8->ARR = 124;
				TIM8->CCR3 = (uint32_t)(((float)TIM8->ARR + 1.0f) / 2.0f + 0.5f);
				HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
				HAL_Delay(single_run_time);
				real_motor2 += single_run_time;
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
			}

			if (usb_data.vision_data.dis3.z <= dis_catch_min)
			{
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET);//后退
			TIM8->ARR = 124;
			TIM8->CCR3 = (uint32_t)(((float)TIM8->ARR + 1.0f) / 2.0f + 0.5f);
			HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
			HAL_Delay(single_run_time);
			real_motor2-=single_run_time;
			HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
			}
			
			if ( usb_data.vision_data.dis3.z >= dis_catch_min&&usb_data.vision_data.dis3.z <= dis_catch_max) //
			{
				window_aim_tick++;
			}
			else
			{
				window_aim_tick = 0;
			}

			if (window_aim_tick >= 500)
			{
				window_aim_tick = 0;
				step_flag = 6; // 前后机械臂调整完成
			}

			// 需要对机械臂做出保护，若超过机械限位，则认为该果实无法摘取，放弃并将机械臂归位
			// 为防止该果实被重新检测而陷入死循环，向前走一段距离，使得该果实被略过
		}
			if (real_motor2 >= max_motor2) // 前后机械臂伸长到机械限位
			{
				if(overstep_tick==0)
				{
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET); // 后退
				TIM8->ARR = 124;
				TIM8->CCR3 = (uint32_t)(((float)TIM8->ARR + 1.0f) / 2.0f + 0.5f);
				HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
				HAL_Delay(max_motor2 - init_motor2);

				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
				overstep_tick=1;	
				overstep_uwtick=uwTick;
				}
				chassis_move_control->vx_set = 0.2f; // 向前快速运动，这个行为要在step_flag被赋值为0之前完成
				if (overstep_tick == 1)			 // 速度被传递出去之后再延时
				{
					if(uwTick-overstep_uwtick>1500)
					{
					real_motor2 = init_motor2;
					overstep_tick = 0;
					step_flag = 0;
					}
				}
			}
			if (real_motor2 <= 0) // 前后机械臂缩短到机械限位
			{
        if(overstep_tick==0)
				{
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET); // 前进
				TIM8->ARR = 124;
				TIM8->CCR3 = (uint32_t)(((float)TIM8->ARR + 1.0f) / 2.0f + 0.5f);
				HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
				HAL_Delay(init_motor2);

				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
				overstep_tick=1;	
				overstep_uwtick=uwTick;
				}
				chassis_move_control->vx_set = 0.2f;
				if (overstep_tick == 1) // 速度被传递出去之后再延时
				{
					if(uwTick-overstep_uwtick>1500)
					{
					real_motor2 = init_motor2;
					overstep_tick = 0;
					step_flag = 0;
					}
			}
		 }
		}
		if (step_flag == 6) // 舵机控制抓取果实
		{
			TIM1->ARR = 19999;
			TIM1->CCR1 = duoji_angle_close;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_Delay(2500);
			step_flag = 7;
		}
		if (step_flag == 7) // 抓取完毕，控制三个电机到指定位置放置
		{
			// 顺序312，为防止上下机械臂打到树桩，先前后机械臂回到初始检测位置，再上下机械臂升到最高位置，最后达秒转动到放置位置

			if (real_motor2 > init_motor2) // 前后机械臂当前伸出长度大于初始位置
			{
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET); // 机械臂后退
				TIM8->ARR = 124;
				TIM8->CCR3 = (uint32_t)(((float)TIM8->ARR + 1.0f) / 2.0f + 0.5f);
				HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
				HAL_Delay(real_motor2 - init_motor2+100);//
				real_motor2 = init_motor2;
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
				HAL_Delay(500);
			}
			else if (real_motor2 < init_motor2) // 前后机械臂当前伸出长度小于初始位置
			{
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET); // 机械臂前进
				TIM8->ARR = 124;
				TIM8->CCR3 = (uint32_t)(((float)TIM8->ARR + 1.0f) / 2.0f + 0.5f);
				HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
				HAL_Delay(init_motor2 - real_motor2);
				real_motor2 = init_motor2;
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
				HAL_Delay(500);
			}

			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET); // 上升运动到最高处
			TIM12->ARR = 124;
			TIM12->CCR1 = (uint32_t)(((float)TIM12->ARR + 1.0f) / 2.0f + 0.5f);
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			HAL_Delay(max_motor1 - real_motor1);
			real_motor1 = max_motor1;
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
			HAL_Delay(500);

			// 达秒电机达到指定角度
			while (1)
			{

				if (CAN_2.position[1] < damiao_lay)
				{
					PosSpeed_CtrlMotor(&hcan2, 0X102, damiao_lay, 1);
				}
				if (CAN_2.position[1] > damiao_lay)
				{
					PosSpeed_CtrlMotor(&hcan2, 0X102, damiao_lay, -1);
				}

				if (my_abs(CAN_2.position[1] - damiao_lay) < 0.1f)
					break;
			}

			step_flag = 8;
		}
		if (step_flag == 8) // 舵机控制放置
		{
			TIM1->ARR = 19999;
			TIM1->CCR1 = duoji_angle_open2;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_Delay(1500);
			step_flag = 9;
		}
		if (step_flag == 9) // 三个电机恢复到检测位置
		{
			// 顺序：先转动达秒，再降低机械臂高度回到检测位置
			// 达秒电机回到初始角度
			while (1)
			{

				if (CAN_2.position[1] < damiao_init)
				{
					PosSpeed_CtrlMotor(&hcan2, 0X102, damiao_init, 1);
				}
				if (CAN_2.position[1] > damiao_init)
				{
					PosSpeed_CtrlMotor(&hcan2, 0X102, damiao_init, -1);
				}

				if (my_abs(CAN_2.position[1] - damiao_init) < 0.1f)
					break;
			}
			// 降低机械臂高度到初始位置
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET); // 下降
			TIM12->ARR = 124;
			TIM12->CCR1 = (uint32_t)(((float)TIM12->ARR + 1.0f) / 2.0f + 0.5f);
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			HAL_Delay(max_motor1 - init_motor1);
			real_motor1 = init_motor1;
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);

			// 在未到激光测距范围内时，循环检测判断
			step_flag = 0;
			chassis_move_control->vy_set = 0;
			chassis_move_control->vx_set = 0;
//			if ((process_mode == GAME_CATCHING1 && laser_x >= CATCH1_END_CHECK_DIS) ||
//				(process_mode == GAME_CATCHING2 && laser_x >= CATCH2_END_CHECK_DIS))
//			{
//				is_stage_ok = 1;
//			}
		}
	}
	// 手动模式
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_MY_CONTROL)
	{

		if (last_chassis_mode != CHASSIS_VECTOR_MY_CONTROL)
		{
			// 使能达秒电机
			while (CAN_2.err[1] == 0)
			{
				CANx_SendStdData(&hcan2, 0x102, Data_Enable, 8);
			}

			damiao_setpos[1] = CAN_2.position[1];

			// 记录达秒电机初始位置
			damiao_init = CAN_2.position[1];
			damiao_lay = damiao_init - 1.25f;

			chassis_move_control->vx_set = 0;
			chassis_move_control->vy_set = 0;
		}

		// 舵机调试
		//  while(1)
		//{
		// 倒斗转动调试
//		TIM1->ARR = 19999;
//		TIM1->CCR4 = duoji_angle_up;
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//		// 夹爪开合调试
//		TIM1->ARR = 19999;
//		TIM1->CCR1 = duoji_angle_open;
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		//}

		// 底盘前后移动、旋转
		chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_control);
		angle_set = -CHASSIS_WZ_RC_SEN * chassis_move_control->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
		chassis_move_control->wz_set = angle_set;
		//chassis_move_control->wz_set = 0;
		// speed limit
		// 速度限幅
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);

		rc_deadband_limit(chassis_move_control->chassis_RC->rc.ch[3], v1_channel, CHASSIS_RC_DEADLINE); // 达妙2

		// 手动控制达秒
		//		damiao_setpos[0] += v0_channel * motor_damiao_RC_SEN;
		//		damiao_setpos[1] += v1_channel * motor_damiao_RC_SEN;
		//		damiao_setpos[2] += v2_channel * motor_damiao_RC_SEN;

		// 手动调试倒斗舵机与达秒电机
		if (switch_is_up(chassis_move_control->chassis_RC->rc.s[1]) && !switch_is_up(last_s_left_daodou_duoji_up))
		{
			left_daodou_duoji_up_times = (left_daodou_duoji_up_times + 1) % 2;
		}
		// 手动调试夹爪舵机
		if (switch_is_down(chassis_move_control->chassis_RC->rc.s[1]) && !switch_is_down(last_s_left_jiazhua_duoji_down))
		{
			left_jiazhua_duoji_down_times = (left_jiazhua_duoji_down_times + 1) % 2;
		}

		// 测试步进电机运动时间
		if (my_abs(last_rcch4) < 200 && my_abs(chassis_move.chassis_RC->rc.ch[4]) >= 200)
		{
			testTick = uwTick;
		}
		if (my_abs(last_rcch4) >= 200 && my_abs(chassis_move.chassis_RC->rc.ch[4]) < 200)
		{
			deltaTick = uwTick - testTick;
		}

		if (my_abs(last_rcch2) < 200 && my_abs(chassis_move.chassis_RC->rc.ch[3]) >= 200)
		{
			testTick = uwTick;
		}
		if (my_abs(last_rcch2) >= 200 && my_abs(chassis_move.chassis_RC->rc.ch[3]) < 200)
		{
			deltaTick = uwTick - testTick;
		}

		last_rcch4 = chassis_move.chassis_RC->rc.ch[4];
		last_rcch2 = chassis_move.chassis_RC->rc.ch[2];

		// 向前拨动为负数，向后拨动为正数
		// 向前拨动，机械臂向前向上，向后拨动，机械臂向后向下
		if (chassis_move.chassis_RC->rc.ch[4] >= 200)
		{
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET); // 向后
			TIM8->ARR = 124;
			TIM8->CCR3 = (uint32_t)(((float)TIM8->ARR + 1.0f) / 2.0f + 0.5f);
			HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		}
		else if (chassis_move.chassis_RC->rc.ch[4] <= -200)
		{

			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET); // 向前
			TIM8->ARR = 124;
			TIM8->CCR3 = (uint32_t)(((float)TIM8->ARR + 1.0f) / 2.0f + 0.5f);
			HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		}
		else if ((chassis_move.chassis_RC->rc.ch[4] > -200) && (chassis_move.chassis_RC->rc.ch[4] < 200)) // 悬停
		{
			HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
		}

		if (chassis_move.chassis_RC->rc.ch[3] >= 200)
		{

			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET); // 向下
			TIM12->ARR = 124;
			TIM12->CCR1 = (uint32_t)(((float)TIM12->ARR + 1.0f) / 2.0f + 0.5f);
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
		}
		else if (chassis_move.chassis_RC->rc.ch[3] <= -200)
		{

			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET); // 向上
			TIM12->ARR = 124;
			TIM12->CCR1 = (uint32_t)(((float)TIM12->ARR + 1.0f) / 2.0f + 0.5f);
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
		}
		else if ((chassis_move.chassis_RC->rc.ch[3] > -200) && (chassis_move.chassis_RC->rc.ch[3] < 200)) // 悬停
		{
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
		}

		// 倒斗转动与达秒控制
		if (left_daodou_duoji_up_times == 0)
		{
			TIM1->ARR = 19999;
			TIM1->CCR4 = duoji_angle_up;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
			while (1)
			{
				if (CAN_2.position[1] < damiao_init)
				{
					PosSpeed_CtrlMotor(&hcan2, 0X102, damiao_init, 1);
				}
				if (CAN_2.position[1] > damiao_init)
				{
					PosSpeed_CtrlMotor(&hcan2, 0X102, damiao_init, -1);
				}

				if (my_abs(CAN_2.position[1] - damiao_init) < 0.1f)
					break;
			}
		}
		if (left_daodou_duoji_up_times == 1)
		{
			TIM1->ARR = 19999;
			TIM1->CCR4 = duoji_angle_down;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

			while (1)
			{
				if (CAN_2.position[1] < damiao_lay)
				{
					PosSpeed_CtrlMotor(&hcan2, 0X102, damiao_lay, 1);
				}
				if (CAN_2.position[1] > damiao_lay)
				{
					PosSpeed_CtrlMotor(&hcan2, 0X102, damiao_lay, -1);
				}

				if (my_abs(CAN_2.position[1] - damiao_lay) < 0.1f)
					break;
			}
		}

		// 夹爪开合
//		if (left_jiazhua_duoji_down_times == 0)
//		{
//			TIM1->ARR = 19999;
//			TIM1->CCR1 = duoji_angle_open1;
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		}
		if (left_jiazhua_duoji_down_times == 0)
		{
			TIM1->ARR = 19999;
			TIM1->CCR1 = duoji_angle_open2;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		}
		if (left_jiazhua_duoji_down_times == 1)
		{
			TIM1->ARR = 19999;
			TIM1->CCR1 = duoji_angle_close;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		}

		last_s_left_daodou_duoji_up = chassis_move_control->chassis_RC->rc.s[1];
		last_s_left_jiazhua_duoji_down = chassis_move_control->chassis_RC->rc.s[1];

		// PosSpeed_CtrlMotor(&hcan2, 0X102, damiao_setpos[1], 5);
	}

	last_chassis_mode = chassis_move_control->chassis_mode;
}

/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
fp32 vx_set_tem, vy_set_tem, offset_angle;
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	// 麦轮
	// because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
	// 旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
	vy_set_tem = vy_set;
	vx_set_tem = vx_set;
	wheel_speed[0] = -vx_set_tem - vy_set_tem + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[1] = vx_set_tem - vy_set_tem + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[2] = vx_set_tem + vy_set_tem + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[3] = -vx_set_tem + vy_set_tem + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	fp32 temp = 0.0f;
	fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 单位
	uint8_t i = 0;

	// mecanum wheel speed calculation
	// 麦轮运动分解
	chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
										  chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

	if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
	{

		for (i = 0; i < 4; i++)
		{
			chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
		}
		// in raw mode, derectly return
		// raw控制直接返回
		return;
	}

	// calculate the max speed in four wheels, limit the max speed
	// 计算轮子控制最大速度，并限制其最大速度
	for (i = 0; i < 4; i++)
	{
		chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
		temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
		if (max_vector < temp)
		{
			max_vector = temp;
		}
	}

	if (max_vector > MAX_WHEEL_SPEED)
	{
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (i = 0; i < 4; i++)
		{
			chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
		}
	}

	// calculate pid
	// 计算pid
	for (i = 0; i < 4; i++)
	{
		PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
	}

	// 功率控制
	chassis_power_control(chassis_move_control_loop);

	// 赋值电流值
	for (i = 0; i < 4; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
	}
}
uint16_t get_chassis_current(void)
{
	return (uint16_t)(fabs(chassis_move.motor_chassis[0].chassis_motor_measure->given_current) + fabs(chassis_move.motor_chassis[1].chassis_motor_measure->given_current) + fabs(chassis_move.motor_chassis[2].chassis_motor_measure->given_current) + fabs(chassis_move.motor_chassis[3].chassis_motor_measure->given_current));
}
