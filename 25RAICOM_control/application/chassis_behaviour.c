#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"
#include "usb_task.h"

/**
 * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
 *                 and chassis control mode is raw. The raw chassis control mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all speed zero.
 * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
 * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
 * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
 *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
 *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle difference between chassis and gimbal
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
 *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle[-PI, PI]
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
 *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
 *                 set value will be sent to can bus.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

// 自定义模式运动控制，底盘与云台分开来进行
static void chassis_my_defined_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector, fp32 *gimbal_yaw, fp32 *gimbal_pitch);

// highlight, the variable chassis behaviour mode
// 留意，这个底盘行为模式变量
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
extern process_mode_e process_mode;

/**
 * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
 * @param[in]      chassis_move_mode: chassis data
 * @retval         none
 */
/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }
  // 拨杆拨到下面，表示无力
  if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[0]) || process_mode == GAME_UNSTART)
  {
    // chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_MOVE;
  }
  // 拨杆拨到中间，表示手动控制
  if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[0]))
  {
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_MY_CONTROL;
  }
  // 拨杆拨到上面，表示自动模式
	if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[0]))
		{
			if(process_mode==GAME_UNSTART){
	chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_MOVE;
	process_mode = GAME_SETUP;
	}
	else if(process_mode==GAME_FINISHED){
	chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_MOVE;
	}else if(process_mode == GAME_SETUP){
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_AUTO_SETTING;
	}else if(process_mode == GAME_MOVING_TO_G1 || process_mode == GAME_MOVING_TO_G2 || process_mode == GAME_MOVING_TO_F || process_mode == GAME_TEST_CATCHING1 || process_mode == GAME_TEST_CATCHING2){
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_AUTO_MOVING;
	}else if(process_mode == GAME_CATCHING1 || process_mode == GAME_CATCHING2){
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_AUTO_CATCH;
	}else if(process_mode == GAME_LAYING){
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_AUTO_LAY;
	}
 }

}

/**
 * @brief          set control set-point. three movement param, according to difference control mode,
 *                 will control corresponding movement.in the function, usually call different control function.
 * @param[out]     vx_set, usually controls vertical speed.
 * @param[out]     vy_set, usually controls horizotal speed.
 * @param[out]     wz_set, usually controls rotation speed.
 * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
 * @retval         none
 */
/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 * @retval         none
 */

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector, fp32 *gimbal_yaw_set, fp32 *gimbal_pitch_set)
{

  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
}

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  *vx_can_set = 0.0f;
  *vy_can_set = 0.0f;
  *wz_can_set = 0.0f;
}

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  *vx_set = 0.0f;
  *vy_set = 0.0f;
  *wz_set = 0.0f;
}

/**
 * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

  *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
 * @brief          自定义状态
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @param[in]      云台左右的旋转速度  正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      云台俯仰的旋转速度，正值 上抬，负值下降
 * @retval         none
 */
static void chassis_my_defined_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector, fp32 *gimbal_yaw, fp32 *gimbal_pitch)
{

  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
  *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}
