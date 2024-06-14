#ifndef HYY_SYSTEM_H_
#define HYY_SYSTEM_H_

/*
 * @brief 返回状态
 */
#define ERR_CONTROLLER_INACTIVE -201//!< 控制器状态错误
#define ERR_MOVETYPE_EMPTY -202 //!< 运动指令为空
#define ERR_MOVETYPE -203 //!< 运动指令错误
#define ERR_MOVETERGET_EMPTY -204 //!< 运动目标为空
#define ERR_MOVETERGET_DOF -205 //!< 运动目标维度错误
#define ERR_DATASPLIT -206 //!< 运动目标数据分割错误
#define ERR_GETMOVETARGET -207 //!< 获取运动目标数据错误
#define ERR_PERSPEEDFORMAT -208 //!< 百分比运动速度错误
#define ERR_TARGET -209 //!< 给定数据错误

#endif