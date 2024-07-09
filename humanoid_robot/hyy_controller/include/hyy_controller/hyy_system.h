#ifndef HYY_SYSTEM_H_
#define HYY_SYSTEM_H_

/*
 * @brief 返回状态
 */
#define ERR_CONTROLLER_INACTIVE -201//!< 控制器状态错误
#define ERR_TYPE_EMPTY -202 //!< 指令为空
#define ERR_TYPE -203 //!< 指令错误
#define ERR_MOVETERGET_EMPTY -204 //!< 运动目标为空
#define ERR_MOVETERGET_DOF -205 //!< 运动目标维度错误
#define ERR_DATASPLIT -206 //!< 运动目标数据分割错误
#define ERR_GETMOVETARGET -207 //!< 获取运动目标数据错误
#define ERR_PERSPEEDFORMAT -208 //!< 百分比运动速度错误

#endif