#ifndef APP_FOC_H
#define APP_FOC_H

#include <stdbool.h>
#include "FOC_CORE.h"

/* =========================================================
 *  应用层 FOC 启动参数（沿用原工程默认值）
 *  这些值在 App_FOC_Init() 中写入 FOC_CORE，之后由 FOC_Start() 快照。
 * ========================================================= */
#define APP_FOC_ALIGN_CURRENT_A       1.0f
#define APP_FOC_ALIGN_TIME_MS         1000U
#define APP_FOC_OPENLOOP_CURRENT_A    1.5f
#define APP_FOC_OPENLOOP_TIME_MS      3000U
#define APP_FOC_OPENLOOP_FREQ_HZ      120.0f
#define APP_FOC_TARGET_RPM            1500

/* 应用自定义故障码：HRTIM FLT1 输入（nFAULT） */
#define APP_FAULT_HRTIM               0x00000001U

/* 仅对齐模式：0x09 命令使用；ALIGN -> OPENLOOP 的同一拍由 ADC 回调停机 */
extern volatile bool App_FOC_AlignOnly;

/* 仅对齐完成事件：ADC 回调置位，UART 任务发送 0x89 后清零 */
extern volatile bool App_FOC_AlignDone;

/** @brief 初始化 FOC 核心并写入默认启动配置（在 CORDIC/HRTIM 初始化之后调用） */
void App_FOC_Init(void);

/**
 * @brief 启动 FOC（对齐与启动分离）
 * @param align_only true=仅对齐，对齐完成后自动停机并清运行状态；
 *                   false=跳过对齐，直接从 OPENLOOP 强拖开始
 * @return FOC_OK 或 FOC 错误码
 * @note 会启动 TIM4 遥测定时器；失败时输出保持关闭。
 */
FOC_Err_t App_FOC_Start(bool align_only);

/** @brief 停止电机并关闭 TIM4 遥测定时器 */
void App_FOC_Stop(void);

/** @brief 当前是否处于运行态（非 IDLE/FAULT） */
bool App_FOC_IsRunning(void);

#endif /* APP_FOC_H */
