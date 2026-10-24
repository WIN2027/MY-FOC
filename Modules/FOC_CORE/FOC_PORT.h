#ifndef FOC_PORT_H
#define FOC_PORT_H

/* 配置宏（FOC_PORT_HEADER 等）必须先可见，保证本头可单独包含 */
#include "FOC_CONFIG.h"

/* =========================================================
 *  核心层依赖的硬件能力清单
 *  实现方式：由 FOC_CONFIG.h 中 FOC_PORT_HEADER 指定的平台 port 头
 *  定义以下全部宏。测试环境定义 FOC_PORT_TEST_MODE 使用空实现。
 *
 *  必需宏（9 个）：
 *    FOC_PORT_PWM_WRITE(a, b, c)         写三相 PWM 比较值（uint16_t）
 *    FOC_PORT_PWM_ENABLE()               使能 PWM 输出（HRTIM/ADC 使能）
 *    FOC_PORT_PWM_DISABLE()              关闭 PWM 输出（HRTIM/ADC 均停）
 *    FOC_PORT_ENTER_CRITICAL()           进入临界区（保存并关闭 PRIMASK）
 *    FOC_PORT_EXIT_CRITICAL()            离开临界区（恢复 PRIMASK）
 *                                        可在任务或中断上下文调用；不得嵌套同名
 *                                        局部变量（实现方应使用唯一变量名）
 *    FOC_PORT_CORDIC_PHASE(x, y, s, m, a)
 *                                        相位模式 CORDIC：
 *                                        输入 (x*scale, y*scale)
 *                                        输出 *mag=幅值, *angle=弧度
 *                                        若目标平台无 CORDIC，可用软件
 *                                        sqrtf + atan2f 实现
 *    FOC_PORT_CORDIC_SELFTEST()          CORDIC 模式自检，返回 bool
 *                                        （若平台无 CORDIC，可返回 true）
 *    FOC_PORT_TS_BEGIN()                 [v1.2] 周期测时开始，返回 uint32_t
 *    FOC_PORT_TS_END(t0)                 [v1.2] 周期测时结束，返回自 t0 起经过的周期数
 *                                        无 DWT/M0 平台可返回 0（foc_dwt 恒 0）
 *
 *  可选宏（有默认值）：
 *    FOC_PORT_CRITICAL_CHECK_NOT_ISR()   [已废弃] 早期用于断言非 ISR 上下文；
 *                                        v1.3 起临界区改为 PRIMASK 保存/恢复，
 *                                        任务与中断均可安全调用，核心层不再调用本宏。
 *                                        保留定义仅为兼容旧 port。
 * ========================================================= */

#include <stdint.h>

#ifdef FOC_PORT_TEST_MODE
    /* 单测桩：CORDIC 改用软件实现，TS 返回 0，ISR 断言为空 */
    #include <math.h>
    #define FOC_PORT_PWM_WRITE(a, b, c)             ((void)0)
    #define FOC_PORT_PWM_ENABLE()                   ((void)0)
    #define FOC_PORT_PWM_DISABLE()                  ((void)0)
    #define FOC_PORT_ENTER_CRITICAL()               ((void)0)
    #define FOC_PORT_EXIT_CRITICAL()                ((void)0)
    #define FOC_PORT_CORDIC_PHASE(x, y, s, m, a)    do { \
        *(m) = sqrtf((x)*(x) + (y)*(y)); \
        *(a) = atan2f((y), (x)); \
    } while (0)
    #define FOC_PORT_CORDIC_SELFTEST()              (1)
    #define FOC_PORT_TS_BEGIN()                     ((uint32_t)0U)
    #define FOC_PORT_TS_END(t0)                     ((void)(t0), (uint32_t)0U)
    #define FOC_PORT_CRITICAL_CHECK_NOT_ISR()       ((void)0)
#else
    #include FOC_PORT_HEADER
    #ifndef FOC_PORT_PWM_WRITE
        #error "port 头未定义 FOC_PORT_PWM_WRITE"
    #endif
    #ifndef FOC_PORT_PWM_ENABLE
        #error "port 头未定义 FOC_PORT_PWM_ENABLE"
    #endif
    #ifndef FOC_PORT_PWM_DISABLE
        #error "port 头未定义 FOC_PORT_PWM_DISABLE"
    #endif
    #ifndef FOC_PORT_ENTER_CRITICAL
        #error "port 头未定义 FOC_PORT_ENTER_CRITICAL"
    #endif
    #ifndef FOC_PORT_EXIT_CRITICAL
        #error "port 头未定义 FOC_PORT_EXIT_CRITICAL"
    #endif
    #ifndef FOC_PORT_CORDIC_PHASE
        #error "port 头未定义 FOC_PORT_CORDIC_PHASE"
    #endif
    #ifndef FOC_PORT_CORDIC_SELFTEST
        #error "port 头未定义 FOC_PORT_CORDIC_SELFTEST"
    #endif
    #ifndef FOC_PORT_TS_BEGIN
        #error "port 头未定义 FOC_PORT_TS_BEGIN（无 DWT 平台请返回 0）"
    #endif
    #ifndef FOC_PORT_TS_END
        #error "port 头未定义 FOC_PORT_TS_END（无 DWT 平台请返回 0）"
    #endif
    /* 可选：未定义时为空 */
    #ifndef FOC_PORT_CRITICAL_CHECK_NOT_ISR
        #define FOC_PORT_CRITICAL_CHECK_NOT_ISR()   ((void)0)
    #endif
#endif

#endif /* FOC_PORT_H */