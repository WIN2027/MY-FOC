#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* =========================================================
 *  平台 port 头文件
 *  移植到新平台时只改这一行（或编译期 -DFOC_PORT_HEADER='"xxx.h"'）
 * ========================================================= */
#ifndef FOC_PORT_HEADER
#define FOC_PORT_HEADER "FOC_PORT_STM32G4.h"
#endif

/* =========================================================
 *  电机默认参数（可运行时通过 FOC_SetMotorParams 覆盖）
 * ========================================================= */
#define FOC_MOTOR_R             0.055f      /* 定子电阻 Ω */
#define FOC_MOTOR_LD            0.000020f   /* d 轴电感 H */
#define FOC_MOTOR_LQ            0.000020f   /* q 轴电感 H */
#define FOC_MOTOR_FLUX_LINK     0.0012f     /* 永磁磁链 Wb */
#define FOC_MOTOR_POLE_PAIRS    7U          /* 极对数 */

/* 运行时电机参数范围（FOC_SetMotorParams 内部校验） */
#define FOC_MOTOR_R_MIN                 0.001f
#define FOC_MOTOR_R_MAX                 100.0f
#define FOC_MOTOR_L_MIN                 0.000001f
#define FOC_MOTOR_L_MAX                 1.0f
#define FOC_MOTOR_FLUX_MIN              0.00001f
#define FOC_MOTOR_FLUX_MAX              10.0f
#define FOC_MOTOR_POLE_PAIRS_MIN        1U
#define FOC_MOTOR_POLE_PAIRS_MAX        255U

/* =========================================================
 *  硬件相关（PWM / 母线 / SVPWM 缩放）
 * ========================================================= */
#define FOC_PWM_FREQ_HZ         48000U
#define FOC_CTRL_FREQ_HZ        48000U       /* 必须等于 FOC_PWM_FREQ_HZ（见文件末尾检查） */
#define FOC_PWM_PERIOD          1770U
#define FOC_PWM_DT_S            (1.0f / (float)FOC_PWM_FREQ_HZ)   /* 周期，单位秒 */
/* 兼容旧名：v1.3 起推荐 FOC_PWM_DT_S（旧名带 _HZ 但实际是秒，易误用） */
#define FOC_PWM_DT_HZ           FOC_PWM_DT_S

#define FOC_VBUS_VOLTAGE        24.0f
#define FOC_I_MAX_DECOUPLE      4.0f

/* 软件过流保护：相电流峰值连续超过阈值 N 个 PWM 周期即触发 FAULT */
#define FOC_SW_OC_TRIP_A        8.0f
#define FOC_SW_OC_TRIP_CNT      3U

#define MIN_CMP                 5.0f
#define MAX_CMP                 ((float)FOC_PWM_PERIOD - 5.0f)

/* =========================================================
 *  速度环分频（每个 PWM 周期调 FOC_SpeedLoopTick）
 * ========================================================= */
#define FOC_SPEED_LOOP_DIV      15U

/* =========================================================
 *  启动参数范围检查（FOC_SetStartupConfig 内部使用）
 *  超范围返回 FOC_ERR_PARAM_OUT_OF_RANGE
 * ========================================================= */
#define FOC_ALIGN_CURRENT_MIN_A         0.1f
#define FOC_ALIGN_CURRENT_MAX_A         5.0f
#define FOC_ALIGN_TIME_MIN_MS           10U
#define FOC_ALIGN_TIME_MAX_MS           10000U

#define FOC_OPENLOOP_CURRENT_MIN_A      0.1f
#define FOC_OPENLOOP_CURRENT_MAX_A      5.0f
#define FOC_OPENLOOP_TIME_MIN_MS        100U
#define FOC_OPENLOOP_TIME_MAX_MS        30000U

/* 强拖目标频率下限：过低 SMO 可能无法收敛（反电动势过小、信噪比差）
 * 请根据电机额定转速与极对数折算为电频率后设定，留 2~3 倍裕量。
 * 例：额定 1500 RPM × 7 对极 / 60 = 175 Hz，下限可设 60 Hz */
#define FOC_OPENLOOP_FREQ_MIN_HZ        60.0f
#define FOC_OPENLOOP_FREQ_MAX_HZ        500.0f

/* 默认目标转速（进入开环强拖时写入 speed_pid.target，可随时 FOC_SetSpeedTarget 覆盖） */
#define FOC_DEFAULT_TARGET_RPM          1500.0f

/* =========================================================
 *  CORDIC 缩放（相位模式：输入 (x*scale, y*scale)，输出 Q1.31 归一化）
 * ========================================================= */
#define CORDIC_VOLTAGE_MAX      24.0f
#define CORDIC_V_SCALE          (2147483648.0f / CORDIC_VOLTAGE_MAX)
#define CORDIC_FREQ_MAX         10000.0f    /* 为 ω 大值留裕量 */
#define CORDIC_FREQ_SCALE       (2147483648.0f / CORDIC_FREQ_MAX)

/* =========================================================
 *  数学常量 & 换算
 * ========================================================= */
#define M_PI_F                  3.141592653f
#define M_2PI_F                 6.283185307f
/* rad/s -> RPM 的换算常数；机械转速需再除以运行时极对数 */
#define FOC_RAD_S_TO_RPM        9.549296585513721f

#define FOC_1_SQRT3_F           0.5773502691896258f   /* 1/sqrt(3) */
#define FOC_SQRT3_2_F           0.8660254037844386f   /* sqrt(3)/2 */

/* =========================================================
 *  SMO 内部限幅与收敛阈值
 * ========================================================= */
#define FOC_SMO_Z_MAX           12.0f       /* 滑模观测器 z 项限幅（V） */
#define FOC_SMO_I_MAX           10.0f       /* 估计电流限幅（A） */
#define FOC_SMO_E_THRESH        0.1f        /* 反电动势更新 theta 的最小幅值 */
#define FOC_SMO_CONV_E_MIN      0.3f        /* 判定收敛所需的最小反电动势 */
#define FOC_SMO_CONV_RPM_MIN    800.0f      /* 判定收敛所需的最小转速（上限值，见 FOC_SMO_CONV_RATIO） */
#define FOC_SMO_CONV_RATIO      0.85f       /* 收敛阈值 = min(RPM_MIN, RATIO × 强拖目标转速)，避免低速配置永远不收敛 */
#define FOC_SMO_CONV_CNT_MIN    10U         /* 连续满足收敛条件的次数 */
#define FOC_SMO_CONV_CNT_MAX    1000U       /* 收敛计数器上限（防溢出） */
#define FOC_SMO_LP_OMEGA_RAD_S  800.0f     /* 反电动势一阶 LPF 截止角速度（rad/s，≈127Hz） */

/* v1.2：Kslide 自动计算（FOC_Init 内按默认目标转速下反电动势峰值推算）
 *   Kslide = FOC_SMO_KSLIDE_MARGIN × ω_e(默认RPM) × ψ_f
 *   ω_e = RPM × pole_pairs × 2π / 60
 *   默认参数：1500×7×2π/60 ≈ 1099 rad/s，E_peak ≈ 1.32 V，Kslide ≈ 2.64 V
 * 保证 z_max ≥ 2×E_peak，避免 LPF 输出被钳制导致相位抖动。 */
#define FOC_SMO_KSLIDE_MARGIN   2.0f        /* Kslide = MARGIN × E_peak */
#define FOC_SMO_KSLIDE_MIN      0.5f        /* Kslide 下限，避免低速时增益过小 */

/* =========================================================
 *  OPENLOOP → TRANSITION → CLOSEDLOOP 过渡参数
 * ========================================================= */
#define FOC_TRANS_ALPHA_MAX     0.999f      /* 混合系数上限（达到即切闭环） */
#define FOC_TRANS_ALPHA_STEP    0.001f      /* 每帧混合系数增量 */

/* =========================================================
 *  可选项
 * ========================================================= */
/* 1=原版控制基线：SMO Kslide 固定 1.0、前馈关闭；
 * 0=模块默认：Kslide 按目标/磁链自动计算、前馈开启。
 * theta_comp 当前已停用，不参与角度输出。 */
#ifndef FOC_LEGACY_CONTROL
#define FOC_LEGACY_CONTROL 1
#endif

/* 开启后核心层状态/代码放入 CCM RAM（需链接脚本支持对应 section） */
// #define FOC_USE_CCM_RAM

/* =========================================================
 *  编译期配置自检（v1.3）
 * ========================================================= */
#if (FOC_PWM_FREQ_HZ == 0U) || (FOC_PWM_PERIOD == 0U) || (FOC_PWM_PERIOD > 0xFFFFU)
#error "FOC_PWM_FREQ_HZ must be > 0 and FOC_PWM_PERIOD must be in (0, 65535]"
#endif
#if (FOC_CTRL_FREQ_HZ != FOC_PWM_FREQ_HZ)
#error "FOC_CTRL_FREQ_HZ must equal FOC_PWM_FREQ_HZ: FOC_SpeedLoopTick is PWM-synchronous"
#endif
#if (FOC_SPEED_LOOP_DIV < 1U) || (FOC_SPEED_LOOP_DIV > 65535U)
#error "FOC_SPEED_LOOP_DIV must be in [1, 65535]"
#endif
#if (FOC_MOTOR_POLE_PAIRS < 1U)
#error "FOC_MOTOR_POLE_PAIRS must be >= 1"
#endif
#if (FOC_SW_OC_TRIP_CNT < 1U)
#error "FOC_SW_OC_TRIP_CNT must be >= 1"
#endif
#if (FOC_PWM_FREQ_HZ < 1000U)
#error "FOC_PWM_FREQ_HZ is suspiciously low"
#endif

/* 浮点宏（FOC_VBUS_VOLTAGE / FOC_SW_OC_TRIP_A）无法用于 #if 或 C11 的
 * _Static_assert（要求整型常量表达式），请在修改时确保二者 > 0；
 * 运行时 FOC_SetVbus 会再次做有限值/范围校验。 */

#endif /* FOC_CONFIG_H */