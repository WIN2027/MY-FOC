#ifndef FOC_CORE_H
#define FOC_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "FOC_CONFIG.h"

/* ---------- 段属性宏 ---------- */
#ifdef FOC_USE_CCM_RAM
#  define FOC_CCM_DATA __attribute__((section(".ccmram_data")))
#  define FOC_CCM_TEXT __attribute__((section(".ccmram_text")))
#else
#  define FOC_CCM_DATA
#  define FOC_CCM_TEXT
#endif

/* ---------- 状态机 ---------- */
typedef enum {
    FOC_STATE_IDLE = 0,
    FOC_STATE_ALIGN,
    FOC_STATE_OPENLOOP,
    FOC_STATE_TRANSITION,
    FOC_STATE_CLOSEDLOOP,
    FOC_STATE_FAULT
} FOC_State_t;

/* ---------- 错误码 ---------- */
typedef enum {
    FOC_OK = 0,
    FOC_ERR_NOT_CONFIGURED,      /* 未通过 FOC_SetStartupConfig 配置 */
    FOC_ERR_PARAM_OUT_OF_RANGE,  /* 配置参数越界/非有限值 */
    FOC_ERR_BUSY,                /* 运行中禁止操作 */
    FOC_ERR_INVALID_STATE,       /* 非法参数/状态 */
    FOC_ERR_HW_SELFTEST          /* 硬件自检失败（如 CORDIC 模式错误） */
} FOC_Err_t;

/* ---------- 故障码（FOC_GetLastFault 返回值） ---------- */
#define FOC_FAULT_NONE              0U
#define FOC_FAULT_CORDIC_MODE       0xC0DE0001U
#define FOC_FAULT_OPENLOOP_TIMEOUT  0xC0DE0002U
#define FOC_FAULT_OVERCURRENT       0xC0DE0003U
#define FOC_FAULT_INVALID_INPUT     0xC0DE0004U

/* ---------- 遥测 ---------- */
typedef struct {
    uint16_t foc_dwt;       /* FOC_Run 入口 -> pwm_write 完成的周期数，饱和 65535
                             * 注意：含高优先级中断抢占时间，非纯算法耗时
                             * TEST_MODE 下恒为 0 */
    float   IU;
    float   IV;
    float   IW;
} FOC_Telemetry_t;

/* ---------- PWM 输出（遥测用） ---------- */
typedef struct {
    uint16_t PWM_U;
    uint16_t PWM_V;
    uint16_t PWM_W;
} FOC_PWM_t;

/* ---------- 启动配置（5 个必须配置项） ---------- */
typedef struct {
    float    align_current_a;       /* 对齐电流 A */
    uint32_t align_time_ms;         /* 对齐时长 ms */
    float    openloop_current_a;    /* 强拖电流 A */
    uint32_t openloop_time_ms;      /* 强拖加速时间 ms */
    float    openloop_freq_hz;      /* 强拖目标电频率 Hz（>= FOC_OPENLOOP_FREQ_MIN_HZ） */
} FOC_StartupConfig_t;

/* ---------- 电机参数（运行时可配置，换电机无需重烧） ---------- */
typedef struct {
    float   R;              /* 定子电阻 Ω */
    float   Ld, Lq;         /* dq 电感 H（SMO 目前使用 Ld，见 README） */
    float   flux_link;      /* 永磁磁链 Wb */
    uint8_t pole_pairs;     /* 极对数 1~255 */
} FOC_MotorConfig_t;

/* ---------- 算法状态结构体（供遥测读取） ---------- */
typedef struct {
    float kp, ki, kd;
    float target, actual;
    float target_final;         // 【新增】上位机下发的最终目标
    float accel_limit;          // 【新增】加速度限制 (RPM/s
    float error, integral, prev_error;
    float out_min, out_max;
    float integral_min, integral_max;
    float sample_time;
    uint16_t speed_loop_cnt;
    float derivative_filtered;
    float filter_alpha;
} PID_Handle_t;

typedef struct {
    float kp, ki;
    float target, actual;
    float error, integral;
    float out_min, out_max, out;
    float integral_min, integral_max;
    float dead_zone;
    float sample_time;
} PI_Handle_t;

typedef struct {
    float R;
    float Ld, Lq;
    float flux_link;
    uint8_t pole_pairs;
    bool  enable_decoupling;
    bool  enable_feedforward;
} MotorParams_t;

typedef struct {
    float R, L, psi_f;
    float coeff_A, coeff_B;
    float Kslide, lp_freq, Band;
    float lp_alpha;
    float i_alpha_est, i_beta_est;
    float e_alpha_est, e_beta_est;
    float theta_est, omega_est;
    float theta_est_prev;
    float inv_Band;
    float v_est;
    float theta_comp;           /* 平滑后的补偿角 */
    float conv_rpm_min;         /* v1.3：按强拖目标转速动态计算的收敛阈值 */
    int16_t  speed_rpm;
    uint16_t converged_cnt;
    bool     converged;
} SMO_Handle_t;

typedef struct {
    float freq_target;
    float freq_actual;
    float freq_slope;
    float theta_open;
    float alpha;
} IF_Handle_t;

/* =========================================================
 *  对外 API（单实例，无 ctx 参数）
 * ========================================================= */
void        FOC_Init(void);
void        FOC_Deinit(void);

/* 启动参数配置：上电后未调用本函数（cfg_valid=false）时 FOC_Start 会被拒绝。
 * v1.3：所有 float 字段做 NaN/Inf 校验，越界/非有限值返回 PARAM_OUT_OF_RANGE。 */
FOC_Err_t   FOC_SetStartupConfig(const FOC_StartupConfig_t *cfg);
FOC_Err_t   FOC_GetStartupConfig(FOC_StartupConfig_t *cfg);

/* 电机参数配置：仅 IDLE/FAULT 可写；写入后同步更新 SMO 系数与 Kslide。
 * pole_pairs 使用运行时值，不再依赖 FOC_MOTOR_POLE_PAIRS 编译期宏。 */
FOC_Err_t   FOC_SetMotorParams(const FOC_MotorConfig_t *cfg);
FOC_Err_t   FOC_GetMotorParams(FOC_MotorConfig_t *cfg);

/* 启动：接受 FOC_STATE_ALIGN（仅对齐）或 FOC_STATE_OPENLOOP（跳过对齐，
 * 直接从强拖开始）。本函数内部会调用 FOC_PORT_PWM_ENABLE() 使能 PWM 输出，
 * 应用层无需再手动使能 HRTIM。
 * v1.3：CORDIC 自检失败返回 FOC_ERR_HW_SELFTEST；启动时快照配置到运行副本。 */
FOC_Err_t   FOC_Start(FOC_State_t mode);

void        FOC_Stop(void);

/* 故障处理：关 PWM、锁存 FAULT（不会被后续状态迁移覆盖）、清运行时状态、记录 code。 */
void        FOC_OnFault(uint32_t code);
uint32_t    FOC_GetLastFault(void);

/* 每 PWM 周期调用，输入为 ADC 给出的相电流安培值。
 * v1.3：非有限输入 → FOC_FAULT_INVALID_INPUT；相电流峰值连续超阈值 →
 * FOC_FAULT_OVERCURRENT；开环收敛超时 → FOC_FAULT_OPENLOOP_TIMEOUT。 */
void        FOC_Run(float Iu_A, float Iv_A);

/* 每 HRTIM 重复事件（速度环分频）调用 */
void        FOC_SpeedLoopTick(void);

void        FOC_SetSpeedTarget(int16_t rpm);
void        FOC_SetCurrentTarget(float iq_a);
void        FOC_SetSpeedPID(float kp, float ki, float kd);
void        FOC_SetCurrentPID(float kp_d, float ki_d, float kp_q, float ki_q);
void        FOC_GetSpeedPID(float *kp, float *ki, float *kd);
void        FOC_GetCurrentPID(float *kp_d, float *ki_d, float *kp_q, float *ki_q);

/* v1.2 新增：限幅设置（避免应用层直接改结构体成员） */
void        FOC_SetSpeedLimits(float out_min, float out_max,
                               float integ_min, float integ_max);
void        FOC_SetCurrentLimits(float out_min, float out_max,
                                 float integ_min, float integ_max);

/* v1.2 新增：在线母线电压补偿（电池放电/母线纹波场景） */
void        FOC_SetVbus(float vbus);

FOC_State_t FOC_GetState(void);
/* v1.3：临界区改为 PRIMASK 保存/恢复，任务与中断上下文都可安全调用 */
void        FOC_GetTelemetry(FOC_Telemetry_t *tele);

/* 硬件层：周期测时后回写（内部饱和到 65535） */
void        FOC_SetCycleCount(uint32_t cycles);

/* 公共 CORDIC 相位 API（核心层内部直接用 FOC_PORT_CORDIC_PHASE 宏） */
void        Cordic_Phase(float x, float y, float scale, float *mag, float *angle);

/* =========================================================
 *  对外只读状态（供遥测打包读取）
 *  v1.2：foc_state 加 volatile，防御 FOC_OnFault 抢占后的编译器缓存
 * ========================================================= */
extern volatile FOC_State_t foc_state;
extern PID_Handle_t         speed_pid;
extern SMO_Handle_t         smo;
extern PI_Handle_t          pi_d, pi_q;
extern FOC_PWM_t            PWM;
extern FOC_Telemetry_t      s_telemetry;

#endif /* FOC_CORE_H */