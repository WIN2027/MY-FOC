#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------- 用户配置宏 ------------------- */
#define FOC_PWM_FREQ_HZ        48000U   // PWM 频率 48kHz
#define ALIGN_DURATION_CYCLES  48000U    // 对齐持续FOC周期数 (1秒 @ 48kHz)
#define FOC_PWM_DT_HZ          (1.0f / FOC_PWM_FREQ_HZ) // dt
#define FOC_PWM_PERIOD         1770U    // 与硬件匹配
#define FOC_CTRL_FREQ_HZ       48000U   // 控制频率与 PWM 同频
#define FOC_VBUS_VOLTAGE       24.0f   // 母线电压 Vdc
#define FOC_I_MAX_DECOUPLE     4.0f    // 电流环前馈解耦限幅最大值
#define FOC_MOTOR_POLE_PAIRS   7U   // 电机极对数
#define FOC_CURRENT_SCALE      0.0179f  // ADC -> 电流系数 (A/LSB)
#define ADC_MIDPOINT           2048.0f   // ADC 零点（12位中心值）
// --- 电压相关缩放（用于反电动势幅值） ---
#define CORDIC_VOLTAGE_MAX     24.0f          // 最大电压幅值（V），留约 20% 余量
#define CORDIC_V_SCALE         (2147483648.0f / CORDIC_VOLTAGE_MAX)

// --- 频率相关缩放（用于补偿角计算） ---
#define CORDIC_FREQ_MAX        5000.0f        // 最大电角速度（rad/s），根据电机额定转速估算
#define CORDIC_FREQ_SCALE      (2147483648.0f / CORDIC_FREQ_MAX)
#define ALIGN_CURRENT_A        1.0f      // 转子对齐电流 (A)
#define OPENLOOP_CURRENT_A     1.5f      // 开环强拖电流 (A)
#define OPENLOOP_TIME_A        3.0f      // 开环强拖时间 (S)
#define M_PI_2_F               1.570796326f
#define M_PI_F                 3.141592653f
#define M_2PI_F                6.283185307f
#define SIN_TABLE_SIZE         128          // 分段数
#define SIN_TABLE_ELEMS        129        // 实际点数
#define SVPWM_SCALE            ((float)FOC_PWM_PERIOD / (float)FOC_VBUS_VOLTAGE) // SVPWM 缩放因子
#define RPM_PER_ELEC_RAD       (9.549296585513721f / (float)FOC_MOTOR_POLE_PAIRS)
#define FOC_DEFAULT_SPEED_RPM  1500.0f
#define MIN_CMP                5.0f
#define MAX_CMP                (FOC_PWM_PERIOD - 5.0f)
/* ------------------- FOC遥测数据结构体 ------------------- */
typedef struct {
    uint16_t foc_dwt;       // FOC单次耗时周期
    int16_t    IU;            // U相电流 (A)
    int16_t    IV;            // V相电流 (A)
    int16_t    IW;            // W相电流 (A)
} FOC_Telemetry_t;
/* ------------------- 比较值结构体 ------------------- */
typedef struct {
    uint16_t PWM_U;
    uint16_t PWM_V;
    uint16_t PWM_W;
} FOC_PWM_t;
/* ------------------- 状态枚举 ------------------- */
typedef enum {
    FOC_STATE_IDLE,
    FOC_STATE_ALIGN,        // 转子对齐
    FOC_STATE_OPENLOOP,     // IF 开环强拖
    FOC_STATE_TRANSITION,   // 开环->闭环过渡
    FOC_STATE_CLOSEDLOOP,   // 闭环速度/电流控制
    FOC_STATE_FAULT
} FOC_State_t;
/* ------------------- PID 控制器结构体（速度环） ------------------- */
typedef struct {
    float kp, ki, kd;
    float target, actual;
    float error, integral, prev_error,prev_output;
    float out_min, out_max;
    float integral_min, integral_max;
    float sample_time;
    float sample_frequency;
    float derivative_filtered;
    float filter_alpha;
} PID_Handle_t;
/* ------------------- PI 控制器结构体（电流环） ------------------- */
typedef struct {
    float kp, ki;
    float target, actual;
    float error, integral;
    float out_min, out_max, out;
    float integral_min, integral_max;
    float dead_zone;
    float sample_time;
} PI_Handle_t;
/* ------------------- 电机参数 ------------------- */
typedef struct {
    float R;          // 定子电阻 Ω
    float Ld, Lq;     // dq 电感 H
    float flux_link;  // 永磁磁链 Wb
    bool enable_decoupling;   // 使能交叉解耦前馈
    bool enable_feedforward;  // 使能电阻压降前馈
} MotorParams_t;
/* ------------------- 滑模观测器 (SMO) ------------------- */
typedef struct {
    float R, L, psi_f;        // 电机参数
    float coeff_A, coeff_B;   //
    float Kslide, lp_freq, Band;
    float lp_alpha;
    float i_alpha, i_beta;    // 实际电流（可选，可不用）
    float i_alpha_est, i_beta_est;
    float e_alpha_est, e_beta_est;
    float z_alpha, z_beta;
    float theta_est, omega_est;
    float theta_est_prev;
    float theta_comp;
    float inv_Band;          // 1.0f / Band
    float Kslide_over_Band;  // Kslide * inv_Band
    float v_est;              // 反电动势幅值
    int16_t  speed_rpm;       // 电机转速
    uint16_t converged_cnt;   // 收敛计数
    bool converged;
} SMO_Handle_t;
/* ------------------- IF 开环启动参数 ------------------- */
typedef struct {
    float freq_start;          // 起始频率
    float freq_target;         // 目标频率
    float freq_actual;         // 实际频率
    float freq_slope;          // 频率斜率 (Hz/s)，预计算 (freq_target / ramp_time)
    float theta_open;          // 开环电角度 (rad) —— 必需
    float omega_elec;
    float alpha;               // 过渡系数
} IF_Handle_t;
/* ------------------- 对外接口函数 ------------------- */
void FOC_Init(void);                            // 初始化所有参数和状态
void FOC_Deinit(void);                          // 重置动态变量（保留配置）
void FOC_Start(FOC_State_t start_mode);         // 启动电机（进入指定模式）
void FOC_Stop(void);                            // 停止电机（进入 IDLE，输出零占空比）
void FOC_SetSpeedTarget(int16_t rpm);           // 设置目标转速 (RPM)
void FOC_SetCurrentTarget(float iq_target_amp); // 手动设置 Q 轴电流目标（仅开环/对齐有效）
void FOC_SetSpeedPID(float kp, float ki, float kd); // 设置速度环 PID 系数
void FOC_SetCurrentPID(float kp_d, float ki_d, float kp_q, float ki_q); // 设置电流环 PI 系数
void FOC_SetOpenloopParams(float freq_end_hz, float ramp_time_sec, float i_start_amp);  // 配置开环参数
void FOC_Run(float Iu, float Iv); // 主控制函数（每个周期调用）
void FOC_GetCurrentPID(float *kp_d, float *ki_d, float *kp_q, float *ki_q);
void FOC_GetSpeedPID(float *kp, float *ki, float *kd);
FOC_State_t FOC_GetState(void);                 // 获取当前状态机状态
void FOC_GetTelemetry(FOC_Telemetry_t *tele);
void SpeedLoop_Update(void);
void Cordic_Phase(float x, float y, float scale, float *mag, float *angle);

extern FOC_State_t foc_state;
extern PID_Handle_t speed_pid;
extern SMO_Handle_t smo;
extern FOC_Telemetry_t s_telemetry;
extern FOC_PWM_t PWM;
extern PI_Handle_t    pi_d, pi_q;
extern float TestA,TestB,TestC;
extern IF_Handle_t    if_handler;

#endif //FOC_H