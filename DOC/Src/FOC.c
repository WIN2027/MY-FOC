#include "FOC.h"
#include <math.h>
#include "arm_math.h"
#include <stdint.h>
#include <string.h>
#include "Init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "UART.h"

float TestA,TestB,TestC;

__attribute__((section(".ccmram_data")))PID_Handle_t   speed_pid;
__attribute__((section(".ccmram_data")))FOC_State_t    foc_state = FOC_STATE_IDLE;
__attribute__((section(".ccmram_data")))SMO_Handle_t   smo;
__attribute__((section(".ccmram_data")))FOC_Telemetry_t s_telemetry = {0};
__attribute__((section(".ccmram_data")))FOC_PWM_t PWM;
__attribute__((section(".ccmram_data")))PI_Handle_t    pi_d, pi_q;
__attribute__((section(".ccmram_data")))IF_Handle_t    if_handler;

/* ------------------- 内部静态变量 ------------------- */
__attribute__((section(".ccmram_data")))static MotorParams_t  motor;
__attribute__((section(".ccmram_data")))static float          theta_elec,id_actual, iq_actual; // 当前 dq 电流
__attribute__((section(".ccmram_data")))static uint32_t align_cycle_counter = 0;
__attribute__((section(".ccmram_data")))static float          vd_out, vq_out;       // dq 输出电压

/* ------------------- 对外接口函数（空实现） ------------------- */
/**
 * @brief 初始化 FOC 模块（参数、状态、预计算系数）
 */
void FOC_Init(void)
{
    // ========== 1. 速度环 PID 初始化 ==========
    speed_pid.kp = 0.001f;                          // 比例系数（示例值，实际根据电机调整）
    speed_pid.ki = 0.2f;                           // 积分系数
    speed_pid.kd = 0.0001f;                            // 微分系数
    speed_pid.target = FOC_DEFAULT_SPEED_RPM;       // 目标转速 (RPM)
    speed_pid.actual = 0.0f;                        // 实际转速
    speed_pid.error = 0.0f;
    speed_pid.integral = 0.0f;
    speed_pid.prev_error = 0.0f;
    speed_pid.prev_output = 0.0f;
    speed_pid.derivative_filtered = 0.0f;
    speed_pid.filter_alpha = 0.1f;
    speed_pid.out_min = -2.5f;      // 最小输出电流 (A)
    speed_pid.out_max = 2.5f;       // 最大输出电流
    speed_pid.integral_min = -4.0f; // 积分下限
    speed_pid.integral_max = 4.0f;  // 积分上限
    speed_pid.sample_time = 25.0f / FOC_CTRL_FREQ_HZ;
    speed_pid.sample_frequency = 0.0f;
    // ========== 2. D 轴电流环 PI 初始化 ==========
    pi_d.kp = 0.088f;
    pi_d.ki = 241.0f;
    pi_d.target = 0.0f;             // d 轴目标电流 (通常为 0)
    pi_d.actual = 0.0f;
    pi_d.error = 0.0f;
    pi_d.integral = 0.0f;
    pi_d.out = 0.0f;
    pi_d.out_min = -2.0f;           // 最小输出电压 (V)
    pi_d.out_max = 2.0f;            // 最大输出电压
    pi_d.integral_min = -4.0f;
    pi_d.integral_max = 4.0f;
    pi_d.dead_zone = 0.02f;        // 死区 (A)
    pi_d.sample_time = FOC_PWM_DT_HZ;

    // ========== 3. Q 轴电流环 PI 初始化 ==========
    pi_q = pi_d;                    // 复制 D 轴参数
    pi_q.kp = 0.088f;
    pi_q.ki = 241.0f;
    // 其他成员与 D 轴相同

    // ========== 4. 电机参数初始化 ==========
    motor.R = 0.055f;               // 定子电阻 (Ω)
    motor.Ld = 0.000020f;           // d 轴电感 (H)
    motor.Lq = 0.000020f;           // q 轴电感 (H)
    motor.flux_link = 0.0012f;      // 永磁磁链 (Wb)
    motor.enable_decoupling = false;
    motor.enable_feedforward = false;

    // ========== 5. 滑模观测器初始化 ==========
    smo.R = motor.R;
    smo.L = motor.Ld;               // 假设 Ld = Lq
    smo.psi_f = motor.flux_link;
    smo.Kslide = 1.0f;              // 滑模增益
    smo.lp_freq = 800.0f;           // 低通滤波截止频率 (rad/s)
    smo.lp_alpha = smo.lp_freq * FOC_PWM_DT_HZ; // 预计算低通系数
    if (smo.lp_alpha > 1.0f) smo.lp_alpha = 1.0f;
    smo.Band = 0.3f;                // 饱和函数边界
    smo.inv_Band = 1.0f / smo.Band; // 用乘法替代除法
    smo.coeff_A = 1.0f - smo.R / smo.L * (1.0f / FOC_CTRL_FREQ_HZ);
    smo.coeff_B = (1.0f / FOC_CTRL_FREQ_HZ) / smo.L;
    smo.theta_comp = 0.0f;
    smo.speed_rpm = 0;
    // 状态清零
    smo.i_alpha = 0.0f;
    smo.i_beta = 0.0f;
    smo.i_alpha_est = 0.0f;
    smo.i_beta_est = 0.0f;
    smo.e_alpha_est = 0.0f;
    smo.e_beta_est = 0.0f;
    smo.z_alpha = 0.0f;
    smo.z_beta = 0.0f;
    smo.theta_est = 0.0f;
    smo.omega_est = 0.0f;
    smo.theta_est_prev = 0.0f;
    smo.v_est = 0.0f;
    smo.converged_cnt = 0;
    smo.converged = false;

    // ========== 6. IF 开环启动参数初始化 ==========
    if_handler.freq_target = 40.0f;   // 默认目标电频率 100Hz（可外部修改）
    if_handler.freq_actual = 0.0f;
    if_handler.freq_slope = (if_handler.freq_target) / OPENLOOP_TIME_A;
    if_handler.theta_open = 0.0f;
    if_handler.alpha = 0.0f;        // 过渡混合系数

    // ========== 7. 全局工作变量初始化 ==========
    foc_state = FOC_STATE_IDLE;     // 初始为空闲状态
    theta_elec = 0.0f;
    id_actual = 0.0f;
    iq_actual = 0.0f;
    vd_out = 0.0f;
    vq_out = 0.0f;
    // ========== 8. FOC遥测数据结构体初始化 ==========
    memset(&s_telemetry, 0, sizeof(s_telemetry));
}
/**
 * @brief 重置 FOC 模块的动态变量（保留配置参数）
 * @note  用于电机停止后重新启动，清除积分和估计状态
 */
void FOC_Deinit(void)
{
    // 1. 状态机置为空闲
    foc_state = FOC_STATE_IDLE;

    // 2. 重置速度环 PID 动态变量
    speed_pid.error = 0.0f;
    speed_pid.integral = 0.0f;
    speed_pid.prev_error = 0.0f;
    speed_pid.prev_output = 0.0f;
    speed_pid.derivative_filtered = 0.0f;

    // 3. 重置电流环 PI 动态变量
    pi_d.error = 0.0f;
    pi_d.integral = 0.0f;
    pi_q.error = 0.0f;
    pi_q.integral = 0.0f;

    // 4. 重置滑模观测器所有估计状态
    smo.i_alpha_est = 0.0f;
    smo.i_beta_est = 0.0f;
    smo.e_alpha_est = 0.0f;
    smo.e_beta_est = 0.0f;
    smo.z_alpha = 0.0f;
    smo.z_beta = 0.0f;
    smo.theta_est = 0.0f;
    smo.omega_est = 0.0f;
    smo.theta_est_prev = 0.0f;
    smo.v_est = 0.0f;
    smo.converged_cnt = 0;
    smo.converged = false;

    // 5. 重置 IF 开环启动状态（频率、角度、混合系数）
    if_handler.freq_target = 100.0f;   // 默认目标电频率 100Hz（可外部修改）
    if_handler.freq_actual = 0.0f;
    if_handler.freq_slope = (if_handler.freq_target) / OPENLOOP_TIME_A;
    if_handler.theta_open = 0.0f;
    if_handler.alpha = 0.0f;        // 过渡混合系数

    // 6. 重置全局工作变量
    theta_elec = 0.0f;
    id_actual = 0.0f;
    iq_actual = 0.0f;
    vd_out = 0.0f;
    vq_out = 0.0f;
}
void FOC_Start(FOC_State_t start_mode)
{

    // 2. 重置所有控制器的动态变量（积分、误差等）
    speed_pid.error = 0.0f;
    speed_pid.integral = 0.0f;
    speed_pid.prev_error = 0.0f;
    speed_pid.prev_output = 0.0f;
    speed_pid.derivative_filtered = 0.0f;

    pi_d.error = 0.0f;
    pi_d.integral = 0.0f;
    pi_q.error = 0.0f;
    pi_q.integral = 0.0f;

    // 3. 重置滑模观测器估计状态
    smo.i_alpha_est = 0.0f;
    smo.i_beta_est = 0.0f;
    smo.e_alpha_est = 0.0f;
    smo.e_beta_est = 0.0f;
    smo.z_alpha = 0.0f;
    smo.z_beta = 0.0f;
    smo.theta_est = 0.0f;
    smo.omega_est = 0.0f;
    smo.theta_est_prev = 0.0f;
    smo.v_est = 0.0f;
    smo.converged = false;

    // 4. 根据启动模式，设置特定初始值
    //    说明：已移除 TRANSITION 启动入口 —— 开环->闭环过渡由 FOC_GetAngle() 在
    //    SMO 收敛后自动触发；也不允许直接闭环启动（SMO 未收敛时 theta_est 无效，
    //    直接闭环会失步），统一引导到开环强拖。
    FOC_State_t run_mode = start_mode;
    switch (start_mode)
    {
        case FOC_STATE_ALIGN:   pi_d.target = ALIGN_CURRENT_A;pi_q.target = 0.0f;
                                theta_elec = 0.0f;align_cycle_counter = 0;break;
        case FOC_STATE_OPENLOOP:
        case FOC_STATE_CLOSEDLOOP:      // 直接闭环启动 -> 引导到开环启动
                                if (start_mode == FOC_STATE_CLOSEDLOOP){run_mode = FOC_STATE_OPENLOOP;}
                                if_handler.freq_target = 120.0f;   // 默认目标电频率 120Hz（可外部修改）
                                if_handler.freq_actual = 0.0f;
                                if_handler.freq_slope = (if_handler.freq_target) / OPENLOOP_TIME_A;
                                if_handler.theta_open = 0.0f;
                                if_handler.alpha = 0.0f;        // 过渡混合系数
                                // 电流给定
                                pi_d.target = 0.0f;pi_q.target = OPENLOOP_CURRENT_A;break;

        default:break;
    }

    // 5. 清空输出电压，设定状态（注意用解析后的 run_mode，而非原始 start_mode）
    vd_out = 0.0f;
    vq_out = 0.0f;
    foc_state = run_mode;
}
void FOC_Stop(void)
{
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
    // // 2. 一次性同步停止主定时器 + A + B + D (HAL 宏支持掩码)
    __HAL_HRTIM_DISABLE(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D);

    // 状态置为空闲
    foc_state = FOC_STATE_IDLE;
    // 清空积分项，防止下次启动时突变
    speed_pid.integral = 0.0f;
    pi_d.integral = 0.0f;
    pi_q.integral = 0.0f;
    // 输出电压清零（下一次 FOC_Run 会输出零占空比）
    vd_out = 0.0f;
    vq_out = 0.0f;
}
void FOC_SetSpeedTarget(int16_t rpm)
{
    speed_pid.target = (float)rpm;
}
void FOC_SetCurrentTarget(float iq_target_amp)
{
    pi_q.target = iq_target_amp;
}
void FOC_SetSpeedPID(float kp, float ki, float kd)
{
    speed_pid.kp = kp;
    speed_pid.ki = ki;
    speed_pid.kd = kd;
}
void FOC_GetSpeedPID(float *kp, float *ki, float *kd)
{
    if (kp) *kp = speed_pid.kp;
    if (ki) *ki = speed_pid.ki;
    if (kd) *kd = speed_pid.kd;
}
void FOC_SetCurrentPID(float kp_d, float ki_d, float kp_q, float ki_q)
{
    pi_d.kp = kp_d;
    pi_d.ki = ki_d;
    pi_q.kp = kp_q;
    pi_q.ki = ki_q;
}
void FOC_GetCurrentPID(float *kp_d, float *ki_d, float *kp_q, float *ki_q)
{
    if (kp_d) *kp_d = pi_d.kp;
    if (ki_d) *ki_d = pi_d.ki;
    if (kp_q) *kp_q = pi_q.kp;
    if (ki_q) *ki_q = pi_q.ki;
}
FOC_State_t FOC_GetState(void)
{
    return foc_state;
}



/* ------------------- 静态函数实现 ------------------- */
/**
 * @brief 使用 CORDIC 相位模式计算向量 (x, y) 的幅值和角度
 * @param x       输入 X 分量（物理单位）
 * @param y       输入 Y 分量（物理单位）
 * @param scale   从物理值到 Q1.31 的缩放倍数（应使最大输入不超过 ±1.0）
 * @param mag     输出幅值（与输入物理单位相同）
 * @param angle   输出角度（弧度，范围 [-π, π)）
 */
__attribute__((section(".ccmram_text"))) void Cordic_Phase(float x, float y, float scale, float *mag, float *angle)
{
    // 1. 缩放为 Q1.31 定点数
    int32_t in[2] = {
        (int32_t)(x * scale),
        (int32_t)(y * scale)
    };
    int32_t out[2];

    // 2. 调用 HAL 库轮询计算（单次，无限超时）
    if (HAL_CORDIC_Calculate(&hcordic, in, out, 1, HAL_MAX_DELAY) != HAL_OK) {
        *mag = 0.0f;
        *angle = 0.0f;
        return;
    }

    // 3. 解析输出（硬件顺序：out[0]=角度 RES1，out[1]=模 RES2）
    //    角度：RES1 = θ/π (Q1.31) → 乘以 π 得弧度
    *angle = ((float)out[0] / 2147483648.0f) * 3.141592653589793f;
    //    模：RES2 = 实际幅值 / 最大幅值 (Q1.31) → 除以 scale 还原物理值
    *mag   = (float)out[1] / scale;
}
__attribute__((section(".ccmram_text")))static void ParkTransform(float i_alpha, float i_beta, float cos_theta, float sin_theta, float *id, float *iq)
{
    *id =  i_alpha * cos_theta + i_beta * sin_theta;
    *iq = -i_alpha * sin_theta + i_beta * cos_theta;
}
__attribute__((section(".ccmram_text")))static void InvParkTransform(float vd, float vq, float cos_theta, float sin_theta, float *v_alpha, float *v_beta)
{
    *v_alpha = vd * cos_theta - vq * sin_theta;
    *v_beta  = vd * sin_theta + vq * cos_theta;
}
__attribute__((section(".ccmram_text")))static void SVPWM(const float v_alpha, const float v_beta)
{
    // 1. Clarke 反变换
    float va = v_alpha;
    const float half_va = 0.5f * v_alpha;
    float vb = -half_va + 0.8660254037844386f * v_beta;
    float vc = -half_va - 0.8660254037844386f * v_beta;

    // 2. 中心化零序分量注入（Min‑Max 法）
    float v_max = fmaxf(va, fmaxf(vb, vc));
    float v_min = fminf(va, fminf(vb, vc));
    float v_offset = 0.5f * FOC_VBUS_VOLTAGE - 0.5f * (v_max + v_min);

    va += v_offset;
    vb += v_offset;
    vc += v_offset;

    // 3. 缩放为定时器比较值
    float cmp_a = va * SVPWM_SCALE;
    float cmp_b = vb * SVPWM_SCALE;
    float cmp_c = vc * SVPWM_SCALE;

    float max_cmp = (float)FOC_PWM_PERIOD - 5.0f;   // 默认接近满占空比（原 MAX_CMP）

    if (cmp_a < MIN_CMP)    cmp_a = (uint16_t)MIN_CMP;
    if (cmp_a > max_cmp)    cmp_a = (uint16_t)max_cmp;

    if (cmp_b < MIN_CMP)    cmp_b = (uint16_t)MIN_CMP;
    if (cmp_b > max_cmp)    cmp_b = (uint16_t)max_cmp;

    if (cmp_c < MIN_CMP)    cmp_c = (uint16_t)MIN_CMP;
    if (cmp_c > max_cmp)    cmp_c = (uint16_t)max_cmp;

    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = (int16_t)cmp_a;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = (int16_t)cmp_b;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (int16_t)cmp_c;

    PWM.PWM_U = (uint16_t)cmp_a;PWM.PWM_V = (uint16_t)cmp_b;PWM.PWM_W = (uint16_t)cmp_c;
}

/**
 * @brief 滑模观测器更新（无感 FOC 角度/速度估计）
 * @param i_alpha   实际 α 轴电流 (A)
 * @param i_beta    实际 β 轴电流 (A)
 * @param v_alpha   α 轴电压 (V)
 * @param v_beta    β 轴电压 (V)
 */
// __attribute__((section(".ccmram_text"))) static void SMO_Update(float i_alpha, float i_beta, float v_alpha, float v_beta)
// {
//     const float dt = FOC_PWM_DT_HZ;   // 建议：后续将宏改为实际周期，避免误解
//
//     // 1. 电流观测误差
//     float err_alpha = smo.i_alpha_est - i_alpha;
//     float err_beta  = smo.i_beta_est  - i_beta;
//
//     // 2. 滑模控制律（饱和函数）
//     float z_alpha, z_beta;
//     if (err_alpha > smo.Band)
//         z_alpha = smo.Kslide;
//     else if (err_alpha < -smo.Band)
//         z_alpha = -smo.Kslide;
//     else
//         z_alpha = smo.Kslide * err_alpha * smo.inv_Band;
//
//     if (err_beta > smo.Band)
//         z_beta = smo.Kslide;
//     else if (err_beta < -smo.Band)
//         z_beta = -smo.Kslide;
//     else
//         z_beta = smo.Kslide * err_beta * smo.inv_Band;
//
//     // 可选限幅
//     const float Z_MAX = 12.0f;
//     if (z_alpha > Z_MAX) z_alpha = Z_MAX;
//     if (z_alpha < -Z_MAX) z_alpha = -Z_MAX;
//     if (z_beta > Z_MAX) z_beta = Z_MAX;
//     if (z_beta < -Z_MAX) z_beta = -Z_MAX;
//
//     // 3. 电流观测器更新
//     smo.i_alpha_est = smo.coeff_A * smo.i_alpha_est + smo.coeff_B * (v_alpha - z_alpha);
//     smo.i_beta_est  = smo.coeff_A * smo.i_beta_est  + smo.coeff_B * (v_beta  - z_beta);
//
//     const float I_MAX = 10.0f;
//     if (smo.i_alpha_est > I_MAX) smo.i_alpha_est = I_MAX;
//     if (smo.i_alpha_est < -I_MAX) smo.i_alpha_est = -I_MAX;
//     if (smo.i_beta_est > I_MAX) smo.i_beta_est = I_MAX;
//     if (smo.i_beta_est < -I_MAX) smo.i_beta_est = -I_MAX;
//
//     // ========== 4. 自适应反电动势低通滤波（新） ==========
//     // 截止角频率与估计速度成正比，保证相位滞后近似恒定
//     const float k_emf = 1.2f;            // 比例系数，典型 1.5 ~ 3.0，可根据噪声/滞后权衡调整
//     const float omega_c_emf_min = 140.0f; // 最小截止角频率 (rad/s)，防止极低速时动态过慢
//     const float omega_c_emf_max = 2000.0f; // 最大截止角频率，受限于采样定理
//
//     // 使用上一拍的估计速度绝对值计算自适应带宽
//     float omega_abs_emf = fabsf(smo.omega_est);
//     float omega_c_emf = k_emf * omega_abs_emf;
//     if (omega_c_emf < omega_c_emf_min) omega_c_emf = omega_c_emf_min;
//     if (omega_c_emf > omega_c_emf_max) omega_c_emf = omega_c_emf_max;
//     float alpha_emf = omega_c_emf * dt;
//     if (alpha_emf > 1.0f) alpha_emf = 1.0f;  // 稳定性保护
//
//     // 一阶低通滤波提取反电动势
//     smo.e_alpha_est = smo.e_alpha_est * (1.0f - alpha_emf) + z_alpha * alpha_emf;
//     smo.e_beta_est  = smo.e_beta_est  * (1.0f - alpha_emf) + z_beta  * alpha_emf;
//
//     // 5. 计算反电动势幅值
//     float e_mag, theta;
//     Cordic_Phase(smo.e_beta_est, -smo.e_alpha_est, CORDIC_V_SCALE, &e_mag, &theta);
//     smo.v_est = e_mag;
//
//     // 6. 估计电角度（含动态相位补偿）
//     const float E_THRESH = 0.1f;
//     if (e_mag > E_THRESH)
//     {
//         if (theta < 0.0f) theta += M_2PI_F;
//
//         // ---- 动态相位补偿，匹配当前自适应滤波带宽 ----
//         float theta_comp = 0.0f;
//         if (omega_c_emf > 1e-6f) {
//             theta_comp = atan2f(smo.omega_est, omega_c_emf);  // 补偿低通造成的相位滞后
//         } else {
//             // 带宽接近0时，角度接近 ±90°
//             theta_comp = (smo.omega_est >= 0.0f) ? M_PI_2_F : -M_PI_2_F;
//         }
//         theta += theta_comp;
//
//         // 归一化到 [0, 2π)
//         while (theta >= M_2PI_F) theta -= M_2PI_F;
//         while (theta < 0.0f)     theta += M_2PI_F;
//
//         smo.theta_est = theta;
//     }
//
//     // ========== 7. 自适应速度低通滤波（新） ==========
//     float delta_theta = smo.theta_est - smo.theta_est_prev;
//     if (delta_theta > M_PI_F)   delta_theta -= M_2PI_F;
//     if (delta_theta < -M_PI_F)  delta_theta += M_2PI_F;
//     float omega_raw = delta_theta / dt;
//
//     const float k_omega = 1.2f;             // 速度滤波比例系数，典型 1.5 ~ 3.0
//     const float omega_c_vel_min = 140.0f;    // 最小截止角频率 (rad/s)
//     const float omega_c_vel_max = 2000.0f;  // 最大截止角频率 (rad/s)
//
//     float omega_abs_vel = fabsf(smo.omega_est);
//     float omega_c_vel = k_omega * omega_abs_vel;
//     if (omega_c_vel < omega_c_vel_min) omega_c_vel = omega_c_vel_min;
//     if (omega_c_vel > omega_c_vel_max) omega_c_vel = omega_c_vel_max;
//     float alpha_omega = omega_c_vel * dt;
//     if (alpha_omega > 1.0f) alpha_omega = 1.0f;
//
//     smo.omega_est = smo.omega_est * (1.0f - alpha_omega) + omega_raw * alpha_omega;
//
//     // 8. 收敛判断
//     const float SPEED_THRESH_RPM = 800.0f;
//     const float speed_rpm = smo.omega_est * RPM_PER_ELEC_RAD;
//     smo.speed_rpm = (uint16_t)speed_rpm;
//     if (e_mag > 0.3f && fabsf(speed_rpm) > SPEED_THRESH_RPM)
//     {
//         if (smo.converged_cnt < 1000) smo.converged_cnt++;
//         if (smo.converged_cnt >= 10)
//             smo.converged = true;
//     }
//     else
//     {
//         smo.converged_cnt = 0;
//         smo.converged = false;
//     }
//
//     smo.theta_est_prev = smo.theta_est;
// }
// __attribute__((section(".ccmram_text")))static void SMO_Update(float i_alpha, float i_beta, float v_alpha, float v_beta)
// {
//     const float dt   = FOC_PWM_DT_HZ;             // 预计算 dt 倒数（避免除法）
//
//     // 1. 电流观测误差
//     float err_alpha = smo.i_alpha_est - i_alpha;
//     float err_beta  = smo.i_beta_est  - i_beta;
//
//     // 2. 滑模控制律（饱和函数，边界层 Band）
//     float z_alpha, z_beta;
//     if (err_alpha > smo.Band)
//         z_alpha = smo.Kslide;
//     else if (err_alpha < -smo.Band)
//         z_alpha = -smo.Kslide;
//     else
//         z_alpha = smo.Kslide * err_alpha * smo.inv_Band;   // 线性区
//
//     if (err_beta > smo.Band)
//         z_beta = smo.Kslide;
//     else if (err_beta < -smo.Band)
//         z_beta = -smo.Kslide;
//     else
//         z_beta = smo.Kslide * err_beta * smo.inv_Band;
//
//     // 可选限幅（防止电压过冲）
//     const float Z_MAX = 12.0f;  // 最大24V系统
//     if (z_alpha > Z_MAX) z_alpha = Z_MAX;
//     if (z_alpha < -Z_MAX) z_alpha = -Z_MAX;
//     if (z_beta > Z_MAX) z_beta = Z_MAX;
//     if (z_beta < -Z_MAX) z_beta = -Z_MAX;
//
//     // 3. 电流观测器更新（前向欧拉）
//     smo.i_alpha_est = smo.coeff_A * smo.i_alpha_est + smo.coeff_B * (v_alpha - z_alpha);
//     smo.i_beta_est  = smo.coeff_A * smo.i_beta_est  + smo.coeff_B * (v_beta  - z_beta);
//
//     // 避免估计电流发散（限幅）
//     const float I_MAX = 10.0f;
//     if (smo.i_alpha_est > I_MAX) smo.i_alpha_est = I_MAX;
//     if (smo.i_alpha_est < -I_MAX) smo.i_alpha_est = -I_MAX;
//     if (smo.i_beta_est > I_MAX) smo.i_beta_est = I_MAX;
//     if (smo.i_beta_est < -I_MAX) smo.i_beta_est = -I_MAX;
//
//     // 4. 低通滤波获取等效反电动势
//     //    滤波系数 alpha_lp = 2*pi*f_cut * dt，但确保 <1
//     smo.e_alpha_est = smo.e_alpha_est * (1.0f - smo.lp_alpha) + z_alpha * smo.lp_alpha;
//     smo.e_beta_est  = smo.e_beta_est  * (1.0f - smo.lp_alpha) + z_beta  * smo.lp_alpha;
//
//     // 5. 计算反电动势幅值
//     float e_mag, theta;
//     Cordic_Phase(smo.e_beta_est, -smo.e_alpha_est, CORDIC_V_SCALE, &e_mag, &theta);
//     smo.v_est = e_mag;
//
//     // 6. 估计电角度（当反电动势足够大时）
//     const float E_THRESH = 0.1f;   // 反电动势阈值 (V)
//     if (e_mag > E_THRESH)
//     {
//         if (theta < 0.0f) theta += M_2PI_F;
//
//         // 使用硬件 CORDIC（固定相位模式）计算补偿角 atan2(omega_est, lp_freq)
//         theta += smo.theta_comp;
//
//         // 归一化到 [0, 2π)
//         while (theta >= M_2PI_F) theta -= M_2PI_F;
//         while (theta < 0.0f)           theta += M_2PI_F;
//
//         smo.theta_est = theta;
//     }
//     // 否则保持上一拍角度
//
//     // 7. 估计电角速度（角度差分 + 低通滤波）
//     float delta_theta = smo.theta_est - smo.theta_est_prev;
//     // 处理角度跳变（2π 边界）
//     if (delta_theta > M_PI_F)   delta_theta -= M_2PI_F;
//     if (delta_theta < -M_PI_F)  delta_theta += M_2PI_F;
//     float omega_raw = delta_theta / dt;
//     // 一阶低通滤波（截止频率 100Hz，即 628 rad/s）
//     const float OMEGA_LP_FREQ = 100.0f;   // rad/s
//     float alpha_omega = OMEGA_LP_FREQ * dt;
//     if (alpha_omega > 1.0f) alpha_omega = 1.0f;
//     smo.omega_est = smo.omega_est * (1.0f - alpha_omega) + omega_raw * alpha_omega;
//
//     // 8. 收敛判断：反电动势足够大且速度超过阈值（例如 200 RPM）
//     const float SPEED_THRESH_RPM = 600.0f;
//     const float speed_rpm = smo.omega_est * RPM_PER_ELEC_RAD; // 机械转速
//     smo.speed_rpm = (uint16_t)speed_rpm;
//     if (e_mag > 0.3f && fabsf(speed_rpm) > SPEED_THRESH_RPM)
//     {
//         if (smo.converged_cnt < 1000) smo.converged_cnt++;
//         if (smo.converged_cnt >= 10)   // 连续10次满足条件认为收敛
//             smo.converged = true;
//     }
//     else
//     {
//         smo.converged_cnt = 0;
//         smo.converged = false;
//     }
//     // 保存上一拍角度
//     smo.theta_est_prev = smo.theta_est;
// }
static void SMO_Update(float i_alpha, float i_beta, float v_alpha, float v_beta)
{
    const float dt   = FOC_PWM_DT_HZ;             // 预计算 dt 倒数（避免除法）

    // 1. 电流观测误差
    float err_alpha = smo.i_alpha_est - i_alpha;
    float err_beta  = smo.i_beta_est  - i_beta;

    // 2. 滑模控制律（饱和函数，边界层 Band）
    float z_alpha, z_beta;
    if (err_alpha > smo.Band)
        z_alpha = smo.Kslide;
    else if (err_alpha < -smo.Band)
        z_alpha = -smo.Kslide;
    else
        z_alpha = smo.Kslide * err_alpha * smo.inv_Band;   // 线性区

    if (err_beta > smo.Band)
        z_beta = smo.Kslide;
    else if (err_beta < -smo.Band)
        z_beta = -smo.Kslide;
    else
        z_beta = smo.Kslide * err_beta * smo.inv_Band;

    // 可选限幅（防止电压过冲）
    const float Z_MAX = 12.0f;  // 最大24V系统
    if (z_alpha > Z_MAX) z_alpha = Z_MAX;
    if (z_alpha < -Z_MAX) z_alpha = -Z_MAX;
    if (z_beta > Z_MAX) z_beta = Z_MAX;
    if (z_beta < -Z_MAX) z_beta = -Z_MAX;

    // 3. 电流观测器更新（前向欧拉）
    smo.i_alpha_est = smo.coeff_A * smo.i_alpha_est + smo.coeff_B * (v_alpha - z_alpha);
    smo.i_beta_est  = smo.coeff_A * smo.i_beta_est  + smo.coeff_B * (v_beta  - z_beta);

    // 避免估计电流发散（限幅）
    const float I_MAX = 10.0f;
    if (smo.i_alpha_est > I_MAX) smo.i_alpha_est = I_MAX;
    if (smo.i_alpha_est < -I_MAX) smo.i_alpha_est = -I_MAX;
    if (smo.i_beta_est > I_MAX) smo.i_beta_est = I_MAX;
    if (smo.i_beta_est < -I_MAX) smo.i_beta_est = -I_MAX;

    // 4. 低通滤波获取等效反电动势
    //    滤波系数 alpha_lp = 2*pi*f_cut * dt，但确保 <1
    smo.e_alpha_est = smo.e_alpha_est * (1.0f - smo.lp_alpha) + z_alpha * smo.lp_alpha;
    smo.e_beta_est  = smo.e_beta_est  * (1.0f - smo.lp_alpha) + z_beta  * smo.lp_alpha;

    // 5. 计算反电动势幅值
    float e_mag, theta;
    Cordic_Phase(smo.e_beta_est, -smo.e_alpha_est, CORDIC_V_SCALE, &e_mag, &theta);
    smo.v_est = e_mag;

    // 6. 估计电角度（当反电动势足够大时）
    const float E_THRESH = 0.1f;   // 反电动势阈值 (V)
    if (e_mag > E_THRESH)
    {
        if (theta < 0.0f) theta += M_2PI_F;

        // 使用硬件 CORDIC（固定相位模式）计算补偿角 atan2(omega_est, lp_freq)
        theta += smo.theta_comp;

        // 归一化到 [0, 2π)
        while (theta >= M_2PI_F) theta -= M_2PI_F;
        while (theta < 0.0f)           theta += M_2PI_F;

        smo.theta_est = theta;
    }
    // 否则保持上一拍角度

    // 7. 估计电角速度（角度差分 + 低通滤波）
    float delta_theta = smo.theta_est - smo.theta_est_prev;
    // 处理角度跳变（2π 边界）
    if (delta_theta > M_PI_F)   delta_theta -= M_2PI_F;
    if (delta_theta < -M_PI_F)  delta_theta += M_2PI_F;
    float omega_raw = delta_theta / dt;
    // 一阶低通滤波（截止频率 100Hz，即 628 rad/s）
    const float OMEGA_LP_FREQ = 100.0f;   // rad/s
    float alpha_omega = OMEGA_LP_FREQ * dt;
    if (alpha_omega > 1.0f) alpha_omega = 1.0f;
    smo.omega_est = smo.omega_est * (1.0f - alpha_omega) + omega_raw * alpha_omega;

    // 8. 收敛判断：反电动势足够大且速度超过阈值（例如 200 RPM）
    const float SPEED_THRESH_RPM = 800.0f;
    float speed_rpm = smo.omega_est * RPM_PER_ELEC_RAD; // 机械转速
    smo.speed_rpm = (int16_t)speed_rpm;
    if (e_mag > 0.3f && fabsf(speed_rpm) > SPEED_THRESH_RPM)
    {
        if (smo.converged_cnt < 1000) smo.converged_cnt++;
        if (smo.converged_cnt >= 10)   // 连续10次满足条件认为收敛
            smo.converged = true;
    }
    else
    {
        smo.converged_cnt = 0;
        smo.converged = false;
    }
    // 保存上一拍角度
    smo.theta_est_prev = smo.theta_est;
}

__attribute__((section(".ccmram_text")))static void CurrentLoop_Update()
{
    // 可选：前馈项（解耦和反电动势补偿）
    float vd_ff = 0.0f;
    float vq_ff = 0.0f;
    if (motor.enable_feedforward)
    {
        // 基本电阻压降前馈
        vd_ff = motor.R * iq_actual;
        vq_ff = motor.R * id_actual;

        if (motor.enable_decoupling)
        {
            float iq_decouple = iq_actual;
            float id_decouple = id_actual;

            // 双向限幅（绝对值）
            if (iq_decouple > FOC_I_MAX_DECOUPLE)       iq_decouple = FOC_I_MAX_DECOUPLE;
            else if (iq_decouple < -FOC_I_MAX_DECOUPLE) iq_decouple = -FOC_I_MAX_DECOUPLE;

            if (id_decouple > FOC_I_MAX_DECOUPLE)       id_decouple = FOC_I_MAX_DECOUPLE;
            else if (id_decouple < -FOC_I_MAX_DECOUPLE) id_decouple = -FOC_I_MAX_DECOUPLE;
            // 交叉解耦项（需要当前电角速度）
            // 注意：smo.omega_est 是电角速度 (rad/s)
            vd_ff -= smo.omega_est * motor.Lq * iq_decouple;
            vq_ff += smo.omega_est * motor.Ld * id_decouple;
        }
        // 可选：反电动势前馈 (仅 q 轴)
        // vq_ff += smo.omega_est * motor.flux_link;
    }

    /* ------------------- D轴计算 ------------------- */
    pi_d.error = pi_d.target - pi_d.actual;
    // 2. 死区处理：如果误差小于死区，视为零误差，防止微颤
    if (fabsf(pi_d.error) < pi_d.dead_zone) {pi_d.error = 0.0f;}
    // 3. 比例项输出
    const float d_pout = pi_d.kp * pi_d.error;
    // 4. 积分项更新（带抗饱和限幅）
    pi_d.integral += pi_d.error * pi_d.sample_time;   // 累加误差 × 时间
    if (pi_d.integral > pi_d.integral_max)pi_d.integral = pi_d.integral_max;
    if (pi_d.integral < -pi_d.integral_max)pi_d.integral = -pi_d.integral_max;
    const float d_iout = pi_d.ki * pi_d.integral;      // 积分项输出
    // 5. 总输出 = 比例 + 积分
    pi_d.out = d_pout + d_iout;

    // 6. 输出限幅
    if (pi_d.out > pi_d.out_max)pi_d.out = pi_d.out_max;
    if (pi_d.out < pi_d.out_min)pi_d.out = pi_d.out_min;

    // 7. 抗饱和处理：如果输出饱和且误差与输出同号，则回退积分，防止积分深度饱和
    if ((pi_d.out >= pi_d.out_max && pi_d.error > 0) || (pi_d.out <= pi_d.out_min && pi_d.error < 0))
    {
        pi_d.integral -= pi_d.error * pi_d.sample_time;           // 回退本次积分量
        // 重新限幅积分（避免过回退）
        if (pi_d.integral > pi_d.integral_max) pi_d.integral = pi_d.integral_max;
        if (pi_d.integral < -pi_d.integral_max) pi_d.integral = -pi_d.integral_max;
    }

    /* ------------------- Q轴计算 ------------------- */
    pi_q.error = pi_q.target - pi_q.actual;
    // 2. 死区处理：如果误差小于死区，视为零误差，防止微颤
    if (fabsf(pi_q.error) < pi_q.dead_zone) {pi_q.error = 0.0f;}
    // 3. 比例项输出
    const float q_pout = pi_q.kp * pi_q.error;

    pi_q.integral += pi_q.error * pi_q.sample_time;   // 累加误差 × 时间
    if (pi_q.integral > pi_q.integral_max)pi_q.integral = pi_q.integral_max;
    if (pi_q.integral < -pi_q.integral_max)pi_q.integral = -pi_q.integral_max;

    const float q_iout = pi_q.ki * pi_q.integral;      // 积分项输出
    // 5. 总输出 = 比例 + 积分
    pi_q.out = q_pout + q_iout;

    // 6. 输出限幅
    if (pi_q.out > pi_q.out_max)pi_q.out = pi_q.out_max;
    if (pi_q.out < pi_q.out_min)pi_q.out = pi_q.out_min;

    // 7. 抗饱和处理：如果输出饱和且误差与输出同号，则回退积分，防止积分深度饱和
    if ((pi_q.out >= pi_q.out_max && pi_q.error > 0) || (pi_q.out <= pi_q.out_min && pi_q.error < 0))
    {
        pi_q.integral -= pi_q.error * pi_q.sample_time;           // 回退本次积分量
        // 重新限幅积分（避免过回退）
        if (pi_q.integral > pi_q.integral_max) pi_q.integral = pi_q.integral_max;
        if (pi_q.integral < -pi_q.integral_max) pi_q.integral = -pi_q.integral_max;
    }

    // 合并前馈和反馈
    vd_out = vd_ff + pi_d.out;
    vq_out = vq_ff + pi_q.out;

    // 电压限幅：最大相电压峰值 = Vdc / sqrt(3)
    float v_max = FOC_VBUS_VOLTAGE * 0.5773502691896258f;

    const float v_sq = vd_out*vd_out + vq_out*vq_out;
    const float v_max_sq = v_max * v_max;
    if (v_sq > v_max_sq)
    {
        const float scale = v_max / sqrtf(v_sq);
        vd_out *= scale;
        vq_out *= scale;
    }
}

__attribute__((section(".ccmram_text")))static float FOC_GetAngle(void)
{
    float theta = 0.0f;

    switch (foc_state)
    {
        case FOC_STATE_ALIGN:theta = 0.0f;break;
        case FOC_STATE_OPENLOOP:
        {
            const float dt = FOC_PWM_DT_HZ;
            if (if_handler.freq_actual != if_handler.freq_target)
            {
                const float new_freq = if_handler.freq_actual + if_handler.freq_slope * dt;
                // 防止越过目标（支持正反向）
                if ((if_handler.freq_slope > 0 && new_freq >= if_handler.freq_target) ||(if_handler.freq_slope < 0 && new_freq <= if_handler.freq_target))
                {if_handler.freq_actual = if_handler.freq_target;} else {if_handler.freq_actual = new_freq;}
            }

            if_handler.theta_open += M_2PI_F * if_handler.freq_actual * dt;
            // 归一化到 [0, 2π)
            if (if_handler.theta_open >= M_2PI_F){if_handler.theta_open -= M_2PI_F;}
            if (if_handler.theta_open < 0.0f){if_handler.theta_open += M_2PI_F;}

            theta = if_handler.theta_open;

            if (smo.converged){foc_state = FOC_STATE_TRANSITION;if_handler.alpha = 0.0f;}

        }break;
        case FOC_STATE_TRANSITION:
        {
            const float dt = FOC_PWM_DT_HZ;
            if (if_handler.freq_actual != if_handler.freq_target)
            {
                const float new_freq = if_handler.freq_actual + if_handler.freq_slope * dt;
                // 防止越过目标（支持正反向）
                if ((if_handler.freq_slope > 0 && new_freq >= if_handler.freq_target) ||(if_handler.freq_slope < 0 && new_freq <= if_handler.freq_target))
                {if_handler.freq_actual = if_handler.freq_target;} else {if_handler.freq_actual = new_freq;}
            }

            if_handler.theta_open += M_2PI_F * if_handler.freq_actual * dt;
            // 归一化到 [0, 2π)
            if (if_handler.theta_open >= M_2PI_F){if_handler.theta_open -= M_2PI_F;}
            if (if_handler.theta_open < 0.0f){if_handler.theta_open += M_2PI_F;}

            // 混合角度：开环角度 + 观测器误差补偿
            float theta_err = smo.theta_est - if_handler.theta_open;
            if (theta_err > M_PI_F)  theta_err -= M_2PI_F;
            if (theta_err < -M_PI_F) theta_err += M_2PI_F;

            const float theta_adj = if_handler.theta_open + theta_err;
            const float alpha = if_handler.alpha;

            theta = (1.0f - alpha) * if_handler.theta_open + alpha * theta_adj;

            // 归一化到 [0, 2π)
            if (theta >= M_2PI_F) theta -= M_2PI_F;
            if (theta < 0.0f)     theta += M_2PI_F;

            if (alpha < 0.999f) if_handler.alpha += 0.001f;
            else foc_state = FOC_STATE_CLOSEDLOOP;
        }break;
        case FOC_STATE_CLOSEDLOOP:theta = smo.theta_est;break;
        default:theta = 0.0f;break;
    }

    return theta;
}

__attribute__((section(".ccmram_text")))void SpeedLoop_Update(void)
{
    speed_pid.actual = smo.speed_rpm;   // 1. 获取当前机械转速 (RPM)

    const float dt = speed_pid.sample_time;   // 2. 控制周期

    // 3. 内联 PID 计算（原 PID_Update 逻辑）
    const float error = speed_pid.target - speed_pid.actual;speed_pid.error = error;

    const float p_out = speed_pid.kp * error;   // 比例项

    // 积分项（带抗饱和）
    // 预计算当前输出（用于饱和判断）
    const float pre_output = speed_pid.kp * error+ speed_pid.ki * speed_pid.integral+ speed_pid.kd * (error - speed_pid.prev_error) / dt;
    const bool saturated = (pre_output >= speed_pid.out_max) || (pre_output <= speed_pid.out_min);
    if (!saturated) {
        speed_pid.integral += error * dt;
        if (speed_pid.integral > speed_pid.integral_max)
            speed_pid.integral = speed_pid.integral_max;
        if (speed_pid.integral < speed_pid.integral_min)
            speed_pid.integral = speed_pid.integral_min;
    }
    const float i_out = speed_pid.ki * speed_pid.integral;

    // 微分项（带一阶低通滤波）
    float d_out = 0.0f;
    if (speed_pid.kd != 0.0f) {
        const float derivative = (error - speed_pid.prev_error) / dt;
        const float alpha = speed_pid.filter_alpha;   // 使用可配置系数
        speed_pid.derivative_filtered = alpha * derivative
                                      + (1.0f - alpha) * speed_pid.derivative_filtered;
        d_out = speed_pid.kd * speed_pid.derivative_filtered;
    }

    float output = p_out + i_out + d_out;
    if (output > speed_pid.out_max) output = speed_pid.out_max;
    if (output < speed_pid.out_min) output = speed_pid.out_min;

    speed_pid.prev_error = error;
    speed_pid.prev_output = output;

    pi_q.target = output;
    pi_d.target = 0.0f;
}

/**
 * @brief FOC 主控制函数，每个 PWM 周期调用一次
 * @param Iu        U 相电流
 * @param Iv        V 相电流
 */
__attribute__((section(".ccmram_text"), flatten))void FOC_Run(const float Iu, const float Iv)
{
    if (foc_state == FOC_STATE_IDLE){PWM.PWM_U = PWM.PWM_V = PWM.PWM_W = 0;return;}

    // ========== 1. 电流采样与转换 ==========
    const float ia = (Iu - ADC_MIDPOINT) * FOC_CURRENT_SCALE;
    const float ib = (Iv - ADC_MIDPOINT) * FOC_CURRENT_SCALE;
    // 第三相由 KCL 计算：ic = -ia - ib（可选，用于诊断）

    // ========== 2. Clark 变换 (αβ 电流) ==========
    const float i_alpha = ia;
    const float i_beta  = (ia + 2.0f * ib) * 0.577350269f;

    // ========== 3. 根据当前状态获取电角度 ==========
    const float theta = FOC_GetAngle();
    // ========== 4. Park 变换 (dq 电流) ==========
    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);
    float id, iq;
    ParkTransform(i_alpha, i_beta, cos_theta, sin_theta, &id, &iq);
    pi_d.actual = id;
    pi_q.actual = iq;

    // ========== 6. 电流环（计算 vd, vq） ==========
    CurrentLoop_Update();

    // ========== 7. 逆 Park 变换 (αβ 电压) ==========
    float v_alpha, v_beta;
    InvParkTransform(vd_out, vq_out, cos_theta, sin_theta, &v_alpha, &v_beta);
    // ========== 8. SVPWM 调制 ==========
    SVPWM(v_alpha, v_beta);

    // ========== 9. 滑模观测器更新（使用本次电压和电流） ==========
    // 观察器需要本次的电压和电流，用于估计下一周期的角度
    SMO_Update(i_alpha, i_beta, v_alpha, v_beta);

    // ========== 10. 状态监控（可选） ==========
    // 例如检查过流、过压等故障，设置 foc_state = FOC_STATE_FAULT

    // ========== 11. 对齐计时 ==========
    if (foc_state == FOC_STATE_ALIGN)
    {
        align_cycle_counter++;
        if (align_cycle_counter >= ALIGN_DURATION_CYCLES)
        {
            align_cycle_counter = 0;
            FOC_Stop();HAL_TIM_Base_Stop_IT(&htim4);
        }
    }

    // ========== 12. 更新遥测数据 ==========
    // 计算 W 相电流
    const float ic = -ia - ib;

    s_telemetry.IU = (int16_t) (ia * 100.0f);
    s_telemetry.IV = (int16_t) (ib * 100.0f);
    s_telemetry.IW = (int16_t) (ic * 100.0f);

}

/**
 * @brief 获取当前 FOC 遥测数据（线程安全）
 * @param tele 指向输出结构体的指针，将填入最新的遥测数据
 */
__attribute__((section(".ccmram_text")))void FOC_GetTelemetry(FOC_Telemetry_t *tele)
{
    if (tele == NULL) return;
    taskENTER_CRITICAL();
    *tele = s_telemetry;
    taskEXIT_CRITICAL();
}
