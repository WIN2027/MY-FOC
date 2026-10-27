#include "FOC_CORE.h"
#include "FOC_PORT.h"
#include <math.h>
#include <string.h>

/* =========================================================
 *  全局状态
 *  v1.2：foc_state 加 volatile，防御 FOC_OnFault 抢占 FOC_Run 后的编译器缓存
 *  v1.3：新增故障锁存 s_fault_latched + 状态迁移守卫，避免 FAULT 被写回覆盖
 * ========================================================= */
FOC_CCM_DATA volatile FOC_State_t foc_state = FOC_STATE_IDLE;
FOC_CCM_DATA PID_Handle_t    speed_pid;
FOC_CCM_DATA SMO_Handle_t    smo;
FOC_CCM_DATA PI_Handle_t     pi_d, pi_q;
FOC_CCM_DATA FOC_PWM_t       PWM;
FOC_CCM_DATA FOC_Telemetry_t s_telemetry = {0};

FOC_CCM_DATA static IF_Handle_t     if_handler;
FOC_CCM_DATA static MotorParams_t   motor;
FOC_CCM_DATA static float           id_actual, iq_actual;
FOC_CCM_DATA static float           vd_out, vq_out;
FOC_CCM_DATA static uint32_t        align_cycle_counter;
FOC_CCM_DATA static uint32_t        align_target_cycles;
FOC_CCM_DATA static FOC_StartupConfig_t startup_cfg;  /* 用户配置 */
FOC_CCM_DATA static FOC_StartupConfig_t run_cfg;      /* FOC_Start 时快照 */
FOC_CCM_DATA static bool            cfg_valid = false;
FOC_CCM_DATA static volatile uint32_t last_fault = FOC_FAULT_NONE;
FOC_CCM_DATA static volatile bool   s_fault_latched = false;
FOC_CCM_DATA static volatile uint32_t s_fault_seq = 0U;
FOC_CCM_DATA static bool            s_initialized = false;
FOC_CCM_DATA static bool            s_cordic_ok = true;
FOC_CCM_DATA static uint32_t        openloop_cycle_counter = 0U;
FOC_CCM_DATA static uint32_t        openloop_timeout_cycles = 0U;
FOC_CCM_DATA static uint8_t         oc_trip_cnt = 0U;

/* v1.2：母线电压在线补偿（可通过 FOC_SetVbus 更新） */
FOC_CCM_DATA static float s_vbus         = FOC_VBUS_VOLTAGE;
FOC_CCM_DATA static float s_svpwm_scale  = (float)FOC_PWM_PERIOD / FOC_VBUS_VOLTAGE;

/* =========================================================
 *  内部：参数有限性检查（NaN/Inf 防护，v1.3）
 * ========================================================= */
static inline bool foc_is_finite(float v)
{
    return isfinite(v) ? true : false;
}

/* 运行时极对数；正常配置为 1~255，兜底避免除零 */
static inline uint8_t foc_pole_pairs(void)
{
    return (motor.pole_pairs != 0U) ? motor.pole_pairs : 1U;
}

/* =========================================================
 *  内部：状态迁移守卫（v1.3）
 *  只有在“当前状态 == expected 且未锁存故障”时才写入 next；
 *  写入期间关中断，避免 FOC_OnFault 抢占后把 FAULT 覆盖掉。
 * ========================================================= */
static bool foc_transition(FOC_State_t expected, FOC_State_t next)
{
    bool ok = false;

    FOC_PORT_ENTER_CRITICAL();
    if (!s_fault_latched && (foc_state == expected))
    {
        foc_state = next;
        ok = true;
    }
    FOC_PORT_EXIT_CRITICAL();

    return ok;
}

/* =========================================================
 *  内部：按当前配置/目标转速重算 Kslide（v1.3）
 *  保证 z_max 覆盖运行范围内可能出现的最大的反电动势峰值。
 * ========================================================= */
static void foc_update_kslide(void)
{
#if FOC_LEGACY_CONTROL
    /* 原版行为：固定滑模增益，不随目标转速/磁链自动放大 */
    smo.Kslide = 1.0f;
#else
    float max_rpm = FOC_DEFAULT_TARGET_RPM;
    const float target = fabsf(speed_pid.target);
    const uint8_t pole_pairs = foc_pole_pairs();

    if (target > max_rpm) max_rpm = target;

    if (cfg_valid)
    {
        const float oloop_rpm = startup_cfg.openloop_freq_hz * 60.0f
                              / (float)pole_pairs;
        if (oloop_rpm > max_rpm) max_rpm = oloop_rpm;
    }

    const float omega_e = max_rpm * (float)pole_pairs * (M_2PI_F / 60.0f);
    float kslide = FOC_SMO_KSLIDE_MARGIN * omega_e * motor.flux_link;

    if (kslide < FOC_SMO_KSLIDE_MIN) kslide = FOC_SMO_KSLIDE_MIN;
    if (kslide > FOC_SMO_Z_MAX)      kslide = FOC_SMO_Z_MAX;   /* 超过 z 限幅无意义 */

    smo.Kslide = kslide;
#endif
}


/* =========================================================
 *  内部：对齐完成 -> 强拖初始化
 * ========================================================= */
static void foc_enter_openloop(void)
{
    const FOC_StartupConfig_t *cfg = &run_cfg;

    if (s_fault_latched || (foc_state != FOC_STATE_ALIGN)) return;

    if_handler.freq_target = cfg->openloop_freq_hz;
    if_handler.freq_actual = 0.0f;
    if_handler.freq_slope  = cfg->openloop_freq_hz * 1000.0f / (float)cfg->openloop_time_ms;
    if_handler.theta_open  = 0.0f;
    if_handler.alpha       = 0.0f;

    pi_d.target = 0.0f;
    pi_q.target = cfg->openloop_current_a;

    /* 对齐结束进入强拖前，清全部 SMO 动态状态，避免 ALIGN 阶段残值
     * 污染 OPENLOOP。启动流程与旧版一致：对齐是独立阶段，强拖从零角度
     * 和零速度估计开始。 */
    smo.i_alpha_est = smo.i_beta_est = 0.0f;
    smo.e_alpha_est = smo.e_beta_est = 0.0f;
    smo.theta_est = smo.omega_est = smo.theta_est_prev = 0.0f;
    smo.v_est = 0.0f;
    smo.converged_cnt = 0;
    smo.converged = false;

    /* v1.3: 收敛阈值按强拖目标转速动态计算。
     * 固定 800 rpm 在 60~80 Hz（7 对极 ≈514~686 rpm）配置下永远不可达；
     * 这里取 min(800, 0.85 × 目标转速)，低速配置也能进入闭环。 */
    {
        const float oloop_rpm = cfg->openloop_freq_hz * 60.0f
                              / (float)foc_pole_pairs();
        float conv = FOC_SMO_CONV_RATIO * oloop_rpm;
        if (conv > FOC_SMO_CONV_RPM_MIN) conv = FOC_SMO_CONV_RPM_MIN;
        if (conv < 50.0f)                conv = 50.0f;
        smo.conv_rpm_min = conv;
    }

    /* v1.3: 开环收敛超时 = 2 × 加速时间 + 500 ms；超时切 FAULT，
     * 避免 SMO 不收敛时无限期强拖。 */
    {
        const uint32_t accel_cycles = (cfg->openloop_time_ms * FOC_PWM_FREQ_HZ) / 1000U;
        openloop_timeout_cycles = 2U * accel_cycles + (FOC_PWM_FREQ_HZ / 2U);
        openloop_cycle_counter  = 0U;
    }

    /* 本地修改：不在这里覆盖 speed_pid.target。
     * 上位机可能在 ALIGN 期间已经下发目标转速；进入强拖/闭环时应保持该值。
     * 上电默认目标仍由 FOC_Init() 写入 FOC_DEFAULT_TARGET_RPM。 */

    (void)foc_transition(FOC_STATE_ALIGN, FOC_STATE_OPENLOOP);
}

/* =========================================================
 *  CORDIC / atan2f 统一入口（保留为公共 API）
 * ========================================================= */
FOC_CCM_TEXT void Cordic_Phase(float x, float y, float scale, float *mag, float *angle)
{
    (void)scale;   /* TEST_MODE 的软件实现不使用 scale；真实 port 使用 */
    FOC_PORT_CORDIC_PHASE(x, y, scale, mag, angle);
}

/* =========================================================
 *  基础变换
 * ========================================================= */
FOC_CCM_TEXT static void ParkTransform(float i_alpha, float i_beta,
                                       float cos_t, float sin_t,
                                       float *id, float *iq)
{
    *id =  i_alpha * cos_t + i_beta * sin_t;
    *iq = -i_alpha * sin_t + i_beta * cos_t;
}

FOC_CCM_TEXT static void InvParkTransform(float vd, float vq,
                                          float cos_t, float sin_t,
                                          float *v_alpha, float *v_beta)
{
    *v_alpha = vd * cos_t - vq * sin_t;
    *v_beta  = vd * sin_t + vq * cos_t;
}

/* =========================================================
 *  SVPWM：核心层唯一调用 FOC_PORT_PWM_WRITE 的位置
 * ========================================================= */
FOC_CCM_TEXT static void SVPWM(float v_alpha, float v_beta,
                               float *v_alpha_applied, float *v_beta_applied)
{
    /* v1.2: FOC_OnFault / FOC_Stop 可能在 FOC_Run 执行中被抢占。
     * PWM 已被关闭，直接返回避免写比较寄存器、污染遥测。
     * 依赖 foc_state 的 volatile 语义，确保每次都从内存重读。 */
    if (s_fault_latched || foc_state == FOC_STATE_FAULT || foc_state == FOC_STATE_IDLE)
    {
        *v_alpha_applied = 0.0f;
        *v_beta_applied  = 0.0f;
        return;
    }

    const float half_va = 0.5f * v_alpha;
    float va = v_alpha;
    float vb = -half_va + FOC_SQRT3_2_F * v_beta;
    float vc = -half_va - FOC_SQRT3_2_F * v_beta;

    const float v_max = fmaxf(va, fmaxf(vb, vc));
    const float v_min = fminf(va, fminf(vb, vc));
    const float v_offset = 0.5f * s_vbus - 0.5f * (v_max + v_min);

    va += v_offset;
    vb += v_offset;
    vc += v_offset;

    float cmp_a = va * s_svpwm_scale;
    float cmp_b = vb * s_svpwm_scale;
    float cmp_c = vc * s_svpwm_scale;

    if (cmp_a < MIN_CMP) cmp_a = MIN_CMP;
    if (cmp_a > MAX_CMP) cmp_a = MAX_CMP;
    if (cmp_b < MIN_CMP) cmp_b = MIN_CMP;
    if (cmp_b > MAX_CMP) cmp_b = MAX_CMP;
    if (cmp_c < MIN_CMP) cmp_c = MIN_CMP;
    if (cmp_c > MAX_CMP) cmp_c = MAX_CMP;

    FOC_PORT_PWM_WRITE((uint16_t)cmp_a, (uint16_t)cmp_b, (uint16_t)cmp_c);

    PWM.PWM_U = (uint16_t)cmp_a;
    PWM.PWM_V = (uint16_t)cmp_b;
    PWM.PWM_W = (uint16_t)cmp_c;

    /* v1.3: 由实际比较值反算施加到电机上的 α/β 电压，供 SMO 使用。
     * 这样过调制/相电压钳位时观测器输入与实际施加电压一致。 */
    {
        const float inv_scale = 1.0f / s_svpwm_scale;
        const float va_app = cmp_a * inv_scale;
        const float vb_app = cmp_b * inv_scale;
        const float vc_app = cmp_c * inv_scale;

        *v_alpha_applied = (2.0f * va_app - vb_app - vc_app) * (1.0f / 3.0f);
        *v_beta_applied  = (vb_app - vc_app) * FOC_1_SQRT3_F;
    }
}

/* =========================================================
 *  SMO 更新（每 PWM 周期一次）
 * ========================================================= */
FOC_CCM_TEXT static void SMO_Update(float i_alpha, float i_beta,
                                    float v_alpha, float v_beta)
{
    const float dt = FOC_PWM_DT_S;

    const float err_alpha = smo.i_alpha_est - i_alpha;
    const float err_beta  = smo.i_beta_est  - i_beta;

    float z_alpha, z_beta;
    if (err_alpha >  smo.Band) z_alpha =  smo.Kslide;
    else if (err_alpha < -smo.Band) z_alpha = -smo.Kslide;
    else z_alpha = smo.Kslide * err_alpha * smo.inv_Band;

    if (err_beta >  smo.Band) z_beta =  smo.Kslide;
    else if (err_beta < -smo.Band) z_beta = -smo.Kslide;
    else z_beta = smo.Kslide * err_beta * smo.inv_Band;

    if (z_alpha >  FOC_SMO_Z_MAX) z_alpha =  FOC_SMO_Z_MAX;
    if (z_alpha < -FOC_SMO_Z_MAX) z_alpha = -FOC_SMO_Z_MAX;
    if (z_beta  >  FOC_SMO_Z_MAX) z_beta  =  FOC_SMO_Z_MAX;
    if (z_beta  < -FOC_SMO_Z_MAX) z_beta  = -FOC_SMO_Z_MAX;

    smo.i_alpha_est = smo.coeff_A * smo.i_alpha_est + smo.coeff_B * (v_alpha - z_alpha);
    smo.i_beta_est  = smo.coeff_A * smo.i_beta_est  + smo.coeff_B * (v_beta  - z_beta);

    if (smo.i_alpha_est >  FOC_SMO_I_MAX) smo.i_alpha_est =  FOC_SMO_I_MAX;
    if (smo.i_alpha_est < -FOC_SMO_I_MAX) smo.i_alpha_est = -FOC_SMO_I_MAX;
    if (smo.i_beta_est  >  FOC_SMO_I_MAX) smo.i_beta_est  =  FOC_SMO_I_MAX;
    if (smo.i_beta_est  < -FOC_SMO_I_MAX) smo.i_beta_est  = -FOC_SMO_I_MAX;

    smo.e_alpha_est = smo.e_alpha_est * (1.0f - smo.lp_alpha) + z_alpha * smo.lp_alpha;
    smo.e_beta_est  = smo.e_beta_est  * (1.0f - smo.lp_alpha) + z_beta  * smo.lp_alpha;

    float e_mag, theta;
    FOC_PORT_CORDIC_PHASE(smo.e_beta_est, -smo.e_alpha_est, CORDIC_V_SCALE,
                          &e_mag, &theta);
    smo.v_est = e_mag;

    /* 观测器输出纯角度；theta_comp 已停用，不再进入观测或输出。 */
    if (e_mag > FOC_SMO_E_THRESH)
    {
        if (theta < 0.0f) theta += M_2PI_F;

        /* ω<0 时 atan2(-e_alpha, e_beta) 偏 π，补偿回来。
         * 用上一帧的 omega_est 判断符号（速度变化远慢于角度），启动时
         * omega_est=0 不补偿，自然过渡到正确值。 */
        if (smo.omega_est < 0.0f)
        {
            theta += M_PI_F;
            if (theta >= M_2PI_F) theta -= M_2PI_F;
        }

        while (theta >= M_2PI_F) theta -= M_2PI_F;
        while (theta < 0.0f)     theta += M_2PI_F;
        smo.theta_est = theta;
    }

    float d_theta = smo.theta_est - smo.theta_est_prev;
    if (d_theta >  M_PI_F) d_theta -= M_2PI_F;
    if (d_theta < -M_PI_F) d_theta += M_2PI_F;
    const float omega_raw = d_theta / dt;

    const float OMEGA_LP = 100.0f;
    float a_omega = OMEGA_LP * dt;
    if (a_omega > 1.0f) a_omega = 1.0f;
    smo.omega_est = smo.omega_est * (1.0f - a_omega) + omega_raw * a_omega;

    const float rpm = smo.omega_est * (FOC_RAD_S_TO_RPM / (float)foc_pole_pairs());

    /* v1.2: int16 饱和，消除浮点→整型转换超范围的 UB */
    if (rpm > 32767.0f)        smo.speed_rpm =  32767;
    else if (rpm < -32768.0f)  smo.speed_rpm = -32768;
    else                       smo.speed_rpm = (int16_t)rpm;

    if (e_mag > FOC_SMO_CONV_E_MIN && fabsf(rpm) > smo.conv_rpm_min)
    {
        if (smo.converged_cnt < FOC_SMO_CONV_CNT_MAX) smo.converged_cnt++;
        if (smo.converged_cnt >= FOC_SMO_CONV_CNT_MIN) smo.converged = true;
    }
    else
    {
        smo.converged_cnt = 0;
        smo.converged = false;
    }

    smo.theta_est_prev = smo.theta_est;
}

/* =========================================================
 *  PI 单步运算统一实现
 *  使用 integral_min / integral_max 双边界，支持不对称限幅
 * ========================================================= */
FOC_CCM_TEXT static void PI_Step(PI_Handle_t *pi)
{
    pi->error = pi->target - pi->actual;
    if (fabsf(pi->error) < pi->dead_zone) pi->error = 0.0f;

    const float p_out = pi->kp * pi->error;
    pi->integral += pi->error * pi->sample_time;
    if (pi->integral > pi->integral_max) pi->integral = pi->integral_max;
    if (pi->integral < pi->integral_min) pi->integral = pi->integral_min;
    pi->out = p_out + pi->ki * pi->integral;

    if (pi->out > pi->out_max) pi->out = pi->out_max;
    if (pi->out < pi->out_min) pi->out = pi->out_min;

    /* 抗积分饱和：输出饱和且误差同向时回退积分 */
    if ((pi->out >= pi->out_max && pi->error > 0) ||
        (pi->out <= pi->out_min && pi->error < 0))
    {
        pi->integral -= pi->error * pi->sample_time;
        if (pi->integral > pi->integral_max) pi->integral = pi->integral_max;
        if (pi->integral < pi->integral_min) pi->integral = pi->integral_min;
    }
}

/* =========================================================
 *  电流环 PI
 * ========================================================= */
FOC_CCM_TEXT static void CurrentLoop_Update(void)
{
    float vd_ff = 0.0f, vq_ff = 0.0f;
    /* v1.3: 前馈只在 SMO 已收敛（TRANSITION/CLOSEDLOOP）后生效。
     * ALIGN/OPENLOOP 阶段 omega_est 尚未收敛，注入前馈会引入错误电压。 */
    if (motor.enable_feedforward &&
        (foc_state == FOC_STATE_TRANSITION || foc_state == FOC_STATE_CLOSEDLOOP))
    {
        /* 标准 PMSM 稳态方程：
         *   vd = R·id - ω·Lq·iq
         *   vq = R·iq + ω·Ld·id + ω·ψf
         * R 项用原始电流（id_actual / iq_actual），
         * 解耦项用限幅电流（id_dc / iq_dc），避免尖峰扰动。 */
        vd_ff = motor.R * id_actual;
        vq_ff = motor.R * iq_actual + smo.omega_est * motor.flux_link;

        if (motor.enable_decoupling)
        {
            float iq_dc = iq_actual;
            float id_dc = id_actual;
            if (iq_dc >  FOC_I_MAX_DECOUPLE) iq_dc =  FOC_I_MAX_DECOUPLE;
            if (iq_dc < -FOC_I_MAX_DECOUPLE) iq_dc = -FOC_I_MAX_DECOUPLE;
            if (id_dc >  FOC_I_MAX_DECOUPLE) id_dc =  FOC_I_MAX_DECOUPLE;
            if (id_dc < -FOC_I_MAX_DECOUPLE) id_dc = -FOC_I_MAX_DECOUPLE;
            vd_ff -= smo.omega_est * motor.Lq * iq_dc;
            vq_ff += smo.omega_est * motor.Ld * id_dc;
        }
    }

    PI_Step(&pi_d);
    PI_Step(&pi_q);

    vd_out = vd_ff + pi_d.out;
    vq_out = vq_ff + pi_q.out;

    /* 电压圆限幅（SVPWM 线性调制区最大值 = Vbus/sqrt(3)） */
    const float v_max = s_vbus * FOC_1_SQRT3_F;
    const float v_sq  = vd_out * vd_out + vq_out * vq_out;
    if (v_sq > v_max * v_max)
    {
        const float s = v_max / sqrtf(v_sq);
        vd_out *= s;
        vq_out *= s;
    }
}

/* =========================================================
 *  IF 强拖更新（OPENLOOP 与 TRANSITION 共用）
 * ========================================================= */
FOC_CCM_TEXT static void IF_Update(void)
{
    const float dt = FOC_PWM_DT_S;

    if (if_handler.freq_actual != if_handler.freq_target)
    {
        const float nf = if_handler.freq_actual + if_handler.freq_slope * dt;
        if ((if_handler.freq_slope > 0 && nf >= if_handler.freq_target) ||
            (if_handler.freq_slope < 0 && nf <= if_handler.freq_target))
            if_handler.freq_actual = if_handler.freq_target;
        else
            if_handler.freq_actual = nf;
    }

    if_handler.theta_open += M_2PI_F * if_handler.freq_actual * dt;
    if (if_handler.theta_open >= M_2PI_F) if_handler.theta_open -= M_2PI_F;
    if (if_handler.theta_open <  0.0f)    if_handler.theta_open += M_2PI_F;
}

/* =========================================================
 *  状态机角度获取
 * ========================================================= */
FOC_CCM_TEXT static float FOC_GetAngle(void)
{
    float theta = 0.0f;

    if (s_fault_latched) return 0.0f;

    switch (foc_state)
    {
        case FOC_STATE_ALIGN:
            theta = 0.0f;
            break;

        case FOC_STATE_OPENLOOP:
        {
            IF_Update();
            theta = if_handler.theta_open;

            /* v1.3: 迁移带守卫；若迁移失败（故障锁存/状态已变），
             * 不覆盖 FAULT，也不重置 alpha。 */
            if (smo.converged && foc_transition(FOC_STATE_OPENLOOP, FOC_STATE_TRANSITION))
            {
                if_handler.alpha = 0.0f;
            }
            break;
        }

        case FOC_STATE_TRANSITION:
        {
            IF_Update();

            float e = smo.theta_est - if_handler.theta_open;
            if (e >  M_PI_F) e -= M_2PI_F;
            if (e < -M_PI_F) e += M_2PI_F;

            /* v1.2: 简化公式 (1-a)*θ_open + a*(θ_open+e) == θ_open + a*e */
            theta = if_handler.theta_open + if_handler.alpha * e;
            if (theta >= M_2PI_F) theta -= M_2PI_F;
            if (theta <  0.0f)    theta += M_2PI_F;

            if (if_handler.alpha < FOC_TRANS_ALPHA_MAX)
            {
                if_handler.alpha += FOC_TRANS_ALPHA_STEP;
            }
            else
            {
                (void)foc_transition(FOC_STATE_TRANSITION, FOC_STATE_CLOSEDLOOP);
            }
            break;
        }

        case FOC_STATE_CLOSEDLOOP:
            /* theta_comp 已停用，闭环直接使用纯观测角。 */
            theta = smo.theta_est;
            break;

        default:
            theta = 0.0f;
            break;
    }
    return theta;
}

/* =========================================================
 *  速度环 PID
 * ========================================================= */
FOC_CCM_TEXT static void SpeedLoop_Update(void)
{
    speed_pid.actual = (float)smo.speed_rpm;
    const float dt = speed_pid.sample_time;

    const float error = speed_pid.target - speed_pid.actual;
    speed_pid.error = error;

    const float p_out = speed_pid.kp * error;

    /* 微分先算，便于与积分预判保持一致 */
    float d_out = 0.0f;
    if (speed_pid.kd != 0.0f)
    {
        const float der   = (error - speed_pid.prev_error) / dt;
        const float alpha = speed_pid.filter_alpha;
        speed_pid.derivative_filtered = alpha * der
                                      + (1.0f - alpha) * speed_pid.derivative_filtered;
        d_out = speed_pid.kd * speed_pid.derivative_filtered;
    }

    /* v1.2: 用候选积分预算输出，判断是否饱和再决定是否积分。
     * 与 PI_Step 的抗饱和逻辑对齐，消除"多积一次"偏差。 */
    float integ_cand = speed_pid.integral + error * dt;
    if (integ_cand > speed_pid.integral_max) integ_cand = speed_pid.integral_max;
    if (integ_cand < speed_pid.integral_min) integ_cand = speed_pid.integral_min;

    const float pre = p_out + speed_pid.ki * integ_cand + d_out;

    /* 饱和且误差同向 → 保持原积分 */
    if (!((pre >= speed_pid.out_max && error > 0.0f) ||
          (pre <= speed_pid.out_min && error < 0.0f)))
    {
        speed_pid.integral = integ_cand;
    }

    const float i_out = speed_pid.ki * speed_pid.integral;

    float out = p_out + i_out + d_out;
    if (out > speed_pid.out_max) out = speed_pid.out_max;
    if (out < speed_pid.out_min) out = speed_pid.out_min;

    speed_pid.prev_error = error;

    pi_q.target = out;
    pi_d.target = 0.0f;
}

/* =========================================================
 *  清运行时动态状态（保留电机/启动配置和 speed_pid.target）
 *  ALIGN-only 完成后、STOP 后、直接 OPENLOOP 启动前都使用同一套清零，
 *  避免上一段 ALIGN/强拖/闭环的积分、角度和 SMO 残值污染下一次启动。
 * ========================================================= */
FOC_CCM_TEXT static void foc_reset_runtime_state(void)
{
    speed_pid.actual = 0.0f;
    speed_pid.error = 0.0f;
    speed_pid.integral = 0.0f;
    speed_pid.prev_error = 0.0f;
    speed_pid.derivative_filtered = 0.0f;
    speed_pid.speed_loop_cnt = 0U;

    pi_d.target = 0.0f;
    pi_d.actual = 0.0f;
    pi_d.error = 0.0f;
    pi_d.integral = 0.0f;
    pi_d.out = 0.0f;

    pi_q.target = 0.0f;
    pi_q.actual = 0.0f;
    pi_q.error = 0.0f;
    pi_q.integral = 0.0f;
    pi_q.out = 0.0f;

    smo.i_alpha_est = smo.i_beta_est = 0.0f;
    smo.e_alpha_est = smo.e_beta_est = 0.0f;
    smo.theta_est = smo.omega_est = smo.theta_est_prev = 0.0f;
    smo.v_est = 0.0f;
    smo.speed_rpm = 0;
    smo.conv_rpm_min = FOC_SMO_CONV_RPM_MIN;
    smo.converged_cnt = 0U;
    smo.converged = false;

    if_handler.freq_target = 0.0f;
    if_handler.freq_actual = 0.0f;
    if_handler.freq_slope = 0.0f;
    if_handler.theta_open = 0.0f;
    if_handler.alpha = 0.0f;

    id_actual = iq_actual = 0.0f;
    vd_out = vq_out = 0.0f;

    align_cycle_counter = 0U;
    align_target_cycles = 0U;
    openloop_cycle_counter = 0U;
    openloop_timeout_cycles = 0U;
    oc_trip_cnt = 0U;
}

/* =========================================================
 *  初始化 / 反初始化 / 配置 / 启停
 * ========================================================= */
void FOC_Init(void)
{
    const bool was_initialized = s_initialized;

    /* 重新初始化时先停输出，并在初始化期间让 FOC_Run 空操作，
     * 避免 ISR 访问尚未初始化完成的结构体。 */
    s_initialized = false;
    if (was_initialized) FOC_PORT_PWM_DISABLE();

    /* ---- 速度环 PID ---- */
    speed_pid.kp = 0.001f;
    speed_pid.ki = 0.2f;
    speed_pid.kd = 0.0001f;
    speed_pid.target = FOC_DEFAULT_TARGET_RPM;
    speed_pid.actual = 0.0f;
    speed_pid.error = 0.0f;
    speed_pid.integral = 0.0f;
    speed_pid.prev_error = 0.0f;
    speed_pid.derivative_filtered = 0.0f;
    speed_pid.filter_alpha = 0.1f;
    speed_pid.out_min = -2.5f;
    speed_pid.out_max =  2.5f;
    speed_pid.integral_min = -4.0f;
    speed_pid.integral_max =  4.0f;
    speed_pid.sample_time = (float)FOC_SPEED_LOOP_DIV / (float)FOC_PWM_FREQ_HZ;
    speed_pid.speed_loop_cnt = 0U;

    /* ---- D 轴 PI ---- */
    pi_d.kp = 0.088f;
    pi_d.ki = 241.0f;
    pi_d.target = 0.0f;
    pi_d.actual = 0.0f;
    pi_d.error = 0.0f;
    pi_d.integral = 0.0f;
    pi_d.out = 0.0f;
    pi_d.out_min = -2.0f;
    pi_d.out_max =  2.0f;
    pi_d.integral_min = -4.0f;
    pi_d.integral_max =  4.0f;
    pi_d.dead_zone = 0.02f;
    pi_d.sample_time = FOC_PWM_DT_S;

    /* ---- Q 轴 PI ---- */
    pi_q = pi_d;

    /* ---- 电机参数（宏配置） ---- */
    motor.R = FOC_MOTOR_R;
    motor.Ld = FOC_MOTOR_LD;
    motor.Lq = FOC_MOTOR_LQ;
    motor.flux_link = FOC_MOTOR_FLUX_LINK;
    motor.pole_pairs = (uint8_t)FOC_MOTOR_POLE_PAIRS;
    motor.enable_decoupling = false;
#if FOC_LEGACY_CONTROL
    motor.enable_feedforward = false;   /* 原版行为：前馈关闭 */
#else
    motor.enable_feedforward = true;
#endif

    /* ---- SMO ---- */
    smo.R = motor.R;
    smo.L = motor.Ld;
    smo.psi_f = motor.flux_link;
    smo.lp_freq = FOC_SMO_LP_OMEGA_RAD_S;
    smo.lp_alpha = smo.lp_freq * FOC_PWM_DT_S;
    if (smo.lp_alpha > 1.0f) smo.lp_alpha = 1.0f;
    smo.Band = 0.3f;
    smo.inv_Band = 1.0f / smo.Band;
    smo.coeff_A = 1.0f - smo.R / smo.L * FOC_PWM_DT_S;
    smo.coeff_B = FOC_PWM_DT_S / smo.L;

    smo.speed_rpm = 0;
    smo.i_alpha_est = smo.i_beta_est = 0.0f;
    smo.e_alpha_est = smo.e_beta_est = 0.0f;
    smo.theta_est = smo.omega_est = smo.theta_est_prev = 0.0f;
    smo.v_est = 0.0f;
    smo.converged_cnt = 0;
    smo.converged = false;
    smo.conv_rpm_min = FOC_SMO_CONV_RPM_MIN;

    /* ---- IF ---- */
    if_handler.freq_target = FOC_OPENLOOP_FREQ_MIN_HZ;
    if_handler.freq_actual = 0.0f;
    if_handler.freq_slope  = 0.0f;
    if_handler.theta_open  = 0.0f;
    if_handler.alpha       = 0.0f;

    /* ---- 母线 & SVPWM 缩放 ---- */
    s_vbus        = FOC_VBUS_VOLTAGE;
    s_svpwm_scale = (float)FOC_PWM_PERIOD / s_vbus;

    /* ---- 全局 ---- */
    foc_state = FOC_STATE_IDLE;
    id_actual = iq_actual = 0.0f;
    vd_out = vq_out = 0.0f;
    align_cycle_counter = 0;
    align_target_cycles = 0;
    openloop_cycle_counter = 0;
    openloop_timeout_cycles = 0;
    oc_trip_cnt = 0U;

    cfg_valid     = false;
    s_fault_latched = false;
    memset(&startup_cfg, 0, sizeof(startup_cfg));
    memset(&run_cfg, 0, sizeof(run_cfg));

    /* v1.3: Kslide 由运行范围内最高转速反电动势峰值计算（见 foc_update_kslide） */
    foc_update_kslide();

    memset(&s_telemetry, 0, sizeof(s_telemetry));
    PWM.PWM_U = PWM.PWM_V = PWM.PWM_W = 0;

    /* CORDIC 自检：失败时锁存故障码并阻止 FOC_Start（v1.3）。 */
    s_cordic_ok = FOC_PORT_CORDIC_SELFTEST() ? true : false;
    last_fault  = s_cordic_ok ? FOC_FAULT_NONE : FOC_FAULT_CORDIC_MODE;
    s_initialized = true;
}

void FOC_Deinit(void)
{
    if (s_initialized) FOC_PORT_PWM_DISABLE();

    FOC_PORT_ENTER_CRITICAL();
    s_fault_latched = false;
    foc_state = FOC_STATE_IDLE;
    FOC_PORT_EXIT_CRITICAL();

    s_initialized = false;

    PWM.PWM_U = PWM.PWM_V = PWM.PWM_W = 0;
    memset(&s_telemetry, 0, sizeof(s_telemetry));

    foc_reset_runtime_state();
}

FOC_Err_t FOC_SetStartupConfig(const FOC_StartupConfig_t *cfg)
{
    if (cfg == NULL) return FOC_ERR_PARAM_OUT_OF_RANGE;

    if (foc_state != FOC_STATE_IDLE && foc_state != FOC_STATE_FAULT)
        return FOC_ERR_BUSY;

    /* v1.3: NaN/Inf 防护。NaN 与任何数比较均为 false，旧的范围检查会放行。 */
    if (!foc_is_finite(cfg->align_current_a) ||
        !foc_is_finite(cfg->openloop_current_a) ||
        !foc_is_finite(cfg->openloop_freq_hz))
        return FOC_ERR_PARAM_OUT_OF_RANGE;

    if (cfg->align_current_a < FOC_ALIGN_CURRENT_MIN_A ||
        cfg->align_current_a > FOC_ALIGN_CURRENT_MAX_A)
        return FOC_ERR_PARAM_OUT_OF_RANGE;

    if (cfg->align_time_ms < FOC_ALIGN_TIME_MIN_MS ||
        cfg->align_time_ms > FOC_ALIGN_TIME_MAX_MS)
        return FOC_ERR_PARAM_OUT_OF_RANGE;

    if (cfg->openloop_current_a < FOC_OPENLOOP_CURRENT_MIN_A ||
        cfg->openloop_current_a > FOC_OPENLOOP_CURRENT_MAX_A)
        return FOC_ERR_PARAM_OUT_OF_RANGE;

    if (cfg->openloop_time_ms < FOC_OPENLOOP_TIME_MIN_MS ||
        cfg->openloop_time_ms > FOC_OPENLOOP_TIME_MAX_MS)
        return FOC_ERR_PARAM_OUT_OF_RANGE;

    if (cfg->openloop_freq_hz < FOC_OPENLOOP_FREQ_MIN_HZ ||
        cfg->openloop_freq_hz > FOC_OPENLOOP_FREQ_MAX_HZ)
        return FOC_ERR_PARAM_OUT_OF_RANGE;

    startup_cfg = *cfg;
    cfg_valid   = true;

    /* 配置变化可能提高运行转速，按新强拖频率重算 Kslide */
    foc_update_kslide();

    return FOC_OK;
}

FOC_Err_t FOC_GetStartupConfig(FOC_StartupConfig_t *cfg)
{
    if (cfg == NULL) return FOC_ERR_PARAM_OUT_OF_RANGE;
    if (!cfg_valid)  return FOC_ERR_NOT_CONFIGURED;
    *cfg = startup_cfg;
    return FOC_OK;
}

FOC_Err_t FOC_SetMotorParams(const FOC_MotorConfig_t *cfg)
{
    if (cfg == NULL) return FOC_ERR_PARAM_OUT_OF_RANGE;
    if (!s_initialized) return FOC_ERR_INVALID_STATE;

    if (foc_state != FOC_STATE_IDLE && foc_state != FOC_STATE_FAULT)
        return FOC_ERR_BUSY;

    if (!foc_is_finite(cfg->R) || !foc_is_finite(cfg->Ld) ||
        !foc_is_finite(cfg->Lq) || !foc_is_finite(cfg->flux_link))
        return FOC_ERR_PARAM_OUT_OF_RANGE;

    if (cfg->R < FOC_MOTOR_R_MIN || cfg->R > FOC_MOTOR_R_MAX) return FOC_ERR_PARAM_OUT_OF_RANGE;
    if (cfg->Ld < FOC_MOTOR_L_MIN || cfg->Ld > FOC_MOTOR_L_MAX) return FOC_ERR_PARAM_OUT_OF_RANGE;
    if (cfg->Lq < FOC_MOTOR_L_MIN || cfg->Lq > FOC_MOTOR_L_MAX) return FOC_ERR_PARAM_OUT_OF_RANGE;
    if (cfg->flux_link < FOC_MOTOR_FLUX_MIN || cfg->flux_link > FOC_MOTOR_FLUX_MAX)
        return FOC_ERR_PARAM_OUT_OF_RANGE;
    if (cfg->pole_pairs < FOC_MOTOR_POLE_PAIRS_MIN)
        return FOC_ERR_PARAM_OUT_OF_RANGE;

    /* SMO 前向欧拉离散化稳定性：R/L * dt 必须小于 1 */
    if ((cfg->R / cfg->Ld) * FOC_PWM_DT_S >= 1.0f)
        return FOC_ERR_PARAM_OUT_OF_RANGE;

    motor.R = cfg->R;
    motor.Ld = cfg->Ld;
    motor.Lq = cfg->Lq;
    motor.flux_link = cfg->flux_link;
    motor.pole_pairs = cfg->pole_pairs;

    /* SMO 当前按 Ld = Lq 的简化模型使用 Ld；Lq 仅保存供后续扩展/读取。 */
    smo.R = motor.R;
    smo.L = motor.Ld;
    smo.psi_f = motor.flux_link;
    smo.coeff_A = 1.0f - smo.R / smo.L * FOC_PWM_DT_S;
    smo.coeff_B = FOC_PWM_DT_S / smo.L;

    /* 极对数/磁链变化会改变反电动势与收敛转速，按新参数重算 Kslide */
    foc_update_kslide();

    return FOC_OK;
}

FOC_Err_t FOC_GetMotorParams(FOC_MotorConfig_t *cfg)
{
    if (cfg == NULL) return FOC_ERR_PARAM_OUT_OF_RANGE;
    if (!s_initialized) return FOC_ERR_INVALID_STATE;

    cfg->R = motor.R;
    cfg->Ld = motor.Ld;
    cfg->Lq = motor.Lq;
    cfg->flux_link = motor.flux_link;
    cfg->pole_pairs = motor.pole_pairs;
    return FOC_OK;
}

FOC_Err_t FOC_Start(FOC_State_t mode)
{
    uint32_t fault_seq_start;
    bool start_ok;
    const bool direct_openloop = (mode == FOC_STATE_OPENLOOP);

    /* ALIGN：只做对齐；OPENLOOP：跳过对齐，直接从强拖开始。 */
    if (mode != FOC_STATE_ALIGN && mode != FOC_STATE_OPENLOOP)
        return FOC_ERR_INVALID_STATE;

    if (!s_initialized) return FOC_ERR_INVALID_STATE;

    if (foc_state != FOC_STATE_IDLE && foc_state != FOC_STATE_FAULT)
        return FOC_ERR_BUSY;

    if (!cfg_valid) return FOC_ERR_NOT_CONFIGURED;

    /* v1.3: CORDIC 模式自检失败时拒绝启动，并保留 last_fault 供诊断 */
    if (!s_cordic_ok) return FOC_ERR_HW_SELFTEST;

    /* 记录当前故障序号：启动期间若出现新故障，序号会变化，最终会中止启动。
     * 这样既能从旧的 FAULT 恢复，又不会把启动过程中刚发生的新故障清掉。 */
    fault_seq_start = s_fault_seq;

    /* 启动时快照配置，运行期间不再读取 startup_cfg */
    run_cfg = startup_cfg;

    /* 每次启动前统一清动态状态，保留配置和 speed_pid.target。 */
    foc_reset_runtime_state();

    if (direct_openloop)
    {
        pi_d.target = 0.0f;
        pi_q.target = run_cfg.openloop_current_a;
    }
    else
    {
        pi_d.target = run_cfg.align_current_a;
        pi_q.target = 0.0f;
    }

    /* v1.2: 用 32 位乘除，避免 uint64 除法引入 libgcc 调用。
     * 上限 10000 × 48000 = 4.8e8 < 2^32，安全。 */
    align_target_cycles = (run_cfg.align_time_ms * FOC_PWM_FREQ_HZ) / 1000U;

    /* 先清故障锁存并进入 ALIGN，再使能 PWM；使能期间若发生故障，
     * FOC_OnFault 会重新锁存，下面会检测到并回滚。 */
    FOC_PORT_ENTER_CRITICAL();
    if (s_fault_seq != fault_seq_start)
    {
        start_ok = false;                  /* 启动过程中出现新故障，保留 FAULT */
    }
    else
    {
        s_fault_latched = false;           /* 允许从旧的 FAULT 恢复 */
        last_fault = FOC_FAULT_NONE;
        foc_state = FOC_STATE_ALIGN;
        start_ok = true;
    }
    FOC_PORT_EXIT_CRITICAL();

    if (!start_ok) return FOC_ERR_INVALID_STATE;

    /* 若在清锁存与使能之间又发生故障，不要使能输出 */
    if (s_fault_latched) return FOC_ERR_INVALID_STATE;

    if (direct_openloop)
    {
        /* 不经过 ALIGN 计时：直接初始化 IF/SMO 并切到 OPENLOOP。
         * foc_enter_openloop() 同时完成目标频率、强拖电流和超时配置。 */
        foc_enter_openloop();
        if (foc_state != FOC_STATE_OPENLOOP)
        {
            FOC_PORT_PWM_DISABLE();
            return FOC_ERR_INVALID_STATE;
        }
    }

    FOC_PORT_PWM_ENABLE();

    if (s_fault_latched)
    {
        FOC_PORT_PWM_DISABLE();
        return FOC_ERR_INVALID_STATE;   /* 启动过程中检测到硬件故障，last_fault 已记录 */
    }

    return FOC_OK;
}

void FOC_Stop(void)
{
    if (s_initialized) FOC_PORT_PWM_DISABLE();

    FOC_PORT_ENTER_CRITICAL();
    s_fault_latched = false;
    foc_state = FOC_STATE_IDLE;
    FOC_PORT_EXIT_CRITICAL();

    PWM.PWM_U = PWM.PWM_V = PWM.PWM_W = 0;
    memset(&s_telemetry, 0, sizeof(s_telemetry));

    /* ALIGN-only 结束后的 Stop 也要清角度/电流环/SMO 残值；
     * 下一次 0x04 直接 OPENLOOP 启动时从干净状态开始。 */
    foc_reset_runtime_state();
}

void FOC_OnFault(uint32_t code)
{
    /* 先关输出，再锁存状态；FOC_Run 会在下一次检查时退出。 */
    if (s_initialized) FOC_PORT_PWM_DISABLE();

    FOC_PORT_ENTER_CRITICAL();
    s_fault_latched = true;
    s_fault_seq++;
    foc_state = FOC_STATE_FAULT;
    FOC_PORT_EXIT_CRITICAL();

    PWM.PWM_U = PWM.PWM_V = PWM.PWM_W = 0;
    memset(&s_telemetry, 0, sizeof(s_telemetry));

    /* 故障后同样清角度/电流环/SMO，避免串口继续发送故障瞬间的陈旧值。 */
    foc_reset_runtime_state();

    last_fault = code;
}

uint32_t FOC_GetLastFault(void) { return last_fault; }

/* =========================================================
 *  主控制函数：每 PWM 周期一次
 * ========================================================= */
FOC_CCM_TEXT void FOC_Run(float Iu_A, float Iv_A)
{
    if (!s_initialized) return;   /* 未初始化时不访问 HRTIM/CORDIC */

    if (s_fault_latched || foc_state == FOC_STATE_IDLE || foc_state == FOC_STATE_FAULT)
    {
        FOC_PORT_PWM_WRITE(0, 0, 0);
        PWM.PWM_U = PWM.PWM_V = PWM.PWM_W = 0;
        s_telemetry.IU = 0;
        s_telemetry.IV = 0;
        s_telemetry.IW = 0;
        s_telemetry.foc_dwt = 0U;
        return;
    }

    /* ---- 输入合法性 + 软件过流（v1.3） ---- */
    const float ic = -Iu_A - Iv_A;

    if (!foc_is_finite(Iu_A) || !foc_is_finite(Iv_A))
    {
        FOC_OnFault(FOC_FAULT_INVALID_INPUT);
        return;
    }

    {
        const float i_peak = fmaxf(fabsf(Iu_A), fmaxf(fabsf(Iv_A), fabsf(ic)));
        if (i_peak > FOC_SW_OC_TRIP_A)
        {
            if (++oc_trip_cnt >= (uint8_t)FOC_SW_OC_TRIP_CNT)
            {
                FOC_OnFault(FOC_FAULT_OVERCURRENT);
                return;
            }
        }
        else
        {
            oc_trip_cnt = 0U;
        }
    }

    /* Clarke */
    const float i_alpha = Iu_A;
    const float i_beta  = (Iu_A + 2.0f * Iv_A) * FOC_1_SQRT3_F;

    /* 角度 */
    const float theta = FOC_GetAngle();
    if (s_fault_latched) return;

    /* ---- 开环收敛超时（v1.3） ---- */
    if ((foc_state == FOC_STATE_OPENLOOP) || (foc_state == FOC_STATE_TRANSITION))
    {
        if (openloop_timeout_cycles != 0U)
        {
            if (++openloop_cycle_counter >= openloop_timeout_cycles)
            {
                FOC_OnFault(FOC_FAULT_OPENLOOP_TIMEOUT);
                return;
            }
        }
    }
    else
    {
        openloop_cycle_counter = 0U;
    }

    float sin_t, cos_t;
    FOC_PORT_CORDIC_SINCOS(theta, &sin_t, &cos_t);

    /* Park */
    float id, iq;
    ParkTransform(i_alpha, i_beta, cos_t, sin_t, &id, &iq);
    pi_d.actual = id;
    pi_q.actual = iq;

    /* 电流环 -> vd_out, vq_out */
    CurrentLoop_Update();

    /* 逆 Park */
    float v_alpha, v_beta;
    InvParkTransform(vd_out, vq_out, cos_t, sin_t, &v_alpha, &v_beta);

    /* SVPWM：内部最后一次 FOC_PORT_PWM_WRITE，并回写实际施加的 α/β 电压 */
    float v_alpha_applied = 0.0f, v_beta_applied = 0.0f;
    SVPWM(v_alpha, v_beta, &v_alpha_applied, &v_beta_applied);
    if (s_fault_latched) return;

    /* SMO 更新：使用实际施加的电压估计下一拍状态（不计入测时） */
    SMO_Update(i_alpha, i_beta, v_alpha_applied, v_beta_applied);

    /* 对齐计时：完成后自动切换到 OPENLOOP */
    if (foc_state == FOC_STATE_ALIGN)
    {
        align_cycle_counter++;
        if (align_cycle_counter >= align_target_cycles)
        {
            align_cycle_counter = 0;
            foc_enter_openloop();
        }
    }

    /* 遥测三相电流（100 倍放大，int16_t） */
    s_telemetry.IU = Iu_A;
    s_telemetry.IV = Iv_A;
    s_telemetry.IW = ic;
}

/* =========================================================
 *  速度环分频（HRTIM 重复事件回调）
 * ========================================================= */
FOC_CCM_TEXT void FOC_SpeedLoopTick(void)
{
    /* 速度环只在 CLOSEDLOOP 执行。 */
#if FOC_LEGACY_CONTROL
    /* 原版速度环回调在非闭环状态也累加计数器。 */
    if (foc_state != FOC_STATE_CLOSEDLOOP && foc_state != FOC_STATE_TRANSITION)
    {
        if (speed_pid.speed_loop_cnt < (uint16_t)FOC_SPEED_LOOP_DIV)
        {
            speed_pid.speed_loop_cnt++;
        }
        return;
    }
#else
    if (foc_state != FOC_STATE_CLOSEDLOOP && foc_state != FOC_STATE_TRANSITION)
    {
        speed_pid.speed_loop_cnt = 0U;
        return;
    }
#endif

    if (++speed_pid.speed_loop_cnt >= (uint16_t)FOC_SPEED_LOOP_DIV)
    {
        speed_pid.speed_loop_cnt = 0U;

        if (foc_state == FOC_STATE_CLOSEDLOOP)
        {
            SpeedLoop_Update();
        }
    }
}

/* =========================================================
 *  参数设置 / 查询
 * ========================================================= */
void FOC_SetSpeedTarget(int16_t rpm)
{
    speed_pid.target = (float)rpm;
    foc_update_kslide();   /* 目标转速提高时同步提高 Kslide */
}

void FOC_SetCurrentTarget(float iq_a)
{
    if (!foc_is_finite(iq_a)) return;
    pi_q.target = iq_a;
}

void FOC_SetSpeedPID(float kp, float ki, float kd)
{
    if (!foc_is_finite(kp) || !foc_is_finite(ki) || !foc_is_finite(kd)) return;
    speed_pid.kp = kp; speed_pid.ki = ki; speed_pid.kd = kd;
}

void FOC_GetSpeedPID(float *kp, float *ki, float *kd)
{
    if (kp) *kp = speed_pid.kp;
    if (ki) *ki = speed_pid.ki;
    if (kd) *kd = speed_pid.kd;
}

void FOC_SetCurrentPID(float kp_d, float ki_d, float kp_q, float ki_q)
{
    if (!foc_is_finite(kp_d) || !foc_is_finite(ki_d) ||
        !foc_is_finite(kp_q) || !foc_is_finite(ki_q)) return;
    pi_d.kp = kp_d; pi_d.ki = ki_d;
    pi_q.kp = kp_q; pi_q.ki = ki_q;
}

void FOC_GetCurrentPID(float *kp_d, float *ki_d, float *kp_q, float *ki_q)
{
    if (kp_d) *kp_d = pi_d.kp;
    if (ki_d) *ki_d = pi_d.ki;
    if (kp_q) *kp_q = pi_q.kp;
    if (ki_q) *ki_q = pi_q.ki;
}

/* v1.2 新增：限幅设置（避免应用层直接改结构体成员，破坏封装） */
void FOC_SetSpeedLimits(float out_min, float out_max,
                        float integ_min, float integ_max)
{
    if (!foc_is_finite(out_min) || !foc_is_finite(out_max) ||
        !foc_is_finite(integ_min) || !foc_is_finite(integ_max)) return;
    if (out_min > out_max || integ_min > integ_max) return;
    speed_pid.out_min = out_min;
    speed_pid.out_max = out_max;
    speed_pid.integral_min = integ_min;
    speed_pid.integral_max = integ_max;
}

void FOC_SetCurrentLimits(float out_min, float out_max,
                          float integ_min, float integ_max)
{
    if (!foc_is_finite(out_min) || !foc_is_finite(out_max) ||
        !foc_is_finite(integ_min) || !foc_is_finite(integ_max)) return;
    if (out_min > out_max || integ_min > integ_max) return;
    pi_d.out_min = out_min; pi_d.out_max = out_max;
    pi_d.integral_min = integ_min; pi_d.integral_max = integ_max;
    pi_q.out_min = out_min; pi_q.out_max = out_max;
    pi_q.integral_min = integ_min; pi_q.integral_max = integ_max;
}

/* v1.2 新增：在线母线电压补偿（电池放电、母线纹波场景） */
void FOC_SetVbus(float vbus)
{
    /* 合理性范围 + 有限性检查（NaN 会绕过普通比较） */
    if (!foc_is_finite(vbus) || vbus < 1.0f || vbus > 200.0f) return;

    FOC_PORT_ENTER_CRITICAL();
    s_vbus = vbus;
    s_svpwm_scale = (float)FOC_PWM_PERIOD / vbus;
    FOC_PORT_EXIT_CRITICAL();
}

FOC_State_t FOC_GetState(void) { return foc_state; }

/* =========================================================
 *  遥测
 * ========================================================= */
FOC_CCM_TEXT void FOC_GetTelemetry(FOC_Telemetry_t *tele)
{
    if (tele == NULL) return;
    /* v1.3: 临界区改为 PRIMASK 保存/恢复，任务与中断上下文都可安全调用。 */
    FOC_PORT_ENTER_CRITICAL();
    *tele = s_telemetry;
    FOC_PORT_EXIT_CRITICAL();
}

void FOC_SetCycleCount(uint32_t cycles)
{
    s_telemetry.foc_dwt = (cycles > 65535U) ? 65535U : (uint16_t)cycles;
}