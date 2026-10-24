#include <stdbool.h>
#include <stdint.h>
#include "FOC_PORT_STM32G4.h"

/* CORDIC 输入 Q1.31 饱和转换，避免 |x*scale| >= 2^31 时的 UB */
static inline int32_t sat_to_q31(float v, float scale)
{
    const float t = v * scale;
    if (t >=  2147483647.0f) return  INT32_MAX;
    if (t <= -2147483648.0f) return  INT32_MIN;
    return (int32_t)t;
}

/* RRDY 等待加超时，避免 CORDIC 异常时在中断里永久自旋。
 * v1.3：超时从 10000 次降到 200 次（约 5 µs @170MHz），
 * 远小于 48 kHz 的 20.8 µs 控制周期；CORDIC 相位模式 6 周期即可出结果。 */
static inline bool cordic_wait_rrdy(uint32_t timeout_cnt)
{
    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U)
    {
        if (--timeout_cnt == 0U) return false;
    }
    return true;
}

/* CORDIC PHASE 模式直写寄存器
 * 输入：(x*scale, y*scale) Q1.31（已饱和）
 * 输出：RES1 = angle/π Q1.31，RES2 = magnitude/max Q1.31
 *
 * 关于 scale：
 *   - 用于电压量 (e_alpha, e_beta) 时，scale = CORDIC_V_SCALE = 2^31/24
 *   - 用于频率量 (lp_freq, omega_est) 时，scale = CORDIC_FREQ_SCALE = 2^31/10000
 *   atan 只依赖比值，scale 差异不影响 *angle；*mag 的物理量纲由 scale 决定。
 *
 * 超时降级返回 (0, 0)，SMO 本帧不更新角度，下一帧再试。 */
void FOC_Port_CordicPhase(float x, float y, float scale, float *mag, float *angle)
{
    const int32_t ix = sat_to_q31(x, scale);
    const int32_t iy = sat_to_q31(y, scale);

    CORDIC->WDATA = (uint32_t)ix;
    CORDIC->WDATA = (uint32_t)iy;

    if (!cordic_wait_rrdy(200U)) { *mag = 0.0f; *angle = 0.0f; return; }
    const int32_t res1 = (int32_t)CORDIC->RDATA;

    if (!cordic_wait_rrdy(200U)) { *mag = 0.0f; *angle = 0.0f; return; }
    const int32_t res2 = (int32_t)CORDIC->RDATA;

    *angle = ((float)res1 / 2147483648.0f) * 3.141592653589793f;
    *mag   = (float)res2 / scale;
}
