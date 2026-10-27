#include "FOC_PORT_STM32G4.h"

/*
 * 所有 CORDIC 与 GPIO/HRTIM 硬件操作均已移至头文件进行 inline 优化，
 * 以确保代码能强制内联并编入 CCM 内存段执行。
 * 本文件保留作为空实现，维持工程结构完整。
 */