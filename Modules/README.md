# FOC 工程内置可移植模块

本目录集中存放从工作区 `模块/` 复制进来的三个组件，FOC 工程只依赖本目录，不再直接引用外部路径。

| 目录 | 来源 | 版本 / commit | 本地修改 |
|---|---|---|---|
| `FOC_ADC` | `模块/FOC_ADC` | v1.3 / `4f93491` | 无，源码与上游一致 |
| `FOC_CORE` | `模块/FOC移植模块` | v1.3 / `5f24278` | `FOC_CORE.c/.h`、`FOC_CONFIG.h`：增加运行时电机参数（R/Ld/Lq/磁链/极对数）与启动/强拖参数接口；极对数改为运行时参与转速/Kslide 换算；`foc_enter_openloop()` 不再覆盖 `speed_pid.target`。启动与对齐分离：`FOC_Start(ALIGN)` 仅对齐，`FOC_Start(OPENLOOP)` 直接强拖；`FOC_Stop()`/`FOC_OnFault()` 清运行时状态。当前 `FOC_LEGACY_CONTROL=0`，恢复自动 Kslide/前馈；`theta_comp` 暂时停用；0x1000 为反电动势。README 对应说明已更新 |
| `UART_Protocol` | `模块/通用回调注册串口模块-实验版` | v1.7 / `c734d78` | 无，源码与上游一致 |

## 编译集成

- 源文件已加入 `CMakeLists.txt`。
- 头文件搜索路径：`Modules/FOC_ADC`、`Modules/FOC_CORE`、`Modules/UART_Protocol`。
- 工程级编译宏：
  - `PROTOCOL_DEFER_HANDLER=1`：协议回调在 1 ms 任务中执行，适配 DRV8320S 的阻塞 SPI/`HAL_Delay`。
  - `FOC_USE_CCM_RAM`：FOC 核心状态/代码放入 CCM。
  - `ADC_FILTER_TEXT_CCM`：ADC 滤波函数放入 CCM（`ADC_SelfTest` 仍在 Flash）。
- `Protocol_HAL_Context_t` 含 DMA 收发缓冲，必须位于普通 SRAM，不能放入 CCM；工程中由 `UART.c` 的静态变量分配在 `.bss`。

## 更新方式

从 `模块/` 同步新版本时，先比对上游差异，再把本目录的本地修改重新应用；不要直接覆盖 `FOC_CORE.c`，否则目标转速保持行为会丢失。
