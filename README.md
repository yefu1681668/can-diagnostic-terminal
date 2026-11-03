# STM32 车载 CAN 数据采集与故障诊断终端

基于 **STM32F103C8T6 + bxCAN + W25Q16** 的车载诊断终端原型：模拟整车 ECU 周期广播传感器数据，实现 **UDS（ISO 14229）诊断服务子集** 与 **ISO-TP（ISO 15765-2）传输层**，并用外部 SPI Flash 做**掉电不丢的黑匣子事件日志**（类 EDR）。

> 一套固件，三种形态：单板自环全流程演示 / 双板真机 ECU / 双板真机诊断终端。

## 功能特性

- **CAN 数据采集**：500kbps 标准帧，100ms 周期广播转速/车速/电压/温度（ID `0x310`，带序号可检测丢帧）
- **UDS 诊断服务子集**（ISO 14229）：
  | SID | 服务 | 说明 |
  |-----|------|------|
  | 0x10 | DiagnosticSessionControl | 默认/扩展会话，P2/P2* 时序参数 |
  | 0x14 | ClearDiagnosticInformation | 清除 DTC |
  | 0x19 | ReadDTCInformation | 子功能 01 数量 / 02 列表（多帧响应） |
  | 0x22 | ReadDataByIdentifier | DID：转速/车速/电压/VIN/广播周期 |
  | 0x2E | WriteDataByIdentifier | 写广播周期（需扩展会话 + 安全解锁） |
  | 0x27 | SecurityAccess | 请求种子/发送密钥（教学简化算法） |
  否定响应码：NRC 0x11/0x12/0x13/0x22/0x31/0x33/0x36
- **ISO-TP 传输层**（ISO 15765-2 子集）：SF / FF / CF 拆包重组，序号校验防丢帧乱序
- **黑匣子事件日志**：故障置位/清码/会话切换/解锁/改参数全部落盘 W25Q16，掉电不丢，上电扫描恢复
- **分层驱动**：`can.c`（中断+环形缓冲）/ `isotp.c` / `uds.c` / `vehicle.c`（信号模拟）/ `blackbox.c` / `cli.c`（命令行）各司其职，换传感器/换协议只动一层
- **上电自检**：CAN 自环 T1（单帧）+ T2（200B 分帧重组）不通过不影响进主流程

## 三种运行形态

| 形态 | NODE_ROLE | CAN 模式 | 说明 |
|------|-----------|----------|------|
| 单板演示（默认） | `ROLE_DEMO` | LOOPBACK | ECU 栈 + 诊断栈同板互答，一块板跑完整诊断流程 |
| 整车 ECU | `ROLE_ECU` | NORMAL | 0x310 广播 + UDS 服务端（0x7DF/0x7E0 应答 0x7E8） |
| 诊断终端 | `ROLE_GATEWAY` | NORMAL | UDS 客户端 + 0x310 监听 + 黑匣子记录 |

切换：改 `Core/Inc/main.h` 里 `NODE_ROLE` 宏，重新编译即可。

## 硬件与接线

| 外设 | 引脚 |
|------|------|
| CAN 收发器（SN65HVD230，3.3V） | PA12→TX，PA11→RX，共 GND，总线两端 120Ω |
| W25Q16 SPI Flash | SPI1（PA5/6/7），CS=PA4 |
| 调试串口（CH340） | PA9=TX，PA10=RX，115200 8N1 |
| 板载 LED | PC13（低电平点亮，500ms 心跳） |

## 快速开始

1. Keil MDK 打开 `app/MDK-ARM/app.uvprojx`，Rebuild（0 Error / 0 Warning）
2. ST-Link 下载（本工程直接烧 `0x08000000`，无 Bootloader）
3. 串口终端（115200）敲命令：

```
cmd> status               # 看实时整车数据
cmd> fault 1              # 注入故障（黑匣子记录）
cmd> session 3            # 进扩展会话
cmd> seedkey              # 安全解锁（自动算密钥）
cmd> dtc                  # UDS 0x19 读 DTC 列表
cmd> cleardtc             # UDS 0x14 清码
cmd> bb                   # 导出黑匣子日志
cmd> canstats             # 收发计数
```

DEMO 模式下每条 UDS 命令都会在板内走一遍真实的 CAN 收发 + ISO-TP 拆包重组（自环），不是函数直接调用。

## 工程结构

```
├── app/                    # Keil MDK 工程（STM32CubeMX 生成框架 + 手写业务层）
│   ├── Core/Src/can.c      # bxCAN 驱动：500kbps 位时序、过滤器、RX 中断环形缓冲、自检
│   ├── Core/Src/isotp.c    # ISO-TP：SF/FF/CF 拆包重组
│   ├── Core/Src/uds.c      # UDS 服务端+客户端：0x10/0x14/0x19/0x22/0x2E/0x27
│   ├── Core/Src/vehicle.c  # 模拟整车信号源 + 故障注入
│   ├── Core/Src/blackbox.c # W25Q 环形事件日志（掉电不丢）
│   ├── Core/Src/cli.c      # 串口命令行（中断收字节 + 主循环解析）
│   └── Core/Src/w25qxx.c   # W25Q SPI Flash 底层驱动
├── docs/                   # 协议与设计文档
└── LICENSE
```

## 已验证

- [x] CAN 500kbps 位时序（APB1 36MHz ÷ 4 ÷ 18TQ，采样点 88.9%）
- [x] ISO-TP 多帧拆包重组（0x19 列表响应 > 8B 自动走 FF+CF）
- [x] UDS 全服务闭环（含 NRC 否定路径、安全访问拒绝路径）
- [x] 黑匣子掉电恢复（拔电重上电日志不丢）
- [ ] 双板 NORMAL 模式真机互通（待第二块 C8T6 + 可靠收发器）

## 已知说明

- `0x27` 密钥算法为教学简化（seed XOR 常数），真实产品应使用带 challenge 的加密算法
- 黑匣子为"写满整池后擦除重写"的简化策略，滚动擦除 + 磨损均衡方案见 `docs/`
- DEMO 模式依赖 bxCAN 回环特性（LBKM），不需要任何外部硬件即可演示

## License

MIT
