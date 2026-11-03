/**
  ******************************************************************************
  * @file    uds.h
  * @brief   UDS (ISO 14229) 诊断服务子集：ECU 服务端 + 诊断仪客户端
  ******************************************************************************
  * 实现的服务（教学子集，覆盖"读-写-安全-清码-读码"闭环）：
  *   0x10 DiagnosticSessionControl   会话控制（默认/扩展）
  *   0x14 ClearDiagnosticInformation 清除 DTC
  *   0x19 ReadDTCInformation         读 DTC（子功能 01 数量 / 02 列表）
  *   0x22 ReadDataByIdentifier       按 DID 读数据
  *   0x2E WriteDataByIdentifier      按 DID 写数据（需扩展会话+解锁）
  *   0x27 SecurityAccess             安全访问（请求种子/发送密钥）
  * 否定响应：0x7F <SID> <NRC>
  ******************************************************************************
  */
#ifndef __UDS_H
#define __UDS_H

#include <stdint.h>

/* ---- DID 表（Data Identifier） ---- */
#define UDS_DID_ENGINE_RPM    0xF40CU   /* 2B 当前转速 */
#define UDS_DID_VEHICLE_SPEED 0xF40DU   /* 2B 当前车速 x0.1 */
#define UDS_DID_BATT_VOLT     0xF442U   /* 2B 电瓶电压 x0.01V */
#define UDS_DID_VIN           0xF190U   /* 17B 车架号（只读） */
#define UDS_DID_BROADCAST_MS  0xFD00U   /* 2B 广播周期（可写，演示 0x2E） */

/* ---- 会话状态 ---- */
#define UDS_SESSION_DEFAULT   0x01U
#define UDS_SESSION_EXTENDED  0x03U

/* ---- 否定响应码 (NRC) ---- */
#define UDS_NRC_SERVICE_NOT_SUPPORTED      0x11U
#define UDS_NRC_SUBFUNC_NOT_SUPPORTED      0x12U
#define UDS_NRC_INCORRECT_MSG_LEN          0x13U
#define UDS_NRC_CONDITIONS_NOT_CORRECT     0x22U
#define UDS_NRC_REQUEST_OUT_OF_RANGE       0x31U
#define UDS_NRC_SECURITY_DENIED            0x33U
#define UDS_NRC_INVALID_KEY                0x36U

void uds_server_init(void);
void uds_server_task(void);                 /* ECU 角色：处理 0x7DF/0x7E0 上的请求 */
void uds_client_task(void);                 /* 诊断仪角色：收 0x7E8 响应并打印 */

/* 客户端便捷封装（CLI 调用）：组请求 -> isotp_send(0x7E0) */
void uds_client_send_raw(const uint8_t *msg, uint16_t len);
void uds_client_session(uint8_t sub);
void uds_client_clear_dtc(void);
void uds_client_read_dtc_list(void);
void uds_client_read_did(uint16_t did);
void uds_client_write_did(uint16_t did, uint16_t value);
void uds_client_security_access(void);

/* 统计（canstats / 演示用） */
uint32_t uds_req_count(void);
uint32_t uds_resp_count(void);
uint32_t uds_nrc_count(void);

#endif /* __UDS_H */
