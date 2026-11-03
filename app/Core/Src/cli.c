/**
  ******************************************************************************
  * @file    cli.c
  * @brief   串口命令行实现
  ******************************************************************************
  */
#include "cli.h"
#include "main.h"
#include "can.h"
#include "vehicle.h"
#include "uds.h"
#include "blackbox.h"
#include "w25qxx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CLI_LINE_MAX  64U
#define CLI_RX_RING   64U

static char    s_line[CLI_LINE_MAX];
static uint8_t s_pos;
static uint8_t s_echo = 1U;

static volatile uint8_t s_rx[CLI_RX_RING];
static volatile uint8_t s_rx_head, s_rx_tail;

/* ---------- UART 接收中断：收一个字节 ---------- */
void cli_uart_isr(void)
{
    if ((USART1->SR & USART_SR_RXNE) != 0U)
    {
        uint8_t c = (uint8_t)(USART1->DR & 0xFFU);
        uint8_t next = (uint8_t)((s_rx_head + 1U) % CLI_RX_RING);
        if (next != s_rx_tail)              /* 满了丢新字节 */
        {
            s_rx[s_rx_head] = c;
            s_rx_head = next;
        }
    }
}

/* ---------- 命令实现 ---------- */
static void cmd_help(void)
{
    printf("commands:\r\n");
    printf("  info                  node info (role/mode/flash/uptime)\r\n");
    printf("  status                live vehicle data snapshot\r\n");
    printf("  broadcast on|off      toggle 100ms CAN broadcast\r\n");
    printf("  fault <0-3>           inject fault 1..3 (0=clear all)\r\n");
    printf("  session <1|3>         UDS 0x10 default/extended\r\n");
    printf("  read <did_hex>        UDS 0x22 e.g. read F40C\r\n");
    printf("  seedkey               UDS 0x27 request seed -> auto key\r\n");
    printf("  write <val_ms>        UDS 0x2E write broadcast period (need ext+unlock)\r\n");
    printf("  dtc                   UDS 0x19 read DTC list\r\n");
    printf("  cleardtc              UDS 0x14 clear all DTC\r\n");
    printf("  bb                    blackbox dump (W25Q log)\r\n");
    printf("  bbclear               erase blackbox log\r\n");
    printf("  canstats              CAN/UDS counters\r\n");
    printf("  echo on|off           command echo\r\n");
}

static void cmd_info(void)
{
    printf("[INFO] role=%s can=%s w25q_id=0x%06lX uptime=%lums\r\n",
           NODE_ROLE_NAME, can_mode_name(),
           (unsigned long)W25Q_ReadID(), (unsigned long)HAL_GetTick());
    printf("[INFO] tx=%lu rx=%lu bb=%lu\r\n",
           (unsigned long)can_tx_count(), (unsigned long)can_rx_count(),
           (unsigned long)blackbox_count());
}

static void cmd_status(void)
{
    vehicle_data_t *v = vehicle_get();
    printf("[STATUS] rpm=%u speed=%u.%u km/h batt=%u.%02uV coolant=%dC faults=0x%02X\r\n",
           v->rpm, v->speed_d10 / 10U, v->speed_d10 % 10U,
           v->batt_mv10 / 100U, v->batt_mv10 % 100U,
           v->coolant_c, vehicle_fault_active_mask());
}

static void cmd_fault(uint8_t idx)
{
    if (idx > 3U)
    {
        printf("usage: fault <0-3>\r\n");
        return;
    }
    if (idx == 0U)
    {
        vehicle_inject_fault(0U);
        blackbox_log(BB_EVT_DTC_CLEAR, 0xFFU, 0U);
        printf("[FAULT] all cleared\r\n");
    }
    else
    {
        vehicle_inject_fault(idx);
        blackbox_log(BB_EVT_FAULT_SET, idx, 0U);
        printf("[FAULT] injected #%u: %s\r\n", idx, vehicle_fault_name(idx));
    }
}

/* 解析两位十六进制（read F40C 里的 DID） */
static int parse_hex16(const char *s, uint16_t *out)
{
    uint32_t v = 0U;
    int i;
    for (i = 0; i < 4; i++)
    {
        char c = s[i];
        if (c == '\0') return -1;
        v <<= 4;
        if ((c >= '0') && (c <= '9'))      v |= (uint32_t)(c - '0');
        else if ((c >= 'a') && (c <= 'f')) v |= (uint32_t)(c - 'a' + 10);
        else if ((c >= 'A') && (c <= 'F')) v |= (uint32_t)(c - 'A' + 10);
        else return -1;
    }
    *out = (uint16_t)v;
    return 0;
}

static void cmd_exec(char *line)
{
    /* 简单分词：命令 + 一个参数 */
    char *cmd = strtok(line, " ");
    char *arg = strtok(NULL, " ");

    if (cmd == NULL) return;

    if      (strcmp(cmd, "help") == 0)      cmd_help();
    else if (strcmp(cmd, "info") == 0)      cmd_info();
    else if (strcmp(cmd, "status") == 0)    cmd_status();
    else if (strcmp(cmd, "broadcast") == 0)
    {
        if (arg && strcmp(arg, "on") == 0)   { vehicle_set_broadcast(1U); printf("[BROADCAST] on\r\n"); }
        else if (arg && strcmp(arg, "off")==0){ vehicle_set_broadcast(0U); printf("[BROADCAST] off\r\n"); }
        else printf("usage: broadcast on|off\r\n");
    }
    else if (strcmp(cmd, "fault") == 0)
    {
        if (arg) cmd_fault((uint8_t)atoi(arg));
        else printf("usage: fault <0-3>\r\n");
    }
    else if (strcmp(cmd, "session") == 0)
    {
        uint8_t sub = (arg && arg[0] == '3') ? UDS_SESSION_EXTENDED : UDS_SESSION_DEFAULT;
        printf("[UDS] -> 0x10 sub=%02X\r\n", sub);
        uds_client_session(sub);
    }
    else if (strcmp(cmd, "read") == 0)
    {
        uint16_t did;
        if (arg && parse_hex16(arg, &did) == 0)
        {
            printf("[UDS] -> 0x22 DID=0x%04X\r\n", did);
            uds_client_read_did(did);
        }
        else printf("usage: read <did_hex> (F40C/F40D/F442/F190/FD00)\r\n");
    }
    else if (strcmp(cmd, "seedkey") == 0)
    {
        printf("[UDS] -> 0x27 sub=01 request seed\r\n");
        uds_client_security_access();       /* 种子回来后自动算 key 发回（uds.c） */
    }
    else if (strcmp(cmd, "write") == 0)
    {
        if (arg)
        {
            uint16_t val = (uint16_t)atoi(arg);
            printf("[UDS] -> 0x2E DID=FD00 val=%u\r\n", val);
            uds_client_write_did(UDS_DID_BROADCAST_MS, val);
        }
        else printf("usage: write <val_ms> (20..1000)\r\n");
    }
    else if (strcmp(cmd, "dtc") == 0)
    {
        printf("[UDS] -> 0x19 sub=02 read DTC list\r\n");
        uds_client_read_dtc_list();
    }
    else if (strcmp(cmd, "cleardtc") == 0)
    {
        printf("[UDS] -> 0x14 clear all\r\n");
        uds_client_clear_dtc();
    }
    else if (strcmp(cmd, "bb") == 0)        blackbox_dump();
    else if (strcmp(cmd, "bbclear") == 0)
    {
        blackbox_format();
        blackbox_log(BB_EVT_POWER_ON, 0U, 0U);   /* 清完留一条上电标记 */
        printf("[BB] cleared\r\n");
    }
    else if (strcmp(cmd, "canstats") == 0)
    {
        printf("[STATS] can tx=%lu rx=%lu ring=%lu | uds req=%lu resp=%lu nrc=%lu\r\n",
               (unsigned long)can_tx_count(), (unsigned long)can_rx_count(),
               (unsigned long)can_ring_count(),
               (unsigned long)uds_req_count(), (unsigned long)uds_resp_count(),
               (unsigned long)uds_nrc_count());
    }
    else if (strcmp(cmd, "echo") == 0)
    {
        if (arg && strcmp(arg, "off") == 0) { s_echo = 0U; printf("echo off\r\n"); }
        else { s_echo = 1U; printf("echo on\r\n"); }
    }
    else
    {
        printf("unknown: %s (try 'help')\r\n", cmd);
    }
}

void cli_init(void)
{
    s_pos = 0U;
    s_rx_head = s_rx_tail = 0U;

    /* 使能 USART1 接收中断（NVIC 在 main 里开） */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    printf("\r\n=== Vehicle CAN Diagnostic Terminal (role=%s) ===\r\n", NODE_ROLE_NAME);
    printf("type 'help' for commands\r\n");
}

void cli_poll(void)
{
    while (s_rx_tail != s_rx_head)
    {
        char c = (char)s_rx[s_rx_tail];
        s_rx_tail = (uint8_t)((s_rx_tail + 1U) % CLI_RX_RING);

        if (c == '\r' || c == '\n')
        {
            if (s_pos > 0U)
            {
                s_line[s_pos] = '\0';
                if (s_echo) printf("\r\n");
                cmd_exec(s_line);
                s_pos = 0U;
                printf("cmd> ");
            }
        }
        else if ((c == 0x08U) || (c == 0x7FU))  /* 退格 */
        {
            if (s_pos > 0U) s_pos--;
        }
        else if ((s_pos + 1U) < CLI_LINE_MAX)
        {
            s_line[s_pos++] = c;
            if (s_echo)
            {
                HAL_UART_Transmit(&huart1, (uint8_t *)&c, 1, 0xFFFF);   /* 回显 */
            }
        }
    }
}
