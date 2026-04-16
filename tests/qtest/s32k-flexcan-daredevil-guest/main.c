/*
 * Manual bare-metal guest used to validate the S32K-focused FlexCAN model
 * against the daredevil-small FlexCAN driver subset that currently matters:
 * init, simple MB setup, classic CAN, and CAN FD blocking mailbox traffic.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "device_registers.h"
#include "flexcan_driver.h"
#include "clocks_and_modes.h"
#include "interrupt_manager.h"

#define CAN_INSTANCE 0U
#define CLASSIC_TX_MB 0U
#define FD_TX_MB 1U
#define CLASSIC_RX_MB 4U
#define FD_RX_MB 5U

#define CLASSIC_GUEST_TX_ID 0x321U
#define CLASSIC_HOST_TX_ID  0x121U
#define CLASSIC_REPLY_ID    0x322U
#define FD_GUEST_TX_ID      0x323U
#define FD_HOST_TX_ID       0x221U
#define FD_REPLY_ID         0x324U

static flexcan_state_t can_state;
static flexcan_msgbuff_t classic_rx_frame;
static flexcan_msgbuff_t fd_rx_frame;

static void semihost_write0(const char *s)
{
    register uint32_t op asm("r0") = 0x04U;
    register const char *msg asm("r1") = s;
    asm volatile("bkpt 0xab" : : "r"(op), "r"(msg) : "memory");
}

static void fail_and_halt(const char *msg)
{
    semihost_write0(msg);
    for (;;) {
        asm volatile("nop");
    }
}

static void WDOG_disable(void)
{
    WDOG->CNT = 0xD928C520U;
    WDOG->TOVAL = 0x0000FFFFU;
    WDOG->CS = 0x00002100U;
}

static void PORT_init(void)
{
    PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK;

    PORTE->PCR[4] = PORT_PCR_MUX(5);
    PORTE->PCR[5] = PORT_PCR_MUX(5);
}

static bool bytes_match(const uint8_t *lhs, const uint8_t *rhs, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        if (lhs[i] != rhs[i]) {
            return false;
        }
    }

    return true;
}

int main(void)
{
    static const uint8_t classic_guest_data[8] = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17 };
    static const uint8_t classic_host_expected[8] = { 0x24, 0x23, 0x22, 0x21, 0x28, 0x27, 0x26, 0x25 };
    static const uint8_t classic_reply_data[8] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38 };
    static const uint8_t fd_guest_data[12] = { 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B };
    static const uint8_t fd_host_expected[12] = { 0x54, 0x53, 0x52, 0x51, 0x58, 0x57, 0x56, 0x55, 0x5C, 0x5B, 0x5A, 0x59 };
    static const uint8_t fd_reply_data[12] = { 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C };

    flexcan_user_config_t cfg;
    flexcan_data_info_t classic_info;
    flexcan_data_info_t fd_info;
    status_t status;

    semihost_write0("DRV_GUEST_BOOT\n");

    WDOG_disable();
    semihost_write0("WDOG_OK\n");
    SOSC_init_8MHz();
    semihost_write0("SOSC_OK\n");
    SPLL_init_160MHz();
    semihost_write0("SPLL_OK\n");
    NormalRUNmode_80MHz();
    semihost_write0("RUNMODE_OK\n");
    PORT_init();
    semihost_write0("PORT_OK\n");
    INT_SYS_EnableIRQGlobal();
    semihost_write0("IRQ_GLOBAL_OK\n");

    FLEXCAN_DRV_GetDefaultConfig(&cfg);
    cfg.max_num_mb = 16U;
    cfg.is_rx_fifo_needed = false;
    cfg.flexcanMode = FLEXCAN_NORMAL_MODE;
    cfg.payload = FLEXCAN_PAYLOAD_SIZE_16;
    cfg.fd_enable = true;
#if FEATURE_CAN_HAS_PE_CLKSRC_SELECT
    cfg.pe_clock = FLEXCAN_CLK_SOURCE_SOSCDIV2;
#endif
    cfg.transfer_type = FLEXCAN_RXFIFO_USING_INTERRUPTS;
    semihost_write0("CFG_PREP_OK\n");

    status = FLEXCAN_DRV_Init(CAN_INSTANCE, &can_state, &cfg);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("DRV_INIT_FAIL\n");
    }
    semihost_write0("DRV_INIT_OK\n");

    FLEXCAN_DRV_SetRxMaskType(CAN_INSTANCE, FLEXCAN_RX_MASK_GLOBAL);
    FLEXCAN_DRV_SetRxMbGlobalMask(CAN_INSTANCE, FLEXCAN_MSG_ID_STD, 0x7FFU);

    classic_info.msg_id_type = FLEXCAN_MSG_ID_STD;
    classic_info.data_length = 8U;
    classic_info.fd_enable = false;
    classic_info.fd_padding = 0x00U;
    classic_info.enable_brs = false;
    classic_info.is_remote = false;

    fd_info.msg_id_type = FLEXCAN_MSG_ID_STD;
    fd_info.data_length = 12U;
    fd_info.fd_enable = true;
    fd_info.fd_padding = 0xCCU;
    fd_info.enable_brs = true;
    fd_info.is_remote = false;

    status = FLEXCAN_DRV_ConfigTxMb(CAN_INSTANCE, CLASSIC_TX_MB, &classic_info, CLASSIC_GUEST_TX_ID);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("CFG_CLASSIC_TX_FAIL\n");
    }
    status = FLEXCAN_DRV_ConfigRxMb(CAN_INSTANCE, CLASSIC_RX_MB, &classic_info, CLASSIC_HOST_TX_ID);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("CFG_CLASSIC_RX_FAIL\n");
    }
    status = FLEXCAN_DRV_ConfigTxMb(CAN_INSTANCE, FD_TX_MB, &fd_info, FD_GUEST_TX_ID);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("CFG_FD_TX_FAIL\n");
    }
    status = FLEXCAN_DRV_ConfigRxMb(CAN_INSTANCE, FD_RX_MB, &fd_info, FD_HOST_TX_ID);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("CFG_FD_RX_FAIL\n");
    }
    semihost_write0("CFG_OK\n");

    status = FLEXCAN_DRV_SendBlocking(CAN_INSTANCE, CLASSIC_TX_MB, &classic_info,
                                      CLASSIC_GUEST_TX_ID, classic_guest_data, 1000U);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("CLASSIC_TX_FAIL\n");
    }
    semihost_write0("CLASSIC_TX_OK\n");

    status = FLEXCAN_DRV_ReceiveBlocking(CAN_INSTANCE, CLASSIC_RX_MB, &classic_rx_frame, 5000U);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("CLASSIC_RX_FAIL\n");
    }
    if ((classic_rx_frame.dataLen != 8U) || !bytes_match(classic_rx_frame.data, classic_host_expected, 8U)) {
        fail_and_halt("CLASSIC_DATA_FAIL\n");
    }
    semihost_write0("CLASSIC_RX_OK\n");

    status = FLEXCAN_DRV_SendBlocking(CAN_INSTANCE, CLASSIC_TX_MB, &classic_info,
                                      CLASSIC_REPLY_ID, classic_reply_data, 1000U);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("CLASSIC_REPLY_FAIL\n");
    }
    semihost_write0("CLASSIC_REPLY_OK\n");

    status = FLEXCAN_DRV_SendBlocking(CAN_INSTANCE, FD_TX_MB, &fd_info,
                                      FD_GUEST_TX_ID, fd_guest_data, 1000U);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("FD_TX_FAIL\n");
    }
    semihost_write0("FD_TX_OK\n");

    status = FLEXCAN_DRV_ReceiveBlocking(CAN_INSTANCE, FD_RX_MB, &fd_rx_frame, 5000U);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("FD_RX_FAIL\n");
    }
    if ((fd_rx_frame.dataLen != 12U) || !bytes_match(fd_rx_frame.data, fd_host_expected, 12U)) {
        fail_and_halt("FD_DATA_FAIL\n");
    }
    semihost_write0("FD_RX_OK\n");

    status = FLEXCAN_DRV_SendBlocking(CAN_INSTANCE, FD_TX_MB, &fd_info,
                                      FD_REPLY_ID, fd_reply_data, 1000U);
    if (status != STATUS_SUCCESS) {
        fail_and_halt("FD_REPLY_FAIL\n");
    }

    semihost_write0("DRV_GUEST_PASS\n");

    for (;;) {
        asm volatile("nop");
    }
}
