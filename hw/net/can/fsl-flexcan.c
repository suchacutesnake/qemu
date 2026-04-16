/*
 * Minimal NXP FlexCAN model focused on S32K1xx basic CAN/CAN FD traffic.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "net/can_emu.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_FSL_FLEXCAN "fsl.flexcan"
OBJECT_DECLARE_SIMPLE_TYPE(FslFlexCANState, FSL_FLEXCAN)

#define FLEXCAN_MMIO_SIZE            0x1000

#define FLEXCAN_MCR                  0x000
#define FLEXCAN_CTRL1                0x004
#define FLEXCAN_TIMER                0x008
#define FLEXCAN_RXMGMASK             0x010
#define FLEXCAN_RX14MASK             0x014
#define FLEXCAN_RX15MASK             0x018
#define FLEXCAN_ECR                  0x01c
#define FLEXCAN_ESR1                 0x020
#define FLEXCAN_IMASK1               0x028
#define FLEXCAN_IFLAG1               0x030
#define FLEXCAN_CTRL2                0x034
#define FLEXCAN_ESR2                 0x038
#define FLEXCAN_CRCR                 0x044
#define FLEXCAN_RXFGMASK             0x048
#define FLEXCAN_RXFIR                0x04c
#define FLEXCAN_CBT                  0x050
#define FLEXCAN_MB_BASE              0x080
#define FLEXCAN_MB_END               0x280
#define FLEXCAN_RXIMR_BASE           0x880
#define FLEXCAN_RXIMR_END            0x900
#define FLEXCAN_CTRL1_PN             0xb00
#define FLEXCAN_CTRL2_PN             0xb04
#define FLEXCAN_WU_MTC               0xb08
#define FLEXCAN_FLT_ID1              0xb0c
#define FLEXCAN_FLT_DLC              0xb10
#define FLEXCAN_PL1_LO               0xb14
#define FLEXCAN_PL1_HI               0xb18
#define FLEXCAN_FLT_ID2_IDMASK       0xb1c
#define FLEXCAN_PL2_PLMASK_LO        0xb20
#define FLEXCAN_PL2_PLMASK_HI        0xb24
#define FLEXCAN_WMB_BASE             0xb40
#define FLEXCAN_WMB_END              0xb80
#define FLEXCAN_FDCTRL               0xc00
#define FLEXCAN_FDCBT                0xc04
#define FLEXCAN_FDCRC                0xc08
#define FLEXCAN_FIFO_OUTPUT_END      0x090
#define FLEXCAN_FIFO_ENGINE_END      0x0e0
#define FLEXCAN_FIFO_FILTER_BASE     0x0e0
#define FLEXCAN_FIFO_DEPTH           6

#define MCR_MDIS                     BIT(31)
#define MCR_FRZ                      BIT(30)
#define MCR_RFEN                     BIT(29)
#define MCR_HALT                     BIT(28)
#define MCR_NOTRDY                   BIT(27)
#define MCR_SOFTRST                  BIT(25)
#define MCR_FRZACK                   BIT(24)
#define MCR_SUPV                     BIT(23)
#define MCR_WRNEN                    BIT(21)
#define MCR_LPMACK                   BIT(20)
#define MCR_SRXDIS                   BIT(17)
#define MCR_IRMQ                     BIT(16)
#define MCR_DMA                      BIT(15)
#define MCR_PNET_EN                  BIT(14)
#define MCR_LPRIOEN                  BIT(13)
#define MCR_AEN                      BIT(12)
#define MCR_FDEN                     BIT(11)
#define MCR_IDAM_MASK                (0x3u << 8)
#define MCR_MAXMB_MASK               0x7fu

#define CTRL1_LPB                    BIT(12)
#define CTRL1_TSYN                   BIT(5)
#define CTRL1_LOM                    BIT(3)

#define CTRL2_RFFN_MASK              (0xfu << 24)
#define CTRL2_TASD_MASK              (0x1fu << 19)
#define CTRL2_MRP                    BIT(18)
#define CTRL2_RRS                    BIT(17)
#define CTRL2_EACEN                  BIT(16)
#define CTRL2_ISOCANFDEN             BIT(12)

#define ESR1_IDLE                    BIT(7)
#define ESR1_FLTCONF_SHIFT           4
#define ESR1_FLTCONF_MASK            (0x3u << ESR1_FLTCONF_SHIFT)

#define FDCTRL_FDRATE                BIT(31)
#define FDCTRL_MBDSR0_MASK           (0x3u << 16)
#define FDCTRL_TDCEN                 BIT(15)
#define FDCTRL_TDCFAIL               BIT(14)
#define FDCTRL_TDCOFF_MASK           (0x1fu << 8)

#define MB_CS_EDL                    BIT(31)
#define MB_CS_BRS                    BIT(30)
#define MB_CS_ESI                    BIT(29)
#define MB_CS_CODE_SHIFT             24
#define MB_CS_CODE_MASK              (0xfu << MB_CS_CODE_SHIFT)
#define MB_CS_SRR                    BIT(22)
#define MB_CS_IDE                    BIT(21)
#define MB_CS_RTR                    BIT(20)
#define MB_CS_DLC_SHIFT              16
#define MB_CS_DLC_MASK               (0xfu << MB_CS_DLC_SHIFT)
#define MB_CS_TIMESTAMP_MASK         0xffffu

#define MB_CODE_RX_INACTIVE          0x0
#define MB_CODE_RX_FULL              0x2
#define MB_CODE_RX_EMPTY             0x4
#define MB_CODE_RX_OVERRUN           0x6
#define MB_CODE_RX_RANSWER           0xa
#define MB_CODE_TX_INACTIVE          0x8
#define MB_CODE_TX_DATA              0xc
#define MB_CODE_TX_TANSWER           0xe

#define FLEXCAN_MAX_MB               32
#define FLEXCAN_FIFO_IDHIT_SHIFT     23
#define FLEXCAN_FIFO_IDHIT_MASK      (0x1ffu << FLEXCAN_FIFO_IDHIT_SHIFT)

#define FLEXCAN_MCR_RESET            0xd890000fu
#define FLEXCAN_CTRL2_RESET          0x00a00000u
#define FLEXCAN_FDCTRL_RESET         0x80000100u

typedef struct FslFlexCANRxFifoEntry {
    qemu_can_frame frame;
    uint16_t timestamp;
    uint16_t idhit;
} FslFlexCANRxFifoEntry;

typedef struct FslFlexCANState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    CanBusState *canbus;
    CanBusClientState bus_client;
    bool pnet;

    uint32_t mcr;
    uint32_t ctrl1;
    uint32_t timer;
    uint32_t rxmgmask;
    uint32_t rx14mask;
    uint32_t rx15mask;
    uint32_t ecr;
    uint32_t esr1;
    uint32_t imask1;
    uint32_t iflag1;
    uint32_t ctrl2;
    uint32_t esr2;
    uint32_t crcr;
    uint32_t rxfgmask;
    uint32_t rxfir;
    uint32_t cbt;
    uint32_t ctrl1_pn;
    uint32_t ctrl2_pn;
    uint32_t wu_mtc;
    uint32_t flt_id1;
    uint32_t flt_dlc;
    uint32_t pl1_lo;
    uint32_t pl1_hi;
    uint32_t flt_id2_idmask;
    uint32_t pl2_plmask_lo;
    uint32_t pl2_plmask_hi;
    uint32_t wmb[16];
    uint32_t fdctrl;
    uint32_t fdcbt;
    uint32_t fdcrc;
    uint32_t rximr[FLEXCAN_MAX_MB];
    uint8_t mb_ram[FLEXCAN_MB_END - FLEXCAN_MB_BASE];
    uint32_t rx_lock_mask;
    uint32_t rx_serviced_mask;
    FslFlexCANRxFifoEntry rx_fifo[FLEXCAN_FIFO_DEPTH];
    uint8_t rx_fifo_head;
    uint8_t rx_fifo_count;
    bool rx_fifo_warning_armed;
} FslFlexCANState;

static inline uint32_t flexcan_mb_get_word(FslFlexCANState *s, hwaddr offset)
{
    return ldl_le_p(&s->mb_ram[offset - FLEXCAN_MB_BASE]);
}

static inline void flexcan_mb_set_word(FslFlexCANState *s, hwaddr offset,
                                       uint32_t value)
{
    stl_le_p(&s->mb_ram[offset - FLEXCAN_MB_BASE], value);
}

static inline uint32_t flexcan_mb_code(uint32_t cs)
{
    return extract32(cs, MB_CS_CODE_SHIFT, 4);
}

static inline uint32_t flexcan_mb_dlc(uint32_t cs)
{
    return extract32(cs, MB_CS_DLC_SHIFT, 4);
}

static inline uint32_t flexcan_mb_set_code(uint32_t cs, uint32_t code)
{
    return (cs & ~MB_CS_CODE_MASK) | (code << MB_CS_CODE_SHIFT);
}

static inline uint32_t flexcan_mb_set_timestamp(uint32_t cs, uint16_t ts)
{
    return (cs & ~MB_CS_TIMESTAMP_MASK) | ts;
}

static inline uint16_t flexcan_get_timestamp(FslFlexCANState *s)
{
    s->timer++;
    return s->timer & 0xffff;
}

static inline bool flexcan_enabled(FslFlexCANState *s)
{
    return !(s->mcr & MCR_MDIS);
}

static inline bool flexcan_frozen(FslFlexCANState *s)
{
    return (s->mcr & MCR_FRZACK) != 0;
}

static inline bool flexcan_active(FslFlexCANState *s)
{
    return flexcan_enabled(s) && !flexcan_frozen(s);
}

static inline int flexcan_payload_size(FslFlexCANState *s)
{
    if (!(s->mcr & MCR_FDEN)) {
        return 8;
    }

    switch (extract32(s->fdctrl, 16, 2)) {
    case 0:
        return 8;
    case 1:
        return 16;
    case 2:
        return 32;
    case 3:
        return 64;
    default:
        return 8;
    }
}

static inline int flexcan_mb_stride(FslFlexCANState *s)
{
    int payload = flexcan_payload_size(s);

    return payload == 8 ? 0x10 : payload + 8;
}

static inline int flexcan_mb_capacity(FslFlexCANState *s)
{
    if (!(s->mcr & MCR_FDEN)) {
        return 32;
    }

    switch (extract32(s->fdctrl, 16, 2)) {
    case 0:
        return 32;
    case 1:
        return 21;
    case 2:
        return 12;
    case 3:
        return 7;
    default:
        return 32;
    }
}

static inline int flexcan_last_mb(FslFlexCANState *s)
{
    int maxmb = s->mcr & MCR_MAXMB_MASK;
    int capacity = flexcan_mb_capacity(s) - 1;

    return MIN(maxmb, capacity);
}

static inline hwaddr flexcan_mb_region_end(FslFlexCANState *s)
{
    return FLEXCAN_MB_BASE + (flexcan_mb_capacity(s) * flexcan_mb_stride(s));
}

static inline bool flexcan_fifo_enabled(FslFlexCANState *s)
{
    return (s->mcr & MCR_RFEN) != 0;
}

static inline int flexcan_fifo_rffn(FslFlexCANState *s)
{
    return extract32(s->ctrl2, 24, 4);
}

static inline int flexcan_fifo_filter_elements(FslFlexCANState *s)
{
    if (!flexcan_fifo_enabled(s)) {
        return 0;
    }

    return 8 * (flexcan_fifo_rffn(s) + 1);
}

static inline int flexcan_fifo_occupied_mbs(FslFlexCANState *s)
{
    if (!flexcan_fifo_enabled(s)) {
        return 0;
    }

    return 8 + (2 * flexcan_fifo_rffn(s));
}

static inline int flexcan_first_mailbox(FslFlexCANState *s)
{
    return flexcan_fifo_enabled(s) ? flexcan_fifo_occupied_mbs(s) : 0;
}

static inline int flexcan_fifo_individual_mask_limit(FslFlexCANState *s)
{
    return MIN(FLEXCAN_MAX_MB, flexcan_fifo_occupied_mbs(s));
}

static inline hwaddr flexcan_fifo_filter_table_end(FslFlexCANState *s)
{
    return FLEXCAN_FIFO_FILTER_BASE + (flexcan_fifo_filter_elements(s) * 4);
}

static inline hwaddr flexcan_mb_offset(FslFlexCANState *s, int mb)
{
    int stride = flexcan_mb_stride(s);
    hwaddr off = FLEXCAN_MB_BASE + (mb * stride);

    if (mb < 0 || mb >= flexcan_mb_capacity(s) || off >= FLEXCAN_MB_END) {
        return UINT32_MAX;
    }

    return off;
}

static inline bool flexcan_mb_is_cs_word(FslFlexCANState *s, hwaddr offset, int *mb)
{
    int stride = flexcan_mb_stride(s);

    if (offset < FLEXCAN_MB_BASE || offset >= flexcan_mb_region_end(s)) {
        return false;
    }

    if (((offset - FLEXCAN_MB_BASE) % stride) != 0) {
        return false;
    }

    if (mb) {
        *mb = (offset - FLEXCAN_MB_BASE) / stride;
    }
    return true;
}

static uint32_t flexcan_read_mask(FslFlexCANState *s, int mb)
{
    if (s->mcr & MCR_IRMQ) {
        return s->rximr[mb];
    }
    if (mb == 14) {
        return s->rx14mask;
    }
    if (mb == 15) {
        return s->rx15mask;
    }
    return s->rxmgmask;
}

static uint32_t flexcan_fifo_read_mask(FslFlexCANState *s, int filter)
{
    if (!(s->mcr & MCR_IRMQ)) {
        return s->rxfgmask;
    }

    if (filter < flexcan_fifo_individual_mask_limit(s)) {
        return s->rximr[filter];
    }

    return s->rxfgmask;
}

static void flexcan_update_irq(FslFlexCANState *s)
{
    uint32_t pending = s->iflag1 & s->imask1;

    if (flexcan_fifo_enabled(s) && (s->mcr & MCR_DMA)) {
        pending &= ~(BIT(5) | BIT(6) | BIT(7));
    }

    qemu_set_irq(s->irq, pending != 0);
}

static void flexcan_sync_mcr_state(FslFlexCANState *s)
{
    if (s->mcr & MCR_MDIS) {
        /*
         * The S32 SDK FlexCAN helpers drive MDIS directly and poll LPMACK
         * during disable/enable sequencing. Model low-power acknowledge here
         * so those waits complete before the controller is reconfigured.
         */
        s->mcr |= MCR_LPMACK;
        s->mcr |= MCR_NOTRDY | MCR_HALT;
        s->mcr &= ~MCR_FRZACK;
        return;
    }

    /* Leaving module-disable clears low-power acknowledge before FRZ/HALT. */
    s->mcr &= ~MCR_LPMACK;

    if ((s->mcr & MCR_FRZ) && (s->mcr & MCR_HALT)) {
        s->mcr |= MCR_FRZACK | MCR_NOTRDY;
    } else {
        s->mcr &= ~(MCR_FRZACK | MCR_NOTRDY);
    }
}

static void flexcan_reset_regs(FslFlexCANState *s, bool keep_mdis)
{
    uint32_t preserved_mdis = keep_mdis ? (s->mcr & MCR_MDIS) : MCR_MDIS;

    s->mcr = FLEXCAN_MCR_RESET;
    if (!preserved_mdis) {
        s->mcr &= ~MCR_MDIS;
        s->mcr |= MCR_FRZ | MCR_HALT | MCR_FRZACK | MCR_NOTRDY;
    }
    s->ctrl1 = 0;
    s->timer = 0;
    s->rxmgmask = 0;
    s->rx14mask = 0;
    s->rx15mask = 0;
    s->ecr = 0;
    s->esr1 = 0;
    s->imask1 = 0;
    s->iflag1 = 0;
    s->ctrl2 = FLEXCAN_CTRL2_RESET;
    s->esr2 = 0;
    s->crcr = 0;
    s->rxfgmask = 0;
    s->rxfir = 0;
    s->cbt = 0;
    s->ctrl1_pn = 0x00000100;
    s->ctrl2_pn = 0;
    s->wu_mtc = 0;
    s->flt_id1 = 0;
    s->flt_dlc = 0x00000008;
    s->pl1_lo = 0;
    s->pl1_hi = 0;
    s->flt_id2_idmask = 0;
    s->pl2_plmask_lo = 0;
    s->pl2_plmask_hi = 0;
    memset(s->wmb, 0, sizeof(s->wmb));
    s->fdctrl = FLEXCAN_FDCTRL_RESET;
    s->fdcbt = 0;
    s->fdcrc = 0;
    s->rx_lock_mask = 0;
    s->rx_serviced_mask = 0;
    s->rx_fifo_head = 0;
    s->rx_fifo_count = 0;
    s->rx_fifo_warning_armed = true;
    flexcan_sync_mcr_state(s);
    flexcan_update_irq(s);
}

static void flexcan_reset(DeviceState *dev)
{
    FslFlexCANState *s = FSL_FLEXCAN(dev);

    flexcan_reset_regs(s, false);
}

static bool flexcan_match_frame(FslFlexCANState *s, int mb,
                                const qemu_can_frame *frame)
{
    hwaddr mb_off = flexcan_mb_offset(s, mb);
    uint32_t cs;
    uint32_t id_reg;
    uint32_t code;
    uint32_t mask;
    bool frame_ide;
    bool frame_rtr;
    bool mb_ide;
    bool mb_rtr;
    uint32_t frame_id;
    uint32_t mb_id;
    uint32_t id_mask;

    if (mb_off == UINT32_MAX) {
        return false;
    }

    cs = flexcan_mb_get_word(s, mb_off);
    code = flexcan_mb_code(cs);
    if (code != MB_CODE_RX_EMPTY && code != MB_CODE_RX_FULL &&
        code != MB_CODE_RX_OVERRUN && code != MB_CODE_RX_RANSWER) {
        return false;
    }

    if ((frame->flags & QEMU_CAN_FRMF_TYPE_FD) && !(s->mcr & MCR_FDEN)) {
        return false;
    }

    frame_ide = !!(frame->can_id & QEMU_CAN_EFF_FLAG);
    frame_rtr = !!(frame->can_id & QEMU_CAN_RTR_FLAG);
    mb_ide = !!(cs & MB_CS_IDE);
    mb_rtr = !!(cs & MB_CS_RTR);
    mask = flexcan_read_mask(s, mb);

    if (!(s->ctrl2 & CTRL2_EACEN)) {
        if (frame_ide != mb_ide) {
            return false;
        }
    } else {
        if ((mask & BIT(30)) && frame_ide != mb_ide) {
            return false;
        }
        if ((mask & BIT(31)) && frame_rtr != mb_rtr) {
            return false;
        }
    }

    if ((s->ctrl2 & CTRL2_RRS) && frame_rtr && code == MB_CODE_RX_RANSWER) {
        return false;
    }

    id_reg = flexcan_mb_get_word(s, mb_off + 4);
    if (frame_ide) {
        frame_id = frame->can_id & QEMU_CAN_EFF_MASK;
        mb_id = id_reg & QEMU_CAN_EFF_MASK;
        id_mask = mask & QEMU_CAN_EFF_MASK;
    } else {
        frame_id = frame->can_id & QEMU_CAN_SFF_MASK;
        mb_id = (id_reg >> 18) & QEMU_CAN_SFF_MASK;
        id_mask = (mask >> 18) & QEMU_CAN_SFF_MASK;
    }

    return ((frame_id ^ mb_id) & id_mask) == 0;
}

static bool flexcan_mailbox_is_free_to_receive(FslFlexCANState *s, int mb)
{
    hwaddr mb_off = flexcan_mb_offset(s, mb);
    uint32_t cs;
    uint32_t code;

    if (mb_off == UINT32_MAX) {
        return false;
    }

    cs = flexcan_mb_get_word(s, mb_off);
    code = flexcan_mb_code(cs);

    if (code == MB_CODE_RX_EMPTY) {
        return true;
    }

    if ((code == MB_CODE_RX_FULL || code == MB_CODE_RX_OVERRUN) &&
        (s->rx_serviced_mask & BIT(mb))) {
        return true;
    }

    return false;
}

static uint32_t flexcan_frame_id_reg(const qemu_can_frame *frame)
{
    if (frame->can_id & QEMU_CAN_EFF_FLAG) {
        return frame->can_id & QEMU_CAN_EFF_MASK;
    }

    return (frame->can_id & QEMU_CAN_SFF_MASK) << 18;
}

static uint32_t flexcan_fifo_output_cs(const FslFlexCANRxFifoEntry *entry)
{
    const qemu_can_frame *frame = &entry->frame;
    uint32_t cs = (entry->idhit & 0x1ffu) << FLEXCAN_FIFO_IDHIT_SHIFT;

    if (frame->can_id & QEMU_CAN_EFF_FLAG) {
        cs |= MB_CS_SRR | MB_CS_IDE;
    }
    if (frame->can_id & QEMU_CAN_RTR_FLAG) {
        cs |= MB_CS_RTR;
    }
    cs |= (frame->can_dlc & 0xf) << MB_CS_DLC_SHIFT;
    cs |= entry->timestamp;

    return cs;
}

static void flexcan_fifo_sync_output(FslFlexCANState *s)
{
    uint32_t data = 0;
    int i;

    if (s->rx_fifo_count == 0) {
        for (i = 0; i < 4; i++) {
            flexcan_mb_set_word(s, FLEXCAN_MB_BASE + (i * 4), 0);
        }
        s->rxfir = 0;
        return;
    }

    flexcan_mb_set_word(s, 0x80, flexcan_fifo_output_cs(&s->rx_fifo[s->rx_fifo_head]));
    flexcan_mb_set_word(s, 0x84,
                        flexcan_frame_id_reg(&s->rx_fifo[s->rx_fifo_head].frame));
    for (i = 0; i < 2; i++) {
        data = s->rx_fifo[s->rx_fifo_head].frame.data[(i * 4) + 0] << 24;
        data |= s->rx_fifo[s->rx_fifo_head].frame.data[(i * 4) + 1] << 16;
        data |= s->rx_fifo[s->rx_fifo_head].frame.data[(i * 4) + 2] << 8;
        data |= s->rx_fifo[s->rx_fifo_head].frame.data[(i * 4) + 3];
        flexcan_mb_set_word(s, 0x88 + (i * 4), data);
    }
    s->rxfir = s->rx_fifo[s->rx_fifo_head].idhit & 0x1ffu;
}

static void flexcan_fifo_rearm_warning(FslFlexCANState *s)
{
    if (s->rx_fifo_count <= 4) {
        s->rx_fifo_warning_armed = true;
    }
}

static void flexcan_fifo_pop(FslFlexCANState *s)
{
    if (s->rx_fifo_count == 0) {
        s->iflag1 &= ~BIT(5);
        flexcan_fifo_sync_output(s);
        flexcan_fifo_rearm_warning(s);
        flexcan_update_irq(s);
        return;
    }

    s->rx_fifo_head = (s->rx_fifo_head + 1) % FLEXCAN_FIFO_DEPTH;
    s->rx_fifo_count--;
    if (s->rx_fifo_count == 0) {
        s->iflag1 &= ~BIT(5);
    } else {
        s->iflag1 |= BIT(5);
    }
    flexcan_fifo_rearm_warning(s);
    flexcan_fifo_sync_output(s);
    flexcan_update_irq(s);
}

static void flexcan_fifo_clear(FslFlexCANState *s)
{
    s->rx_fifo_head = 0;
    s->rx_fifo_count = 0;
    s->rx_fifo_warning_armed = true;
    if (s->mcr & MCR_DMA) {
        s->iflag1 &= ~BIT(5);
    }
    flexcan_fifo_sync_output(s);
}

static bool flexcan_fifo_match_a(const qemu_can_frame *frame,
                                 uint32_t filter, uint32_t mask)
{
    uint32_t value = (frame->can_id & QEMU_CAN_RTR_FLAG) ? BIT(31) : 0;

    if (frame->can_id & QEMU_CAN_EFF_FLAG) {
        value |= BIT(30);
        value |= (frame->can_id & QEMU_CAN_EFF_MASK) << 1;
    } else {
        value |= (frame->can_id & QEMU_CAN_SFF_MASK) << 19;
    }

    return ((value ^ filter) & mask) == 0;
}

static bool flexcan_fifo_match_b(const qemu_can_frame *frame,
                                 uint16_t filter, uint16_t mask)
{
    uint16_t value = (frame->can_id & QEMU_CAN_RTR_FLAG) ? BIT(15) : 0;

    if (frame->can_id & QEMU_CAN_EFF_FLAG) {
        value |= BIT(14);
        value |= ((frame->can_id & QEMU_CAN_EFF_MASK) >> 15) & 0x3fff;
    } else {
        value |= ((frame->can_id & QEMU_CAN_SFF_MASK) << 3) & 0x3fff;
    }

    return ((value ^ filter) & mask) == 0;
}

static bool flexcan_fifo_match_c(const qemu_can_frame *frame,
                                 uint8_t filter, uint8_t mask)
{
    uint8_t value;

    if (frame->can_id & QEMU_CAN_EFF_FLAG) {
        value = ((frame->can_id & QEMU_CAN_EFF_MASK) >> 21) & 0xff;
    } else {
        value = ((frame->can_id & QEMU_CAN_SFF_MASK) >> 3) & 0xff;
    }

    return ((value ^ filter) & mask) == 0;
}

static bool flexcan_fifo_matches(FslFlexCANState *s, const qemu_can_frame *frame,
                                 uint16_t *idhit)
{
    int elements = flexcan_fifo_filter_elements(s);
    int idam = extract32(s->mcr, 8, 2);
    int i;

    if (!flexcan_fifo_enabled(s) || (frame->flags & QEMU_CAN_FRMF_TYPE_FD)) {
        return false;
    }

    for (i = 0; i < elements; i++) {
        uint32_t filter = flexcan_mb_get_word(s, FLEXCAN_FIFO_FILTER_BASE + (i * 4));
        uint32_t mask = flexcan_fifo_read_mask(s, i);

        switch (idam) {
        case 0:
            if (flexcan_fifo_match_a(frame, filter, mask)) {
                *idhit = i;
                return true;
            }
            break;
        case 1:
            if (flexcan_fifo_match_b(frame, filter >> 16, mask >> 16)) {
                *idhit = i * 2;
                return true;
            }
            if (flexcan_fifo_match_b(frame, filter & 0xffff, mask & 0xffff)) {
                *idhit = (i * 2) + 1;
                return true;
            }
            break;
        case 2:
            if (flexcan_fifo_match_c(frame, (filter >> 24) & 0xff,
                                     (mask >> 24) & 0xff)) {
                *idhit = i * 4;
                return true;
            }
            if (flexcan_fifo_match_c(frame, (filter >> 16) & 0xff,
                                     (mask >> 16) & 0xff)) {
                *idhit = (i * 4) + 1;
                return true;
            }
            if (flexcan_fifo_match_c(frame, (filter >> 8) & 0xff,
                                     (mask >> 8) & 0xff)) {
                *idhit = (i * 4) + 2;
                return true;
            }
            if (flexcan_fifo_match_c(frame, filter & 0xff, mask & 0xff)) {
                *idhit = (i * 4) + 3;
                return true;
            }
            break;
        default:
            break;
        }
    }

    return false;
}

static void flexcan_fifo_push(FslFlexCANState *s, const qemu_can_frame *frame,
                              uint16_t idhit)
{
    int tail;

    if (s->rx_fifo_count >= FLEXCAN_FIFO_DEPTH) {
        if (!(s->mcr & MCR_DMA)) {
            s->iflag1 |= BIT(7);
        }
        flexcan_update_irq(s);
        return;
    }

    tail = (s->rx_fifo_head + s->rx_fifo_count) % FLEXCAN_FIFO_DEPTH;
    s->rx_fifo[tail].frame = *frame;
    s->rx_fifo[tail].timestamp = flexcan_get_timestamp(s);
    s->rx_fifo[tail].idhit = idhit;
    s->rx_fifo_count++;

    if (!(s->mcr & MCR_DMA) && s->rx_fifo_count == 5 && s->rx_fifo_warning_armed) {
        s->iflag1 |= BIT(6);
        s->rx_fifo_warning_armed = false;
    }

    s->iflag1 |= BIT(5);
    if (s->rx_fifo_count == 1) {
        flexcan_fifo_sync_output(s);
    }
    flexcan_update_irq(s);
}

static void flexcan_unlock_rx_mbs(FslFlexCANState *s)
{
    uint32_t mask = s->rx_lock_mask;
    int mb;

    if (!mask) {
        return;
    }

    for (mb = 0; mb < FLEXCAN_MAX_MB; mb++) {
        hwaddr mb_off;
        uint32_t code;

        if (!(mask & BIT(mb))) {
            continue;
        }
        mb_off = flexcan_mb_offset(s, mb);
        if (mb_off == UINT32_MAX) {
            continue;
        }
        code = flexcan_mb_code(flexcan_mb_get_word(s, mb_off));
        if (code == MB_CODE_RX_FULL || code == MB_CODE_RX_OVERRUN) {
            s->rx_serviced_mask |= BIT(mb);
        }
    }

    s->rx_lock_mask = 0;
}

static void flexcan_lock_rx_mb(FslFlexCANState *s, int mb, uint32_t cs)
{
    uint32_t code = flexcan_mb_code(cs);

    flexcan_unlock_rx_mbs(s);
    if (code == MB_CODE_RX_FULL || code == MB_CODE_RX_OVERRUN) {
        s->rx_lock_mask = BIT(mb);
    }
}

static void flexcan_write_frame_to_mb(FslFlexCANState *s, int mb,
                                      const qemu_can_frame *frame)
{
    hwaddr mb_off = flexcan_mb_offset(s, mb);
    uint32_t cs = flexcan_mb_get_word(s, mb_off);
    uint32_t old_code = flexcan_mb_code(cs);
    uint32_t new_code = old_code == MB_CODE_RX_EMPTY ? MB_CODE_RX_FULL
                                                     : MB_CODE_RX_OVERRUN;
    uint32_t id_reg = 0;
    uint32_t i;
    uint32_t len = can_dlc2len(frame->can_dlc);
    uint32_t max_payload = flexcan_payload_size(s);
    uint16_t timestamp = flexcan_get_timestamp(s);

    if (frame->can_id & QEMU_CAN_EFF_FLAG) {
        cs |= MB_CS_IDE | MB_CS_SRR;
        id_reg = frame->can_id & QEMU_CAN_EFF_MASK;
    } else {
        cs &= ~(MB_CS_IDE | MB_CS_SRR);
        id_reg = (frame->can_id & QEMU_CAN_SFF_MASK) << 18;
    }

    if (frame->can_id & QEMU_CAN_RTR_FLAG) {
        cs |= MB_CS_RTR;
    } else {
        cs &= ~MB_CS_RTR;
    }

    if (frame->flags & QEMU_CAN_FRMF_TYPE_FD) {
        cs |= MB_CS_EDL;
        if (frame->flags & QEMU_CAN_FRMF_BRS) {
            cs |= MB_CS_BRS;
        } else {
            cs &= ~MB_CS_BRS;
        }
        if (frame->flags & QEMU_CAN_FRMF_ESI) {
            cs |= MB_CS_ESI;
        } else {
            cs &= ~MB_CS_ESI;
        }
    } else {
        cs &= ~(MB_CS_EDL | MB_CS_BRS | MB_CS_ESI);
    }

    cs &= ~MB_CS_DLC_MASK;
    cs |= frame->can_dlc << MB_CS_DLC_SHIFT;
    cs = flexcan_mb_set_code(cs, new_code);
    cs = flexcan_mb_set_timestamp(cs, timestamp);

    flexcan_mb_set_word(s, mb_off + 4, id_reg);
    memset(&s->mb_ram[mb_off + 8 - FLEXCAN_MB_BASE], 0,
           flexcan_mb_stride(s) - 8);
    for (i = 0; i < MIN(len, max_payload); i++) {
        s->mb_ram[mb_off + 8 - FLEXCAN_MB_BASE + i] = frame->data[i];
    }
    flexcan_mb_set_word(s, mb_off, cs);
    s->rx_lock_mask &= ~BIT(mb);
    s->rx_serviced_mask &= ~BIT(mb);

    s->iflag1 |= BIT(mb);
    if ((s->ctrl1 & CTRL1_TSYN) && mb == 0) {
        s->timer = 0;
    }
    flexcan_update_irq(s);
}

static void flexcan_receive_one(FslFlexCANState *s, const qemu_can_frame *frame)
{
    int mb;
    int first_free_mb = -1;
    int first_matched_mb = -1;
    int last_nonfree_mb = -1;
    int first_mb = flexcan_first_mailbox(s);
    uint16_t idhit = 0;
    bool fifo_matched = false;
    bool fifo_free = false;

    if (!flexcan_active(s)) {
        return;
    }

    if (flexcan_fifo_enabled(s)) {
        fifo_matched = flexcan_fifo_matches(s, frame, &idhit);
        fifo_free = s->rx_fifo_count < FLEXCAN_FIFO_DEPTH;
    }

    for (mb = first_mb; mb <= flexcan_last_mb(s); mb++) {
        if (flexcan_match_frame(s, mb, frame)) {
            if (first_matched_mb < 0) {
                first_matched_mb = mb;
            }
            if (flexcan_mailbox_is_free_to_receive(s, mb)) {
                if (first_free_mb < 0) {
                    first_free_mb = mb;
                }
                if (!(s->mcr & MCR_IRMQ)) {
                    break;
                }
            } else {
                last_nonfree_mb = mb;
                if (!(s->mcr & MCR_IRMQ)) {
                    break;
                }
            }
        }
    }

    if (!flexcan_fifo_enabled(s)) {
        if (!(s->mcr & MCR_IRMQ)) {
            if (first_matched_mb >= 0) {
                flexcan_write_frame_to_mb(s, first_matched_mb, frame);
            }
        } else if (first_free_mb >= 0) {
            flexcan_write_frame_to_mb(s, first_free_mb, frame);
        } else if (last_nonfree_mb >= 0) {
            flexcan_write_frame_to_mb(s, last_nonfree_mb, frame);
        }
        return;
    }

    if (s->ctrl2 & CTRL2_MRP) {
        if (first_free_mb >= 0) {
            flexcan_write_frame_to_mb(s, first_free_mb, frame);
            return;
        }
        if (first_matched_mb < 0) {
            if (fifo_matched && fifo_free) {
                flexcan_fifo_push(s, frame, idhit);
            } else if (fifo_matched && !fifo_free && !(s->mcr & MCR_DMA)) {
                s->iflag1 |= BIT(7);
                flexcan_update_irq(s);
            }
            return;
        }

        if (!(s->mcr & MCR_IRMQ)) {
            flexcan_write_frame_to_mb(s, first_matched_mb, frame);
            return;
        }

        if (fifo_matched && fifo_free) {
            flexcan_fifo_push(s, frame, idhit);
        } else if (last_nonfree_mb >= 0) {
            flexcan_write_frame_to_mb(s, last_nonfree_mb, frame);
        } else if (fifo_matched && !fifo_free && !(s->mcr & MCR_DMA)) {
            s->iflag1 |= BIT(7);
            flexcan_update_irq(s);
        }
        return;
    }

    if (fifo_matched && fifo_free) {
        flexcan_fifo_push(s, frame, idhit);
        return;
    }

    if (!(s->mcr & MCR_IRMQ)) {
        if (first_matched_mb >= 0) {
            flexcan_write_frame_to_mb(s, first_matched_mb, frame);
        } else if (fifo_matched && !fifo_free && !(s->mcr & MCR_DMA)) {
            s->iflag1 |= BIT(7);
            flexcan_update_irq(s);
        }
        return;
    }

    if (first_free_mb >= 0) {
        flexcan_write_frame_to_mb(s, first_free_mb, frame);
    } else if (last_nonfree_mb >= 0) {
        flexcan_write_frame_to_mb(s, last_nonfree_mb, frame);
    } else if (fifo_matched && !fifo_free && !(s->mcr & MCR_DMA)) {
        s->iflag1 |= BIT(7);
        flexcan_update_irq(s);
    }
}

static bool flexcan_can_receive(CanBusClientState *client)
{
    FslFlexCANState *s = container_of(client, FslFlexCANState, bus_client);

    return flexcan_active(s);
}

static ssize_t flexcan_receive(CanBusClientState *client,
                               const qemu_can_frame *frames,
                               size_t frames_cnt)
{
    FslFlexCANState *s = container_of(client, FslFlexCANState, bus_client);
    size_t i;

    for (i = 0; i < frames_cnt; i++) {
        flexcan_receive_one(s, &frames[i]);
    }

    return frames_cnt;
}

static CanBusClientInfo flexcan_canbus_info = {
    .can_receive = flexcan_can_receive,
    .receive = flexcan_receive,
};

static void flexcan_do_tx(FslFlexCANState *s, int mb)
{
    hwaddr mb_off = flexcan_mb_offset(s, mb);
    qemu_can_frame frame = { 0 };
    uint32_t cs;
    uint32_t id;
    uint32_t len;
    uint32_t i;
    uint32_t tx_code;
    bool loopback = !!(s->ctrl1 & CTRL1_LPB);

    if (mb_off == UINT32_MAX || !flexcan_active(s) || (s->ctrl1 & CTRL1_LOM)) {
        return;
    }

    cs = flexcan_mb_get_word(s, mb_off);
    tx_code = flexcan_mb_code(cs);
    if (tx_code != MB_CODE_TX_DATA && tx_code != MB_CODE_TX_TANSWER) {
        return;
    }

    id = flexcan_mb_get_word(s, mb_off + 4);
    if (cs & MB_CS_IDE) {
        frame.can_id = (id & QEMU_CAN_EFF_MASK) | QEMU_CAN_EFF_FLAG;
    } else {
        frame.can_id = ((id >> 18) & QEMU_CAN_SFF_MASK);
    }
    if (cs & MB_CS_RTR) {
        frame.can_id |= QEMU_CAN_RTR_FLAG;
    }

    frame.can_dlc = flexcan_mb_dlc(cs);
    len = can_dlc2len(frame.can_dlc);
    for (i = 0; i < MIN(len, (uint32_t)flexcan_payload_size(s)); i++) {
        frame.data[i] = s->mb_ram[mb_off + 8 - FLEXCAN_MB_BASE + i];
    }

    if ((s->mcr & MCR_FDEN) && (cs & MB_CS_EDL)) {
        frame.flags |= QEMU_CAN_FRMF_TYPE_FD;
        if ((s->fdctrl & FDCTRL_FDRATE) && (cs & MB_CS_BRS)) {
            frame.flags |= QEMU_CAN_FRMF_BRS;
        }
        if (cs & MB_CS_ESI) {
            frame.flags |= QEMU_CAN_FRMF_ESI;
        }
    }

    if (loopback) {
        flexcan_receive_one(s, &frame);
    } else {
        can_bus_client_send(&s->bus_client, &frame, 1);
        if (!(s->mcr & MCR_SRXDIS)) {
            flexcan_receive_one(s, &frame);
        }
    }

    cs = flexcan_mb_set_timestamp(cs, flexcan_get_timestamp(s));
    if (tx_code == MB_CODE_TX_TANSWER) {
        cs = flexcan_mb_set_code(cs, MB_CODE_RX_RANSWER);
    } else if (cs & MB_CS_RTR) {
        cs = flexcan_mb_set_code(cs, MB_CODE_RX_EMPTY);
    } else {
        cs = flexcan_mb_set_code(cs, MB_CODE_TX_INACTIVE);
    }
    flexcan_mb_set_word(s, mb_off, cs);

    s->iflag1 |= BIT(mb);
    flexcan_update_irq(s);
}

static uint32_t flexcan_dynamic_esr1(FslFlexCANState *s)
{
    uint32_t esr1 = s->esr1 | ESR1_IDLE;

    if (s->ctrl1 & CTRL1_LOM) {
        esr1 &= ~ESR1_FLTCONF_MASK;
        esr1 |= 0x1u << ESR1_FLTCONF_SHIFT;
    }

    return esr1;
}

static bool flexcan_accepts(void *opaque, hwaddr offset, unsigned size,
                            bool is_write, MemTxAttrs attrs)
{
    FslFlexCANState *s = opaque;
    (void)attrs;

    if (size != 4 || (offset & 0x3)) {
        return false;
    }

    switch (offset) {
    case FLEXCAN_MCR:
    case FLEXCAN_CTRL1:
    case FLEXCAN_TIMER:
    case FLEXCAN_RXMGMASK:
    case FLEXCAN_RX14MASK:
    case FLEXCAN_RX15MASK:
    case FLEXCAN_ECR:
    case FLEXCAN_ESR1:
    case FLEXCAN_IMASK1:
    case FLEXCAN_IFLAG1:
    case FLEXCAN_CTRL2:
    case FLEXCAN_RXFGMASK:
    case FLEXCAN_CBT:
    case FLEXCAN_FDCTRL:
    case FLEXCAN_FDCBT:
        return true;
    case FLEXCAN_ESR2:
    case FLEXCAN_CRCR:
    case FLEXCAN_RXFIR:
    case FLEXCAN_FDCRC:
        return !is_write;
    case FLEXCAN_CTRL1_PN:
    case FLEXCAN_CTRL2_PN:
    case FLEXCAN_WU_MTC:
    case FLEXCAN_FLT_ID1:
    case FLEXCAN_FLT_DLC:
    case FLEXCAN_PL1_LO:
    case FLEXCAN_PL1_HI:
    case FLEXCAN_FLT_ID2_IDMASK:
    case FLEXCAN_PL2_PLMASK_LO:
    case FLEXCAN_PL2_PLMASK_HI:
        return s->pnet;
    default:
        break;
    }

    if (offset >= FLEXCAN_MB_BASE && (offset + size) <= flexcan_mb_region_end(s)) {
        return true;
    }

    if (offset >= FLEXCAN_RXIMR_BASE && offset < FLEXCAN_RXIMR_END) {
        return true;
    }

    if (s->pnet && offset >= FLEXCAN_WMB_BASE && offset < FLEXCAN_WMB_END) {
        return !is_write;
    }

    return false;
}

static uint64_t flexcan_read(void *opaque, hwaddr offset, unsigned size)
{
    FslFlexCANState *s = opaque;

    switch (offset) {
    case FLEXCAN_MCR:
        return s->mcr;
    case FLEXCAN_CTRL1:
        return s->ctrl1;
    case FLEXCAN_TIMER:
        flexcan_unlock_rx_mbs(s);
        return flexcan_get_timestamp(s);
    case FLEXCAN_RXMGMASK:
        return s->rxmgmask;
    case FLEXCAN_RX14MASK:
        return s->rx14mask;
    case FLEXCAN_RX15MASK:
        return s->rx15mask;
    case FLEXCAN_ECR:
        return s->ecr;
    case FLEXCAN_ESR1:
        return flexcan_dynamic_esr1(s);
    case FLEXCAN_IMASK1:
        return s->imask1;
    case FLEXCAN_IFLAG1:
        return s->iflag1;
    case FLEXCAN_CTRL2:
        return s->ctrl2;
    case FLEXCAN_ESR2:
        return s->esr2;
    case FLEXCAN_CRCR:
        return s->crcr;
    case FLEXCAN_RXFGMASK:
        return s->rxfgmask;
    case FLEXCAN_RXFIR:
        return s->rxfir;
    case FLEXCAN_CBT:
        return s->cbt;
    case FLEXCAN_CTRL1_PN:
        return s->ctrl1_pn;
    case FLEXCAN_CTRL2_PN:
        return s->ctrl2_pn;
    case FLEXCAN_WU_MTC:
        return s->wu_mtc;
    case FLEXCAN_FLT_ID1:
        return s->flt_id1;
    case FLEXCAN_FLT_DLC:
        return s->flt_dlc;
    case FLEXCAN_PL1_LO:
        return s->pl1_lo;
    case FLEXCAN_PL1_HI:
        return s->pl1_hi;
    case FLEXCAN_FLT_ID2_IDMASK:
        return s->flt_id2_idmask;
    case FLEXCAN_PL2_PLMASK_LO:
        return s->pl2_plmask_lo;
    case FLEXCAN_PL2_PLMASK_HI:
        return s->pl2_plmask_hi;
    case FLEXCAN_FDCTRL:
        return s->fdctrl;
    case FLEXCAN_FDCBT:
        return s->fdcbt;
    case FLEXCAN_FDCRC:
        return s->fdcrc;
    default:
        if (offset >= FLEXCAN_MB_BASE && offset < FLEXCAN_MB_END) {
            uint32_t value = flexcan_mb_get_word(s, offset);
            int mb;

            if (flexcan_fifo_enabled(s) && (s->mcr & MCR_DMA) && offset == 0x8c) {
                flexcan_fifo_pop(s);
            } else if (flexcan_mb_is_cs_word(s, offset, &mb) &&
                       mb >= flexcan_first_mailbox(s)) {
                flexcan_lock_rx_mb(s, mb, value);
            }

            return value;
        }
        if (offset >= FLEXCAN_RXIMR_BASE && offset < FLEXCAN_RXIMR_END) {
            return s->rximr[(offset - FLEXCAN_RXIMR_BASE) >> 2];
        }
        if (offset >= FLEXCAN_WMB_BASE && offset < FLEXCAN_WMB_END) {
            return s->wmb[(offset - FLEXCAN_WMB_BASE) >> 2];
        }
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented read @0x%" HWADDR_PRIx "\n",
                      TYPE_FSL_FLEXCAN, offset);
        return 0;
    }
}

static void flexcan_write_mcr(FslFlexCANState *s, uint32_t value)
{
    uint32_t old_mcr = s->mcr;
    bool was_enabled = flexcan_enabled(s);
    bool freeze_only = flexcan_frozen(s) || !was_enabled;
    uint32_t writable;

    writable = MCR_MDIS | MCR_FRZ | MCR_HALT | MCR_SOFTRST |
               MCR_WRNEN | MCR_SRXDIS | MCR_IRMQ | MCR_DMA |
               MCR_PNET_EN | MCR_LPRIOEN | MCR_AEN | MCR_FDEN |
               MCR_IDAM_MASK | MCR_MAXMB_MASK | MCR_RFEN;

    if (!freeze_only) {
        writable &= ~(MCR_RFEN | MCR_WRNEN | MCR_SRXDIS | MCR_IRMQ |
                      MCR_DMA | MCR_PNET_EN | MCR_LPRIOEN | MCR_AEN |
                      MCR_FDEN | MCR_IDAM_MASK | MCR_MAXMB_MASK);
    }

    s->mcr = (s->mcr & ~writable) | (value & writable);
    if ((s->mcr & MCR_FDEN) && (s->mcr & MCR_RFEN)) {
        s->mcr &= ~MCR_RFEN;
    }
    if (!!(old_mcr & MCR_RFEN) != !!(s->mcr & MCR_RFEN)) {
        s->iflag1 &= ~0xffu;
        s->rx_fifo_head = 0;
        s->rx_fifo_count = 0;
        s->rx_fifo_warning_armed = true;
        flexcan_fifo_sync_output(s);
    }

    if ((value & MCR_SOFTRST) && !(old_mcr & MCR_SOFTRST)) {
        flexcan_reset_regs(s, true);
        return;
    }

    if ((old_mcr & MCR_MDIS) && !(s->mcr & MCR_MDIS)) {
        s->mcr |= MCR_FRZ | MCR_HALT | MCR_FRZACK | MCR_NOTRDY;
    }
    flexcan_sync_mcr_state(s);
}

static void flexcan_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned size)
{
    FslFlexCANState *s = opaque;
    uint32_t v = value;

    switch (offset) {
    case FLEXCAN_MCR:
        flexcan_write_mcr(s, v);
        break;
    case FLEXCAN_CTRL1:
        if (flexcan_frozen(s)) {
            s->ctrl1 = v;
        }
        break;
    case FLEXCAN_RXMGMASK:
        if (flexcan_frozen(s)) {
            s->rxmgmask = v;
        }
        break;
    case FLEXCAN_RX14MASK:
        if (flexcan_frozen(s)) {
            s->rx14mask = v;
        }
        break;
    case FLEXCAN_RX15MASK:
        if (flexcan_frozen(s)) {
            s->rx15mask = v;
        }
        break;
    case FLEXCAN_ESR1:
        s->esr1 &= ~v;
        break;
    case FLEXCAN_IMASK1:
        s->imask1 = v;
        break;
    case FLEXCAN_IFLAG1:
        if ((s->mcr & MCR_RFEN) && (v & BIT(0)) && flexcan_frozen(s)) {
            flexcan_fifo_clear(s);
            if (s->mcr & MCR_DMA) {
                s->iflag1 &= ~BIT(5);
            }
            v &= ~BIT(0);
        }
        if ((s->mcr & MCR_RFEN) && (v & BIT(5)) && !(s->mcr & MCR_DMA)) {
            flexcan_fifo_pop(s);
            v &= ~BIT(5);
        }
        if ((s->mcr & MCR_RFEN) && (s->mcr & MCR_DMA)) {
            v &= ~(BIT(5) | BIT(6) | BIT(7));
        }
        s->iflag1 &= ~v;
        break;
    case FLEXCAN_CTRL2:
        if (flexcan_frozen(s)) {
            uint32_t writable = CTRL2_RFFN_MASK | CTRL2_TASD_MASK |
                                CTRL2_MRP | CTRL2_RRS | CTRL2_EACEN |
                                CTRL2_ISOCANFDEN;
            s->ctrl2 = (s->ctrl2 & ~writable) | (v & writable);
        }
        break;
    case FLEXCAN_RXFGMASK:
        if (flexcan_frozen(s)) {
            s->rxfgmask = v;
        }
        break;
    case FLEXCAN_CBT:
        if (flexcan_frozen(s)) {
            s->cbt = v;
        }
        break;
    case FLEXCAN_CTRL1_PN:
        s->ctrl1_pn = v;
        break;
    case FLEXCAN_CTRL2_PN:
        s->ctrl2_pn = v;
        break;
    case FLEXCAN_WU_MTC:
        s->wu_mtc &= ~v;
        break;
    case FLEXCAN_FLT_ID1:
        s->flt_id1 = v;
        break;
    case FLEXCAN_FLT_DLC:
        s->flt_dlc = v;
        break;
    case FLEXCAN_PL1_LO:
        s->pl1_lo = v;
        break;
    case FLEXCAN_PL1_HI:
        s->pl1_hi = v;
        break;
    case FLEXCAN_FLT_ID2_IDMASK:
        s->flt_id2_idmask = v;
        break;
    case FLEXCAN_PL2_PLMASK_LO:
        s->pl2_plmask_lo = v;
        break;
    case FLEXCAN_PL2_PLMASK_HI:
        s->pl2_plmask_hi = v;
        break;
    case FLEXCAN_FDCTRL:
        if (flexcan_frozen(s)) {
            uint32_t runtime = FDCTRL_FDRATE;
            uint32_t freeze_only = FDCTRL_MBDSR0_MASK | FDCTRL_TDCEN |
                                   FDCTRL_TDCOFF_MASK;
            s->fdctrl = (s->fdctrl & ~(runtime | freeze_only)) |
                        (v & (runtime | freeze_only));
        } else {
            s->fdctrl = (s->fdctrl & ~FDCTRL_FDRATE) | (v & FDCTRL_FDRATE);
        }
        if (v & FDCTRL_TDCFAIL) {
            s->fdctrl &= ~FDCTRL_TDCFAIL;
        }
        break;
    case FLEXCAN_FDCBT:
        if (flexcan_frozen(s)) {
            s->fdcbt = v;
        }
        break;
    default:
        if (offset >= FLEXCAN_MB_BASE && offset < FLEXCAN_MB_END) {
            int mb;

            if (flexcan_mb_is_cs_word(s, offset, &mb)) {
                flexcan_unlock_rx_mbs(s);
                s->rx_lock_mask &= ~BIT(mb);
                s->rx_serviced_mask &= ~BIT(mb);
            }
            flexcan_mb_set_word(s, offset, v);
            if (flexcan_mb_is_cs_word(s, offset, &mb) &&
                mb >= flexcan_first_mailbox(s)) {
                flexcan_do_tx(s, mb);
            }
        } else if (offset >= FLEXCAN_RXIMR_BASE && offset < FLEXCAN_RXIMR_END) {
            if (flexcan_frozen(s)) {
                s->rximr[(offset - FLEXCAN_RXIMR_BASE) >> 2] = v;
            }
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "%s: unimplemented write @0x%" HWADDR_PRIx
                          " = 0x%08x\n", TYPE_FSL_FLEXCAN, offset, v);
        }
        break;
    }

    flexcan_update_irq(s);
}

static const MemoryRegionOps flexcan_ops = {
    .read = flexcan_read,
    .write = flexcan_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .valid.accepts = flexcan_accepts,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static Property flexcan_properties[] = {
    DEFINE_PROP_BOOL("pnet", FslFlexCANState, pnet, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void flexcan_instance_init(Object *obj)
{
    FslFlexCANState *s = FSL_FLEXCAN(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &flexcan_ops, s,
                          TYPE_FSL_FLEXCAN, FLEXCAN_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    object_property_add_link(obj, "canbus", TYPE_CAN_BUS,
                             (Object **)&s->canbus,
                             qdev_prop_allow_set_link_before_realize,
                             0);
}

static void flexcan_realize(DeviceState *dev, Error **errp)
{
    FslFlexCANState *s = FSL_FLEXCAN(dev);

    s->bus_client.info = &flexcan_canbus_info;
    s->bus_client.model = g_strdup(TYPE_FSL_FLEXCAN);
    s->bus_client.fd_mode = true;

    if (s->canbus) {
        can_bus_insert_client(s->canbus, &s->bus_client);
    }
}

static void flexcan_unrealize(DeviceState *dev)
{
    FslFlexCANState *s = FSL_FLEXCAN(dev);

    can_bus_remove_client(&s->bus_client);
    g_free(s->bus_client.model);
    s->bus_client.model = NULL;
}

static void flexcan_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = flexcan_realize;
    dc->unrealize = flexcan_unrealize;
    dc->reset = flexcan_reset;
    dc->desc = "NXP FlexCAN controller (S32K1xx-focused)";
    device_class_set_props(dc, flexcan_properties);
}

static const TypeInfo flexcan_type_info = {
    .name = TYPE_FSL_FLEXCAN,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FslFlexCANState),
    .instance_init = flexcan_instance_init,
    .class_init = flexcan_class_init,
};

static void flexcan_register_types(void)
{
    type_register_static(&flexcan_type_info);
}

type_init(flexcan_register_types)
