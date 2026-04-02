/*
 * Minimal S32K1xx-style Cortex-M4 board with one FlexCAN instance.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "exec/address-spaces.h"
#include "hw/boards.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "net/can_emu.h"

#define TYPE_S32K1XX_CAN_MACHINE MACHINE_TYPE_NAME("s32k1xx-can")
OBJECT_DECLARE_SIMPLE_TYPE(S32K1xxCanMachineState, S32K1XX_CAN_MACHINE)

#define S32K1XX_FLASH_SIZE           0x180000
#define S32K1XX_SRAM_SIZE            0x3f000
#define S32K1XX_FLASH_BASE           0x00000000
#define S32K1XX_SRAM_BASE            0x1ffe0000
#define S32K1XX_FLEXCAN0_BASE        0x40024000
#define S32K1XX_FLEXCAN1_BASE        0x40025000
#define S32K1XX_FLEXCAN2_BASE        0x4002b000
#define S32K1XX_WDOG_BASE            0x40052000
#define S32K1XX_SCG_BASE             0x40064000
#define S32K1XX_PCC_BASE             0x40065000
#define S32K1XX_PORTE_BASE           0x4004d000
#define S32K1XX_PTE_BASE             0x400ff100
#define S32K1XX_FLEXCAN0_IRQ         81
#define S32K1XX_FLEXCAN1_IRQ         88
#define S32K1XX_FLEXCAN2_IRQ         95
#define S32K1XX_NUM_IRQ              128
#define S32K1XX_SYSCLK_HZ            80000000ULL
#define S32K1XX_PCC_REG_COUNT        122

#define S32K1XX_WDOG_CS              0x00
#define S32K1XX_WDOG_CNT             0x04
#define S32K1XX_WDOG_TOVAL           0x08
#define S32K1XX_WDOG_WIN             0x0c

#define S32K1XX_SCG_CSR              0x10
#define S32K1XX_SCG_RCCR             0x14
#define S32K1XX_SCG_SOSCCSR          0x100
#define S32K1XX_SCG_SOSCDIV          0x104
#define S32K1XX_SCG_SOSCCFG          0x108
#define S32K1XX_SCG_SIRCDIV          0x204
#define S32K1XX_SCG_SPLLCSR          0x600
#define S32K1XX_SCG_SPLLDIV          0x604
#define S32K1XX_SCG_SPLLCFG          0x608
#define S32K1XX_SCG_SCS_MASK         0x0f000000u
#define S32K1XX_SCG_SOSCVLD_MASK     0x01000000u
#define S32K1XX_SCG_SPLLVLD_MASK     0x01000000u
#define S32K1XX_SCG_REGS             (0x700 / sizeof(uint32_t))

#define S32K1XX_GPIO_PDOR            0x00
#define S32K1XX_GPIO_PSOR            0x04
#define S32K1XX_GPIO_PCOR            0x08
#define S32K1XX_GPIO_PTOR            0x0c
#define S32K1XX_GPIO_PDIR            0x10
#define S32K1XX_GPIO_PDDR            0x14
#define S32K1XX_GPIO_PIDR            0x18

struct S32K1xxCanMachineState {
    MachineState parent;

    ARMv7MState armv7m;
    MemoryRegion flash;
    MemoryRegion sram;
    MemoryRegion wdog;
    MemoryRegion scg;
    MemoryRegion pcc;
    MemoryRegion porte;
    MemoryRegion pte;
    CanBusState *canbus0;
    CanBusState *canbus1;
    CanBusState *canbus2;
    uint32_t wdog_cs;
    uint32_t wdog_cnt;
    uint32_t wdog_toval;
    uint32_t wdog_win;
    uint32_t scg_regs[S32K1XX_SCG_REGS];
    uint32_t pcc_regs[S32K1XX_PCC_REG_COUNT];
    uint32_t porte_pcr[32];
    uint32_t pte_pdor;
    uint32_t pte_pdir;
    uint32_t pte_pddr;
};

static void s32k1xx_realize_flexcan(S32K1xxCanMachineState *s,
                                    DeviceState *armv7m,
                                    CanBusState *canbus,
                                    hwaddr base,
                                    int irq,
                                    bool pnet)
{
    DeviceState *flexcan = qdev_new("fsl.flexcan");
    SysBusDevice *sbd;

    qdev_prop_set_bit(flexcan, "pnet", pnet);
    if (canbus) {
        object_property_set_link(OBJECT(flexcan), "canbus",
                                 OBJECT(canbus), &error_abort);
    }

    sbd = SYS_BUS_DEVICE(flexcan);
    sysbus_realize(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, base);
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(armv7m, irq));
}

static uint64_t s32k1xx_wdog_read(void *opaque, hwaddr offset, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;

    if (size != 4) {
        return 0;
    }

    switch (offset) {
    case S32K1XX_WDOG_CS:
        return s->wdog_cs;
    case S32K1XX_WDOG_CNT:
        return s->wdog_cnt;
    case S32K1XX_WDOG_TOVAL:
        return s->wdog_toval;
    case S32K1XX_WDOG_WIN:
        return s->wdog_win;
    default:
        return 0;
    }
}

static void s32k1xx_wdog_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;

    if (size != 4) {
        return;
    }

    switch (offset) {
    case S32K1XX_WDOG_CS:
        s->wdog_cs = value;
        break;
    case S32K1XX_WDOG_CNT:
        s->wdog_cnt = value;
        break;
    case S32K1XX_WDOG_TOVAL:
        s->wdog_toval = value;
        break;
    case S32K1XX_WDOG_WIN:
        s->wdog_win = value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps s32k1xx_wdog_ops = {
    .read = s32k1xx_wdog_read,
    .write = s32k1xx_wdog_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t s32k1xx_scg_read(void *opaque, hwaddr offset, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;
    unsigned index = offset >> 2;

    if (size != 4 || index >= S32K1XX_SCG_REGS) {
        return 0;
    }

    switch (offset) {
    case S32K1XX_SCG_SOSCCSR:
        return s->scg_regs[index] | S32K1XX_SCG_SOSCVLD_MASK;
    case S32K1XX_SCG_SPLLCSR:
        return s->scg_regs[index] | S32K1XX_SCG_SPLLVLD_MASK;
    default:
        return s->scg_regs[index];
    }
}

static void s32k1xx_scg_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;
    unsigned index = offset >> 2;

    if (size != 4 || index >= S32K1XX_SCG_REGS) {
        return;
    }

    switch (offset) {
    case S32K1XX_SCG_SOSCCSR:
        s->scg_regs[index] = value | S32K1XX_SCG_SOSCVLD_MASK;
        break;
    case S32K1XX_SCG_SPLLCSR:
        s->scg_regs[index] = value | S32K1XX_SCG_SPLLVLD_MASK;
        break;
    case S32K1XX_SCG_RCCR:
        s->scg_regs[index] = value;
        s->scg_regs[S32K1XX_SCG_CSR >> 2] =
            (s->scg_regs[S32K1XX_SCG_CSR >> 2] & ~S32K1XX_SCG_SCS_MASK) |
            (value & S32K1XX_SCG_SCS_MASK);
        break;
    default:
        s->scg_regs[index] = value;
        break;
    }
}

static const MemoryRegionOps s32k1xx_scg_ops = {
    .read = s32k1xx_scg_read,
    .write = s32k1xx_scg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t s32k1xx_pcc_read(void *opaque, hwaddr offset, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;
    unsigned index = offset >> 2;

    if (size != 4 || index >= S32K1XX_PCC_REG_COUNT) {
        return 0;
    }

    return s->pcc_regs[index];
}

static void s32k1xx_pcc_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;
    unsigned index = offset >> 2;

    if (size != 4 || index >= S32K1XX_PCC_REG_COUNT) {
        return;
    }

    s->pcc_regs[index] = value;
}

static const MemoryRegionOps s32k1xx_pcc_ops = {
    .read = s32k1xx_pcc_read,
    .write = s32k1xx_pcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t s32k1xx_porte_read(void *opaque, hwaddr offset, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;
    unsigned index = offset >> 2;

    if (size != 4 || index >= ARRAY_SIZE(s->porte_pcr)) {
        return 0;
    }

    return s->porte_pcr[index];
}

static void s32k1xx_porte_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;
    unsigned index = offset >> 2;

    if (size != 4 || index >= ARRAY_SIZE(s->porte_pcr)) {
        return;
    }

    s->porte_pcr[index] = value;
}

static const MemoryRegionOps s32k1xx_porte_ops = {
    .read = s32k1xx_porte_read,
    .write = s32k1xx_porte_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t s32k1xx_pte_read(void *opaque, hwaddr offset, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;

    if (size != 4) {
        return 0;
    }

    switch (offset) {
    case S32K1XX_GPIO_PDOR:
        return s->pte_pdor;
    case S32K1XX_GPIO_PDIR:
        return s->pte_pdir;
    case S32K1XX_GPIO_PDDR:
        return s->pte_pddr;
    case S32K1XX_GPIO_PIDR:
        return 0;
    default:
        return 0;
    }
}

static void s32k1xx_pte_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    S32K1xxCanMachineState *s = opaque;

    if (size != 4) {
        return;
    }

    switch (offset) {
    case S32K1XX_GPIO_PDOR:
        s->pte_pdor = value;
        break;
    case S32K1XX_GPIO_PSOR:
        s->pte_pdor |= value;
        break;
    case S32K1XX_GPIO_PCOR:
        s->pte_pdor &= ~value;
        break;
    case S32K1XX_GPIO_PTOR:
        s->pte_pdor ^= value;
        break;
    case S32K1XX_GPIO_PDDR:
        s->pte_pddr = value;
        break;
    default:
        break;
    }

    s->pte_pdir = s->pte_pdor;
}

static const MemoryRegionOps s32k1xx_pte_ops = {
    .read = s32k1xx_pte_read,
    .write = s32k1xx_pte_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void s32k1xx_can_init(MachineState *machine)
{
    S32K1xxCanMachineState *s = S32K1XX_CAN_MACHINE(machine);
    DeviceState *armv7m;
    Clock *sysclk;
    MemoryRegion *system_memory = get_system_memory();

    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, S32K1XX_SYSCLK_HZ);

    memory_region_init_rom(&s->flash, NULL, "s32k1xx.flash",
                           S32K1XX_FLASH_SIZE, &error_fatal);
    memory_region_add_subregion(system_memory, S32K1XX_FLASH_BASE, &s->flash);

    memory_region_init_ram(&s->sram, NULL, "s32k1xx.sram",
                           S32K1XX_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(system_memory, S32K1XX_SRAM_BASE, &s->sram);

    memory_region_init_io(&s->wdog, NULL, &s32k1xx_wdog_ops, s,
                          "s32k1xx.wdog", 0x1000);
    memory_region_add_subregion(system_memory, S32K1XX_WDOG_BASE, &s->wdog);

    memory_region_init_io(&s->scg, NULL, &s32k1xx_scg_ops, s,
                          "s32k1xx.scg", 0x1000);
    memory_region_add_subregion(system_memory, S32K1XX_SCG_BASE, &s->scg);

    memory_region_init_io(&s->pcc, NULL, &s32k1xx_pcc_ops, s,
                          "s32k1xx.pcc", 0x1000);
    memory_region_add_subregion(system_memory, S32K1XX_PCC_BASE, &s->pcc);

    memory_region_init_io(&s->porte, NULL, &s32k1xx_porte_ops, s,
                          "s32k1xx.porte", 0x1000);
    memory_region_add_subregion(system_memory, S32K1XX_PORTE_BASE, &s->porte);

    memory_region_init_io(&s->pte, NULL, &s32k1xx_pte_ops, s,
                          "s32k1xx.pte", 0x1000);
    memory_region_add_subregion(system_memory, S32K1XX_PTE_BASE, &s->pte);

    s->scg_regs[S32K1XX_SCG_CSR >> 2] = 0x02000000;
    s->scg_regs[S32K1XX_SCG_SOSCCSR >> 2] = S32K1XX_SCG_SOSCVLD_MASK;
    s->scg_regs[S32K1XX_SCG_SPLLCSR >> 2] = S32K1XX_SCG_SPLLVLD_MASK;

    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", S32K1XX_NUM_IRQ);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    qdev_prop_set_bit(armv7m, "vfp", true);
    qdev_prop_set_bit(armv7m, "dsp", true);
    qdev_connect_clock_in(armv7m, "cpuclk", sysclk);
    qdev_connect_clock_in(armv7m, "refclk", sysclk);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), &error_fatal);

    s32k1xx_realize_flexcan(s, armv7m, s->canbus0, S32K1XX_FLEXCAN0_BASE,
                            S32K1XX_FLEXCAN0_IRQ, true);
    s32k1xx_realize_flexcan(s, armv7m, s->canbus1, S32K1XX_FLEXCAN1_BASE,
                            S32K1XX_FLEXCAN1_IRQ, false);
    s32k1xx_realize_flexcan(s, armv7m, s->canbus2, S32K1XX_FLEXCAN2_BASE,
                            S32K1XX_FLEXCAN2_IRQ, false);

    if (machine->kernel_filename) {
        armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename,
                           0, S32K1XX_FLASH_SIZE);
    }
}

static void s32k1xx_can_machine_instance_init(Object *obj)
{
    S32K1xxCanMachineState *s = S32K1XX_CAN_MACHINE(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    object_property_add_link(obj, "canbus0", TYPE_CAN_BUS,
                             (Object **)&s->canbus0,
                             object_property_allow_set_link,
                             0);
    object_property_add_link(obj, "canbus1", TYPE_CAN_BUS,
                             (Object **)&s->canbus1,
                             object_property_allow_set_link,
                             0);
    object_property_add_link(obj, "canbus2", TYPE_CAN_BUS,
                             (Object **)&s->canbus2,
                             object_property_allow_set_link,
                             0);
}

static void s32k1xx_can_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };

    mc->desc = "Minimal S32K1xx-style CAN/CAN FD board (Cortex-M4 + FlexCAN)";
    mc->init = s32k1xx_can_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_id = "sram";
    mc->ignore_memory_transaction_failures = true;
}

static const TypeInfo s32k1xx_can_machine_typeinfo = {
    .name = TYPE_S32K1XX_CAN_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(S32K1xxCanMachineState),
    .instance_init = s32k1xx_can_machine_instance_init,
    .class_init = s32k1xx_can_machine_class_init,
};

static void s32k1xx_can_machine_init_register_types(void)
{
    type_register_static(&s32k1xx_can_machine_typeinfo);
}

type_init(s32k1xx_can_machine_init_register_types)
