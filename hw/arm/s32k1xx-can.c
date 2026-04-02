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

#define S32K1XX_FLASH_SIZE           0x80000
#define S32K1XX_SRAM_SIZE            0x10000
#define S32K1XX_FLASH_BASE           0x00000000
#define S32K1XX_SRAM_BASE            0x1fff0000
#define S32K1XX_SRAM_ALIAS_BASE      0x20000000
#define S32K1XX_FLEXCAN0_BASE        0x40024000
#define S32K1XX_FLEXCAN0_IRQ         81
#define S32K1XX_NUM_IRQ              128
#define S32K1XX_SYSCLK_HZ            80000000ULL

struct S32K1xxCanMachineState {
    MachineState parent;

    ARMv7MState armv7m;
    MemoryRegion flash;
    MemoryRegion sram;
    MemoryRegion sram_alias;
    CanBusState *canbus0;
};

static void s32k1xx_can_init(MachineState *machine)
{
    S32K1xxCanMachineState *s = S32K1XX_CAN_MACHINE(machine);
    DeviceState *armv7m;
    DeviceState *flexcan;
    SysBusDevice *sbd;
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

    memory_region_init_alias(&s->sram_alias, OBJECT(machine),
                             "s32k1xx.sram.alias", &s->sram, 0,
                             S32K1XX_SRAM_SIZE);
    memory_region_add_subregion(system_memory, S32K1XX_SRAM_ALIAS_BASE,
                                &s->sram_alias);

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

    flexcan = qdev_new("fsl.flexcan");
    if (s->canbus0) {
        object_property_set_link(OBJECT(flexcan), "canbus",
                                 OBJECT(s->canbus0), &error_abort);
    }
    sbd = SYS_BUS_DEVICE(flexcan);
    sysbus_realize(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, S32K1XX_FLEXCAN0_BASE);
    sysbus_connect_irq(sbd, 0,
                       qdev_get_gpio_in(DEVICE(&s->armv7m),
                                        S32K1XX_FLEXCAN0_IRQ));

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
