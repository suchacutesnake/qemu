# S32K FlexCAN daredevil guest

This directory keeps the bare-metal guest used to validate that the local
FlexCAN model can boot the `daredevil-small` FlexCAN driver through:

- `FLEXCAN_DRV_Init`
- bit-timing setup from `FLEXCAN_DRV_GetDefaultConfig`
- simple RX/TX mailbox configuration
- blocking classic CAN send/receive
- blocking CAN FD send/receive

It is not wired into the default QEMU build. The guest is intended for manual
reproduction in a local environment that already has:

- the NXP S32K148 startup/linker files
- the `daredevil-small/refs/platform/drivers/src/flexcan` driver sources
- a SocketCAN interface such as `vcan0`

The committed sources mirror the local validation overlay used during bring-up:

- `device_registers.h`: local override for endianness and DMA-disable macros
- `osif.h` / `osif_stub.c`: minimal semaphore API used by blocking driver calls
- `interrupt_stub.c`: minimal NVIC helpers for the driver IRQ path
- `main.c`: the actual guest scenario and semihosting progress trace
