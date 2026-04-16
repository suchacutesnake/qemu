#ifndef DEVICE_REGISTERS_H
#define DEVICE_REGISTERS_H

#include <stddef.h>
#include <stdint.h>

#include "/root/myqemu/S32K148_Project_FlexCan/include/S32K148.h"
#include "/root/myqemu/S32K148_Project_FlexCan/include/S32K148_features.h"
#include "/root/myqemu/daredevil-small/refs/platform/devices/devassert.h"

#define CORE_LITTLE_ENDIAN

#ifndef REV_BYTES_32
#define REV_BYTES_32(src, dst) ((dst) = __builtin_bswap32((uint32_t)(src)))
#endif

#ifndef INT_VECTOR_Reg
#define INT_VECTOR_Reg (S32_SCB->VTOR)
#endif

#ifdef FEATURE_CAN_HAS_DMA_ENABLE
#undef FEATURE_CAN_HAS_DMA_ENABLE
#endif
#define FEATURE_CAN_HAS_DMA_ENABLE (0)

#endif
