#include <stddef.h>

#include "interrupt_manager.h"

void INT_SYS_InstallHandler(IRQn_Type irqNumber, const isr_t newHandler, isr_t * const oldHandler)
{
    isr_t *vectors = (isr_t *)(uintptr_t)S32_SCB->VTOR;
    uint32_t index = (uint32_t)irqNumber + 16U;

    if ((oldHandler != NULL) && (vectors != NULL)) {
        *oldHandler = vectors[index];
    }
    if (vectors != NULL) {
        vectors[index] = newHandler;
    }
}

void INT_SYS_EnableIRQ(IRQn_Type irqNumber)
{
    uint32_t irqn = (uint32_t)irqNumber;
    S32_NVIC->ISER[irqn >> 5U] = (1UL << (irqn & 31U));
}

void INT_SYS_DisableIRQ(IRQn_Type irqNumber)
{
    uint32_t irqn = (uint32_t)irqNumber;
    S32_NVIC->ICER[irqn >> 5U] = (1UL << (irqn & 31U));
}

void INT_SYS_EnableIRQGlobal(void)
{
    asm volatile("cpsie i" : : : "memory");
}

void INT_SYS_DisableIRQGlobal(void)
{
    asm volatile("cpsid i" : : : "memory");
}

void INT_SYS_SetPriority(IRQn_Type irqNumber, uint8_t priority)
{
    S32_NVIC->IP[(uint32_t)irqNumber] = priority;
}

uint8_t INT_SYS_GetPriority(IRQn_Type irqNumber)
{
    return S32_NVIC->IP[(uint32_t)irqNumber];
}

void INT_SYS_ClearPending(IRQn_Type irqNumber)
{
    uint32_t irqn = (uint32_t)irqNumber;
    S32_NVIC->ICPR[irqn >> 5U] = (1UL << (irqn & 31U));
}

void INT_SYS_SetPending(IRQn_Type irqNumber)
{
    uint32_t irqn = (uint32_t)irqNumber;
    S32_NVIC->ISPR[irqn >> 5U] = (1UL << (irqn & 31U));
}

uint32_t INT_SYS_GetPending(IRQn_Type irqNumber)
{
    uint32_t irqn = (uint32_t)irqNumber;
    return (S32_NVIC->ISPR[irqn >> 5U] >> (irqn & 31U)) & 1UL;
}

uint32_t INT_SYS_GetActive(IRQn_Type irqNumber)
{
    uint32_t irqn = (uint32_t)irqNumber;
    return (S32_NVIC->IABR[irqn >> 5U] >> (irqn & 31U)) & 1UL;
}
