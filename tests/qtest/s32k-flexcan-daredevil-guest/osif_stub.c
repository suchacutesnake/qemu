#include "device_registers.h"
#include "osif.h"

static inline void irq_disable(void)
{
    asm volatile("cpsid i" : : : "memory");
}

static inline void irq_enable(void)
{
    asm volatile("cpsie i" : : : "memory");
}

status_t OSIF_SemaCreate(volatile semaphore_t *sem, uint32_t initValue)
{
    sem->value = (int32_t)initValue;
    return STATUS_SUCCESS;
}

status_t OSIF_SemaDestroy(volatile semaphore_t *sem)
{
    sem->value = 0;
    return STATUS_SUCCESS;
}

status_t OSIF_SemaWait(volatile semaphore_t *sem, uint32_t timeout_ms)
{
    volatile uint32_t spins = (timeout_ms + 1U) * 20000U;

    while (spins-- > 0U) {
        irq_disable();
        if (sem->value > 0) {
            sem->value--;
            irq_enable();
            return STATUS_SUCCESS;
        }
        irq_enable();
        asm volatile("nop");
    }

    return STATUS_TIMEOUT;
}

status_t OSIF_SemaPost(volatile semaphore_t *sem)
{
    irq_disable();
    sem->value++;
    irq_enable();
    return STATUS_SUCCESS;
}
