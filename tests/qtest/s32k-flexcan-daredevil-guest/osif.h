#ifndef OSIF_H
#define OSIF_H

#include <stdint.h>
#include "status.h"

typedef struct {
    volatile int32_t value;
} semaphore_t;

status_t OSIF_SemaCreate(volatile semaphore_t *sem, uint32_t initValue);
status_t OSIF_SemaDestroy(volatile semaphore_t *sem);
status_t OSIF_SemaWait(volatile semaphore_t *sem, uint32_t timeout_ms);
status_t OSIF_SemaPost(volatile semaphore_t *sem);

#endif
