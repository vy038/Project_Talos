/**
 * @file freertos_sim.c
 * @brief pthread-backed FreeRTOS primitive implementations for Lightsim
 *
 * Implements queues, semaphores, task creation, and task notifications using
 * pthreads so the real firmware tasks run concurrently, matching the actual
 * RTOS behaviour as closely as a Linux process allows.
 *
 * stdout = JSON (never written here)
 * stderr = log output via fprintf
 */

#include "esp_idf_shims.h"
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

/* ========================================================================== */
/*  Thread-local current task pointer                                         */
/* ========================================================================== */

static pthread_key_t  sim_task_key;
static pthread_once_t sim_task_key_once = PTHREAD_ONCE_INIT;

static void make_task_key(void) {
    pthread_key_create(&sim_task_key, NULL);
}

static sim_task_t *current_task(void) {
    pthread_once(&sim_task_key_once, make_task_key);
    return (sim_task_t *)pthread_getspecific(sim_task_key);
}

/* ========================================================================== */
/*  Task creation / control                                                   */
/* ========================================================================== */

typedef struct {
    TaskFunction_t  func;
    void           *params;
    sim_task_t     *task;
} task_wrapper_args_t;

static void *task_wrapper(void *arg) {
    task_wrapper_args_t *w = (task_wrapper_args_t *)arg;
    pthread_once(&sim_task_key_once, make_task_key);
    pthread_setspecific(sim_task_key, w->task);
    TaskFunction_t func   = w->func;
    void          *params = w->params;
    free(w);
    func(params);
    return NULL;
}

/* Registry of tasks by name, so sim-only code (main_sim.c) can suspend a
 * task without the firmware needing to hand out a TaskHandle_t. */
#define SIM_TASK_REGISTRY_MAX 16
static struct { char name[32]; sim_task_t *task; } sim_task_registry[SIM_TASK_REGISTRY_MAX];
static int sim_task_registry_count = 0;

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t func, const char *name,
    uint32_t stack, void *params, UBaseType_t prio,
    TaskHandle_t *handle_out, BaseType_t core)
{
    (void)stack; (void)prio; (void)core;

    sim_task_t *t = (sim_task_t *)calloc(1, sizeof(sim_task_t));
    pthread_mutex_init(&t->notify_mutex,  NULL);
    pthread_cond_init (&t->notify_cond,   NULL);
    pthread_mutex_init(&t->suspend_mutex, NULL);
    pthread_cond_init (&t->suspend_cond,  NULL);

    task_wrapper_args_t *w = (task_wrapper_args_t *)malloc(sizeof(task_wrapper_args_t));
    w->func   = func;
    w->params = params;
    w->task   = t;

    if (handle_out) *handle_out = t;

    if (name && sim_task_registry_count < SIM_TASK_REGISTRY_MAX) {
        strncpy(sim_task_registry[sim_task_registry_count].name, name,
                sizeof(sim_task_registry[0].name) - 1);
        sim_task_registry[sim_task_registry_count].task = t;
        sim_task_registry_count++;
    }

    pthread_create(&t->thread, NULL, task_wrapper, w);
    return pdPASS;
}

/* Permanently suspend a task by the name it was created with (sim-only,
 * used to stop the autonomous state-machine task once teleop takes over). */
void sim_suspend_task_by_name(const char *name) {
    for (int i = 0; i < sim_task_registry_count; i++) {
        if (strcmp(sim_task_registry[i].name, name) == 0) {
            vTaskSuspend(sim_task_registry[i].task);
            return;
        }
    }
}

void sim_check_suspend(void) {
    sim_task_t *t = current_task();
    if (!t) return;
    pthread_mutex_lock(&t->suspend_mutex);
    while (t->suspended) {
        pthread_cond_wait(&t->suspend_cond, &t->suspend_mutex);
    }
    pthread_mutex_unlock(&t->suspend_mutex);
}

void vTaskSuspend(TaskHandle_t handle) {
    sim_task_t *t = (sim_task_t *)handle;
    if (!t) return;
    pthread_mutex_lock(&t->suspend_mutex);
    t->suspended = 1;
    pthread_mutex_unlock(&t->suspend_mutex);
    /* If this task is suspending itself, block here */
    if (t == current_task()) {
        pthread_mutex_lock(&t->suspend_mutex);
        while (t->suspended) {
            pthread_cond_wait(&t->suspend_cond, &t->suspend_mutex);
        }
        pthread_mutex_unlock(&t->suspend_mutex);
    }
}

void vTaskResume(TaskHandle_t handle) {
    sim_task_t *t = (sim_task_t *)handle;
    if (!t) return;
    pthread_mutex_lock(&t->suspend_mutex);
    t->suspended = 0;
    pthread_cond_signal(&t->suspend_cond);
    pthread_mutex_unlock(&t->suspend_mutex);
}

/* ========================================================================== */
/*  Task notifications                                                        */
/* ========================================================================== */

void xTaskNotifyGive(TaskHandle_t handle) {
    sim_task_t *t = (sim_task_t *)handle;
    if (!t) return;
    pthread_mutex_lock(&t->notify_mutex);
    t->notify_value++;
    pthread_cond_signal(&t->notify_cond);
    pthread_mutex_unlock(&t->notify_mutex);
}

uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout) {
    sim_task_t *t = current_task();
    if (!t) return 0;

    pthread_mutex_lock(&t->notify_mutex);

    if (timeout == portMAX_DELAY) {
        while (t->notify_value == 0) {
            pthread_cond_wait(&t->notify_cond, &t->notify_mutex);
        }
    } else if (timeout > 0) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += timeout / 1000;
        ts.tv_nsec += (long)(timeout % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        while (t->notify_value == 0) {
            if (pthread_cond_timedwait(&t->notify_cond, &t->notify_mutex, &ts) == ETIMEDOUT)
                break;
        }
    }

    uint32_t val = t->notify_value;
    if (clear_on_exit) t->notify_value = 0;
    else if (val > 0)  t->notify_value--;
    pthread_mutex_unlock(&t->notify_mutex);
    return val;
}

/* ========================================================================== */
/*  Queues                                                                    */
/* ========================================================================== */

QueueHandle_t xQueueCreate(UBaseType_t depth, UBaseType_t item_size) {
    sim_queue_t *q = (sim_queue_t *)calloc(1, sizeof(sim_queue_t));
    q->buf       = (uint8_t *)calloc(depth, item_size);
    q->item_size = item_size;
    q->depth     = depth;
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init (&q->cond,  NULL);
    return q;
}

BaseType_t xQueueSend(QueueHandle_t xq, const void *item, TickType_t ticks) {
    (void)ticks;
    sim_queue_t *q = (sim_queue_t *)xq;
    pthread_mutex_lock(&q->mutex);
    if (q->count >= q->depth) {
        pthread_mutex_unlock(&q->mutex);
        return pdFALSE;
    }
    memcpy(q->buf + q->head * q->item_size, item, q->item_size);
    q->head = (q->head + 1) % q->depth;
    q->count++;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t xq, void *buf, TickType_t ticks) {
    sim_queue_t *q = (sim_queue_t *)xq;
    pthread_mutex_lock(&q->mutex);

    if (ticks == 0) {
        if (q->count == 0) { pthread_mutex_unlock(&q->mutex); return pdFALSE; }
    } else if (ticks == portMAX_DELAY) {
        while (q->count == 0) pthread_cond_wait(&q->cond, &q->mutex);
    } else {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += ticks / 1000;
        ts.tv_nsec += (long)(ticks % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        while (q->count == 0) {
            if (pthread_cond_timedwait(&q->cond, &q->mutex, &ts) == ETIMEDOUT) {
                pthread_mutex_unlock(&q->mutex);
                return pdFALSE;
            }
        }
    }

    memcpy(buf, q->buf + q->tail * q->item_size, q->item_size);
    q->tail = (q->tail + 1) % q->depth;
    q->count--;
    pthread_mutex_unlock(&q->mutex);
    return pdTRUE;
}

BaseType_t xQueueOverwrite(QueueHandle_t xq, const void *item) {
    sim_queue_t *q = (sim_queue_t *)xq;
    pthread_mutex_lock(&q->mutex);
    /* Always write, reset to single-item state */
    q->head = 0; q->tail = 0; q->count = 0;
    memcpy(q->buf, item, q->item_size);
    q->head  = 1;
    q->count = 1;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
    return pdTRUE;
}

/* ========================================================================== */
/*  Semaphores                                                                */
/* ========================================================================== */

static SemaphoreHandle_t make_sem(int initial, int max) {
    sim_sem_t *s = (sim_sem_t *)calloc(1, sizeof(sim_sem_t));
    pthread_mutex_init(&s->mutex, NULL);
    pthread_cond_init (&s->cond,  NULL);
    s->count = initial;
    s->max   = max;
    return s;
}

SemaphoreHandle_t xSemaphoreCreateBinary(void) { return make_sem(0, 1); }
SemaphoreHandle_t xSemaphoreCreateMutex(void)  { return make_sem(1, 1); }

BaseType_t xSemaphoreTake(SemaphoreHandle_t xs, TickType_t ticks) {
    sim_sem_t *s = (sim_sem_t *)xs;
    pthread_mutex_lock(&s->mutex);

    if (ticks == 0) {
        if (s->count == 0) { pthread_mutex_unlock(&s->mutex); return pdFALSE; }
    } else if (ticks == portMAX_DELAY) {
        while (s->count == 0) pthread_cond_wait(&s->cond, &s->mutex);
    } else {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += ticks / 1000;
        ts.tv_nsec += (long)(ticks % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        while (s->count == 0) {
            if (pthread_cond_timedwait(&s->cond, &s->mutex, &ts) == ETIMEDOUT) {
                pthread_mutex_unlock(&s->mutex);
                return pdFALSE;
            }
        }
    }

    s->count--;
    pthread_mutex_unlock(&s->mutex);
    return pdTRUE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t xs) {
    sim_sem_t *s = (sim_sem_t *)xs;
    pthread_mutex_lock(&s->mutex);
    if (s->count < s->max) {
        s->count++;
        pthread_cond_signal(&s->cond);
    }
    pthread_mutex_unlock(&s->mutex);
    return pdTRUE;
}
