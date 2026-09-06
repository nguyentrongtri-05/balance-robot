#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f4xx.h"

/* =========================================================================
 * C?u hình l?p l?ch và nhân FreeRTOS
 * ========================================================================= */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      (168000000UL) /* 168MHz trên STM32F407 */
#define configTICK_RATE_HZ                      ((TickType_t)1000) /* 1 tick = 1ms */
#define configMAX_PRIORITIES                    (5)
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                   ((size_t)(32 * 1024)) /* 32 KB Heap */
#define configMAX_TASK_NAME_LEN                 (16)
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          0

/* Ð? r?ng bi?n d?m Tick cho FreeRTOS V11 (Dùng 32-bit, không d?nh nghia 16-bit) */
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS

/* Hook Callbacks */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* C?p phát b? nh? */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/* Các tính nang nâng cao không s? d?ng */
#define configUSE_TIMERS                        0
#define configUSE_CO_ROUTINES                   0
#define configNUMBER_OF_CORES                   1

/* =========================================================================
 * C?u hình m?c uu tiên ng?t cho ARM Cortex-M4 (NVIC 4 bits)
 * ========================================================================= */
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* =========================================================================
 * Khai báo các hàm API du?c phép s? d?ng
 * ========================================================================= */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1

/* =========================================================================
 * Ánh x? tên ng?t FreeRTOS vào vector b?ng ng?t STM32
 * ========================================================================= */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

#endif /* FREERTOS_CONFIG_H */