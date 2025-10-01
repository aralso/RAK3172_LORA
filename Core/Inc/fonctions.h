/*
 * fonctions.h
 *
 *  Created on: Oct 1, 2025
 *      Author: Tocqueville
 */

#ifndef INC_FONCTIONS_H_
#define INC_FONCTIONS_H_

#include "main.h"
#include <stdio.h>
#include <stdarg.h>

#define END_NODE
#define My_Address 'U'

#define WATCHDOG

// Structure pour les événements
typedef struct {
    uint8_t type;           // Type d'événement
    uint8_t source;         // Source de l'événement
    uint16_t data;          // Données de l'événement
} event_t;


typedef enum  {
    EVENT_BUTTON = 0,
    EVENT_LORA_RX,
    EVENT_LORA_TX,
    EVENT_UART_RX,
    EVENT_ERROR,
    EVENT_WAKE_UP,
    EVENT_SLEEP,
    EVENT_SYSTEM_RESET,
    EVENT_TIMER_24h,
    EVENT_TIMER_20min
} EventId_t;

// Sources d'événements
#define SOURCE_BUTTON           0x01
#define SOURCE_LORA             0x02
#define SOURCE_UART             0x03
#define SOURCE_TIMER            0x04
#define SOURCE_SYSTEM           0x05

// === SYSTÈME DE WATCHDOG ===
// Identifiants des tâches pour le watchdog
typedef enum {
    WATCHDOG_TASK_DEFAULT = 0,
    WATCHDOG_TASK_LORA_RX,
    WATCHDOG_TASK_LORA_TX,
    WATCHDOG_TASK_APPLI,
    WATCHDOG_TASK_UART1,
    WATCHDOG_TASK_UART_TX,
    WATCHDOG_TASK_COUNT  // Nombre total de tâches surveillées
} watchdog_task_id_t;

// Structure pour le suivi des tâches
typedef struct {
    uint32_t last_heartbeat;    // Timestamp du dernier heartbeat
    uint32_t timeout_ms;        // Timeout en millisecondes
    uint8_t is_active;          // Tâche active ou non
    uint8_t error_count;        // Nombre d'erreurs consécutives
} watchdog_task_info_t;

// Configuration du watchdog
#define WATCHDOG_TIMEOUT_MS        30000   // 30 secondes par défaut
#define WATCHDOG_ERROR_THRESHOLD   3       // Nombre d'erreurs avant reset
#define WATCHDOG_CHECK_INTERVAL    5000    // Vérification toutes les 5 secondes

// Fonctions du système watchdog
void watchdog_init(void);
void watchdog_task_heartbeat(watchdog_task_id_t task_id);
void watchdog_task_start(watchdog_task_id_t task_id);
void watchdog_task_stop(watchdog_task_id_t task_id);
void watchdog_check_all_tasks(void);
void watchdog_reset_system(void);
uint8_t watchdog_is_task_alive(watchdog_task_id_t task_id);
void watchdog_print_status(void);
void watchdog_test_task_block(watchdog_task_id_t task_id, uint32_t duration_ms);

// Fonctions de diagnostic du reset
void display_reset_cause(void);
const char* get_reset_cause_string(uint32_t reset_flags);
void save_diagnostic_data(void);
void load_diagnostic_data(void);
void check_stack_usage(void);


#endif /* INC_FONCTIONS_H_ */
