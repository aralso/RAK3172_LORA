/*
 * functions.h
 *
 *  Created on: Sep 26, 2025
 *      Author: Tocqueville
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_


#include "main.h"
#include <stdio.h>
#include <stdarg.h>

#define END_NODE
#define EMETTEUR 'U'


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

#define MESS_BUFFER_SIZE 500
#define MESS_LG_MAX	100

// Niveaux de verbosité
#define LOG_LEVEL_ERROR    1
#define LOG_LEVEL_WARNING  2
#define LOG_LEVEL_INFO     3
#define LOG_LEVEL_DEBUG    4
#define LOG_LEVEL_VERBOSE  5

#define TIMER_PERIOD_MS  7000   // 50s

// Niveau de verbosité global (modifiable)
#define CURRENT_LOG_LEVEL  LOG_LEVEL_DEBUG

extern uint8_t code_erreur, comptage_erreur;
extern uint8_t err_donnee1, err_donnee2;


// Code erreur

     // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
     // Erreurs 20 a 7F : 4 fois        (Appli >0x70)
     // Erreurs 80 a FF : tout le temps (Appli > 0xD0)

#define code_erreur_envoi 0x20
#define code_erreur_dequeue 0x21

#define erreur_analog               0x42  // 40 et 41:periph
#define erreur_demar                0x43
#define erreur_util_rtos            0x43
#define erreur_tab                  0x44



#define dest_erreur	'1'
#define dest_log '1'

// Fonction principale de logging
void print_log(uint8_t level, const char* format, ...);

// Macros pour faciliter l'utilisation
#define LOG_ERROR(...)   print_log(LOG_LEVEL_ERROR,   __VA_ARGS__)
#define LOG_WARNING(...) print_log(LOG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_INFO(...)    print_log(LOG_LEVEL_INFO,    __VA_ARGS__)
#define LOG_DEBUG(...)   print_log(LOG_LEVEL_DEBUG,   __VA_ARGS__)
#define LOG_VERBOSE(...) print_log(LOG_LEVEL_VERBOSE, __VA_ARGS__)

// Fonction pour changer le niveau de verbosité à l'exécution
void set_log_level(uint8_t level);

// Fonction pour obtenir le niveau actuel
uint8_t get_log_level(void);
void init_functions(void);
uint8_t envoie_mess_ASC(const char* format, ...);
uint8_t envoie_mess_bin(const uint8_t *buf);
uint8_t deci (uint8_t val);
void envoi_code_erreur(void);

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

#endif /* INC_FUNCTIONS_H_ */
