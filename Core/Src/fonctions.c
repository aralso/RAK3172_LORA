/*
 * fonctions.c
 *
 *  Created on: Oct 1, 2025
 *      Author: Tocqueville
 */

#include <communication.h>
#include <fonctions.h>
#include "cmsis_os.h"
#include "timers.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// === SYSTÈME DE WATCHDOG ===
// Tableau de suivi des tâches
static watchdog_task_info_t watchdog_tasks[WATCHDOG_TASK_COUNT];

#define nb_erreurs_enregistrees  20
#define nb_erreurs_envoyees      30
#define nb_erreurs_unique        0x20
#define nb_erreurs_4_fois        0x60

uint8_t code_erreur, comptage_erreur;
uint8_t err_donnee1, err_donnee2;
uint8_t enr_erreur[nb_erreurs_enregistrees];          // Enr_erreur enregistre les 30 premiÃ¨res erreurs
uint8_t erreurs_unique[nb_erreurs_unique/8];    // 1:erreur dÃ©ja envoyÃ©e, plus d'envoi
uint8_t erreurs_4_fois[nb_erreurs_4_fois/4];       // Nb d'erreurs dÃ©ja envoyÃ©es, max 4

uint8_t nb_reset=0;


extern UART_HandleTypeDef huart2;
extern QueueHandle_t Event_QueueHandle;
extern osThreadId_t Appli_TaskHandle;
extern osThreadId_t LORA_TX_TaskHandle;
extern osThreadId_t LORA_RX_TaskHandle;
extern osThreadId_t Uart_RX_TaskHandle;

// Timers
TimerHandle_t HTimer_24h;
TimerHandle_t HTimer_20min;
TimerHandle_t HTimer_Watchdog;  // Timer pour la vérification périodique du watchdog

static void WatchdogTimerCallback(TimerHandle_t xTimer);
static void Timer24hCallback(TimerHandle_t xTimer);
static void Timer20minCallback(TimerHandle_t xTimer);

void init_functions(void)
{
	// Afficher la cause du reset au démarrage
	display_reset_cause();


	// creation timers
	 HTimer_24h = xTimerCreate(
	        "Timer24h",                          // Nom
	        pdMS_TO_TICKS(24*3600*1000),     // Période en ticks
	        pdTRUE,                             // Auto-reload
	        (void*)0,                           // ID optionnel
	        Timer24hCallback                     // Callback
	    );
	    if (HTimer_24h != NULL) xTimerStart(HTimer_24h, 0);

		 HTimer_20min = xTimerCreate(
		        "Timer20min",                          // Nom
		        pdMS_TO_TICKS(TIMER_PERIOD_MS),     // Période en ticks
		        pdTRUE,                             // Auto-reload
		        (void*)0,                           // ID optionnel
		        Timer20minCallback                     // Callback
		    );
		    if (HTimer_20min != NULL) xTimerStart(HTimer_20min, 0);

	// Initialisation du système de watchdog
	watchdog_init();
}


static void Timer24hCallback(TimerHandle_t xTimer)
{
	event_t evt = { EVENT_TIMER_24h, 0, 0 };
	if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
	{
	    LOG_ERROR("Event queue full - message lost");
	}
}

static void Timer20minCallback(TimerHandle_t xTimer)
{
	event_t evt = { EVENT_TIMER_20min, 0, 0 };
	if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
	{
	    LOG_ERROR("Event queue full - message lost");
	}
}

/**
 * @brief Retourne le temps restant avant expiration du timer, en millisecondes.
 * @param t Handle du timer FreeRTOS
 * @return Temps restant en ms (0 si expiré ou inactif)
 */
uint32_t TimerGetRemainingMs(TimerHandle_t t)
{
    if (t == NULL || xTimerIsTimerActive(t) == pdFALSE)
    {
        return 0; // Timer non actif ou invalide
    }

    TickType_t expiry = xTimerGetExpiryTime(t);
    TickType_t now = xTaskGetTickCount();

    if (expiry <= now)
    {
        return 0; // Déjà expiré ou en cours de callback
    }

    TickType_t remainingTicks = expiry - now;
    return (uint32_t) ((remainingTicks * 1000) / configTICK_RATE_HZ);
}

void check_stack_usage(void)
{
    UBaseType_t stack_high_water_mark;

    // Vérifier chaque tâche
    LOG_INFO("=== STACK USAGE REPORT ===");

    // Appli_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(Appli_TaskHandle);
    LOG_INFO("Appli_Task: %lu bytes free", stack_high_water_mark);

    // LORA_TX_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(LORA_TX_TaskHandle);
    LOG_INFO("LORA_TX_Task: %lu bytes free", stack_high_water_mark);

    // LORA_RX_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(LORA_RX_TaskHandle);
    LOG_INFO("LORA_RX_Task: %lu bytes free", stack_high_water_mark);

    // Uart_TX_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(Uart_TX_TaskHandle);
    LOG_INFO("Uart_TX_Task: %lu bytes free", stack_high_water_mark);

    // Uart1_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(Uart_RX_TaskHandle);
    LOG_INFO("Uart_RX_Task: %lu bytes free", stack_high_water_mark);

    //LOG_INFO("=== END STACK REPORT ===");
}


void envoi_code_erreur (void)        // envoie l'erreur a dest_erreur_reset
{
    uint8_t i3;

    // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
    // Erreurs 20 a 7F : 3 fois        (Appli >0x70)  comm:2,3 util/periph:4  radio:5  tab:6 conc/appli:7
    // Erreurs 80 a FF : tout le temps (Appli > 0xD0)

    if (comptage_erreur < nb_erreurs_envoyees)  // 30 envoyees, 20 enregistrees
    {
       i3 = 0;
       if (comptage_erreur < nb_erreurs_enregistrees)  enr_erreur[comptage_erreur]=code_erreur;
        comptage_erreur++;
        if (code_erreur < (nb_erreurs_4_fois+nb_erreurs_unique))
        {
            if (code_erreur < nb_erreurs_unique) // 0x20  envoi_une seule fois
            {
                if (code_erreur < nb_erreurs_unique)
                {
                    uint8_t index, bit;
                    index = code_erreur/8;  // si code=0xA =>index=1, bit=2;
                    bit = code_erreur % 8;
                    if (! (erreurs_unique[index] & (1<<bit)))
                    {
                        i3=1;
                        erreurs_unique[index] |= (1<<bit);
                    }
                }
            }
            else      // envoi 4 fois max
            {
                uint8_t index, bit, cpt;
                index = (code_erreur-nb_erreurs_unique)/4;  // si code=0xA =>index=3, bit=4;
                bit = (code_erreur % 4)*2;
                cpt = (erreurs_4_fois[index] >> bit) & 3 ;
                if  (cpt < 3)
                {
                    i3=1;
                    erreurs_4_fois[index] += (1 << bit);
                }
            }
        }
        else i3=1;  //  0x80 Ã  0xFF

        if (i3)
          {
          #ifdef dest_erreur
           uint8_t message[30];
           message[0] = dest_erreur;
           message[1] ='E';
           message[2] ='L';
           message[3] = nb_reset + 48;
           message[4] = comptage_erreur + 48;
           message[5] = ':';
           message[6] =deci(code_erreur >> 4 );
           message[7] =deci(code_erreur & 15);
           message[8] = '-';
           message[9] = err_donnee1+48;
           message[10] = err_donnee2+48;
           i3=9;
           if (err_donnee1) i3=10;
           if (err_donnee2) i3=11;
           message[i3] = 0;
           envoie_mess_ASC((const char*)message);
          #endif
          }
    }
    code_erreur=0;
    err_donnee1=0;
    err_donnee2=0;
}

uint8_t deci (uint8_t val) //transforme un char hexa en son charactere ASCII   0->48 9->57 A->65 F->70 G->71
{
  uint8_t resul;
  if (val > 36)
    resul = 91;  // caractere apres Z pour indiquer une erreur
  else if (val < 10)
    resul = val + 48;
  else
    resul = val + 55;
  return resul;
}

// === IMPLÉMENTATION DU SYSTÈME DE WATCHDOG ===

/**
 * @brief Initialise le système de watchdog
 */
void watchdog_init(void)
{
    LOG_INFO("Initializing watchdog system...");

    // Initialiser toutes les tâches comme inactives
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        watchdog_tasks[i].last_heartbeat = 0;
        watchdog_tasks[i].timeout_ms = WATCHDOG_TIMEOUT_MS;
        watchdog_tasks[i].is_active = 0;
        watchdog_tasks[i].error_count = 0;
    }

    // Créer le timer de vérification du watchdog
    HTimer_Watchdog = xTimerCreate(
        "WatchdogTimer",                    // Nom
        pdMS_TO_TICKS(WATCHDOG_CHECK_INTERVAL), // Période
        pdTRUE,                            // Auto-reload
        (void*)0,                          // ID
        WatchdogTimerCallback              // Callback
    );

	#ifdef WATCHDOG
		if (HTimer_Watchdog != NULL) {
			xTimerStart(HTimer_Watchdog, 0);
			LOG_INFO("Watchdog timer started");
		} else {
			LOG_ERROR("Failed to create watchdog timer");
		}
	#endif
}

/**
 * @brief Callback du timer de watchdog
 */
static void WatchdogTimerCallback(TimerHandle_t xTimer)
{
    watchdog_check_all_tasks();
}

/**
 * @brief Démarre la surveillance d'une tâche
 */
void watchdog_task_start(watchdog_task_id_t task_id)
{
    if (task_id < WATCHDOG_TASK_COUNT) {
        watchdog_tasks[task_id].is_active = 1;
        watchdog_tasks[task_id].last_heartbeat = xTaskGetTickCount();
        watchdog_tasks[task_id].error_count = 0;
        LOG_DEBUG("Watchdog started for task %d", task_id);
    }
}

/**
 * @brief Arrête la surveillance d'une tâche
 */
void watchdog_task_stop(watchdog_task_id_t task_id)
{
    if (task_id < WATCHDOG_TASK_COUNT) {
        watchdog_tasks[task_id].is_active = 0;
        LOG_DEBUG("Watchdog stopped for task %d", task_id);
    }
}

/**
 * @brief Enregistre un heartbeat d'une tâche
 */
void watchdog_task_heartbeat(watchdog_task_id_t task_id)
{
    if (task_id < WATCHDOG_TASK_COUNT && watchdog_tasks[task_id].is_active) {
        watchdog_tasks[task_id].last_heartbeat = xTaskGetTickCount();
        watchdog_tasks[task_id].error_count = 0; // Reset du compteur d'erreurs
    }
}

/**
 * @brief Vérifie si une tâche est vivante
 */
uint8_t watchdog_is_task_alive(watchdog_task_id_t task_id)
{
    if (task_id >= WATCHDOG_TASK_COUNT || !watchdog_tasks[task_id].is_active) {
        return 1; // Tâche non surveillée ou inactive = considérée comme vivante
    }

    uint32_t current_time = xTaskGetTickCount();
    uint32_t elapsed_ms = (current_time - watchdog_tasks[task_id].last_heartbeat) * 1000 / configTICK_RATE_HZ;

    return (elapsed_ms < watchdog_tasks[task_id].timeout_ms);
}

/**
 * @brief Vérifie toutes les tâches surveillées
 */
void watchdog_check_all_tasks(void)
{
    uint32_t current_time = xTaskGetTickCount();
    uint8_t critical_errors = 0;

    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        if (!watchdog_tasks[i].is_active) {
            continue; // Tâche non surveillée
        }

        uint32_t elapsed_ms = (current_time - watchdog_tasks[i].last_heartbeat) * 1000 / configTICK_RATE_HZ;

        if (elapsed_ms >= watchdog_tasks[i].timeout_ms) {
            watchdog_tasks[i].error_count++;
            LOG_ERROR("Task %d timeout! Elapsed: %lu ms, Error count: %d",
                     i, elapsed_ms, watchdog_tasks[i].error_count);

            if (watchdog_tasks[i].error_count >= WATCHDOG_ERROR_THRESHOLD) {
                LOG_ERROR("Task %d exceeded error threshold, system will reset!", i);
                critical_errors++;
            }
        }
    }

    // Si trop d'erreurs critiques, redémarrer le système
    if (critical_errors > 0) {
        watchdog_reset_system();
    }
}

/**
 * @brief Affiche le statut de toutes les tâches surveillées
 */
void watchdog_print_status(void)
{
    LOG_INFO("=== WATCHDOG STATUS ===");

    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        if (watchdog_tasks[i].is_active) {
            uint32_t current_time = xTaskGetTickCount();
            uint32_t elapsed_ms = (current_time - watchdog_tasks[i].last_heartbeat) * 1000 / configTICK_RATE_HZ;

            LOG_INFO("Task %d: Active, Last heartbeat: %lu ms ago, Errors: %d",
                    i, elapsed_ms, watchdog_tasks[i].error_count);
        } else {
            LOG_INFO("Task %d: Inactive", i);
        }
    }

    LOG_INFO("=== END WATCHDOG STATUS ===");
}

/**
 * @brief Redémarre le système en cas d'erreur critique
 */
void watchdog_reset_system(void)
{
    LOG_ERROR("WATCHDOG: Critical system failure detected, initiating reset...");

    // Envoyer un message d'erreur avant le reset
    envoie_mess_ASC("WATCHDOG: System reset due to task failures");

    // Attendre un peu pour que le message soit envoyé
    osDelay(1000);

    // Forcer un reset système
    NVIC_SystemReset();
}

/**
 * @brief Fonction de test pour simuler une tâche bloquée
 * @param task_id: ID de la tâche à bloquer
 * @param duration_ms: Durée du blocage en millisecondes
 */
void watchdog_test_task_block(watchdog_task_id_t task_id, uint32_t duration_ms)
{
    LOG_WARNING("WATCHDOG TEST: Blocking task %d for %lu ms", task_id, duration_ms);

    // Arrêter temporairement la surveillance de cette tâche
    watchdog_task_stop(task_id);

    // Simuler un blocage
    osDelay(duration_ms);

    // Redémarrer la surveillance
    watchdog_task_start(task_id);

    LOG_INFO("WATCHDOG TEST: Task %d unblocked", task_id);
}

// === DIAGNOSTIC DE LA CAUSE DU RESET ===

/**
 * @brief Affiche la cause du reset au démarrage
 */
void display_reset_cause(void)
{
    // Charger les données de diagnostic de la session précédente
    load_diagnostic_data();

    // Lire les flags de reset depuis RCC
    uint32_t reset_flags = RCC->CSR;

    LOG_INFO("=== SYSTEM RESET DIAGNOSTIC ===");
    LOG_INFO("Reset flags: 0x%08lX", reset_flags);

    // Analyser chaque cause de reset
    if (reset_flags & RCC_CSR_LPWRRSTF) {
        LOG_INFO("Reset cause: Low Power (LPWR)");
    }
    if (reset_flags & RCC_CSR_WWDGRSTF) {
        LOG_INFO("Reset cause: Window Watchdog (WWDG)");
    }
    if (reset_flags & RCC_CSR_IWDGRSTF) {
        LOG_INFO("Reset cause: Independent Watchdog (IWDG)");
    }
    if (reset_flags & RCC_CSR_SFTRSTF) {
        LOG_INFO("Reset cause: Software Reset (NVIC_SystemReset)");
    }
    if (reset_flags & RCC_CSR_BORRSTF) {
        LOG_INFO("Reset cause: Brown On Reset (BOR)");
    }
    if (reset_flags & RCC_CSR_PINRSTF) {
        LOG_INFO("Reset cause: External Pin Reset (NRST)");
    }
    if (reset_flags & RCC_CSR_OBLRSTF) {
        LOG_INFO("Reset cause: Option Byte Loader Reset");
    }

    // Si aucun flag n'est défini, c'est un reset inconnu
    if (!(reset_flags & (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_IWDGRSTF |
                         RCC_CSR_SFTRSTF | RCC_CSR_BORRSTF | RCC_CSR_PINRSTF |
                         RCC_CSR_OBLRSTF ))) {
        LOG_WARNING("Reset cause: Unknown or multiple causes");
    }

    // Afficher le nombre de resets
    static uint32_t reset_count = 0;
    reset_count++;
    LOG_INFO("Reset count since last power-on: %lu", reset_count);

    // Afficher le temps d'uptime si disponible
    uint32_t uptime_ms = HAL_GetTick();
    LOG_INFO("System uptime: %lu ms", uptime_ms);

    LOG_INFO("=== END RESET DIAGNOSTIC ===");

    // Effacer les flags de reset pour le prochain démarrage
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

/**
 * @brief Retourne une chaîne décrivant la cause du reset
 * @param reset_flags: Flags de reset du registre RCC->CSR
 * @return Chaîne descriptive de la cause
 */
const char* get_reset_cause_string(uint32_t reset_flags)
{
    if (reset_flags & RCC_CSR_LPWRRSTF) {
        return "Low Power Reset";
    }
    if (reset_flags & RCC_CSR_WWDGRSTF) {
        return "Window Watchdog Reset";
    }
    if (reset_flags & RCC_CSR_IWDGRSTF) {
        return "Independent Watchdog Reset";
    }
    if (reset_flags & RCC_CSR_SFTRSTF) {
        return "Software Reset";
    }
    if (reset_flags & RCC_CSR_BORRSTF) {
        return "Brown On Reset";
    }
    if (reset_flags & RCC_CSR_PINRSTF) {
        return "External Pin Reset";
    }
    if (reset_flags & RCC_CSR_OBLRSTF) {
        return "Option Byte Loader Reset";
    }

    return "Unknown Reset Cause";
}

// === SAUVEGARDE DE DONNÉES DE DIAGNOSTIC ===

// Structure pour sauvegarder des informations de diagnostic
typedef struct {
    uint32_t magic_number;        // Nombre magique pour valider la structure
    uint32_t reset_count;         // Compteur de resets
    uint32_t last_uptime_ms;      // Temps de fonctionnement avant le reset
    uint32_t last_reset_cause;    // Cause du dernier reset
    uint32_t watchdog_errors;     // Nombre d'erreurs watchdog
    uint32_t timestamp;           // Timestamp du dernier enregistrement
} diagnostic_data_t;

#define DIAGNOSTIC_MAGIC_NUMBER   0xDEADBEEF
#define DIAGNOSTIC_DATA_ADDR      (0x20000000 + 0x2000)  // Adresse en RAM backup

/**
 * @brief Sauvegarde des données de diagnostic en RAM backup
 */
void save_diagnostic_data(void)
{
    static uint32_t reset_count = 0;
    static uint32_t watchdog_errors = 0;

    diagnostic_data_t *diag = (diagnostic_data_t*)DIAGNOSTIC_DATA_ADDR;

    // Incrémenter le compteur de resets
    reset_count++;

    // Sauvegarder les données
    diag->magic_number = DIAGNOSTIC_MAGIC_NUMBER;
    diag->reset_count = reset_count;
    diag->last_uptime_ms = HAL_GetTick();
    diag->last_reset_cause = RCC->CSR;
    diag->watchdog_errors = watchdog_errors;
    diag->timestamp = HAL_GetTick();

    LOG_DEBUG("Diagnostic data saved: reset_count=%lu, uptime=%lu ms",
              reset_count, diag->last_uptime_ms);
}

/**
 * @brief Récupère et affiche les données de diagnostic sauvegardées
 */
void load_diagnostic_data(void)
{
    diagnostic_data_t *diag = (diagnostic_data_t*)DIAGNOSTIC_DATA_ADDR;

    // Vérifier si les données sont valides
    if (diag->magic_number == DIAGNOSTIC_MAGIC_NUMBER) {
        LOG_INFO("=== PREVIOUS SESSION DIAGNOSTIC ===");
        LOG_INFO("Previous reset count: %lu", diag->reset_count);
        LOG_INFO("Previous uptime: %lu ms", diag->last_uptime_ms);
        LOG_INFO("Previous reset cause: %s", get_reset_cause_string(diag->last_reset_cause));
        LOG_INFO("Watchdog errors: %lu", diag->watchdog_errors);
        LOG_INFO("Last save timestamp: %lu ms", diag->timestamp);
        LOG_INFO("=== END PREVIOUS SESSION ===");
    } else {
        LOG_INFO("No previous diagnostic data found (first boot)");
    }
}


// Hook appelé si une tâche dépasse sa pile
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    char msg[100];
    int len = snprintf(msg, sizeof(msg), "-- Stack overflow in task: %s\r\n", pcTaskName);
    msg[0] = dest_log;
    msg[1] = My_Address;
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);

    HAL_Delay(2000);
    NVIC_SystemReset();
    //taskDISABLE_INTERRUPTS();
    //for(;;);
}

// Hook appelé si malloc échoue
void vApplicationMallocFailedHook(void)
{
    char msg[] = "-- Malloc failed!\r\n";
    msg[0] = dest_log;
    msg[1] = My_Address;
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);

    HAL_Delay(2000);
    NVIC_SystemReset();
    //taskDISABLE_INTERRUPTS();
    //for(;;);
}

