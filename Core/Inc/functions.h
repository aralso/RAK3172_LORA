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

// Structure pour les événements
typedef struct {
    uint8_t type;           // Type d'événement
    uint8_t source;         // Source de l'événement
    uint16_t data;          // Données de l'événement
} event_t;

// Types d'événements
#define EVENT_INIT              0x01
#define EVENT_BUTTON_PRESSED    0x02
#define EVENT_LORA_RECEIVED    0x03
#define EVENT_LORA_SENT        0x04
#define EVENT_UART_RECEIVED    0x05
#define EVENT_TIMER_EXPIRED     0x06
#define EVENT_ERROR             0x07
#define EVENT_WAKE_UP           0x08
#define EVENT_SLEEP             0x09
#define EVENT_SYSTEM_RESET     0x0A

// Sources d'événements
#define SOURCE_BUTTON           0x01
#define SOURCE_LORA             0x02
#define SOURCE_UART             0x03
#define SOURCE_TIMER            0x04
#define SOURCE_SYSTEM           0x05

// Niveaux de verbosité
#define LOG_LEVEL_ERROR    1
#define LOG_LEVEL_WARNING  2
#define LOG_LEVEL_INFO     3
#define LOG_LEVEL_DEBUG    4
#define LOG_LEVEL_VERBOSE  5

// Niveau de verbosité global (modifiable)
#define CURRENT_LOG_LEVEL  LOG_LEVEL_DEBUG

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


#endif /* INC_FUNCTIONS_H_ */
